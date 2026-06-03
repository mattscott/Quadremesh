# 04 — QuadriFlow integer-grid-map port (P6 completion)

The plan to make `ExtractionMode.PureQuad` **topology-safe** by faithfully porting QuadriFlow's
integer solve + extraction. Reference source is cloned read-only at
`tools/reference-oracle/_external/quadriflow/src` (MIT; read for understanding, reimplemented clean).
We validate every step against the committed QuadriFlow goldens (`tests/golden/quadriflow/`).

## Why the first attempt fell short

`IntegerOffsetSolver` (committed) cancels a **decoupled** per-dimension divergence using a global
vertex spanning-tree comb. Two errors vs the real algorithm:
1. The constraint must be **rotation-coupled**: `Σ_j rshift90(edge_diff[e_j], orient[f][j]) = 0`. The
   per-edge 90° rotation `orient[f][j]` mixes the two lattice axes — ignoring it solves the wrong
   problem near singularities.
2. The comb must be a **disjoint-orient-tree over faces that excludes singularities**, leaving them as
   clean cone points. A vertex spanning tree instead smears a seam through them.

Result: 88% even-gon quads but wrong genus (giant faces at singularities). Benchmarked gap to
QuadriFlow: **266 vs 8** irregular vertices on the sphere (33×).

## The reference pipeline (`Parametrizer::ComputeIndexMap`, parametrizer.cpp:21)

```
BuildEdgeInfo()            edge_diff, edge_values, face_edgeIds       parametrizer-int.cpp:12
BuildIntegerConstraints()  face_edgeOrients, comb, balance, vars      parametrizer-int.cpp:53
ComputeMaxFlow()           min-cost flow -> consistent edge_diff      parametrizer-int.cpp:419 + flow.hpp
subdivide_edgeDiff()       split edges with |edge_diff|>1 to units    subdivide.cpp
FixFlipHierarchy/Sat()     local boolean SAT removes flipped quads    parametrizer-flip.cpp, localsat.cpp
optimize_positions_*()     final vertex placement                     optimizer.cpp
AdvancedExtractQuad()      build quad vertices + faces from edge_diff  parametrizer-mesh.cpp:198
FixValence()               doublet / valence cleanup                  parametrizer-mesh.cpp
```

Our existing components cover the front end (fields P3/P4, `HalfEdgeMesh`, `MinCostFlow`,
`SingularityDetector`). This roadmap is the integer back end.

---

## Foundations — DONE ✅

| Step | What | C++ ref | Test (status) |
|------|------|---------|---------------|
| F1 | `Vec2i`, `Rshift90`, cross-field `Match`/`Rank`, `DisjointOrientTree` | field-math.hpp:79/85, disajoint-tree.hpp | `LatticeMathTests` ✅ |

---

## Stage A — Integer constraint setup *(no flow yet; all locally checkable)*

| Step | What | C++ ref | Testable acceptance |
|------|------|---------|---------------------|
| A1 ✅ | `face_edgeIds[f][j]` + `edge_values[e]` (endpoints) — map each face's 3 half-edges to undirected edge ids | parametrizer-int.cpp:12–50 | every interior edge is referenced by exactly 2 faces; each face has 3 distinct edge ids — `QuadriFlowSolverTests` ✅ |
| A2 ✅ | `edge_diff[e]` = round of the position-field offset `lo→hi` in `lo`'s lattice frame (lo = lower vertex id) | parametrizer-int.cpp:27–34 | reconstructs the rounded offset; on a flat patch with axis-aligned anchors equals exact integer steps — `QuadriFlowSolverTests` ✅ |
| A3 ✅ | `face_edgeOrients[f][j]` from cross ranks (rank1 = v1→v0, rank2 = v2→v0) | parametrizer-int.cpp:76–104 | **telescoping**: `Σ_j rshift90(edge_diff_j, orient[f][j]) = (0,0)` per face on a flat aligned patch — `QuadriFlowSolverTests` ✅ |
| A4 ✅ | Orientation singularities (per face) | ComputeOrientationSingularities; we have `SingularityDetector` | Σ indices = χ (sphere 8, torus 0); sphere has more singular faces than torus — `QuadriFlowSolverTests` ✅ |
| A5 ✅ | Comb: `DisjointOrientTree` merge non-singular adjacent faces first, then singular; add `Orient(f)` to `face_edgeOrients` | parametrizer-int.cpp:115–167 | >90% of non-singular shared edges combed-consistent (orients differ by 2); >70% faces have zero divergence — `QuadriFlowSolverTests` ✅ |

**Stage A milestone ✅:** per-face integer divergence `D_f = Σ_j rshift90(edge_diff[e_j], orient[f][j])`
is computable and **zero on the vast majority of faces**, non-zero only in singular neighbourhoods.

---

## Stage B — Min-cost-flow integer solve

| Step | What | C++ ref | Testable acceptance |
|------|------|---------|---------------------|
| B1 ✅ | `edge_to_constraints` + per-constraint divergence (`initial`) via the `rshift90((e*2+1,e*2+2),orient)` encoding | optimizer.cpp:1231–1252 | `initial[f*2+k]` reproduces `FaceDivergence(f)` exactly — `QuadriFlowSolverTests` ✅ |
| B2 ✅ | Balance: nudge same-sign/boundary `edge_diff` so each flow component's net divergence = 0 | parametrizer-int.cpp:293–416 | flow then fully routes (see B3) — `QuadriFlowSolverTests` ✅ |
| B3 ✅ | Conservation-arc graph (opposite-sign components) + source/sink from divergence → `MinCostFlow` min-cost-max-flow → apply (`edge_diff -= sgn·flow`), `edge_capacity` retry | optimizer.cpp:1230–1336 + flow.hpp:175 | **every non-singular face has D_f = (0,0)** after solve (sphere & torus: 0 residual) — `QuadriFlowSolverTests` ✅ |
| B4 *(opt)* | Multires `DownsampleEdgeGraph` for speed | hierarchy.cpp | deferred (flat solve is fine at corpus scale) |

**Stage B milestone ✅:** a globally consistent integer grid map — `edge_diff` closes **every**
non-singular face (0 residual on sphere & torus), singularities are isolated cone points. This is the
core fix the first attempt lacked, verified independent of extraction.

---

## Stage C — Extraction (`AdvancedExtractQuad`)

| Step | What | C++ ref | Testable acceptance |
|------|------|---------|---------------------|
| C1 | `subdivide_edgeDiff`: split any edge with `|edge_diff|>1` into unit steps | subdivide.cpp | all `edge_diff` become 0 or unit afterwards |
| C2 | Collapse `edge_diff==(0,0)` edges (union-find) → output vertices `O_compact`, `Vset` | parametrizer-mesh.cpp:198+ | collapse partitions input vertices; positions averaged + projected |
| C3 | Build quad faces from the integer structure (`o2e`/`v2o` dedge tracing) → `F_compact` | parametrizer-mesh.cpp / parametrizer.cpp:148–230 | output is **all quads, manifold, watertight, correct χ** |
| C4 | `FixValence` + doublet removal | parametrizer-mesh.cpp | no valence-1/2; irregular count drops |

**Stage C milestone:** end-to-end pure-quad mesh with correct topology.

> **Finding (reuse experiment, after Stage B):** feeding the now-consistent `edge_diff` into the
> existing rotation-system extractor is **not** sufficient — sphere irregular vertices land at ~410–440
> vs QuadriFlow's 8, with or without finer field resolution. The geometric rotation-system face trace
> cannot reproduce the integer iso-structure. **C3 must port `BuildTriangleManifold`** (dedge-cycle
> tracing on the collapsed graph), which consumes `DownsampleEdgeGraph` (multiresolution zero-diff
> collapse, hierarchy.cpp) and `subdivide_edgeDiff`. This is ~600–800 lines, the highest-risk part of
> the whole port. The integer solve (Stage B) is correct; the gap is purely extraction.

---

## Stage D — Validate against the oracle

| Step | What | Testable acceptance |
|------|------|---------------------|
| D1 | Wire the new solver behind `ExtractionMode.PureQuad`; run `ReferenceComparisonTests` | irregular-vertex count **within ~2–4×** of the QuadriFlow goldens (sphere ≈8, torus ≈22) |

---

## Stage E — Robustness polish: flip fixing *(post-core; additive)*

The core (A–C) can still leave a few **flipped or degenerate quads** where the integer field folds.
QuadriFlow fixes these locally.

| Step | What | C++ ref | Testable acceptance |
|------|------|---------|---------------------|
| E1 | Quad energy metric (per-quad angle distortion; detects flips via sign/area) | `QuadEnergy`, parametrizer-flip.cpp:15–69 | flagged quads = exactly the inverted/degenerate ones on a synthetic folded patch |
| E2 | `FixFlipHierarchy`: coarse-to-fine local re-assignment of `edge_diff` to lower total flip energy | parametrizer-flip.cpp:168 | flip count strictly decreases; no new non-manifold edges |
| E3 | `FixFlipSat`: encode the remaining local validity as boolean SAT and solve | parametrizer-flip.cpp:175 + localsat.cpp | zero flipped quads after; quad angle energy below threshold |

> **Dependency note:** the reference shells out to a SAT solver (`localsat.cpp` runs MiniSAT via
> `system()`, or the bundled MapleCOMSPS) and gates it behind `-sat`. For a **clean-room managed
> build we won't shell out** — E3 becomes either a small managed DPLL over the tiny local clauses
> (bounded variables) or a combinatorial local fixer. E2 alone removes most flips; E3 is the last mile.
>
> **Decision (gated on E2's residual flip count):** prefer **local enumeration** over the tiny
> bounded patches (no solver). Only if hard residual cases remain do we reimplement a CDCL engine —
> see the MiniSat fallback below.

### E3 fallback — if a general SAT solver is genuinely needed: port MiniSat (clean-room)

Reference: **[niklasso/minisat](https://github.com/niklasso/minisat)** — the canonical minimal SAT
solver by Eén & Sörensson, explicitly designed to be reimplemented. **License: MIT** (verified) →
compatible with our clean-room posture; reimplement (don't copy), attribute in `NOTICE` alongside
Instant Meshes / QuadriFlow. No NuGet, no native dep — stays inside `Quadremesh.Core`'s
zero-dependency rule.

**What to port (and what to skip):**

| Repo path | Role | Port? |
|-----------|------|-------|
| `minisat/core/Solver.cc` (~700 LOC) | the CDCL engine: `propagate` (watched literals), `analyze` (1-UIP conflict learning), `search`, VSIDS activity, restarts | **yes** — the only substantive piece |
| `minisat/core/Solver.h` (~340 LOC) | solver state / public API (`newVar`, `addClause`, `solve`) | yes (as the C# class surface) |
| `minisat/core/SolverTypes.h` (~400 LOC) | `Lit`, `lbool`, `Clause` (packed literal storage) | yes, but trim — a `struct Lit`, an enum tri-state, and a `int[]`-backed clause suffice |
| `minisat/mtl/` (`Vec`, `Heap`, `IntMap`, `Sort`, `Alg`) | template containers | **skip** — replace with BCL: `List<T>`, `PriorityQueue<T,double>` (VSIDS order), arrays |
| `minisat/utils/`, `minisat/simp/`, `Dimacs.h`, `Main.cc` | CLI, parsing, simplifying preprocessor | **skip** — we call the solver in-process on tiny instances; no DIMACS, no preprocessing |

**Estimated clean-room size:** ~600–900 lines of C# (the templates fold into the BCL, so it's
smaller than the raw C++ line count). Self-contained, single file or a small `Sat/` folder in Core.

**Validation if we build it:** unit-test against known SAT/UNSAT instances (pigeonhole `PHP(n+1,n)`
is UNSAT; small random 3-SAT with a planted solution is SAT) before wiring into `FixFlipSat`. The
local QuadriFlow instances are tiny (`max_depth = 0`, ~`3·n` vars over a handful of edges), so even a
plain **DPLL** (no clause learning) would suffice — CDCL is only worth it if profiling says so.

## Stage F — Position smoothing *(post-core; cosmetic)*

Final placement of the output vertices so the quads are well-shaped and faithful to the surface.

| Step | What | C++ ref | Testable acceptance |
|------|------|---------|---------------------|
| F1 | `optimize_positions_fixed`: least-squares solve for output vertex positions consistent with `edge_diff`, then project to the surface | optimizer.cpp:967 | per-quad shape energy drops vs the raw averaged positions; max surface (Hausdorff) deviation stays below threshold |
| F2 | `optimize_positions_sharp`: same, honouring sharp/boundary constraints (needs Stage G) | optimizer.cpp:733 | sharp/boundary vertices stay on their constraint curves |

> Cosmetic for closed smooth inputs (our averaged-anchor + surface projection already lands on the
> surface); matters most once features (Stage G) are in.

## Stage G — Sharp edges & boundaries *(this is P7 scope; tracked here for completeness)*

Hard-surface creases and open boundaries. **Belongs to P7 (art direction)**, but it threads through
the integer solve, so it's listed here so the whole picture is visible.

| Step | What | C++ ref | Testable acceptance |
|------|------|---------|---------------------|
| G1 | `ComputeSharpEdges`: mark crease edges (dihedral angle) and boundary edges | parametrizer-mesh.cpp:124 | creases on the hard-surface corpus box detected; cylinder boundary edges marked |
| G2 | Constrain the integer solve on sharp/boundary edges (`allow_changes`, axis-locking `edge_diff`) | parametrizer.cpp:33–72, 60–95 | sharp edges become continuous quad edge loops; `edge_diff` along them stays axis-aligned |
| G3 | Boundary loop preservation + `FixHoles` | parametrizer-flip.cpp:72–166 | open meshes (cylinder) keep their boundary loops; output is watertight where input was |

> Until G lands, hard-surface (box) and open (cylinder) inputs may still get wrong genus in PureQuad
> — exactly the cases flagged in the P5/P6 commits. The closed smooth corpus (sphere, torus, head)
> is fully covered by A–C.

## Priority summary

```
Topology-safe pure quad (the goal):   A1–A5 → B1–B3 → C1–C3 → D1   [core, required]
Clean, flip-free output:              + E1–E3                      [robustness, additive]
Best-shaped quads:                    + F1                          [cosmetic]
Hard-surface & open meshes:           + G1–G3                       [P7 — separate phase]
```

## Risk & sequencing notes

- Each step is **independently testable** — we never integrate a stage before its local test passes.
- The highest-risk steps are **B3** (flow graph construction — many sign/index conventions) and **C3**
  (dedge face tracing). We validate B3 by the zero-divergence invariant *before* touching extraction.
- Minimum viable pure-quad = **A1–A5 + B1–B3 + C1–C3**. SAT/position polish is additive.
- Every numeric claim is checkable against the committed goldens via the reference oracle.

## Current status

- ✅ F1 (foundations) implemented + tested (`LatticeMathTests`).
- ⏸ Paused before Stage A for roadmap review.

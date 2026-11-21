# 05 — Research + plan: topology-safe pure-quad at singularities

Focused companion to [04-quadriflow-integer-port.md](04-quadriflow-integer-port.md). The integer solve
(Stage A+B) is done and validated: `edge_diff` closes every non-singular face. The remaining problem
is **extraction**: turning the consistent `edge_diff` into a *manifold, watertight, all-quad* mesh,
including correct behaviour at the orientation singularities (cone points).

## Current state (commit 1e0ccbd)

`QuadriFlowSolver.ExtractQuadMesh` collapses zero-offset edges and builds each quad from the two
triangles sharing a (±1,±1) diagonal edge (parametrizer-flip.cpp:438–460). Result on the corpus:

- **100% quads**, irregular-vertex count **3–7× QuadriFlow** (sphere 55 vs 8, torus 56 vs 22) — down
  from ~50× with the old rotation-system extractor.
- **Not manifold:** sphere χ=−314, 179 non-manifold edges, 681 boundary edges (holes).

Two distinct defects, two distinct causes:

| Defect | Cause | Fix |
|--------|-------|-----|
| Non-manifold edges | A collapsed group spans multiple fans; quads on both sides wrongly share one vertex | **Part A vertex-splitting** (fan-walk + colour-split) |
| Holes (boundary edges) | At 1:1 resolution the arbitrary triangulation diagonals don't align with the lattice, so some quad cells have no single (1,1) diagonal edge | **Finer field resolution + downsample**, so quads emerge from the integer iso-structure, not triangulation diagonals |

## QuadriFlow's full extraction pipeline (the reference)

`Parametrizer::ComputeIndexMap` (parametrizer.cpp:21) after the integer solve:

```
subdivide_edgeDiff()        split any edge with |edge_diff component| > 1 into unit steps   subdivide.cpp
AdvancedExtractQuad()       collapse + manifold-ize + build quads                            parametrizer-flip.cpp:198
  ├ DownsampleEdgeGraph(-1) full zero-diff collapse → coarse F2E / E2F / EdgeDiff / FQ       hierarchy.cpp
  ├ disajoint_tree          merge zero-diff edges → O_compact, Vset (output vertices)
  └ BuildTriangleManifold   Part A: dedge fan-walk → triangle_vertices, colour-split         parametrizer-flip.cpp:267–410
                            Part B: quads from (±1,±1) diagonals  ← DONE                     parametrizer-flip.cpp:438–460
FixValence()                doublet / valence-2 cleanup                                       parametrizer-mesh.cpp
(FixHoles)                  fill boundary loops with quads                                    parametrizer-flip.cpp:72–166
```

### Part A — manifold-ization (the fiddliest piece)

`BuildTriangleManifold` lines 291–410. Two phases:

1. **Fan assignment (291–322).** Build the coarse dual `E2E[face*3+corner]` (twin half-edge). For each
   unassigned corner, walk the fan around its vertex via `E2E` (`deid = E2E[deid/3*3 + (deid+2)%3]`),
   assigning every corner in that fan one output vertex id. Handles the boundary case by also walking
   the other direction.
2. **Colour-splitting (324–410).** Iterate until stable: for each output vertex, walk its incident
   dedges; a repeated directed `(v1,v2)` pair (tracked in `loc`) means the fan closed and re-opened —
   a pinch — so split the vertex into a new colour/id. This is what guarantees manifoldness. Degenerate
   triangles (`pt[i]==pt[j]`) get their `E2E` links severed first.

### The singularity / cone-point handling

Orientation singularities are faces left out of the comb; their `edge_diff` does **not** sum to zero
(the cone charge stays there). In extraction:

- Around a singular face the lattice has a defect — the fan around a nearby output vertex is **valence
  3 or 5**, not 4. Part A's fan-walk naturally produces these irregular vertices (cone points); the
  quads around them are still well-formed.
- The colour-split is what keeps the cone neighbourhood manifold (a singularity is exactly where a
  naïve collapse would pinch). So **Part A is the singularity-safety mechanism** — there is no separate
  "singularity code"; correct fan-walk + split *is* the cone-point handling.
- `subdivide_edgeDiff` matters near singularities too: the integer solve can leave a `|diff|=2` edge
  adjacent to a cone, which must be split so the cone is a clean valence-3/5 vertex.

## Plan — bite-sized, testable (extends Stage C of doc 04)

| Step | What | Test |
|------|------|------|
| C0 | Decouple field resolution: run fields/integer at ℓ/2 (or finer), lattice scale = ℓ | more zero-diff edges; collapse ratio ≈ 4:1 |
| C1 | `subdivide_edgeDiff`: on a mutable mesh, split edges with `|component|>1`; re-derive `edge_diff`/orients/ids for the new edges | after: every edge_diff component ∈ {−1,0,1} |
| C2 | Full zero-diff collapse → coarse `F2E`/`E2F`/`EdgeDiff`/`FQ` (own union-find + quotient build; not the full multires `DownsampleEdgeGraph`) | coarse edges are unit/diagonal; coarse faces non-degenerate |
| C3a | Part A fan-walk → `triangle_vertices` | every coarse-triangle corner assigned; fans consistent |
| C3b | Part A colour-split | resulting quad graph is **edge-manifold** (no edge in >2 quads) |
| C3c | Part B quads from diagonals (DONE) on the manifold-ized vertices | **all-quad, watertight, χ matches input** (sphere 2, torus 0) |
| C4 | `FixValence` + doublet removal; optional `FixHoles` | no valence-1/2; no boundary on closed input |
| D1 | Re-measure vs oracle | irregular vertices **within ≤2–4×** (sphere ≈8, torus ≈22) |

## Risk notes

- **C3b colour-split** is the single fiddliest function in the port (the `loc`/colour bookkeeping). Test
  it in isolation on a synthetic pinch (two fans sharing one collapsed vertex) before the corpus.
- **C2 vs full DownsampleEdgeGraph:** we only need the *coarsest* fully-collapsed level, so a direct
  union-find quotient (with rotation bookkeeping for the surviving edges' orients) should replace the
  reference's multiresolution machinery — smaller and lower-risk, but the orient bookkeeping across a
  collapse must be ported carefully (`mToUpperOrients`).
- **C1 subdivide** needs a mutable mesh (e.g. reuse `DynamicTriangleMesh`); re-deriving per-edge
  fields after a split is where bugs hide — assert `edge_diff` consistency after every split in tests.
- Keep `QuadDominant` the default until D1 passes; `PureQuad` stays opt-in.

## Update — holes need FixFlip, not (only) FixHoles

After porting `QuadriFlowExtractor` (coarse graph + Part A fan-walk/colour-split + Part B), topology
improved markedly (sphere χ −314 → −120, non-manifold edges 179 → 58) but **holes remain** (sphere
384 boundary edges). Root cause confirmed: a quad is built only when its lattice diagonal is a single
(±1,±1) triangulation edge, and the isotropic triangulation's diagonals don't align with the integer
lattice — so cells whose diagonal isn't a mesh edge produce no quad. **This is what `FixFlip`
(roadmap 04 Stage E) fixes**: it flips triangulation edges so each quad cell has a (1,1) diagonal,
*before* extraction. So the dependency is the reverse of what 04 assumed — flip-fixing is a
*prerequisite* for hole-free extraction, not an after-the-fact polish.

Revised remaining order: **subdivide_edgeDiff → FixFlip (align triangulation) → coarse collapse →
Part A → Part B → FixHoles (residual) → FixValence**. The extractor (C2/C3/Part B) is done; the gap
is the triangulation-alignment front end (subdivide + FixFlip).

## Status

- ✅ Integer solve (doc 04 Stage A+B).
- ✅ `QuadriFlowExtractor`: coarse graph (C2) + Part A manifold-ize (C3) + Part B quads.
- ✅ **`subdivide_edgeDiff`** (`IntegerGridMesh.SubdivideEdgeDiff`) — splits every offset to a unit
  step on a mutable flat-array mesh. Invariant test C1: max |edge_diff| ≤ 1, divergence preserved.
- ✅ **FixFlip** (`EdgeGraphHierarchy`) — full `DownsampleEdgeGraph(-1)` + `Hierarchy::FixFlip`
  (signed-area foldover removal) + `PropagateEdge`, on the edge graph only (no geometry).
- ✅ Pipeline wired (`QuadRemesher` PureQuad): **FixFlip → subdivide(1) → FixFlip → subdivide(1)** →
  manifold extraction → FixValence. **Both closed primitives watertight, 100 % quad, correct genus,
  at/near the QuadriFlow goldens:**
  - **sphere**: χ=2, 0 non-manifold, 0 boundary, **irregular = 8 (== golden)**.
  - **torus**: χ=0, 0 non-manifold, 0 boundary, **irregular = 24 (≈ golden ~22)**.
- ✅ **The torus fix was extraction ordering, not the field.** Diagnosis: the torus orientation field
  has ~8 spurious singularities (net index 0 — *fine*, that alone → ~16–24 irregular verts), but the
  position field folds over near them (|edge_diff| up to 4 across one edge, solve residual still 0).
  Those negative-area lattice cells made the *first* `subdivide` choke (its split "spoke" regenerates
  an over-long offset, so it hit its split cap and left holes → 138 irregular). Running **`FixFlip`
  first** shrinks those foldover edge-rings away before subdivide sees them: 138 → **24**. Sphere is
  untouched (no foldovers → leading FixFlip is a no-op there). So the field was *not* the bottleneck;
  more field sweeps actually make the torus far worse (138 → 983), i.e. the smoother does not robustly
  converge past ~20 sweeps — a separate latent P3 issue, no longer on the critical path for topology.
- ✅ **FixValence** (`QuadMeshCleanup`, port of `parametrizer-mesh.cpp` + `compute_direct_graph_quad`):
  remove valence-2 doublets → split pinched (multi-fan) vertices → decrease valence > 5 → drop
  zero-valence. Runs on the all-quad output before surface projection. Effect: torus irregular
  154 → **138**; sphere unchanged at 8 (no doublets/high-valence); both stay watertight.
- ⏳ Remaining: P7 (sharp edges / open boundaries for box & cylinder). The orientation smoother's
  non-convergence past ~20 sweeps is a latent P3 issue but no longer blocks topology. `FixHoles` is
  not needed (output already watertight). Note: `subdivide_edgeDiff` needs no `DynamicTriangleMesh` —
  `IntegerGridMesh` (flat growable arrays) is the mutable mesh.

## Implementation notes (do not relearn)

- **`Lattice2D.Rshift90` must NOT normalize the amount mod 4.** It is bit-identical to the reference
  `rshift90`: the parity (`amount & 1`) and sign (`amount >= 2`) tests run on the raw amount. So
  `amount = 4` is a 180° turn, `amount = −2` is the identity. `AnalyzeOrient`/`FixOrient` produce such
  out-of-range orients and rely on this. (The old unit test asserting period-4 wrap was wrong vs the
  reference and was corrected.)
- **FixFlip needs only the edge graph** (face→edge-ids, face→orients, per-edge diff, allow=all-1) — no
  V/N/Q/O. `subdivide_edgeDiff` is the only stage that needs a mutable mesh.
- **`IntegerGridMesh` is internal**; `Quadremesh.Tests` sees it via `InternalsVisibleTo`.

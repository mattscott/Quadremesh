# 6. C# portability analysis

The decision: **reimplement the Instant Meshes algorithm in managed C#** (clean-room from the
paper, cross-referencing the BSD source), with **QuadriFlow's min-cost-flow extraction** as a
Phase-2 upgrade. This doc lays out *why* against the alternatives, and what the C# implementation
actually requires.

## 6.1 Decision matrix

| Factor | Instant Meshes / QuadriFlow (A) | autoremesher / QuadWild (B) |
|---|---|---|
| Reference license | **BSD / MIT** — adaptable + reimplementable | **GPL-3.0** — viral; can't copy code |
| Core solver | local Gauss–Seidel; +min-cost-flow | **mixed-integer** (CoMISo / **Gurobi commercial**) |
| Managed-C# solver availability | full (hand-rolled) | none adequate |
| Native deps to replace | ~none | OpenVDB, CGAL, libigl, Geogram, CoMISo |
| Robustness on dirty input | high | medium |
| Speed | seconds | tens of s → min |
| Output quality | good → very good | best |
| **Port effort** | **Low → Medium** | High → impractical |

The licensing point is worth stating precisely: **algorithms are not copyrightable — only code is.**
We may freely reimplement *any* of these methods from their papers. But where we want to lean on an
existing implementation while porting, only **BSD/MIT** lets us do so without forcing the whole C#
library to GPL. That, plus the solver-dependency gap, makes Family A the only practical base.

## 6.2 Target platform & language choices

- **.NET 8** (LTS), C# 12. Cross-platform (Windows primary, Linux/macOS via .NET).
- **`Span<T>` / struct-of-arrays** layout for vertex/field data — cache-friendly, GC-light.
- **No unsafe required** for correctness; may add `unsafe`/SIMD (`System.Numerics.Vector`) hot-path
  optimizations later, behind benchmarks.
- Parallelism via `System.Threading.Tasks.Parallel` for per-island and per-sweep work (Gauss–Seidel
  red-black coloring enables safe parallel sweeps).

## 6.3 Required building blocks and their C# story

| Block | Needed for | Status / source |
|---|---|---|
| `Vector3` (double), `Matrix3`, tangent frames | everywhere | hand-rolled, clean-room |
| Half-edge / adjacency mesh | all stages | new `HalfEdgeMesh`, clean-room (`ConnectedMesh` read for reference only) |
| Priority queue / `MinHeap` | hierarchy coarsening | hand-rolled, clean-room |
| Union-Find (DSU) | extraction vertex merge | ~40 LOC |
| Graph coarsening (multiresolution) | field solver speed | **new**, ~200 LOC; mirrors decimation matching |
| kd-tree / BVH | re-projection, isotropic remesh | **new**, standard |
| Isotropic triangle remesh | Stage 0 pre-pass | **new** (split/collapse/flip/tangential relax) |
| Min-cost network flow | QuadriFlow extraction (Phase B) | **new**, ~few hundred LOC, no native dep |
| Sparse linear solver | **not needed for the core** | (only if we ever add a global param mode) — CSparse.NET / Math.NET if so |

Every "new" item is bounded and self-contained. Nothing requires a native interop layer or a
commercial solver.

## 6.4 What we deliberately do NOT do

- **No P/Invoke into Instant Meshes/QuadriFlow C++.** Defeats the point (portability, single
  managed artifact, debuggability). We reimplement. (The C++ tools *are* built locally — but only
  as an offline reference oracle to generate golden outputs for tests, never shipped or linked;
  see §6.6.)
- **No mixed-integer solver.** If a global-parametrization (Family B) mode is ever wanted, it's a
  separate optional module — explicitly out of the core roadmap.
- **No GPL code.** autoremesher/QuadWild are read for *understanding* only; not copied.
- **No code lifted from Nanomesh.** `Quadremesh.Core` is clean-room (§6.5).

## 6.5 Relationship to Nanomesh (clean-room, no code reuse)

**Decision:** `Quadremesh.Core` is written **clean-room**. No source is copied or ported from
Nanomesh. This keeps the dependency graph pristine and auditable, avoids inheriting any latent
bugs/architecture issues, and removes the "audit-before-copy" gate from the critical path.

Nanomesh stays its own separate project and may be **read for reference/understanding only**
(the same posture we take toward the BSD/MIT C++ references):

- `Base/` math types, `Collections/` (`MinHeap`, `FastHashSet`) — conceptual reference; we
  re-derive our own equivalents.
- `Mesh/ConnectedMesh` connectivity model — *pattern* reference for the new half-edge mesh.
- `Algo/Decimation` — *pattern* reference for hierarchy coarsening.
- `Import/Export` OBJ — *pattern* reference; our `Quadremesh.IO` is written fresh (and must handle
  quads, which Nanomesh's triangle-only IO does not).

> The Nanomesh audit ([../Plans/03-followups.md](../Plans/03-followups.md#f1--audit-the-nanomesh-library-requested))
> is therefore **decoupled and no longer gating** — it survives only as an optional contribute-back
> side-quest / sanity-check against our own clean-room connectivity design.

## 6.6 Reference oracle for testing (build IM + QuadriFlow locally)

Several acceptance tests (P3/P4/P6) require diffing our output against **reference Instant Meshes /
QuadriFlow** results. Because both are BSD/MIT, we build them **locally as an offline test oracle**
(never shipped, never linked into the managed artifact):

- Build Instant Meshes and QuadriFlow from source on the dev/CI box (pinned commits).
- A harness drives each tool **deterministically** (fixed seed/params) over `tests/corpus/` inputs
  and captures golden outputs: the extracted quad mesh, plus — where the tool can dump them — the
  per-vertex orientation/position fields and singularity set.
- Goldens are committed under `tests/golden/` (regenerated by an explicit, documented script, not
  on every CI run).
- Diff metrics + tolerances are defined per stage (field angular error, singularity-count delta,
  Hausdorff/quad-% on the mesh). These back the "within tolerance / within ~2–4×" wording in the
  roadmap.

This makes the reference comparison reproducible and reviewable rather than a hand-wave.

## 6.7 Principal risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| **Extraction robustness** (watertight quads near singularities) | High | Build a large test corpus early; ship quad-dominant first; add QuadriFlow integer solve |
| Field solver converging to vortices / local minima | Medium | Multiresolution hierarchy (not flat solve); validated against reference outputs |
| Performance vs C++ (managed overhead) | Medium | SoA layout, red-black parallel sweeps, SIMD hot paths; target seconds not ms |
| Dirty/non-manifold input crashes | Medium | Robust Stage-0 cleanup; per-island isolation; defensive degrade |
| Scope creep into Family-B quality | Medium | Phase gating; pure-quad is Phase B, hard-surface mode is post-1.0 |

## 6.8 Verdict

Implement **Instant Meshes** in C# as the core (Phases 1–4), then add **QuadriFlow's**
min-cost-flow extraction (Phase 5). This is the lowest-risk path to a QuadRemesher-class tool that
is permissively licensed, dependency-light, cross-platform, and fully managed.

→ Roadmap: [../Plans/](../Plans/).

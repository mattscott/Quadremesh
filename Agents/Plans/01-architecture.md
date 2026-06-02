# 01 — Architecture

## Solution layout

```
Quadremesh.sln
├─ src/
│  ├─ Quadremesh.Core/            # the library (no IO, no CLI deps)
│  │  ├─ Math/                    # Vector3 (double), Matrix3, frames, MathUtils
│  │  ├─ Mesh/                    # HalfEdgeMesh, attributes, normals/areas
│  │  ├─ Acceleration/            # KdTree, BVH, Union-Find
│  │  ├─ Remeshing/               # IsotropicRemesher (Stage 0)
│  │  ├─ Hierarchy/               # MultiresHierarchy (graph coarsening)
│  │  ├─ Fields/                  # OrientationFieldSolver, PositionFieldSolver
│  │  ├─ Extraction/              # LatticeExtractor (P5), IntegerSolver/MinCostFlow (P6)
│  │  ├─ Constraints/             # FeatureDetector, GuideCurve, SymmetryPlane, SizingField
│  │  └─ QuadRemesher.cs          # top-level orchestrator + Options
│  ├─ Quadremesh.IO/              # OBJ/PLY/STL/glTF readers+writers
│  └─ Quadremesh.Cli/             # `quadremesh in.obj out.obj --target 20000 ...`
└─ tests/
   ├─ Quadremesh.Tests/           # unit + golden-file tests
   └─ corpus/                     # shared input meshes (see master plan)
```

`Quadremesh.Core` has **zero third-party runtime dependencies** for the core algorithm. Optional
helper libs (Math.NET/CSparse) only enter if a future global-param mode is added — not in v1.0.

## Core data structures

### Half-edge mesh (`Mesh/HalfEdgeMesh.cs`)
A compact, index-based connectivity structure (struct-of-arrays). Written **clean-room** for this
project — no code is lifted from Nanomesh (see [06 §6.5](../Docs/06-csharp-portability-analysis.md#65-relationship-to-nanomesh-clean-room-no-code-reuse)).
Nanomesh's `ConnectedMesh` (position/sibling/relative node model) may be *read for reference only*;
ours is designed from the start for **quad-and-triangle** output (Nanomesh's is triangle-only).
Responsibilities:
- adjacency iteration (1-ring, edge, face circulators)
- per-vertex normals, Voronoi/barycentric areas, cotangent edge weights
- non-manifold-tolerant construction (mark & isolate, don't crash)
- manifold/orientation validation

### Field state (SoA, `Fields/`)
```
struct VertexFields {
   Vector3[] position;     // v_i
   Vector3[] normal;       // n_i
   Vector3[] tangent;      // t_i (frame)
   Vector3[] orient;       // o_i  (Stage 1 output: 4-RoSy direction)
   Vector3[] posField;     // p_i  (Stage 2 output: lattice anchor)
   double[]  area;         // a_i  (smoothing weight)
   double[]  scale;        // ℓ_i  (sizing field; uniform unless adaptive)
}
```
Parallel arrays, not per-vertex objects — cache-friendly and GC-light.

### Multiresolution hierarchy (`Hierarchy/MultiresHierarchy.cs`)
A list of levels; each level stores a coarsened graph plus parent↔children maps and accumulated
weights/areas. Built by greedy edge-matching (heaviest edges first) using a `MinHeap`. Used by both
field solvers: solve coarse → prolong → refine.

## Pipeline orchestration (`QuadRemesher.cs`)

```csharp
public sealed class QuadRemesherOptions {
    public int?    TargetQuadCount;     // OR
    public double? TargetEdgeLength;
    public bool    AdaptiveSizing  = false;   // curvature-driven ℓ_i
    public bool    PreserveSharp   = true;     // feature detection
    public double  SharpAngleDeg   = 35;
    public IReadOnlyList<GuideCurve> Guides = ...;   // optional
    public SymmetryPlane? Symmetry  = null;
    public ExtractionMode Extraction = ExtractionMode.PureQuad; // or QuadDominant
    public int     Seed = 0;            // determinism
    public int     Threads = -1;        // -1 = all cores
}

public static class QuadRemesher {
    public static QuadMesh Remesh(TriangleMesh input, QuadRemesherOptions opt);
}
```

Internal flow (maps 1:1 to phases):
```
TriangleMesh
  → IsotropicRemesher.Run        (P2)
  → BuildFrames + Areas + Weights
  → MultiresHierarchy.Build      (P3/P4)
  → OrientationFieldSolver.Solve (P3)   [+ feature/guide/symmetry constraints]
  → PositionFieldSolver.Solve    (P4)   [+ sizing field]
  → Extraction:
       QuadDominant → LatticeExtractor.Extract              (P5)
       PureQuad     → IntegerSolver(MinCostFlow) + Extract  (P6)
  → PostProcess (cleanup, project-to-surface, reorient)
  → QuadMesh
```

## Key interfaces (stable seams for testing & future swaps)

```csharp
interface IFieldSolver        { void Solve(MultiresHierarchy h, VertexFields f, SolveOptions o); }
interface IExtractor          { QuadMesh Extract(VertexFields f, HalfEdgeMesh m); }
interface ISizingField        { double LengthAt(int vertex); }

// Constraints are applied once per sweep with full field + mesh access, because not all of
// them are per-vertex. Feature/guide constraints are *local* (Dirichlet clamp on their own
// vertices); the symmetry plane is *non-local* — it couples mirror-pair vertices across the
// plane and so needs the pair map, not a single index. A per-vertex helper covers the common
// local case without forcing every constraint through it.
interface IConstraint         { void Apply(VertexFields f, HalfEdgeMesh m); }  // sweep-level
interface ILocalConstraint    { void ApplyAt(VertexFields f, int vertex); }    // feature/guide sugar
```

These seams let us: validate each solver in isolation against reference data; swap the P5 extractor
for the P6 integer extractor without touching solvers; and add new constraint types (guides,
hard-surface patches) without changing the core.

## Determinism & parallelism

- All randomness flows from `Options.Seed`.
- Gauss–Seidel sweeps parallelized via **red-black (2-coloring) of the graph** so neighbors never
  update simultaneously; deterministic given fixed coloring.
- Per-connected-component ("island") work parallelized across cores.
- **Bit-exact reproducibility caveat:** per-vertex updates are deterministic (each vertex is
  written by exactly one thread, reading only neighbors). But floating-point addition is
  non-associative, so any *parallel reduction* (field energy, total area, residual norms) can
  reorder adds and break bit-exact output. Hot-path reductions must use a fixed/sequential
  accumulation order (or compensated/Kahan summation) — never a nondeterministic `Parallel`
  reduce. The 2-coloring itself must be deterministic given the seed.

## Public surface (v1.0)

- `QuadRemesher.Remesh(...)` (one-shot) and a streaming/`IProgress<>` variant for long runs.
- `TriangleMesh` / `QuadMesh` POCOs + `Quadremesh.IO` loaders.
- CLI mirrors the options.

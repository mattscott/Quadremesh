# 2. The two algorithm families

Every robust quad-remesher in the literature and in shipping products reduces to one of two
families. Both share the same conceptual core — **a cross/orientation field, then a parametrization
whose integer grid lines become quad edges** — but they differ in *how* they enforce global
consistency.

```
            INPUT TRIANGLE MESH
                    │
        ┌───────────┴───────────┐
        │   (shared front-end)  │
        │  isotropic remesh     │
        │  cross/orientation    │
        │  field (4-RoSy)       │
        └───────────┬───────────┘
                    │
        ┌───────────┴────────────────────────────┐
        │                                         │
   FAMILY A                                  FAMILY B
   Local field-aligned                       Global integer-grid
   (Instant Meshes, QuadriFlow)              parametrization (MIQ, QuadCover, QuadWild)
        │                                         │
   position field via                       seamless global UV via
   LOCAL Gauss–Seidel                        MIXED-INTEGER solve
   on a multiresolution                      (CoMISo / Gurobi)
   hierarchy                                       │
        │                                         │
   extract quads from                        extract quads from
   lattice                                    integer iso-lines
        │                                         │
        └───────────────────┬─────────────────────┘
                            │
                    QUAD-DOMINANT MESH
```

## 2.1 Family A — Local field-aligned meshes

**Representatives:** Instant Meshes (2015), QuadriFlow (2018).
**Shipping in:** Modo (Instant Meshes), Blender (QuadriFlow).

**Key idea:** Never build a global parametrization or solve a global linear/integer system in
the inner loop. Instead, represent the orientation field and the position field as **per-vertex
variables**, and optimize each with **purely local smoothing operations** (a Gauss–Seidel sweep:
update each vertex from its neighbors), accelerated by a **multiresolution hierarchy**
(coarsen → solve → prolong → refine).

**Strengths**
- No dependency on a mixed-integer or sparse linear solver in the core. ← *decisive for a C# port*
- Near-linear time; scales to millions of triangles; runs in seconds.
- Very robust on dirty input (scans, sculpts) — it degrades gracefully.
- The hierarchy avoids local minima that flat local solvers fall into.

**Weaknesses**
- Basic extraction is **quad-dominant**, not guaranteed pure-quad (triangles/pentagons appear
  near singularities).
- More singularities than the global methods *unless* you add QuadriFlow's integer step.

## 2.2 Family B — Global integer-grid parametrization

**Representatives:** MIQ "Mixed-Integer Quadrangulation" (Bommes 2009), "Integer-Grid Maps"
(Bommes 2013), QuadCover, **QuadWild** (Pietroni 2021, feature-line driven).
**Shipping in:** research tools, autoremesher (QuadCover variant), QRemeshify (QuadWild wrapper).

**Key idea:** Solve for a **global seamless UV map** of the surface such that its integer
grid (u,v ∈ ℤ) *is* the quad mesh. Aligning the gradient of that map to the cross field and
forcing integer translations across seams is a **mixed-integer quadratic program (MIQP)**.

**Strengths**
- Highest-quality, most-regular output; superb feature-line alignment.
- Pure-quad by construction; explicit control over singularity placement.
- QuadWild's patch-layout approach handles sharp creases beautifully.

**Weaknesses**
- Requires a **mixed-integer / seamless-parametrization solver**: **CoMISo** (C++) or commercial
  **Gurobi** (QuadWild's default). No good managed-C# equivalent exists. ← *decisive against a port*
- Slower; more brittle on dirty/non-manifold input (the global solve can fail outright).
- Much larger, harder-to-port codebase (libigl + CGAL + CoMISo + Geogram + OpenVDB).

## 2.3 Why Family A wins for *our* project

| Criterion | Family A (Instant Meshes / QuadriFlow) | Family B (MIQ / QuadWild) |
|---|---|---|
| Core solver dependency | **None** (local) → QuadriFlow adds one min-cost-flow | Mixed-integer (CoMISo / **Gurobi commercial**) |
| License of reference impl | **BSD / MIT** ✅ | **GPL-3.0** ⚠️ |
| Robustness on scans/sculpts | High | Medium |
| Speed | Seconds | Tens of seconds → minutes |
| Output quality | Good → very good (with QuadriFlow) | Best |
| Port effort to C# | Low → Medium | High → impractical |

For a clean-room managed-C# library that must run without commercial solvers and without
GPL contamination, **Family A is the only sensible base**. We adopt **Instant Meshes** as the
core and treat **QuadriFlow's** global integer step as a quality upgrade.

Full per-repo analysis: [03-open-source-landscape.md](03-open-source-landscape.md).
The algorithmic detail we will actually implement: [04-instant-meshes-deep-dive.md](04-instant-meshes-deep-dive.md).

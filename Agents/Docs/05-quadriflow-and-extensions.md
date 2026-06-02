# 5. QuadriFlow & extensions — the singularity-reducing upgrade

Instant Meshes produces a good quad-dominant mesh, but its extraction is *greedy*: singularities
fall where the field happens to be inconsistent, and stray triangles/pentagons appear. **QuadriFlow**
(Huang, Zhou, Nießner, Shewchuk, Guibas — SGP 2018) keeps the entire Instant Meshes front-end and
replaces the *greedy* integer decisions with a **globally consistent integer solve**, yielding
~4× fewer singularities and a pure-quad result, still in under ~10 s/model. License: **MIT**.

## 5.1 What QuadriFlow changes

The orientation field and position field are computed **exactly as in Instant Meshes** (Stages 1–2
of [04](04-instant-meshes-deep-dive.md) are unchanged). The difference is how the **integer
variables** are chosen during/after the position-field stage.

In Instant Meshes, each edge's integer offset `(k1,k2)` is decided locally by rounding. Those local
roundings can be **mutually inconsistent**, which forces extra singularities and non-quad faces.
QuadriFlow instead poses the integer choices as a **constrained global optimization**:

1. **Linear consistency constraints.** Around every face and every input loop, the integer edge
   jumps must sum consistently (a "no contradiction" condition). These form a system of linear
   integer constraints.
2. **Minimum-cost network flow.** The problem of choosing integer offsets that satisfy the
   constraints while staying closest to the field's preferred (rounded) values is reformulated as
   a **minimum-cost flow** on a graph derived from the mesh. This is solvable in polynomial time
   with a network-simplex / successive-shortest-path solver — **no mixed-integer solver needed.**
3. **Local boolean satisfiability.** A handful of small local consistency problems are resolved
   with boolean SAT to guarantee a valid, T-junction-free quad tessellation.

The output is a mesh where edges meet consistently → **pure quads**, far fewer singularities, and
no stray triangles.

## 5.2 Why this is still portable to C#

- The expensive global solver in Family B (CoMISo/Gurobi) is **not** used. QuadriFlow's heaviest
  numeric component is a **min-cost flow**, which is a classic combinatorial algorithm
  (network simplex or SSP) — a few hundred lines of pure managed C#, no native dependency.
- The boolean local problems are tiny and bounded.
- Everything else is shared with the Instant Meshes core we already build.

## 5.3 Recommended adoption strategy

Ship in two phases (mirrors the Plans roadmap):

| Phase | Extraction | Output | Effort |
|---|---|---|---|
| **Phase A** | Instant Meshes greedy lattice extraction | quad-**dominant**, some singularities | lower |
| **Phase B** | QuadriFlow min-cost-flow integer solve | **pure quad**, ~4× fewer singularities | +min-cost-flow + SAT |

Phase A gives a working, demoable remesher fastest. Phase B is a contained, well-understood upgrade
that swaps the extraction back-end without touching the field solvers.

## 5.4 Ideas worth borrowing from the GPL/Family-B world (study, don't copy)

These won't be ported, but inform later features:

- **autoremesher — island parallelism & VDB pre-clean.** Splitting the input into connected
  components and remeshing each in parallel is a cheap throughput win; an OpenVDB-style volumetric
  reconstruction is a robust way to clean self-intersecting scan input before remeshing. We can
  approximate the pre-clean with a voxel-shell/marching-cubes step later.
- **QuadWild — feature-line patch layout.** For *sharp-edge-heavy* models (hard-surface, CAD), the
  state of the art is to first lay out coarse polygonal **patches whose boundaries are the feature
  lines**, then tessellate each patch to pure quads. This is the best-known way to get crisp creases
  and is the natural direction for a future "hard-surface mode" — built on top of, not instead of,
  the field solver.

## 5.5 Net

QuadriFlow is the *same algorithm* as Instant Meshes with a smarter, still-solver-light integer
back-end. Adopting Instant Meshes first and QuadriFlow second is a low-risk path from "working" to
"production quality" without ever taking on a mixed-integer solver or a GPL dependency.

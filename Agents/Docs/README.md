# Quadremesh — Research Docs

This folder holds the background research that justifies the technical direction of the
**Quadremesh** project: a from-scratch, managed-C# automatic quad-remesher (auto-retopology)
in the same problem space as Exoside's commercial **QuadRemesher** / Pixologic's **ZRemesher**.

## Reading order

| # | Doc | What it answers |
|---|-----|-----------------|
| 1 | [01-problem-and-quadremesher.md](01-problem-and-quadremesher.md) | What auto-retopology is, and what QuadRemesher actually does (and what it doesn't tell us). |
| 2 | [02-algorithm-families.md](02-algorithm-families.md) | The two algorithm families that solve this problem, and how they differ. |
| 3 | [03-open-source-landscape.md](03-open-source-landscape.md) | The four candidate repos analyzed (Instant Meshes, QuadriFlow, autoremesher, QuadWild/QRemeshify, CurioMesh). |
| 4 | [04-instant-meshes-deep-dive.md](04-instant-meshes-deep-dive.md) | The core math of the recommended algorithm, in implementation-level detail. |
| 5 | [05-quadriflow-and-extensions.md](05-quadriflow-and-extensions.md) | The optional global-integer upgrade that reduces singularities. |
| 6 | [06-csharp-portability-analysis.md](06-csharp-portability-analysis.md) | Why this family is the right C# target: licensing, dependencies, math. |
| 7 | [07-references.md](07-references.md) | All sources, papers, and repos. |

## One-paragraph conclusion

The robust, production-grade auto-retopo tools (Modo, Blender) are all built on **field-aligned
parametrization**: compute a smooth **cross/orientation field** over the surface, then a
**position field** whose integer lattice lines become quad edges, then extract the quads.
**Instant Meshes** (BSD-licensed, no heavy solver dependency, purely local Gauss–Seidel
optimization on a multiresolution hierarchy) is the ideal base for a clean managed-C# port.
**QuadriFlow** (MIT) is the same algorithm plus a global min-cost-flow integer step that cuts
singularities ~4×, and is the recommended Phase-2 upgrade. The GPL-licensed, solver-heavy
options (autoremesher, QuadWild) are excellent algorithms but poor porting targets.

See **[../Plans/](../Plans/)** for the implementation roadmap that follows from this research.

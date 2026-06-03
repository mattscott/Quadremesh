# Quadremesh — Implementation Plans

The build plan for a managed-C# automatic quad-remesher based on the research in
[../Docs/](../Docs/). Direction in one line: **reimplement Instant Meshes (BSD) in C#, then add
QuadriFlow's (MIT) min-cost-flow extraction.**

| Doc | Contents |
|-----|----------|
| [00-master-plan.md](00-master-plan.md) | Vision, scope, non-goals, milestones, success criteria, risks |
| [01-architecture.md](01-architecture.md) | Project/module layout, core data structures, public API |
| [02-phase-roadmap.md](02-phase-roadmap.md) | Phase-by-phase tasks, deliverables, acceptance tests |
| [03-followups.md](03-followups.md) | Tracked side-quests (Nanomesh audit, hard-surface mode, etc.) |
| [04-quadriflow-integer-port.md](04-quadriflow-integer-port.md) | P6 completion: faithful QuadriFlow integer-grid-map port, broken into testable stages A–G (incl. MiniSat fallback for flip-fixing) |
| [05-topology-safe-singularities.md](05-topology-safe-singularities.md) | Research + plan: manifold, watertight, all-quad extraction incl. cone-point (singularity) handling — the remaining Stage-C work |

**Phase ladder at a glance**

```
P0 Scaffold ─► P1 Mesh+IO ─► P2 Isotropic prep ─► P3 Orientation field ─►
P4 Position field ─► P5 Extraction (quad-dominant) ─► P6 QuadriFlow (pure quad) ─►
P7 Features/guides/sizing ─► P8 Perf+robustness ─► P9 Packaging/CLI/API
```

The first *demoable* milestone (a quad-dominant mesh out) is end of **P5**.
The first *production-quality* milestone (pure quad, few singularities) is end of **P6**.

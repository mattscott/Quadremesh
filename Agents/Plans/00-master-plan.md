# 00 — Master plan

## Vision

A permissively-licensed, dependency-light, cross-platform **managed-C# (.NET 8) library + CLI**
that turns arbitrary triangle meshes (sculpts, scans, isosurfaces) into clean, field-aligned,
quad-dominant → pure-quad meshes — i.e. an open, embeddable QuadRemesher/ZRemesher-class
auto-retopology engine.

## Why this is achievable now (vs. the prior failed attempt)

The research ([../Docs/](../Docs/)) identified that the production-grade algorithm (Instant Meshes,
shipping in Modo; QuadriFlow, shipping in Blender) is **purely local optimization on a
multiresolution graph hierarchy** — *no mixed-integer solver, no native dependency, permissively
licensed.* The prior attempt conflated decimation with remeshing and never built the field solvers.
This plan builds them in the correct order, each independently testable.

## Scope (v1.0)

**In:**
- Triangle-mesh input (OBJ first; glTF/PLY/STL later).
- Isotropic triangle pre-pass.
- 4-RoSy **orientation field** solver (multiresolution).
- 4-PoSy **position field** solver (multiresolution).
- **Quad-dominant extraction** (P5), then **pure-quad** via QuadriFlow integer solve (P6).
- Controls: target edge length / target quad count, adaptive (curvature) sizing.
- **Feature preservation** (sharp edges) and **guide curves**.
- Symmetry plane.
- CLI tool + clean public C# API; OBJ in/out.

**Out (v1.0 non-goals):**
- Global mixed-integer parametrization (Family B / MIQ / QuadWild) — explicitly avoided.
- GPU acceleration.
- GUI / DCC plugins (Blender/Maya) — post-1.0; API designed to enable them.
- Texture/UV transfer, material preservation — post-1.0.
- Volumetric (VDB) reconstruction of broken scans — post-1.0 (a robustness nicety).

## Success criteria (v1.0 "done")

1. On a 500k–2M-tri sculpt, produces a **pure-quad** mesh at a target resolution in **< 20 s**
   on a desktop CPU.
2. **Edge flow follows curvature**; singularity count within ~2–4× of reference QuadriFlow on the
   same input (measured on a shared corpus).
3. **Sharp features preserved** when feature mode is enabled.
4. **Does not crash** on a corpus of dirty/non-manifold inputs; degrades to best-effort.
5. Reproducible: deterministic output for a fixed seed/params.
6. Single managed artifact, no native binaries, MIT/BSD-clean dependency graph.

## Milestones

| Milestone | Phases | Outcome |
|---|---|---|
| **M1 — Skeleton** | P0–P1 | Builds, loads/saves OBJ, half-edge mesh + tests |
| **M2 — Clean input** | P2 | Isotropic remesh pre-pass working on corpus |
| **M3 — Fields** | P3–P4 | Orientation + position fields converge; visualized/validated |
| **M4 — First quads** | P5 | **Demoable** quad-dominant output end-to-end |
| **M5 — Production quality** | P6 | **Pure quad**, low singularities (QuadriFlow integer solve) |
| **M6 — Art-directable** | P7 | Features, guides, adaptive sizing, symmetry |
| **M7 — Ship** | P8–P9 | Perf/robustness hardened; CLI + API + NuGet package |

## Top risks (see [../Docs/06](../Docs/06-csharp-portability-analysis.md#66-principal-risks))

1. **Extraction robustness** — the #1 risk. Mitigate: build the test corpus in P1, ship
   quad-dominant before pure-quad, lean on QuadriFlow's consistency in P6.
2. **Field local minima** — mitigate with the multiresolution hierarchy (P3 includes it from day one).
3. **Managed perf** — mitigate with SoA layout + red-black parallel sweeps + SIMD hot paths (P8).

## Working method

- Each phase has explicit **deliverables + acceptance tests** ([02-phase-roadmap.md](02-phase-roadmap.md)).
- `Quadremesh.Core` is written **clean-room** — no code lifted from Nanomesh or the C++ references;
  both are read for understanding only ([../Docs/06 §6.5](../Docs/06-csharp-portability-analysis.md#65-relationship-to-nanomesh-clean-room-no-code-reuse)).
- Validate field stages against **reference Instant Meshes/QuadriFlow outputs** on identical inputs
  (visual + singularity-count diff) before moving on. References are built locally as an offline
  test oracle that emits committed golden files ([../Docs/06 §6.6](../Docs/06-csharp-portability-analysis.md#66-reference-oracle-for-testing-build-im--quadriflow-locally)).
- Keep a **test corpus** under `tests/corpus/` (a sphere, a torus, a cylinder, a scanned head, a
  hard-surface part, a non-manifold mess) used by every phase.
- TDD where practical; golden-file tests for end-to-end stages.

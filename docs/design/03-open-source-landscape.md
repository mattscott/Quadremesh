# 3. Open-source landscape

The five projects surveyed, with the algorithm each actually implements and its suitability as a
porting source.

## 3.1 Summary table

| Repo | Algorithm | Family | Lang | License | Heavy deps | As a port source |
|---|---|---|---|---|---|---|
| **wjakob/instant-meshes** | Instant Field-Aligned Meshes (Jakob et al. SIGGRAPH Asia 2015) | A | C++ | **BSD-style** ✅ | none critical | **Best base** ✅ |
| **hjwdzh/QuadriFlow** | QuadriFlow (Huang et al. SGP 2018) | A+ | C++ | **MIT** ✅ | Boost (replaceable) | **Phase-2 upgrade** ✅ |
| **huxingyi/autoremesher** | OpenVDB remesh → QuadCover param → quad extract | B | C++ | **GPL-3.0** ⚠️ | OpenVDB, libigl, CGAL, Geogram | reference only ❌ |
| **ksami/QRemeshify** | Blender wrapper around QuadWild + Bi-MDF | B | Python | **GPL-3.0** ⚠️ | QuadWild binaries | not an algorithm |
| **CurioCrafter/CurioMesh** | Blender wrapper around QuadriFlow (+ experimental "TRIAD-Q Lite") | A | Python | **MIT** | Blender's QuadriFlow | not an algorithm |

## 3.2 wjakob/instant-meshes — the recommended base

- Implements *"Instant Field-Aligned Meshes"* (Jakob, Tarini, Panozzo, Sorkine-Hornung,
  ACM TOG / SIGGRAPH Asia 2015).
- Two-stage: solve an **orientation field**, then a **position field**, each by local smoothing
  over a **multiresolution hierarchy**. No global linear/integer solver.
- Supports 2/4/6-RoSy orientation symmetry and 2/4-PoSy position symmetry; for quads we use
  **4-RoSy + 4-PoSy**.
- Production-proven: **Modo** integrated it for auto-retopo in 10.2+.
- **BSD-style license** → its source can be studied and adapted (with attribution) without GPL
  contamination, and the algorithm (published) can be freely reimplemented.
- Dependencies in the repo (TBB, nanogui, etc.) are about the *GUI/perf*, not the algorithm —
  the algorithm itself is self-contained.

## 3.3 hjwdzh/QuadriFlow — the quality upgrade

- Implements *"QuadriFlow: A Scalable and Robust Method for Quadrangulation"* (Huang, Zhou,
  Nießner, Shewchuk, Guibas, SGP 2018).
- Reuses the Instant Meshes orientation+position objective, then **resolves global consistency**
  by solving a **minimum-cost network-flow** problem plus **local boolean satisfiability** to
  decide integer edge "jumps."
- Result: **~4× fewer singularities** than Instant Meshes, **pure quad**, **<10 s/model** —
  validated on 17,791 ShapeNet objects.
- Shipping in **Blender** as a Remesh mode.
- **MIT license.** Extra dependency is Boost (used for the flow solve), which is **replaceable**
  by a managed network-simplex implementation in C#.

## 3.4 huxingyi/autoremesher — reference only (GPL)

- Pipeline (per DeepWiki): mesh separation into islands → **OpenVDB** volumetric remesh →
  isotropic remesh → **QuadCover** parametrization (replaced MIQ in v1.0.0-beta.1) → quad
  extraction → recombine.
- Family B (global parametrization). Pulls in OpenVDB + libigl + CGAL + Geogram — a very heavy
  dependency stack.
- **GPL-3.0** → cannot copy code into a permissively-licensed C# project; the QuadCover/MIQ math
  is published and reimplementable, but the solver burden is high.
- **Verdict:** valuable for ideas (island parallelism, VDB pre-clean), not a port target.

## 3.5 ksami/QRemeshify — wrapper, not an algorithm

- A **Blender Python add-on** that drives **QuadWild + Bi-MDF solver** binaries.
- The interesting part is upstream: **QuadWild** (Pietroni et al. 2021), a Family-B method that
  builds a feature-aligned cross-field, a coarse polygonal patch layout with T-junctions, then a
  pure-quad per-patch tessellation. The original needs **Gurobi**; the `quadwild-bimdf` variant
  swaps in an open **Bi-MDF** solver.
- **GPL-3.0**, Python glue. Not directly portable, but its **feature-line patch-layout** ideas are
  the gold standard for sharp-edge handling and worth studying for a future feature-preservation pass.

## 3.6 CurioCrafter/CurioMesh — wrapper, not an algorithm

- **Blender Python add-on**; default engine is just **Blender's built-in QuadriFlow**. Adds
  material/UV preservation plumbing and an experimental "TRIAD-Q Lite" (feature extraction + field
  estimation + triangle-pairing) that is not production-grade.
- **MIT**, but offers no portable core beyond confirming QuadriFlow as the practical default.

## 3.7 Takeaway

The whole landscape converges on **field-aligned parametrization**. The two permissively-licensed,
solver-light, production-proven implementations — **Instant Meshes (BSD)** and **QuadriFlow (MIT)** —
are the same algorithm family and the obvious foundation. Everything else is either a GPL
solver-heavy Family-B tool or a thin Blender wrapper around one of these two.

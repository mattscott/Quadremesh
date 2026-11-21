# 7. References & sources

## Papers (the algorithms)

- **Instant Field-Aligned Meshes** — Wenzel Jakob, Marco Tarini, Daniele Panozzo, Olga
  Sorkine-Hornung. ACM TOG 34(6), SIGGRAPH Asia 2015. *(The core algorithm Quadremesh implements.)*
- **QuadriFlow: A Scalable and Robust Method for Quadrangulation** — Jingwei Huang, Yichao Zhou,
  Matthias Nießner, Jonathan Shewchuk, Leonidas Guibas. SGP / Computer Graphics Forum, 2018.
  https://yichaozhou.com/publication/1805quadriflow/
- **Reliable Feature-Line Driven Quad-Remeshing (QuadWild)** — Nico Pietroni, Stefano Nuvoli,
  Thomas Alderighi, Paolo Cignoni, Marco Tarini. SIGGRAPH 2021.
  https://history.siggraph.org/learning/reliable-feature-line-driven-quad-remeshing-by-pietroni-nuvoli-alderighi-cignoni-and-tarini/
- **Mixed-Integer Quadrangulation (MIQ)** — David Bommes, Henrik Zimmer, Leif Kobbelt.
  SIGGRAPH 2009.
- **Integer-Grid Maps for Reliable Quad Meshing** — David Bommes et al., 2013.

## Repositories analyzed

- **Instant Meshes** (BSD-style) — https://github.com/wjakob/instant-meshes
  - License: https://github.com/wjakob/instant-meshes/blob/master/LICENSE.txt
- **QuadriFlow** (MIT) — https://github.com/hjwdzh/QuadriFlow
- **autoremesher** (GPL-3.0) — https://github.com/huxingyi/autoremesher
  - Pipeline overview: https://deepwiki.com/huxingyi/autoremesher/1-overview
- **QuadWild** (GPL-3.0) — https://github.com/nicopietroni/quadwild
  - Bi-MDF solver variant: https://github.com/cgg-bern/quadwild-bimdf
- **QRemeshify** (GPL-3.0, Blender add-on) — https://github.com/ksami/QRemeshify
- **CurioMesh** (MIT, Blender add-on) — https://github.com/CurioCrafter/CurioMesh

## QuadRemesher (commercial, closed-source — context only)

- Product: https://www.exoside.com/
- User documentation (PDF): https://www.exoside.com/quadremesherdata/QuadRemesher_1.3_UserDoc.pdf
- CG Channel writeup: https://www.cgchannel.com/2019/10/check-out-neat-automatic-retopology-tool-quadremesher/
- BlenderNation: https://www.blendernation.com/2019/10/08/quad-remesher-auto-retopology-add-on-released-for-blender/

## Candidate C# helper libraries (permissive)

- **Math.NET Numerics** (MIT) — dense/sparse linear algebra, if ever needed — https://numerics.mathdotnet.com/
- **CSparse.NET** (MIT) — sparse direct solvers — https://github.com/wo80/CSparse.NET

## License summary (for porting decisions)

| Project | License | Code-copy OK into MIT/BSD C#? | Algorithm reimplementable? |
|---|---|---|---|
| Instant Meshes | BSD-style | Yes (with attribution) | Yes |
| QuadriFlow | MIT | Yes (with attribution) | Yes |
| autoremesher | GPL-3.0 | **No** | Yes (from paper) |
| QuadWild | GPL-3.0 | **No** | Yes (from paper) |

> Reminder: algorithms/maths from published papers are not copyrightable; only source code is.
> We may reimplement any method. We only *copy/adapt code* from BSD/MIT sources.

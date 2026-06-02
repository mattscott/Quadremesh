# Quadremesh

A permissively-licensed, dependency-light, cross-platform **managed-C# (.NET 8)** library + CLI that
turns arbitrary triangle meshes into clean, field-aligned, quad-dominant → pure-quad meshes — an
open, embeddable auto-retopology engine in the QuadRemesher / ZRemesher class.

It is a **clean-room** reimplementation of [Instant Meshes][im] (field solvers + extraction), with
[QuadriFlow][qf]'s min-cost-flow extraction as a pure-quad upgrade. No native dependencies, no
mixed-integer solver, MIT/BSD-clean.

> **Status: P0 — scaffold.** The solution builds and tests run; the remesh pipeline is not yet
> implemented. See [`Agents/Plans`](Agents/Plans) for the full roadmap and
> [`Agents/Docs`](Agents/Docs) for the research it is based on.

## Layout

```
src/
  Quadremesh.Core/   the library (field solvers, hierarchy, extraction) — zero third-party deps
  Quadremesh.IO/     mesh readers/writers (OBJ first)
  Quadremesh.Cli/    `quadremesh in.obj out.obj --target 20000 ...`
tests/
  Quadremesh.Tests/  unit + golden-file tests
  corpus/            shared input meshes (added in P1)
tools/
  reference-oracle/  offline harness that builds Instant Meshes/QuadriFlow to generate golden outputs
```

## Build & test

```sh
dotnet build
dotnet test
```

Requires the .NET 8 runtime (the SDK may be newer; `global.json` rolls forward).

## License

MIT — see [LICENSE](LICENSE). Algorithmic attributions are in [NOTICE](NOTICE).

[im]: https://github.com/wjakob/instant-meshes
[qf]: https://github.com/hjwdzh/QuadriFlow

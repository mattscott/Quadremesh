# Quadremesh

[![ci](https://github.com/mattscott/quadremesh/actions/workflows/ci.yml/badge.svg)](https://github.com/mattscott/quadremesh/actions/workflows/ci.yml) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A permissively-licensed, dependency-light, cross-platform **managed-C# (.NET 8)** library + CLI that
turns arbitrary triangle meshes into clean, field-aligned, quad-dominant → pure-quad meshes — an
open, embeddable auto-retopology engine in the QuadRemesher / ZRemesher class.

It is a **clean-room** reimplementation of [Instant Meshes][im] (field solvers + extraction), with
[QuadriFlow][qf]'s min-cost-flow extraction as a pure-quad upgrade. No native dependencies, no
mixed-integer solver, MIT/BSD-clean.

> **Status: working pipeline, under active development.** Triangle mesh in, quad mesh out. The
> faithful QuadriFlow path is validated stage by stage against a locally built reference QuadriFlow:
> the orientation field, position field and integer flow reproduce it bit-for-bit, and the dense
> benchmark meshes come out watertight with the correct genus (see
> [`docs/plans/07`](docs/plans/07-field-stack-report.md) and [`08`](docs/plans/08-networksimplex-flow-port.md)).
> See [`docs/plans`](docs/plans) for the roadmap and [`docs/design`](docs/design) for the research
> it is based on.

## CLI

```sh
quadremesh input.obj output.obj --target 5000      # aim for ~5,000 quads
quadremesh input.fbx output.fbx --edge 0.02        # or set a target edge length
quadremesh input.obj output.obj --pure --faithful  # pure-quad, faithful QuadriFlow path
```

OBJ and binary FBX are supported for both input and output.

## Layout

```
src/
  Quadremesh.Core/   the library (field solvers, hierarchy, extraction) — zero third-party deps
  Quadremesh.IO/     mesh readers/writers (OBJ, binary FBX)
  Quadremesh.Cli/    `quadremesh in.obj out.obj --target 20000 ...`
tests/
  Quadremesh.Tests/  unit + golden-file tests
  corpus/            shared input meshes; external/ documents the downloaded benchmark set
tools/
  reference-oracle/  offline harness that builds Instant Meshes/QuadriFlow to generate golden outputs
```

## Build & test

```sh
dotnet build
dotnet test
```

Requires the .NET 8 runtime (the SDK may be newer; `global.json` rolls forward).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Please report security issues privately ([SECURITY.md](SECURITY.md)).

## License

MIT — see [LICENSE](LICENSE). Algorithmic attributions are in [NOTICE](NOTICE).

[im]: https://github.com/wjakob/instant-meshes
[qf]: https://github.com/hjwdzh/QuadriFlow

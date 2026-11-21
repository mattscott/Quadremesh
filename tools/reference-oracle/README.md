# Reference oracle

An **offline, dev/CI-only** harness that builds [QuadriFlow][qf] (MIT) — and optionally
[Instant Meshes][im] (BSD-3) — from source and runs them over the test corpus to produce **golden
quad meshes** for comparison. These C++ tools are never shipped, never linked into the managed
artifact; they exist solely to validate `Quadremesh.Core`'s output against a known-good reference
(see [`docs/design/06` §6.6](../../docs/design/06-csharp-portability-analysis.md)).

## What it produces

For each manifold corpus mesh, QuadriFlow is run at a fixed face target and its output quad mesh is
saved to `tests/golden/quadriflow/<mesh>.obj` (committed). The managed test
`ReferenceComparisonTests` then loads each golden, computes its **irregular-vertex count**
(singularity proxy) and quad fraction with our own tooling, runs our remesher on the same input, and
asserts our singularity count is within the acceptance tolerance (~2–4×). When a golden is absent the
test logs `SKIP` and passes, so the suite stays green on machines without the C++ toolchain.

## Pinned references

Commits are pinned in [`build-references.ps1`](build-references.ps1) (QuadriFlow `810b7a0`, Instant
Meshes `7b31608`). Bump deliberately — pinning keeps goldens reproducible.

## Toolchain

The script auto-discovers Visual Studio's C++ tools + bundled CMake via `vswhere` (or falls back to
`cmake` on PATH). **Run it with PowerShell 7 (`pwsh`)** — Windows PowerShell 5.1 mis-handles native
tools' stderr under `$ErrorActionPreference = 'Stop'`.

QuadriFlow's external dependencies are **Eigen** (header-only) and **Boost** (Graph is header-only;
`program_options` is a compiled lib); LEMON is vendored. Install the deps however you like; the
quickest path is conda-forge:

```sh
conda create -n quadref -c conda-forge eigen libboost-devel -y
```

then point CMake at that environment (`-DCMAKE_PREFIX_PATH=<env>`, `-DEIGEN_INCLUDE_DIR=<env>/include/eigen3`).

## Usage

```powershell
pwsh tools/reference-oracle/build-references.ps1 -Build      # discover toolchain, clone, build
pwsh tools/reference-oracle/download-corpus.ps1              # (optional) fetch high-density benchmark meshes
pwsh tools/reference-oracle/build-references.ps1 -Generate   # run over tests/corpus (+ external/) -> tests/golden
```

`download-corpus.ps1` fetches five real, diverse, high-density meshes (sharp-feature CAD, organic,
high-genus, dense scan) into `tests/corpus/external/`, so the goldens benchmark our output on inputs
far denser than the procedural primitives. Those inputs are gitignored (dev/CI-only); see
`tests/corpus/external/PROVENANCE.md`. `-Generate` scans `external/` automatically.

`_external/` (clones) and `build/` are gitignored; only the generated goldens under `tests/golden/`
are committed. Nothing here is needed to build or consume `Quadremesh.Core`.

[im]: https://github.com/wjakob/instant-meshes
[qf]: https://github.com/hjwdzh/QuadriFlow

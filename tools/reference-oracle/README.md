# Reference oracle

An **offline, dev/CI-only** harness that builds [Instant Meshes][im] (BSD-3) and [QuadriFlow][qf]
(MIT) from source and drives them deterministically over the test corpus to produce **golden
outputs** for diffing. These C++ tools are never shipped, never linked into the managed artifact —
they exist solely to validate `Quadremesh.Core`'s field solvers and extraction against a known-good
reference (see [`Agents/Docs/06` §6.6](../../Agents/Docs/06-csharp-portability-analysis.md)).

## What it produces

For each corpus mesh, run at a fixed seed/params:

| Output | From | Used by |
|--------|------|---------|
| extracted quad mesh (`.obj`) | both tools | P5/P6 mesh diff (quad %, Hausdorff) |
| per-vertex orientation/position field dump (where the tool can emit it) | Instant Meshes | P3/P4 field diff |
| singularity set | both tools | P3/P6 singularity-count delta |

Goldens are written to `tests/golden/<tool>/<mesh>/` and **committed**. They are regenerated only by
running this script explicitly — not on every CI run.

## Status (P0)

Skeleton only. The corpus lands in **P1** and field/singularity dumps are wired per stage as
**P3+** stabilize. Running the script today will report what is not yet available and exit cleanly.

## Pinned references

Edit the commit pins in [`build-references.ps1`](build-references.ps1) when intentionally bumping the
oracle. Pinning keeps goldens reproducible.

## Usage

```powershell
# Build the reference tools (clones into _external/, builds into build/ — both gitignored):
pwsh tools/reference-oracle/build-references.ps1 -Build

# Generate goldens for the corpus (P1+):
pwsh tools/reference-oracle/build-references.ps1 -Generate
```

Requires `git` and `cmake` plus a C++ toolchain (MSVC on Windows, gcc/clang on Linux). These are dev
prerequisites only; nothing here is needed to build or consume `Quadremesh.Core`.

[im]: https://github.com/wjakob/instant-meshes
[qf]: https://github.com/hjwdzh/QuadriFlow

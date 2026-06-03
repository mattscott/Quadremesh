# External benchmark meshes — provenance & attribution

High-density, diverse research meshes used **only** to benchmark our quad output against the
QuadriFlow oracle (dev/CI-only; never shipped, never linked into `Quadremesh.Core`). The `.obj` files
are **not committed** — fetch them with `tools/reference-oracle/download-corpus.ps1`. Only the derived
QuadriFlow goldens under `tests/golden/quadriflow/` are committed.

All five are fetched (pinned commit) from
**[alecjacobson/common-3d-test-models](https://github.com/alecjacobson/common-3d-test-models)**, which
mirrors the classic graphics test meshes the [QuadriFlow paper](https://stanford.edu/~jingweih/papers/quadriflow/)
benchmarks on, so our results are directly comparable.

| File | ~Triangles | Stress axis | Original source |
|------|-----------|-------------|-----------------|
| `fandisk.obj`       | 12,946  | hard-surface CAD, sharp creases (P7) | Pratt & Whitney via H. Hoppe (1994) |
| `rocker-arm.obj`    | 20,088  | mechanical, genus-1 through-hole     | AIM@SHAPE |
| `armadillo.obj`     | 99,976  | organic, limbs/extremities (genus 0) | Stanford 3D Scanning Repository |
| `happy.obj`         | 98,601  | Happy Buddha — high genus, thin features | Stanford 3D Scanning Repository |
| `xyzrgb_dragon.obj` | 249,882 | very high-density organic scan       | Stanford 3D Scanning Repository |

These carry research-use provenance rather than a formal CC0/CC-BY license, so they are kept out of the
shipped artifact and out of version control (consistent with the bundled QuadriFlow build, which is
likewise dev-only). If a formally-licensed alternative is preferred for the high-density-organic slot,
**Nefertiti** (CC-BY, attribution to Nora Al-Badri & Jan Nikolai Nelles) from Keenan Crane's CC0
[model repository](https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/) is a drop-in.

Please credit the original sources above (Stanford 3D Scanning Repository; AIM@SHAPE; H. Hoppe) when
reporting any benchmark numbers derived from these meshes.

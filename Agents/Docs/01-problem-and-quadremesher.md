# 1. The problem & how QuadRemesher works

## 1.1 What "auto-retopology" / quad-remeshing is

The input is an arbitrary triangle surface — a sculpt, a photogrammetry/lidar scan, a
marching-cubes isosurface, or a messy CAD tessellation. Such meshes are **unsuitable for
animation, UV-mapping, or subdivision**: triangle density is uneven, edges don't follow the
shape's features, and there's no coherent "edge flow."

**Quad-remeshing** rebuilds that surface as a new mesh that is:

- **Quad-dominant** (ideally pure quads) — quads subdivide cleanly (Catmull-Clark) and deform well.
- **Field-aligned** — edge loops follow principal curvature and feature lines (the "edge flow"
  an artist would hand-author).
- **Uniform or controllably graded** in element size (a *sizing field*).
- **Low and regular** in singularity count (vertices with valence ≠ 4), because singularities
  are where edge loops terminate and where deformation artifacts concentrate.

This is fundamentally a **global** problem: you cannot decide the orientation of one quad
without considering its neighbors all the way around the surface. That is why naive
triangle-pairing or local greedy methods fail, and why the field-based formulation won.

## 1.2 What QuadRemesher (Exoside) is

| | |
|---|---|
| **Author** | Maxime Rouca |
| **Vendor** | Exoside |
| **Lineage** | Same author wrote **ZRemesher** (ZBrush) and ZBrush's **Decimation Master** |
| **Form factor** | Commercial plugin for Blender, Maya, 3ds Max, Modo, Houdini, Cinema 4D, etc. |
| **Algorithm** | **Proprietary and unpublished** — there is no paper or open source |

Because the algorithm is closed, **we cannot "port QuadRemesher."** We can only study the
*open* algorithms that occupy the same space, and the *observable behavior* of QuadRemesher,
which tells us which family it belongs to.

### Observable features → algorithmic implications

| QuadRemesher feature (from docs/marketing) | What it implies about the algorithm |
|---|---|
| Converts triangles / scans / mixed polygons → even quads | Resampling + global parametrization, not triangle-pairing |
| **Guide curves** force edge loops to follow drawn strokes | Hard **directional constraints on a cross/orientation field** |
| **Adaptive Size** varies density with curvature | A **sizing field** modulating the parametrization scale |
| Preserves **hard edges / creases** | Feature-aligned field + seam/cut handling |
| **Symmetry** support | Symmetry plane folded into the field solve |
| Target poly count / edge length | Global **scale** parameter of the position field |

Every one of these maps cleanly onto the **field-aligned parametrization** family
(see [02-algorithm-families.md](02-algorithm-families.md)). QuadRemesher almost certainly
sits in that family with a particularly well-tuned, feature-aware field solve and a robust
quad-extraction stage — which is exactly the part that is hard and where commercial polish lives.

## 1.3 What "good" looks like (acceptance bar for our tool)

A credible open competitor should, on typical input (a 500k–2M triangle sculpt):

1. Produce a **quad-dominant** mesh at a user-specified target resolution.
2. Have **edge flow visibly following curvature** (loops around limbs, along ridges).
3. Keep **singularities low and isolated** (target: within ~2–4× of QuadriFlow/Instant Meshes).
4. **Preserve sharp features** when asked.
5. Run in **seconds to low tens of seconds**, not minutes.
6. Be **robust** to non-manifold edges, small holes, and self-intersections (degrade, don't crash).

Items 1–3 are achievable with the Instant Meshes algorithm. Item 3's *best* results need the
QuadriFlow global step. Items 4–6 are where most engineering effort (and most of the risk) lives.

## 1.4 Why the previous C# attempt likely failed

The prior effort (`Nanomesh`) is a solid **triangle decimator** (quadric edge-collapse) with a
half-edge-style `ConnectedMesh` structure — but decimation is *not* remeshing. A quad-remesher
needs three things the decimator never had:

1. A **cross-field solver** (orientation field) with a multiresolution hierarchy.
2. A **position-field solver** (the parametrization).
3. A **robust quad-extraction** stage.

The decimation code is reusable as a *preprocessing* front-end (producing a clean isotropic
triangle mesh), but the remeshing core is new work. This research exists to make sure that work
targets the right algorithm. See [06-csharp-portability-analysis.md](06-csharp-portability-analysis.md).

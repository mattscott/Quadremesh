# 4. Instant Meshes — implementation-level deep dive

This is the algorithm Quadremesh will implement. Reference: *Instant Field-Aligned Meshes*,
Jakob, Tarini, Panozzo, Sorkine-Hornung, ACM TOG 34(6), SIGGRAPH Asia 2015.

The whole method is **four stages**, each of which is "compute per-vertex variables by repeated
local averaging over a coarse-to-fine hierarchy." There is **no global linear or integer solve**
in the core — this is the property that makes it portable and fast.

```
  Stage 0  Input prep        → clean, ~isotropic triangle mesh + per-vertex normals, areas, adjacency
  Stage 1  Orientation field → per-vertex tangent direction o_i, smooth modulo 90° (4-RoSy)
  Stage 2  Position field    → per-vertex lattice anchor p_i, snapped to an integer grid
  Stage 3  Extraction        → collapse the lattice into quad vertices + faces
```

---

## 4.1 Stage 0 — Input preparation

Inputs needed per vertex `i`:
- position `v_i ∈ ℝ³`
- unit normal `n_i`
- a local area weight `a_i` (e.g. barycentric/Voronoi area) — used as smoothing weight
- the 1-ring adjacency (neighbor list), with per-edge weights `w_ij` (cotangent or uniform)

Two scalars define the target:
- **target edge length** `ℓ` (or equivalently a target vertex/face count → derive `ℓ` from total area)
- **orientation symmetry** = 4 (quads), **position symmetry** = 4

A pre-pass should make the triangle mesh roughly **isotropic** at a resolution comparable to `ℓ`
(uniform-ish triangles). Instant Meshes is tolerant, but isotropy improves field quality and
extraction robustness. (This is where a ported decimation/remesh front-end is used.)

> **Tangent frame.** For each vertex build an orthonormal tangent basis `(t_i, b_i)` with
> `t_i ⟂ n_i`. All directions are expressed in this plane. Any direction is stored as a unit
> 3-vector lying in the tangent plane (the paper keeps directions extrinsic in ℝ³ and re-projects).

---

## 4.2 Stage 1 — Orientation field (4-RoSy cross field)

### Goal
Assign each vertex a tangent direction `o_i` (a unit vector in its tangent plane) such that the
field is **as smooth as possible** across edges, where "smooth" is measured **modulo 90°** because
a quad's edge directions are indistinguishable under 90° rotation.

### The 4-RoSy representative trick
A 4-RoSy direction at vertex `i` represents the set `{o_i, R90 o_i, R180 o_i, R270 o_i}` where
`Rθ` is rotation by θ about `n_i`. To compare directions at neighboring vertices `i, j` you pick
the pair of representatives that are closest:

```
// returns the rotation of d_j (one of 4) that best matches d_i, both already in their tangent planes
Vector3 closestRepresentative(Vector3 d_i, Vector3 n_i, Vector3 d_j, Vector3 n_j):
    best = d_j; bestDot = -inf
    cur  = d_j
    for k in 0..3:
        s = dot(d_i, cur)
        if s > bestDot: bestDot = s; best = cur
        cur = rotate90(cur, n_j)     // rotate 90° about n_j in its tangent plane
    return best
```

### Smoothness energy (what we are minimizing)
```
E_orient = Σ_(i,j)∈edges  w_ij · ‖ o_i − closestRep(o_i, o_j) ‖²
```
subject to each `o_i` being a unit vector in the tangent plane of `n_i`.

### Local update rule (Gauss–Seidel sweep)
We never solve this globally. We sweep over vertices; for each `i` we replace `o_i` by the
**weighted average of its neighbors' closest representatives**, then re-project and renormalize:

```
update_orientation(i):
    acc = w_i_self * o_i           // small self-weight for stability
    for j in neighbors(i):
        r = closestRepresentative(o_i, n_i, o_j, n_j)
        acc += w_ij * r
    acc = acc − dot(acc, n_i) * n_i   // project into tangent plane of i
    o_i = normalize(acc)
```

One sweep touches every vertex once; a handful of sweeps per hierarchy level converges.

### Constraints (this is where features & guides plug in)
- **Sharp edges / creases:** fix `o_i` to align with the crease direction and skip its update
  (Dirichlet constraint), or heavily weight it.
- **Guide curves (QuadRemesher-style):** constrain `o_i` along the stroke tangent — identical mechanism.
- **Symmetry plane:** mirror updates across the plane.

### Multiresolution hierarchy (the speed/robustness secret)
A flat Gauss–Seidel solve converges slowly and gets stuck in local minima (e.g. field "vortices").
Instant Meshes builds a **hierarchy** of progressively coarser graphs and solves coarse→fine:

```
build_hierarchy(mesh):
    level[0] = mesh graph (vertices = nodes, edges weighted by w_ij)
    while level[L].nodes > threshold:
        match nodes greedily into pairs (collapse heaviest edges first)   // graph coarsening
        level[L+1] = quotient graph; record parent/child + accumulated weights/areas
    return levels

solve_field(levels):
    init o at coarsest level (e.g. random or from one neighbor)
    for L from coarsest down to 0:
        for sweep in 1..K: for each node i at level L: update_orientation(i)
        if L > 0: prolong o from level L to level L−1 (children inherit parent direction)
```

Coarsening = exactly the kind of greedy edge-matching the existing decimation infrastructure
already knows how to do; the same `MinHeap`/priority structures apply.

### Singularities
After convergence, a vertex is a **field singularity** where the cross field cannot be combed
consistently around its 1-ring (the accumulated rotation index ≠ 0). These are detected, not
chosen, in Instant Meshes — and they become irregular vertices in the output. Reducing them is
exactly what QuadriFlow adds (see [05](05-quadriflow-and-extensions.md)).

---

## 4.3 Stage 2 — Position field (the parametrization)

### Goal
Assign each vertex a **lattice anchor** `p_i` — a point in `i`'s tangent plane representing the
nearest lattice corner of the target quad grid. The grid at `i` is spanned by `o_i` and
`o_i × n_i` (the two orthogonal quad-edge directions) at spacing `ℓ`. Smoothness here means:
**across an edge, the two anchors differ by an (almost) integer number of lattice steps.**

### The position "representative" trick (4-PoSy)
Just as orientation is defined modulo 90° rotation, position is defined **modulo integer lattice
translation**. To compare anchors `p_i, p_j`, you snap `p_j` to the lattice cell of `p_i`:

```
// snap p_j into the lattice defined at i, so it lands in the same unit cell as p_i
Vector3 latticeRepresentative(p_i, v_i, o_i, n_i, p_j, ℓ):
    e1 = o_i;  e2 = cross(n_i, o_i)            // orthonormal quad-edge directions at i
    d  = p_j − p_i
    k1 = round(dot(d, e1) / ℓ)
    k2 = round(dot(d, e2) / ℓ)
    return p_j − ℓ*(k1*e1 + k2*e2)             // p_j shifted by an integer # of cells
```
(For 4-PoSy with an orientation that may itself be rotated, you combine this with the orientation
representative so the lattices are co-aligned. The paper interleaves the two.)

### Smoothness energy
```
E_pos = Σ_(i,j)∈edges  w_ij · ‖ p_i − latticeRep(p_i, p_j) ‖²
```
each `p_i` constrained to `i`'s tangent plane near `v_i`.

### Local update rule
```
update_position(i):
    acc = w_i_self * p_i
    wsum = w_i_self
    for j in neighbors(i):
        r = latticeRepresentative(p_i, v_i, o_i, n_i, p_j, ℓ)
        acc += w_ij * r;  wsum += w_ij
    p = acc / wsum
    p = v_i + project_to_tangent(p − v_i, n_i)   // keep anchor on tangent plane at v_i
    p_i = p
```

Solved on the **same hierarchy**, coarse→fine, same as orientation. Initialize `p_i = v_i`.

### Sizing field (adaptive density)
Make `ℓ` a per-vertex `ℓ_i` driven by curvature (smaller on high-curvature regions). Everywhere
`ℓ` appears above it becomes the local `ℓ_i` (averaged across the edge). This reproduces
QuadRemesher's "Adaptive Size."

---

## 4.4 Stage 3 — Extraction (lattice → quad mesh)

This is the **hardest, most bug-prone** stage and where most engineering polish goes.

Conceptually: the converged position field tiles the surface with a grid. Each integer lattice
corner becomes one **output vertex**; lattice corners adjacent along `o`/`o×n` become **output edges**;
four corners around a cell become a **quad face**.

A workable extraction (the Instant Meshes approach, simplified):

1. **Collapse to lattice vertices.** Walk the input vertices; any cluster of input vertices that
   round to the same lattice corner (within a cell) merges into a single output vertex. A
   union-find over edges where `latticeRep` gives `k1=k2=0` does this.
2. **Build edges.** For each input edge `(i,j)`, the integer offset `(k1,k2)` tells you which
   neighboring lattice cell `j` belongs to; create output edges between adjacent output vertices
   (offset of one unit in one axis).
3. **Build faces.** Trace minimal cycles (prefer 4-cycles) in the output edge graph to form quads;
   leftover 3- and 5-cycles near singularities become triangles/pentagons (quad-*dominant*).
4. **Cleanup.** Remove doublets/valence-2 vertices, collapse degenerate edges, fill tiny holes,
   reorient faces consistently, snap output vertices back onto the original surface (project to
   nearest point on input mesh to preserve detail).

> Pure-quad output requires either QuadriFlow's integer step (so 3-/5-cycles don't arise) or a
> post-process that pairs stray triangles. Plan to ship **quad-dominant first**, pure-quad later.

---

## 4.5 What we need numerically (and what we don't)

| Need | Required? | C# story |
|---|---|---|
| Dense small linear algebra (3-vectors, frames) | yes | trivial, hand-rolled `Vector3`/`Matrix3` |
| Sparse linear solver | **no** (core is local) | not needed for Instant Meshes core |
| Mixed-integer / Gurobi / CoMISo | **no** | avoided entirely — the whole point |
| Priority queue / union-find | yes | `MinHeap` + a small DSU |
| Graph coarsening (hierarchy) | yes | new, ~200 LOC; mirrors decimation matching |
| Min-cost flow | only for QuadriFlow upgrade | managed network-simplex, ~few hundred LOC |
| kd-tree / BVH (re-projection, isotropic remesh) | yes | new, standard |

**Bottom line:** the Instant Meshes core is local averaging on a graph hierarchy. No exotic solver.
This is precisely why it is the right algorithm to bring to managed C#.

Continue to [05-quadriflow-and-extensions.md](05-quadriflow-and-extensions.md) for the singularity-reducing upgrade,
or [06-csharp-portability-analysis.md](06-csharp-portability-analysis.md) for the licensing/dependency verdict.

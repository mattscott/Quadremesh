using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Fields;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// Faithful port of QuadriFlow's integer-grid-map back end (Agents/Plans/04-quadriflow-integer-port.md).
/// Built up stage by stage; each stage is independently testable. This produces a globally consistent
/// integer offset per edge from which a topology-safe pure-quad mesh is extracted.
/// </summary>
/// <remarks>Reimplemented clean-room from the QuadriFlow source (MIT), read for understanding only.</remarks>
public sealed class QuadriFlowSolver
{
    private readonly HalfEdgeMesh _mesh;
    private readonly Vector3[] _orientation;
    private readonly Vector3[] _normal;
    private readonly Vector3[] _anchor;
    private readonly double[] _scale;

    // A1 — per-face edge topology (BuildEdgeInfo, parametrizer-int.cpp:12).
    private readonly int[,] _faceCorners;  // [face, 0..2] vertex ids in winding order
    private readonly int[,] _faceEdgeIds;  // [face, 0..2] undirected edge id of edge (corner j -> corner j+1)

    // A2 — per-edge integer lattice offset (edge_diff), measured lo -> hi in lo's frame.
    private readonly Vec2i[] _edgeDiff;

    // A3 — per-face per-edge rotation that expresses each edge's diff in the face's frame.
    private readonly int[,] _faceEdgeOrients;

    // A4 — faces carrying an orientation singularity (left out of the comb, so they stay cone points).
    private readonly bool[] _singular;

    // A5 — for each edge, the (face, slot) incidences (≤2). Built once, reused by the comb and the flow.
    private readonly (int Face, int Slot)[][] _edgeIncidence;

    // Sharp-feature locks (P7): allow_changes[e*2+dim] == 0 freezes that edge_diff component so the
    // solve/flip cannot move a quad edge off a crease. All 1 (free) unless ApplySharpConstraints runs.
    private readonly int[] _allowChanges;

    /// <summary>Builds the integer-grid-map inputs (Stages A1–A2) from the converged fields.</summary>
    public QuadriFlowSolver(
        HalfEdgeMesh mesh, Vector3[] orientation, Vector3[] normal, Vector3[] anchor, double[] scale)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentNullException.ThrowIfNull(orientation);
        ArgumentNullException.ThrowIfNull(normal);
        ArgumentNullException.ThrowIfNull(anchor);
        ArgumentNullException.ThrowIfNull(scale);

        _mesh = mesh;
        _orientation = orientation;
        _normal = normal;
        _anchor = anchor;
        _scale = scale;
        _faceCorners = new int[mesh.FaceCount, 3];
        _faceEdgeIds = new int[mesh.FaceCount, 3];
        _edgeDiff = new Vec2i[mesh.EdgeCount];
        _faceEdgeOrients = new int[mesh.FaceCount, 3];
        _singular = new bool[mesh.FaceCount];
        _allowChanges = new int[mesh.EdgeCount * 2];
        Array.Fill(_allowChanges, 1);
        BuildEdgeInfo();
        ComputeEdgeDiff();
        ComputeFaceEdgeOrients();
        _edgeIncidence = BuildEdgeIncidence();
        ComputeSingularities();
        Comb();
    }

    /// <summary>Number of triangular faces.</summary>
    public int FaceCount => _mesh.FaceCount;

    /// <summary>Number of undirected edges.</summary>
    public int EdgeCount => _mesh.EdgeCount;

    /// <summary>Vertex id of corner <paramref name="j"/> (0..2) of face <paramref name="face"/>, in winding order.</summary>
    public int FaceCorner(int face, int j) => _faceCorners[face, j];

    /// <summary>Undirected edge id of edge <paramref name="j"/> (corner j → corner j+1) of face <paramref name="face"/>.</summary>
    public int FaceEdgeId(int face, int j) => _faceEdgeIds[face, j];

    /// <summary>Endpoints (v0 &lt; v1) of undirected edge <paramref name="edge"/>.</summary>
    public (int V0, int V1) EdgeValue(int edge) => _mesh.Edge(edge);

    /// <summary>Integer lattice offset of edge <paramref name="edge"/> (v0 → v1, in v0's lattice frame).</summary>
    public Vec2i EdgeDiff(int edge) => _edgeDiff[edge];

    /// <summary>Rotation (0–3) that expresses edge <paramref name="j"/> of face <paramref name="face"/> in the face's frame.</summary>
    public int FaceEdgeOrient(int face, int j) => _faceEdgeOrients[face, j];

    /// <summary>Whether face <paramref name="face"/> carries an orientation singularity.</summary>
    public bool IsSingular(int face) => _singular[face];

    /// <summary>The (face, slot) incidences of edge <paramref name="edge"/> (2 interior, 1 boundary).</summary>
    public IReadOnlyList<(int Face, int Slot)> EdgeIncidence(int edge) => _edgeIncidence[edge];

    /// <summary>Number of vertices in the field mesh.</summary>
    internal int VertexCount => _mesh.VertexCount;

    /// <summary>Per-vertex orientation field (one cross axis).</summary>
    internal Vector3[] Orientation => _orientation;

    /// <summary>Per-vertex surface normal.</summary>
    internal Vector3[] Normal => _normal;

    /// <summary>Per-vertex position-field anchor.</summary>
    internal Vector3[] Anchor => _anchor;

    /// <summary>Per-vertex lattice scale.</summary>
    internal double[] Scale => _scale;

    /// <summary>Field-mesh vertex positions.</summary>
    internal Vector3[] Positions => _mesh.Positions;

    /// <summary>Per-edge-component change permission (0 = frozen sharp lock). Indexed edge*2+dim.</summary>
    internal int[] AllowChanges => _allowChanges;

    /// <summary>
    /// Applies sharp-feature constraints (P7, QuadriFlow parametrizer.cpp:36–72). For each sharp edge
    /// it zeroes the <see cref="EdgeDiff"/> component perpendicular to the crease (so a quad grid line
    /// runs along the crease) and, where both endpoints are sharp, freezes the zeroed components so the
    /// solve / flip / extraction keep the quad edge on the crease. Call after construction, before
    /// <see cref="Solve"/>. <paramref name="sharpEdge"/> is indexed by undirected edge id.
    /// </summary>
    public void ApplySharpConstraints(bool[] sharpEdge)
    {
        ArgumentNullException.ThrowIfNull(sharpEdge);

        var sharpVert = new bool[_mesh.VertexCount];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            if (sharpEdge[e])
            {
                (int lo, int hi) = _mesh.Edge(e);
                sharpVert[lo] = true;
                sharpVert[hi] = true;
            }
        }

        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            if (!sharpEdge[e])
            {
                continue;
            }

            (int lo, int hi) = _mesh.Edge(e);
            Vec2i d = _edgeDiff[e];
            if (d.X != 0 && d.Y != 0)
            {
                // Decide which lattice axis the crease runs along, in lo's frame (axis0 = q, axis1 = qy).
                Vector3 delta = _anchor[hi] - _anchor[lo];
                Vector3 q = _orientation[lo];
                Vector3 qy = Vector3.Cross(_normal[lo], q);
                _edgeDiff[e] = System.Math.Abs(Vector3.Dot(q, delta)) > System.Math.Abs(Vector3.Dot(qy, delta))
                    ? new Vec2i(d.X, 0)  // edge along q → keep q, drop qy
                    : new Vec2i(0, d.Y); // edge along qy → keep qy, drop q
            }

            if (sharpVert[lo] && sharpVert[hi])
            {
                if (_edgeDiff[e].X == 0)
                {
                    _allowChanges[(e * 2) + 0] = 0;
                }

                if (_edgeDiff[e].Y == 0)
                {
                    _allowChanges[(e * 2) + 1] = 0;
                }
            }
        }
    }

    /// <summary>Builds the dedge twin map (face*3+slot → opposite dedge, −1 on a boundary).</summary>
    internal int[] BuildDedgeTwins()
    {
        var e2e = new int[_mesh.FaceCount * 3];
        Array.Fill(e2e, -1);
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            if (_edgeIncidence[e].Length != 2)
            {
                continue;
            }

            (int f0, int j0) = _edgeIncidence[e][0];
            (int f1, int j1) = _edgeIncidence[e][1];
            e2e[(f0 * 3) + j0] = (f1 * 3) + j1;
            e2e[(f1 * 3) + j1] = (f0 * 3) + j0;
        }

        return e2e;
    }

    /// <summary>Number of singular faces.</summary>
    public int SingularCount
    {
        get
        {
            int n = 0;
            foreach (bool s in _singular)
            {
                if (s)
                {
                    n++;
                }
            }

            return n;
        }
    }

    /// <summary>
    /// Integer divergence of face <paramref name="face"/>: Σ_j rshift90(edge_diff[e_j], orient[f][j]).
    /// Zero on a consistent (non-singular) face once the field is combed and the flow has run.
    /// </summary>
    public Vec2i FaceDivergence(int face)
    {
        var sum = new Vec2i(0, 0);
        for (int j = 0; j < 3; j++)
        {
            sum += Lattice2D.Rshift90(_edgeDiff[_faceEdgeIds[face, j]], _faceEdgeOrients[face, j]);
        }

        return sum;
    }

    private void BuildEdgeInfo()
    {
        // Map each half-edge to its undirected edge id (each half-edge belongs to exactly one edge).
        var halfEdgeToEdge = new int[_mesh.HalfEdgeCount];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            int h = _mesh.EdgeHalfEdge(e);
            halfEdgeToEdge[h] = e;
            int twin = _mesh.Twin(h);
            if (twin >= 0)
            {
                halfEdgeToEdge[twin] = e;
            }
        }

        // Walk each face's three half-edges; corner j and edge j align with the half-edge loop.
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            int h = _mesh.FaceHalfEdge(f);
            for (int j = 0; j < 3; j++)
            {
                _faceCorners[f, j] = _mesh.Origin(h);
                _faceEdgeIds[f, j] = halfEdgeToEdge[h];
                h = _mesh.Next(h);
            }
        }
    }

    private void ComputeEdgeDiff()
    {
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            (int lo, int hi) = _mesh.Edge(e);
            Vector3 axis0 = _orientation[lo];
            Vector3 axis1 = Vector3.Cross(_normal[lo], axis0);
            Vector3 offset = (_anchor[hi] - _anchor[lo]) / _scale[lo];
            int k0 = (int)System.Math.Round(Vector3.Dot(offset, axis0));
            int k1 = (int)System.Math.Round(Vector3.Dot(offset, axis1));
            _edgeDiff[e] = new Vec2i(k0, k1);
        }
    }

    private void ComputeFaceEdgeOrients()
    {
        // Port of parametrizer-int.cpp:76–104. rank1 = v1→v0, rank2 = v2→v0 cross-field transitions.
        // The per-edge orient rotates that edge's diff (stored in its lower-id frame) into v0's frame,
        // and the "+2" handles edges traversed against their stored direction.
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            int v0 = _faceCorners[f, 0];
            int v1 = _faceCorners[f, 1];
            int v2 = _faceCorners[f, 2];
            int rank1 = Lattice2D.Rank(_orientation[v0], _normal[v0], _orientation[v1], _normal[v1]);
            int rank2 = Lattice2D.Rank(_orientation[v0], _normal[v0], _orientation[v2], _normal[v2]);

            _faceEdgeOrients[f, 0] = v1 < v0 ? (rank1 + 2) % 4 : 0;
            _faceEdgeOrients[f, 1] = v2 < v1 ? (rank2 + 2) % 4 : rank1;
            _faceEdgeOrients[f, 2] = v2 < v0 ? rank2 : 2;
        }
    }

    private (int Face, int Slot)[][] BuildEdgeIncidence()
    {
        var lists = new List<(int, int)>[_mesh.EdgeCount];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            lists[e] = new List<(int, int)>(2);
        }

        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                lists[_faceEdgeIds[f, j]].Add((f, j));
            }
        }

        var result = new (int, int)[_mesh.EdgeCount][];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            result[e] = lists[e].ToArray();
        }

        return result;
    }

    private void ComputeSingularities()
    {
        // A face is singular where the cross field cannot be combed consistently around it. Reuse the
        // validated per-face holonomy detector (sphere → 8, torus → 0); singular faces are excluded
        // from the comb so they remain isolated cone points.
        SingularityResult sing = SingularityDetector.Detect(_mesh, _orientation, _normal);
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            _singular[f] = sing.FaceIndex[f] != 0;
        }
    }

    private void Comb()
    {
        // Disjoint-orient-tree comb (parametrizer-int.cpp:115–167): merge non-singular adjacent faces
        // first to fix a consistent global frame, then attach singular faces; finally rotate every
        // face's edge orientations into the root frame.
        var tree = new DisjointOrientTree(_mesh.FaceCount);

        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            if (_edgeIncidence[e].Length != 2)
            {
                continue;
            }

            (int f0, int j0) = _edgeIncidence[e][0];
            (int f1, int j1) = _edgeIncidence[e][1];
            if (_singular[f0] || _singular[f1])
            {
                continue;
            }

            MergeAcross(tree, f0, j0, f1, j1);
        }

        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            if (_edgeIncidence[e].Length != 2)
            {
                continue;
            }

            (int f0, int j0) = _edgeIncidence[e][0];
            (int f1, int j1) = _edgeIncidence[e][1];
            if (!_singular[f0] && !_singular[f1])
            {
                continue; // already merged above
            }

            MergeAcross(tree, f0, j0, f1, j1);
        }

        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            int orient = tree.Orient(f);
            for (int j = 0; j < 3; j++)
            {
                _faceEdgeOrients[f, j] = (_faceEdgeOrients[f, j] + orient) % 4;
            }
        }
    }

    private void MergeAcross(DisjointOrientTree tree, int f0, int j0, int f1, int j1)
    {
        int orient1 = _faceEdgeOrients[f0, j0];
        int orient0 = (_faceEdgeOrients[f1, j1] + 2) % 4;
        tree.Merge(f0, f1, orient0, orient1);
    }

    // --- Stage B — min-cost-flow integer solve ----------------------------------------------------

    /// <summary>
    /// Constraint graph (B1, parametrizer-int.cpp/optimizer.cpp:1231–1252). For each edge component
    /// (e*2+dim) records the ≤2 face-constraints (face*2+k) it appears in and their signs; and the
    /// per-constraint current divergence. The `rshift90((e*2+1, e*2+2), orient)` trick carries both the
    /// component-to-constraint mapping and the sign.
    /// </summary>
    private (int[,] EdgeToConstraints, int[] Initial) BuildConstraintGraph()
    {
        int components = _mesh.EdgeCount * 2;
        var edgeToConstraints = new int[components, 4];
        for (int i = 0; i < components; i++)
        {
            edgeToConstraints[i, 0] = -1;
            edgeToConstraints[i, 2] = -1;
        }

        var initial = new int[_mesh.FaceCount * 2];
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                int e = _faceEdgeIds[f, j];
                Vec2i index = Lattice2D.Rshift90(new Vec2i((e * 2) + 1, (e * 2) + 2), _faceEdgeOrients[f, j]);
                for (int k = 0; k < 2; k++)
                {
                    int signed = index[k];
                    int abs = System.Math.Abs(signed);
                    int s = signed / abs;
                    int ind = abs - 1;          // 0-based edge component id (e*2 + dim)
                    int equationId = (f * 2) + k;
                    if (edgeToConstraints[ind, 0] == -1)
                    {
                        edgeToConstraints[ind, 0] = equationId;
                        edgeToConstraints[ind, 1] = s;
                    }
                    else
                    {
                        edgeToConstraints[ind, 2] = equationId;
                        edgeToConstraints[ind, 3] = s;
                    }

                    initial[equationId] += s * _edgeDiff[ind / 2][ind % 2];
                }
            }
        }

        return (edgeToConstraints, initial);
    }

    /// <summary>Per-constraint divergence (face*2+dim), exposed for testing B1.</summary>
    public int[] ConstraintDivergence() => BuildConstraintGraph().Initial;

    /// <summary>
    /// Stage B3 — the min-cost-flow integer solve. Routes the per-constraint divergence through
    /// conservation arcs (one per opposite-sign edge component) so every reachable face closes up,
    /// adjusting <see cref="EdgeDiff"/> minimally (optimizer.cpp:1230–1336). Retries with growing edge
    /// capacity until the flow saturates or the cap is hit.
    /// </summary>
    public void Solve(int maxIterations = 10)
    {
        Balance();
        (int[,] edgeToConstraints, _) = BuildConstraintGraph();
        int constraintCount = _mesh.FaceCount * 2;
        int edgeCapacity = 2;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            int[] initial = RecomputeInitial(edgeToConstraints, constraintCount);

            var flow = new MinCostFlow();
            int source = flow.AddNode();
            var node = new int[constraintCount];
            for (int c = 0; c < constraintCount; c++)
            {
                node[c] = flow.AddNode();
            }

            int sink = flow.AddNode();

            // Conservation arcs: one per edge component whose two constraints have opposite signs.
            var arcs = new List<(int Component, int Forward, int Reverse)>();
            for (int i = 0; i < edgeToConstraints.GetLength(0); i++)
            {
                int eq0 = edgeToConstraints[i, 0];
                int eq1 = edgeToConstraints[i, 2];
                if (_allowChanges[i] == 0 || eq0 == -1 || eq1 == -1 ||
                    edgeToConstraints[i, 1] != -edgeToConstraints[i, 3])
                {
                    continue; // frozen sharp lock, or not a conservation component
                }

                int v1 = eq0;
                int v2 = eq1;
                if (edgeToConstraints[i, 1] < 0)
                {
                    (v1, v2) = (v2, v1);
                }

                int cur = _edgeDiff[i / 2][i % 2];
                int fc = System.Math.Max(0, cur + edgeCapacity);
                int rc = System.Math.Max(0, -cur + edgeCapacity);
                int fwd = fc > 0 ? flow.AddEdge(node[v1], node[v2], fc, 1) : -1;
                int rev = rc > 0 ? flow.AddEdge(node[v2], node[v1], rc, 1) : -1;
                arcs.Add((i, fwd, rev));
            }

            long supply = 0;
            for (int c = 0; c < constraintCount; c++)
            {
                if (initial[c] > 0)
                {
                    flow.AddEdge(source, node[c], initial[c], 0);
                    supply += initial[c];
                }
                else if (initial[c] < 0)
                {
                    flow.AddEdge(node[c], sink, -initial[c], 0);
                }
            }

            (long routed, _) = flow.MinCostMaxFlow(source, sink);

            // applyTo: edge_diff[var] -= sgn·flow (forward sgn +1, reverse sgn −1).
            foreach ((int component, int fwd, int rev) in arcs)
            {
                long net = (fwd >= 0 ? flow.Flow(fwd) : 0) - (rev >= 0 ? flow.Flow(rev) : 0);
                _edgeDiff[component / 2] = AdjustComponent(_edgeDiff[component / 2], component % 2, -(int)net);
            }

            if (routed == supply)
            {
                break;
            }

            edgeCapacity++;
        }
    }

    /// <summary>
    /// Stage B2 — pre-balancing (parametrizer-int.cpp:293–416). Conservation arcs alone leave some
    /// flow components unbalanced (same-sign and boundary edge components inject net charge). This
    /// nudges those modifiable components' diffs so every component's summed divergence is zero,
    /// making the subsequent flow able to fully route. Deterministic (no random shuffle).
    /// </summary>
    private void Balance()
    {
        int nC = _mesh.FaceCount * 2;
        (int[,] edgeToConstraints, _) = BuildConstraintGraph();

        // Group constraints joined by conservation (opposite-sign) arcs; frozen sharp components are
        // not merged, so they stay fixed (matches the flow's allow_changes skip).
        var uf = new UnionFind(nC);
        for (int i = 0; i < edgeToConstraints.GetLength(0); i++)
        {
            if (_allowChanges[i] != 0 && edgeToConstraints[i, 0] != -1 && edgeToConstraints[i, 2] != -1 &&
                edgeToConstraints[i, 1] == -edgeToConstraints[i, 3])
            {
                uf.Union(edgeToConstraints[i, 0], edgeToConstraints[i, 2]);
            }
        }

        var compact = new Dictionary<int, int>();
        var component = new int[nC];
        for (int c = 0; c < nC; c++)
        {
            int root = uf.Find(c);
            if (!compact.TryGetValue(root, out int id))
            {
                id = compact.Count;
                compact[root] = id;
            }

            component[c] = id;
        }

        var totalFlows = new int[compact.Count];
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            Vec2i d = FaceDivergence(f);
            totalFlows[component[(f * 2) + 0]] += d.X;
            totalFlows[component[(f * 2) + 1]] += d.Y;
        }

        // Per edge component: which constraints it touches, and its net sign (0 = conservation,
        // ±2 = same-sign, ±1 = boundary).
        int comps = _mesh.EdgeCount * 2;
        var firstConstraint = new int[comps];
        var hasSecond = new bool[comps];
        var netSign = new int[comps];
        Array.Fill(firstConstraint, -1);
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                int orient = _faceEdgeOrients[f, j];
                Vec2i sign = Lattice2D.Rshift90(new Vec2i(1, 1), orient);
                int eid = _faceEdgeIds[f, j];
                Vec2i index = Lattice2D.Rshift90(new Vec2i(eid * 2, (eid * 2) + 1), orient);
                for (int k = 0; k < 2; k++)
                {
                    int comp = System.Math.Abs(index[k]);
                    if (firstConstraint[comp] == -1)
                    {
                        firstConstraint[comp] = (f * 2) + k;
                    }
                    else
                    {
                        hasSecond[comp] = true;
                    }

                    netSign[comp] += sign[k];
                }
            }
        }

        // Collect modifiable components per flow component, split by parity (step).
        var modified = new List<(int Comp, int Dir)>[2, compact.Count];
        for (int s = 0; s < 2; s++)
        {
            for (int c = 0; c < compact.Count; c++)
            {
                modified[s, c] = new List<(int, int)>();
            }
        }

        for (int comp = 0; comp < comps; comp++)
        {
            if (_allowChanges[comp] == 0 || firstConstraint[comp] == -1 ||
                (hasSecond[comp] && netSign[comp] == 0))
            {
                continue; // frozen sharp lock, or a pure conservation component — not modifiable
            }

            int find = component[firstConstraint[comp]];
            int step = System.Math.Abs(netSign[comp]) % 2;
            int cur = _edgeDiff[comp / 2][comp % 2];
            int tf = totalFlows[find];
            if (tf > 0)
            {
                if (netSign[comp] > 0 && cur > -1)
                {
                    modified[step, find].Add((comp, -1));
                }
                else if (netSign[comp] < 0 && cur < 1)
                {
                    modified[step, find].Add((comp, 1));
                }
            }
            else if (tf < 0)
            {
                if (netSign[comp] < 0 && cur > -1)
                {
                    modified[step, find].Add((comp, -1));
                }
                else if (netSign[comp] > 0 && cur < 1)
                {
                    modified[step, find].Add((comp, 1));
                }
            }
        }

        for (int c = 0; c < compact.Count; c++)
        {
            for (int ii = 0; ii < 2; ii++)
            {
                if (totalFlows[c] == 0)
                {
                    continue;
                }

                int dir = totalFlows[c] > 0 ? -1 : 1;
                int magnitude = System.Math.Abs(totalFlows[c]);
                int maxNum = ii == 0
                    ? System.Math.Min(magnitude / 2, modified[ii, c].Count)
                    : System.Math.Min(magnitude, modified[ii, c].Count);
                for (int n = 0; n < maxNum; n++)
                {
                    (int comp, int delta) = modified[ii, c][n];
                    _edgeDiff[comp / 2] = AdjustComponent(_edgeDiff[comp / 2], comp % 2, delta);
                    totalFlows[c] += ii == 0 ? 2 * dir : dir;
                }
            }
        }
    }

    /// <summary>
    /// Stage C — extract the pure-quad mesh from the solved <see cref="EdgeDiff"/>. Collapses
    /// zero-offset edges into output vertices, then builds each quad from the two triangles sharing a
    /// (±1,±1) diagonal edge (parametrizer-flip.cpp:438–460). Assumes the field mesh is at the quad
    /// resolution (one quad ≈ two triangles); finer-resolution downsampling is deferred.
    /// </summary>
    public QuadMesh ExtractQuadMesh()
    {
        // Collapse edges with zero offset — their endpoints are the same lattice corner.
        var uf = new UnionFind(_mesh.VertexCount);
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            if (_edgeDiff[e].X == 0 && _edgeDiff[e].Y == 0)
            {
                (int a, int b) = _mesh.Edge(e);
                uf.Union(a, b);
            }
        }

        var rootToOut = new Dictionary<int, int>();
        var outOf = new int[_mesh.VertexCount];
        var posAccum = new List<Vector3>();
        var count = new List<int>();
        for (int v = 0; v < _mesh.VertexCount; v++)
        {
            int root = uf.Find(v);
            if (!rootToOut.TryGetValue(root, out int id))
            {
                id = rootToOut.Count;
                rootToOut[root] = id;
                posAccum.Add(Vector3.Zero);
                count.Add(0);
            }

            outOf[v] = id;
            posAccum[id] += _mesh.Positions[v];
            count[id]++;
        }

        var outPos = new Vector3[posAccum.Count];
        for (int i = 0; i < outPos.Length; i++)
        {
            outPos[i] = posAccum[i] / count[i];
        }

        // A quad is two triangles sharing a (±1,±1) diagonal; collect both halves per diagonal.
        var quads = new Dictionary<long, (int[] First, int[]? Second)>();
        Span<int> corners = stackalloc int[3];
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                corners[j] = outOf[_faceCorners[f, j]];
            }

            for (int j = 0; j < 3; j++)
            {
                Vec2i d = Lattice2D.Rshift90(_edgeDiff[_faceEdgeIds[f, j]], _faceEdgeOrients[f, j]);
                if (System.Math.Abs(d.X) != 1 || System.Math.Abs(d.Y) != 1)
                {
                    continue;
                }

                int v1 = corners[j];
                int v2 = corners[(j + 1) % 3];
                int v3 = corners[(j + 2) % 3];
                long key = ((long)System.Math.Min(v1, v2) << 32) | (uint)System.Math.Max(v1, v2);
                if (quads.TryGetValue(key, out (int[] First, int[]? Second) slot))
                {
                    quads[key] = (slot.First, new[] { v1, v2, v3 });
                }
                else
                {
                    quads[key] = (new[] { v1, v2, v3 }, null);
                }
            }
        }

        var faces = new List<int[]>();
        foreach ((int[] first, int[]? second) in quads.Values)
        {
            if (second is not null && first[2] != second[2])
            {
                faces.Add(new[] { first[1], first[2], first[0], second[2] });
            }
        }

        return CompactQuadMesh(outPos, faces);
    }

    private static QuadMesh CompactQuadMesh(Vector3[] outPos, List<int[]> faces)
    {
        var remap = new int[outPos.Length];
        Array.Fill(remap, -1);
        var positions = new List<Vector3>();
        var compact = new List<int[]>(faces.Count);
        foreach (int[] face in faces)
        {
            var mapped = new int[face.Length];
            for (int i = 0; i < face.Length; i++)
            {
                if (remap[face[i]] < 0)
                {
                    remap[face[i]] = positions.Count;
                    positions.Add(outPos[face[i]]);
                }

                mapped[i] = remap[face[i]];
            }

            compact.Add(mapped);
        }

        return new QuadMesh(positions.ToArray(), compact);
    }

    private int[] RecomputeInitial(int[,] edgeToConstraints, int constraintCount)
    {
        var initial = new int[constraintCount];
        for (int i = 0; i < edgeToConstraints.GetLength(0); i++)
        {
            int comp = _edgeDiff[i / 2][i % 2];
            int eq0 = edgeToConstraints[i, 0];
            int eq1 = edgeToConstraints[i, 2];
            if (eq0 != -1)
            {
                initial[eq0] += edgeToConstraints[i, 1] * comp;
            }

            if (eq1 != -1)
            {
                initial[eq1] += edgeToConstraints[i, 3] * comp;
            }
        }

        return initial;
    }

    private static Vec2i AdjustComponent(Vec2i v, int k, int delta) =>
        k == 0 ? new Vec2i(v.X + delta, v.Y) : new Vec2i(v.X, v.Y + delta);
}

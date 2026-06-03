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
}

using Quadremesh.Core.Math;

namespace Quadremesh.Core.Mesh;

/// <summary>
/// Index-based, struct-of-arrays half-edge connectivity for triangle <em>and</em> polygon meshes.
/// Built clean-room (no code lifted from Nanomesh — see docs/design/06 §6.5) and tolerant of
/// non-manifold input: defective edges are flagged in <see cref="Defects"/>, never thrown.
/// </summary>
/// <remarks>
/// Half-edges are stored per face as directed loops. Boundary half-edges are not materialised;
/// a boundary half-edge has <see cref="Twin"/> = -1. The 1-ring used by the field solvers is exposed
/// as an <em>undirected</em> vertex adjacency (<see cref="Neighbors"/>), which is correct on boundary
/// vertices and robust under non-manifold input; ordered angular circulation can be layered on later
/// when a stage needs it.
/// </remarks>
public sealed class HalfEdgeMesh
{
    // Per half-edge (interior only).
    private readonly int[] _origin;   // tail vertex of the half-edge
    private readonly int[] _next;     // next half-edge around the same face
    private readonly int[] _twin;     // opposite half-edge, or -1 on a boundary
    private readonly int[] _face;     // owning face

    // Per vertex / per face.
    private readonly int[] _vertexHalfEdge; // one outgoing half-edge, or -1 if isolated
    private readonly int[] _faceHalfEdge;   // one half-edge of the face
    private readonly int[] _faceSize;       // sides per face

    // Undirected edges.
    private readonly int[] _edgeV0;
    private readonly int[] _edgeV1;
    private readonly int[] _edgeHalfEdge;   // one incident half-edge of the edge

    // Vertex adjacency in CSR form (undirected, unique neighbours).
    private readonly int[] _adjStart;       // length VertexCount + 1
    private readonly int[] _adjVertex;      // neighbour vertex id
    private readonly int[] _adjEdge;        // edge id joining the pair

    private HalfEdgeMesh(
        Vector3[] positions,
        int[] origin, int[] next, int[] twin, int[] face,
        int[] vertexHalfEdge, int[] faceHalfEdge, int[] faceSize,
        int[] edgeV0, int[] edgeV1, int[] edgeHalfEdge,
        int[] adjStart, int[] adjVertex, int[] adjEdge,
        MeshDefects defects)
    {
        Positions = positions;
        _origin = origin;
        _next = next;
        _twin = twin;
        _face = face;
        _vertexHalfEdge = vertexHalfEdge;
        _faceHalfEdge = faceHalfEdge;
        _faceSize = faceSize;
        _edgeV0 = edgeV0;
        _edgeV1 = edgeV1;
        _edgeHalfEdge = edgeHalfEdge;
        _adjStart = adjStart;
        _adjVertex = adjVertex;
        _adjEdge = adjEdge;
        Defects = defects;
    }

    /// <summary>Vertex positions (shared reference with the source mesh).</summary>
    public Vector3[] Positions { get; }

    /// <summary>Defects discovered during construction.</summary>
    public MeshDefects Defects { get; }

    /// <summary>Number of vertices.</summary>
    public int VertexCount => Positions.Length;

    /// <summary>Number of (interior) half-edges.</summary>
    public int HalfEdgeCount => _origin.Length;

    /// <summary>Number of faces.</summary>
    public int FaceCount => _faceHalfEdge.Length;

    /// <summary>Number of undirected edges.</summary>
    public int EdgeCount => _edgeV0.Length;

    // --- Half-edge accessors -----------------------------------------------------------------

    /// <summary>Tail (origin) vertex of half-edge <paramref name="h"/>.</summary>
    public int Origin(int h) => _origin[h];

    /// <summary>Head (destination) vertex of half-edge <paramref name="h"/>.</summary>
    public int Head(int h) => _origin[_next[h]];

    /// <summary>Next half-edge around the same face.</summary>
    public int Next(int h) => _next[h];

    /// <summary>Opposite half-edge, or -1 if <paramref name="h"/> is on a boundary.</summary>
    public int Twin(int h) => _twin[h];

    /// <summary>Owning face of half-edge <paramref name="h"/>.</summary>
    public int Face(int h) => _face[h];

    // --- Face accessors ----------------------------------------------------------------------

    /// <summary>Number of sides of face <paramref name="f"/>.</summary>
    public int FaceSize(int f) => _faceSize[f];

    /// <summary>One half-edge belonging to face <paramref name="f"/>.</summary>
    public int FaceHalfEdge(int f) => _faceHalfEdge[f];

    /// <summary>Writes the vertex indices of face <paramref name="f"/> into <paramref name="corners"/> and returns the count.</summary>
    public int GetFaceVertices(int f, Span<int> corners)
    {
        int h = _faceHalfEdge[f];
        int k = 0;
        do
        {
            corners[k++] = _origin[h];
            h = _next[h];
        }
        while (h != _faceHalfEdge[f]);

        return k;
    }

    // --- Edge accessors ----------------------------------------------------------------------

    /// <summary>Endpoints of undirected edge <paramref name="e"/> (v0 &lt; v1).</summary>
    public (int V0, int V1) Edge(int e) => (_edgeV0[e], _edgeV1[e]);

    /// <summary>One half-edge incident to edge <paramref name="e"/>.</summary>
    public int EdgeHalfEdge(int e) => _edgeHalfEdge[e];

    // --- Vertex adjacency --------------------------------------------------------------------

    /// <summary>One outgoing half-edge of vertex <paramref name="v"/>, or -1 if isolated.</summary>
    public int VertexHalfEdge(int v) => _vertexHalfEdge[v];

    /// <summary>The unique 1-ring neighbour vertices of <paramref name="v"/> (undirected).</summary>
    public ReadOnlySpan<int> Neighbors(int v) =>
        _adjVertex.AsSpan(_adjStart[v], _adjStart[v + 1] - _adjStart[v]);

    /// <summary>The edge ids joining <paramref name="v"/> to each of its <see cref="Neighbors"/> (parallel array).</summary>
    public ReadOnlySpan<int> NeighborEdges(int v) =>
        _adjEdge.AsSpan(_adjStart[v], _adjStart[v + 1] - _adjStart[v]);

    /// <summary>Valence (number of 1-ring neighbours) of <paramref name="v"/>.</summary>
    public int Valence(int v) => _adjStart[v + 1] - _adjStart[v];

    // --- Construction ------------------------------------------------------------------------

    /// <summary>Builds connectivity from a triangle mesh.</summary>
    public static HalfEdgeMesh FromTriangleMesh(TriangleMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        int triangleCount = mesh.TriangleCount;
        var faces = new int[triangleCount][];
        for (int t = 0; t < triangleCount; t++)
        {
            (int a, int b, int c) = mesh.Triangle(t);
            faces[t] = new[] { a, b, c };
        }

        return Build(mesh.Positions, faces);
    }

    /// <summary>Builds connectivity from a polygon mesh.</summary>
    public static HalfEdgeMesh FromQuadMesh(QuadMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        var faces = new int[mesh.FaceCount][];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            faces[f] = mesh.Face(f).ToArray();
        }

        return Build(mesh.Positions, faces);
    }

    /// <summary>Builds connectivity from positions and explicit polygon faces.</summary>
    public static HalfEdgeMesh Build(Vector3[] positions, IReadOnlyList<int[]> faces)
    {
        ArgumentNullException.ThrowIfNull(positions);
        ArgumentNullException.ThrowIfNull(faces);

        int nV = positions.Length;
        int nF = faces.Count;

        int nHE = 0;
        for (int f = 0; f < nF; f++)
        {
            nHE += faces[f].Length;
        }

        var origin = new int[nHE];
        var next = new int[nHE];
        var twin = new int[nHE];
        var face = new int[nHE];
        Array.Fill(twin, -1);

        var vertexHalfEdge = new int[nV];
        Array.Fill(vertexHalfEdge, -1);
        var faceHalfEdge = new int[nF];
        var faceSize = new int[nF];

        // Pass 1: lay out per-face half-edge loops.
        int h = 0;
        for (int f = 0; f < nF; f++)
        {
            int[] corners = faces[f];
            int k = corners.Length;
            int baseH = h;
            faceHalfEdge[f] = baseH;
            faceSize[f] = k;
            for (int i = 0; i < k; i++)
            {
                origin[h] = corners[i];
                next[h] = baseH + ((i + 1) % k);
                face[h] = f;
                vertexHalfEdge[corners[i]] = h;
                h++;
            }
        }

        // Pass 2: pair twins, register undirected edges, flag non-manifold defects.
        var directedToHe = new Dictionary<long, int>(nHE);
        var undirectedToEdge = new Dictionary<long, int>(nHE);
        var edgeIncidence = new Dictionary<long, int>(nHE);
        var edgeV0 = new List<int>(nHE / 2);
        var edgeV1 = new List<int>(nHE / 2);
        var edgeHalfEdge = new List<int>(nHE / 2);
        int nonManifoldEdges = 0;

        for (h = 0; h < nHE; h++)
        {
            int o = origin[h];
            int d = origin[next[h]];

            // Twin pairing on directed edges.
            long oppKey = ((long)d * nV) + o;
            if (directedToHe.TryGetValue(oppKey, out int opp) && twin[opp] == -1 && twin[h] == -1)
            {
                twin[h] = opp;
                twin[opp] = h;
                directedToHe.Remove(oppKey);
            }
            else
            {
                long key = ((long)o * nV) + d;
                if (!directedToHe.TryAdd(key, h))
                {
                    // Same directed edge seen twice: inconsistent orientation / non-manifold.
                    nonManifoldEdges++;
                }
            }

            // Undirected edge registration + incidence count.
            int lo = System.Math.Min(o, d);
            int hi = System.Math.Max(o, d);
            long uKey = ((long)lo * nV) + hi;
            if (undirectedToEdge.TryAdd(uKey, edgeV0.Count))
            {
                edgeV0.Add(lo);
                edgeV1.Add(hi);
                edgeHalfEdge.Add(h);
                edgeIncidence[uKey] = 1;
            }
            else
            {
                int incidence = ++edgeIncidence[uKey];
                if (incidence == 3)
                {
                    nonManifoldEdges++; // count each over-incident edge once
                }
            }
        }

        int boundaryEdges = 0;
        foreach (int incidence in edgeIncidence.Values)
        {
            if (incidence == 1)
            {
                boundaryEdges++;
            }
        }

        // Pass 3: vertex adjacency (CSR) from undirected edges.
        int nE = edgeV0.Count;
        var degree = new int[nV];
        for (int e = 0; e < nE; e++)
        {
            degree[edgeV0[e]]++;
            degree[edgeV1[e]]++;
        }

        var adjStart = new int[nV + 1];
        for (int v = 0; v < nV; v++)
        {
            adjStart[v + 1] = adjStart[v] + degree[v];
        }

        var adjVertex = new int[adjStart[nV]];
        var adjEdge = new int[adjStart[nV]];
        var cursor = new int[nV];
        Array.Copy(adjStart, cursor, nV);
        for (int e = 0; e < nE; e++)
        {
            int a = edgeV0[e];
            int b = edgeV1[e];
            adjVertex[cursor[a]] = b;
            adjEdge[cursor[a]] = e;
            cursor[a]++;
            adjVertex[cursor[b]] = a;
            adjEdge[cursor[b]] = e;
            cursor[b]++;
        }

        int isolated = 0;
        for (int v = 0; v < nV; v++)
        {
            if (degree[v] == 0)
            {
                isolated++;
            }
        }

        var defects = new MeshDefects
        {
            NonManifoldEdges = nonManifoldEdges,
            BoundaryEdges = boundaryEdges,
            IsolatedVertices = isolated,
        };

        return new HalfEdgeMesh(
            positions, origin, next, twin, face,
            vertexHalfEdge, faceHalfEdge, faceSize,
            edgeV0.ToArray(), edgeV1.ToArray(), edgeHalfEdge.ToArray(),
            adjStart, adjVertex, adjEdge,
            defects);
    }
}

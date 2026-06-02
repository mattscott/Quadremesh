using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Remeshing;

/// <summary>
/// A mutable, manifold triangle mesh supporting the local operators the isotropic remesher needs:
/// edge <see cref="Split"/>, <see cref="Collapse"/>, and <see cref="Flip"/>. Connectivity is kept in
/// a directed-edge map (each directed edge belongs to exactly one triangle), so every operator is
/// expressed as a small set of triangle add/removes rather than fragile pointer surgery. Removed
/// slots are recycled via free lists; call <see cref="ToTriangleMesh"/> to compact and export.
/// </summary>
public sealed class DynamicTriangleMesh
{
    private const double AreaEpsilon = 1e-16;

    private readonly List<Vector3> _position = new();
    private readonly List<bool> _vertexRemoved = new();
    private readonly List<HashSet<int>> _vertexTris = new();
    private readonly Stack<int> _freeVertices = new();

    private readonly List<int> _t0 = new();
    private readonly List<int> _t1 = new();
    private readonly List<int> _t2 = new();
    private readonly List<bool> _triRemoved = new();
    private readonly Stack<int> _freeTris = new();

    // Directed edge (a -> b) -> triangle that contains it. Manifold invariant: at most one.
    private readonly Dictionary<long, int> _edgeToTri = new();

    /// <summary>Number of triangles dropped during construction because they violated manifoldness.</summary>
    public int DroppedTriangles { get; private set; }

    /// <summary>Number of live vertices.</summary>
    public int LiveVertexCount { get; private set; }

    /// <summary>Number of live triangles.</summary>
    public int LiveTriangleCount { get; private set; }

    /// <summary>Upper bound on vertex ids (including removed slots).</summary>
    public int VertexCapacity => _position.Count;

    /// <summary>Upper bound on triangle ids (including removed slots).</summary>
    public int TriangleCapacity => _t0.Count;

    /// <summary>Builds a dynamic mesh from a triangle mesh, dropping any triangle that would break manifoldness.</summary>
    public static DynamicTriangleMesh FromTriangleMesh(TriangleMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        var dyn = new DynamicTriangleMesh();
        foreach (Vector3 p in mesh.Positions)
        {
            dyn.AddVertex(p);
        }

        for (int t = 0; t < mesh.TriangleCount; t++)
        {
            (int a, int b, int c) = mesh.Triangle(t);
            if (dyn.AddTriangle(a, b, c) < 0)
            {
                dyn.DroppedTriangles++;
            }
        }

        return dyn;
    }

    // --- Accessors ----------------------------------------------------------------------------

    /// <summary>Position of vertex <paramref name="v"/>.</summary>
    public Vector3 Position(int v) => _position[v];

    /// <summary>Sets the position of vertex <paramref name="v"/>.</summary>
    public void SetPosition(int v, Vector3 p) => _position[v] = p;

    /// <summary>Whether vertex <paramref name="v"/> has been removed.</summary>
    public bool VertexRemoved(int v) => _vertexRemoved[v];

    /// <summary>Whether triangle <paramref name="t"/> has been removed.</summary>
    public bool TriangleRemoved(int t) => _triRemoved[t];

    /// <summary>The three corner vertices of triangle <paramref name="t"/>.</summary>
    public (int A, int B, int C) Triangle(int t) => (_t0[t], _t1[t], _t2[t]);

    /// <summary>The incident (live) triangles of vertex <paramref name="v"/>.</summary>
    public IReadOnlyCollection<int> IncidentTriangles(int v) => _vertexTris[v];

    /// <summary>Whether the undirected edge (u, w) lies on a boundary (present in only one direction).</summary>
    public bool EdgeIsBoundary(int u, int w) =>
        _edgeToTri.ContainsKey(Key(u, w)) ^ _edgeToTri.ContainsKey(Key(w, u));

    /// <summary>Whether vertex <paramref name="v"/> touches any boundary edge.</summary>
    public bool IsBoundaryVertex(int v)
    {
        foreach (int nb in Neighbors(v))
        {
            if (EdgeIsBoundary(v, nb))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>The unique 1-ring neighbour vertices of <paramref name="v"/>.</summary>
    public HashSet<int> Neighbors(int v)
    {
        var set = new HashSet<int>();
        foreach (int t in _vertexTris[v])
        {
            (int a, int b, int c) = Triangle(t);
            if (a != v)
            {
                set.Add(a);
            }

            if (b != v)
            {
                set.Add(b);
            }

            if (c != v)
            {
                set.Add(c);
            }
        }

        return set;
    }

    // --- Operators ----------------------------------------------------------------------------

    /// <summary>
    /// Splits interior edge (a, b) at <paramref name="midpoint"/>, returning the new vertex id (or -1
    /// if the edge is on a boundary). The two incident triangles become four.
    /// </summary>
    public int Split(int a, int b, Vector3 midpoint)
    {
        if (!TryGetEdgeTriangles(a, b, out int t1, out int t2, out int c, out int d))
        {
            return -1; // boundary or non-manifold edge
        }

        int m = AddVertex(midpoint);
        RemoveTriangle(t1);
        RemoveTriangle(t2);
        AddTriangle(a, m, c);
        AddTriangle(m, b, c);
        AddTriangle(b, m, d);
        AddTriangle(m, a, d);
        return m;
    }

    /// <summary>
    /// Flips interior edge (a, b) to the opposite diagonal, improving valence. Returns false (no change)
    /// if the edge is on a boundary, the new diagonal already exists, or the flip would invert a triangle.
    /// </summary>
    public bool Flip(int a, int b)
    {
        if (!TryGetEdgeTriangles(a, b, out int t1, out int t2, out int c, out int d))
        {
            return false;
        }

        if (c == d || _edgeToTri.ContainsKey(Key(c, d)) || _edgeToTri.ContainsKey(Key(d, c)))
        {
            return false; // would duplicate an edge
        }

        // New triangles (c, a, d) and (c, d, b) must keep the original surface orientation.
        Vector3 oldRef = TriangleNormal(a, b, c) + TriangleNormal(b, a, d);
        if (!OrientationPreserved(oldRef, _position[c], _position[a], _position[d]) ||
            !OrientationPreserved(oldRef, _position[c], _position[d], _position[b]))
        {
            return false;
        }

        RemoveTriangle(t1);
        RemoveTriangle(t2);
        AddTriangle(c, a, d);
        AddTriangle(c, d, b);
        return true;
    }

    /// <summary>
    /// Collapses edge (a, b): vertex <paramref name="a"/> is merged into <paramref name="b"/>, which moves
    /// to <paramref name="target"/>. Returns false (no change) if either endpoint is on a boundary, the
    /// link condition fails (would create a non-manifold), or any surviving triangle would invert/degenerate.
    /// </summary>
    public bool Collapse(int a, int b, Vector3 target)
    {
        if (!TryGetEdgeTriangles(a, b, out int t1, out int t2, out int c, out int d))
        {
            return false;
        }

        if (IsBoundaryVertex(a) || IsBoundaryVertex(b))
        {
            return false; // boundaries are pinned in the pre-pass
        }

        // Link condition: the only common neighbours of a and b may be the two edge apexes c and d.
        HashSet<int> na = Neighbors(a);
        HashSet<int> nb = Neighbors(b);
        foreach (int n in na)
        {
            if (n != c && n != d && nb.Contains(n))
            {
                return false;
            }
        }

        // Validate every surviving triangle (around a → relabelled to b, and around b) with b at target.
        if (!CollapseKeepsOrientation(a, b, c, d, t1, t2, target))
        {
            return false;
        }

        int[] incidentA = _vertexTris[a].ToArray();
        RemoveTriangle(t1);
        RemoveTriangle(t2);
        foreach (int t in incidentA)
        {
            if (t == t1 || t == t2)
            {
                continue;
            }

            (int x, int y, int z) = Triangle(t);
            RemoveTriangle(t);
            AddTriangle(x == a ? b : x, y == a ? b : y, z == a ? b : z);
        }

        _position[b] = target;
        RemoveVertex(a);
        return true;
    }

    /// <summary>Compacts live vertices/triangles and exports an immutable <see cref="TriangleMesh"/>.</summary>
    public TriangleMesh ToTriangleMesh()
    {
        var remap = new int[_position.Count];
        Array.Fill(remap, -1);
        var positions = new List<Vector3>(LiveVertexCount);
        for (int v = 0; v < _position.Count; v++)
        {
            if (!_vertexRemoved[v])
            {
                remap[v] = positions.Count;
                positions.Add(_position[v]);
            }
        }

        var indices = new List<int>(LiveTriangleCount * 3);
        for (int t = 0; t < _t0.Count; t++)
        {
            if (_triRemoved[t])
            {
                continue;
            }

            indices.Add(remap[_t0[t]]);
            indices.Add(remap[_t1[t]]);
            indices.Add(remap[_t2[t]]);
        }

        return new TriangleMesh(positions.ToArray(), indices.ToArray());
    }

    // --- Internal element management ----------------------------------------------------------

    private int AddVertex(Vector3 p)
    {
        LiveVertexCount++;
        if (_freeVertices.Count > 0)
        {
            int v = _freeVertices.Pop();
            _position[v] = p;
            _vertexRemoved[v] = false;
            _vertexTris[v].Clear();
            return v;
        }

        _position.Add(p);
        _vertexRemoved.Add(false);
        _vertexTris.Add(new HashSet<int>());
        return _position.Count - 1;
    }

    private void RemoveVertex(int v)
    {
        _vertexRemoved[v] = true;
        _vertexTris[v].Clear();
        _freeVertices.Push(v);
        LiveVertexCount--;
    }

    private int AddTriangle(int a, int b, int c)
    {
        // Reject anything that would put two triangles on the same directed edge (non-manifold).
        if (a == b || b == c || c == a ||
            _edgeToTri.ContainsKey(Key(a, b)) ||
            _edgeToTri.ContainsKey(Key(b, c)) ||
            _edgeToTri.ContainsKey(Key(c, a)))
        {
            return -1;
        }

        int t;
        if (_freeTris.Count > 0)
        {
            t = _freeTris.Pop();
            _t0[t] = a;
            _t1[t] = b;
            _t2[t] = c;
            _triRemoved[t] = false;
        }
        else
        {
            t = _t0.Count;
            _t0.Add(a);
            _t1.Add(b);
            _t2.Add(c);
            _triRemoved.Add(false);
        }

        _edgeToTri[Key(a, b)] = t;
        _edgeToTri[Key(b, c)] = t;
        _edgeToTri[Key(c, a)] = t;
        _vertexTris[a].Add(t);
        _vertexTris[b].Add(t);
        _vertexTris[c].Add(t);
        LiveTriangleCount++;
        return t;
    }

    private void RemoveTriangle(int t)
    {
        (int a, int b, int c) = Triangle(t);
        RemoveDirected(a, b, t);
        RemoveDirected(b, c, t);
        RemoveDirected(c, a, t);
        _vertexTris[a].Remove(t);
        _vertexTris[b].Remove(t);
        _vertexTris[c].Remove(t);
        _triRemoved[t] = true;
        _freeTris.Push(t);
        LiveTriangleCount--;
    }

    private void RemoveDirected(int a, int b, int owner)
    {
        long key = Key(a, b);
        if (_edgeToTri.TryGetValue(key, out int t) && t == owner)
        {
            _edgeToTri.Remove(key);
        }
    }

    /// <summary>Resolves the two triangles and apex vertices of interior edge (a, b).</summary>
    private bool TryGetEdgeTriangles(int a, int b, out int t1, out int t2, out int c, out int d)
    {
        t2 = c = d = -1;
        if (!_edgeToTri.TryGetValue(Key(a, b), out t1) || !_edgeToTri.TryGetValue(Key(b, a), out t2))
        {
            t1 = -1;
            return false;
        }

        c = Apex(t1, a, b);
        d = Apex(t2, b, a);
        return true;
    }

    private int Apex(int t, int a, int b)
    {
        (int x, int y, int z) = Triangle(t);
        if (x != a && x != b)
        {
            return x;
        }

        return y != a && y != b ? y : z;
    }

    private bool CollapseKeepsOrientation(int a, int b, int c, int d, int t1, int t2, Vector3 target)
    {
        foreach (int t in _vertexTris[a])
        {
            if (t == t1 || t == t2)
            {
                continue;
            }

            (int x, int y, int z) = Triangle(t);
            if (!OrientationPreservedAfterMove(t, x == a ? b : x, y == a ? b : y, z == a ? b : z, b, target))
            {
                return false;
            }
        }

        foreach (int t in _vertexTris[b])
        {
            if (t == t1 || t == t2)
            {
                continue;
            }

            (int x, int y, int z) = Triangle(t);
            if (!OrientationPreservedAfterMove(t, x, y, z, b, target))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Checks that triangle (nx, ny, nz), with vertex <paramref name="moved"/> placed at <paramref name="target"/>, keeps the orientation of original triangle <paramref name="oldTri"/>.</summary>
    private bool OrientationPreservedAfterMove(int oldTri, int nx, int ny, int nz, int moved, Vector3 target)
    {
        (int ox, int oy, int oz) = Triangle(oldTri);
        Vector3 oldRef = TriangleNormal(ox, oy, oz);
        Vector3 p0 = nx == moved ? target : _position[nx];
        Vector3 p1 = ny == moved ? target : _position[ny];
        Vector3 p2 = nz == moved ? target : _position[nz];
        return OrientationPreserved(oldRef, p0, p1, p2);
    }

    private Vector3 TriangleNormal(int a, int b, int c) =>
        MathUtils.TriangleNormalAreaWeighted(_position[a], _position[b], _position[c]);

    private static bool OrientationPreserved(Vector3 reference, Vector3 p0, Vector3 p1, Vector3 p2)
    {
        Vector3 n = MathUtils.TriangleNormalAreaWeighted(p0, p1, p2);
        return n.LengthSquared > AreaEpsilon && Vector3.Dot(reference, n) > 0.0;
    }

    private static long Key(int a, int b) => ((long)a << 32) | (uint)b;
}

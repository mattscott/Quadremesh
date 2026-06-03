using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// Topology-safe pure-quad extraction from a solved <see cref="QuadriFlowSolver"/>: collapse
/// zero-offset edges into a coarse graph, then port QuadriFlow's <c>BuildTriangleManifold</c>
/// (parametrizer-flip.cpp:267–460) — a dedge fan-walk + colour-split that manifold-izes the result
/// and keeps singularities as clean cone points — and build each quad from the two coarse triangles
/// sharing a (±1,±1) diagonal. Reimplemented clean-room (QuadriFlow is MIT, read for understanding).
/// </summary>
public sealed class QuadriFlowExtractor
{
    private readonly QuadriFlowSolver _solver;
    private readonly Vector3[] _positions;

    // Coarse graph (after collapsing zero-offset edges).
    private int[] _coarseOf = Array.Empty<int>();        // original vertex -> coarse vertex
    private int _coarseVertexCount;
    private int[,] _cfCorners = new int[0, 0];           // coarse face -> 3 coarse vertices
    private Vec2i[,] _cfDiff = new Vec2i[0, 0];          // coarse face edge j -> edge_diff (original frame)
    private int[,] _cfOrient = new int[0, 0];            // coarse face edge j -> orient
    private int[,] _f2e = new int[0, 0];                 // coarse face -> 3 coarse edge ids
    private int[] _e2e = Array.Empty<int>();             // dedge -> opposite dedge (face*3+corner), -1 boundary

    /// <summary>Creates an extractor for a solved field.</summary>
    public QuadriFlowExtractor(QuadriFlowSolver solver, Vector3[] positions)
    {
        _solver = solver ?? throw new ArgumentNullException(nameof(solver));
        _positions = positions ?? throw new ArgumentNullException(nameof(positions));
    }

    /// <summary>Number of coarse faces (non-degenerate triangles after collapse).</summary>
    public int CoarseFaceCount => _cfCorners.GetLength(0);

    /// <summary>Builds the manifold pure-quad mesh.</summary>
    public QuadMesh Extract()
    {
        BuildCoarseGraph();
        int[,] triangleVertices = BuildTriangleVertices(out int outVertexCount, out int[] outGroup);
        return BuildQuads(triangleVertices, outVertexCount, outGroup);
    }

    // --- C2: collapse zero-offset edges, build the coarse triangle graph + its dual --------------

    private void BuildCoarseGraph()
    {
        int vCount = _positions.Length;
        var uf = new UnionFind(vCount);
        for (int e = 0; e < _solver.EdgeCount; e++)
        {
            Vec2i d = _solver.EdgeDiff(e);
            if (d.X == 0 && d.Y == 0)
            {
                (int a, int b) = _solver.EdgeValue(e);
                uf.Union(a, b);
            }
        }

        var rootToCoarse = new Dictionary<int, int>();
        _coarseOf = new int[vCount];
        for (int v = 0; v < vCount; v++)
        {
            int root = uf.Find(v);
            if (!rootToCoarse.TryGetValue(root, out int id))
            {
                id = rootToCoarse.Count;
                rootToCoarse[root] = id;
            }

            _coarseOf[v] = id;
        }

        _coarseVertexCount = rootToCoarse.Count;

        // Coarse faces = original faces whose 3 corners land in 3 distinct coarse vertices.
        var cfCorners = new List<(int A, int B, int C)>();
        var cfDiff = new List<(Vec2i, Vec2i, Vec2i)>();
        var cfOrient = new List<(int, int, int)>();
        var coarseEdge = new Dictionary<long, int>();
        var f2e = new List<int[]>();
        for (int f = 0; f < _solver.FaceCount; f++)
        {
            int a = _coarseOf[_solver.FaceCorner(f, 0)];
            int b = _coarseOf[_solver.FaceCorner(f, 1)];
            int c = _coarseOf[_solver.FaceCorner(f, 2)];
            if (a == b || b == c || a == c)
            {
                continue; // degenerate after collapse
            }

            cfCorners.Add((a, b, c));
            cfDiff.Add((_solver.EdgeDiff(_solver.FaceEdgeId(f, 0)),
                        _solver.EdgeDiff(_solver.FaceEdgeId(f, 1)),
                        _solver.EdgeDiff(_solver.FaceEdgeId(f, 2))));
            cfOrient.Add((_solver.FaceEdgeOrient(f, 0), _solver.FaceEdgeOrient(f, 1), _solver.FaceEdgeOrient(f, 2)));

            var e3 = new int[3];
            for (int j = 0; j < 3; j++)
            {
                int u = j == 0 ? a : j == 1 ? b : c;
                int w = j == 0 ? b : j == 1 ? c : a;
                long key = ((long)System.Math.Min(u, w) << 32) | (uint)System.Math.Max(u, w);
                if (!coarseEdge.TryGetValue(key, out int ce))
                {
                    ce = coarseEdge.Count;
                    coarseEdge[key] = ce;
                }

                e3[j] = ce;
            }

            f2e.Add(e3);
        }

        int cfCount = cfCorners.Count;
        _cfCorners = new int[cfCount, 3];
        _cfDiff = new Vec2i[cfCount, 3];
        _cfOrient = new int[cfCount, 3];
        _f2e = new int[cfCount, 3];
        for (int cf = 0; cf < cfCount; cf++)
        {
            (_cfCorners[cf, 0], _cfCorners[cf, 1], _cfCorners[cf, 2]) = cfCorners[cf];
            (_cfDiff[cf, 0], _cfDiff[cf, 1], _cfDiff[cf, 2]) = cfDiff[cf];
            (_cfOrient[cf, 0], _cfOrient[cf, 1], _cfOrient[cf, 2]) = cfOrient[cf];
            for (int j = 0; j < 3; j++)
            {
                _f2e[cf, j] = f2e[cf][j];
            }
        }

        BuildDual(coarseEdge.Count);
    }

    /// <summary>Builds the dedge dual E2E from the coarse face→edge incidence.</summary>
    private void BuildDual(int coarseEdgeCount)
    {
        int cfCount = _cfCorners.GetLength(0);
        // For each coarse edge, the (≤2) incident dedges (face*3+corner).
        var first = new int[coarseEdgeCount];
        var second = new int[coarseEdgeCount];
        Array.Fill(first, -1);
        Array.Fill(second, -1);
        for (int cf = 0; cf < cfCount; cf++)
        {
            for (int j = 0; j < 3; j++)
            {
                int ce = _f2e[cf, j];
                if (first[ce] == -1)
                {
                    first[ce] = (cf * 3) + j;
                }
                else
                {
                    second[ce] = (cf * 3) + j;
                }
            }
        }

        _e2e = new int[cfCount * 3];
        Array.Fill(_e2e, -1);
        for (int ce = 0; ce < coarseEdgeCount; ce++)
        {
            if (first[ce] != -1 && second[ce] != -1)
            {
                _e2e[first[ce]] = second[ce];
                _e2e[second[ce]] = first[ce];
            }
        }
    }

    // --- C3: BuildTriangleManifold Part A (fan-walk + colour-split) ------------------------------

    private int[,] BuildTriangleVertices(out int numV, out int[] outGroup)
    {
        int cfCount = _cfCorners.GetLength(0);
        var tv = new int[cfCount, 3];
        for (int i = 0; i < cfCount; i++)
        {
            tv[i, 0] = tv[i, 1] = tv[i, 2] = -1;
        }

        numV = 0;
        var group = new List<int>(); // output vertex -> coarse vertex (for geometry)

        // Fan assignment: walk the fan around each corner's coarse vertex, giving the whole fan one id.
        for (int i = 0; i < cfCount; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (tv[i, j] != -1)
                {
                    continue;
                }

                group.Add(_cfCorners[i, j]);
                int deid0 = (i * 3) + j;
                int deid = deid0;
                do
                {
                    tv[deid / 3, deid % 3] = numV;
                    deid = _e2e[(deid / 3 * 3) + ((deid + 2) % 3)];
                }
                while (deid != deid0 && deid != -1);

                if (deid == -1)
                {
                    deid = deid0;
                    while (true)
                    {
                        deid = _e2e[deid];
                        if (deid == -1)
                        {
                            break;
                        }

                        deid = (deid / 3 * 3) + ((deid + 1) % 3);
                        tv[deid / 3, deid % 3] = numV;
                    }
                }

                numV++;
            }
        }

        ColourSplit(tv, ref numV, group);
        outGroup = group.ToArray();
        return tv;
    }

    /// <summary>The colour-split: split an output vertex whose dedge fan loops back (a pinch).</summary>
    private void ColourSplit(int[,] tv, ref int numV, List<int> group)
    {
        int cfCount = tv.GetLength(0);
        int numV0;
        do
        {
            numV0 = numV;
            var vertToDedge = new List<int>[numV];
            for (int i = 0; i < numV; i++)
            {
                vertToDedge[i] = new List<int>();
            }

            for (int i = 0; i < cfCount; i++)
            {
                if (tv[i, 0] == tv[i, 1] || tv[i, 1] == tv[i, 2] || tv[i, 0] == tv[i, 2])
                {
                    for (int j = 0; j < 3; j++)
                    {
                        int t = _e2e[(i * 3) + j];
                        if (t != -1)
                        {
                            _e2e[t] = -1;
                        }

                        _e2e[(i * 3) + j] = -1;
                    }
                }
                else
                {
                    for (int j = 0; j < 3; j++)
                    {
                        vertToDedge[tv[i, j]].Add((i * 3) + j);
                    }
                }
            }

            var colors = new int[cfCount * 3];
            Array.Fill(colors, -1);
            for (int vi = 0; vi < vertToDedge.Length; vi++)
            {
                int numColor = 0;
                foreach (int startDeid in vertToDedge[vi])
                {
                    if (colors[startDeid] != -1)
                    {
                        continue;
                    }

                    var l = new LinkedList<int>();
                    int deid = startDeid;
                    do
                    {
                        l.AddLast(deid);
                        deid = _e2e[(deid / 3 * 3) + ((deid + 2) % 3)];
                    }
                    while (deid != -1 && deid != startDeid);

                    if (deid == -1)
                    {
                        deid = startDeid;
                        while (true)
                        {
                            deid = _e2e[deid];
                            if (deid == -1)
                            {
                                break;
                            }

                            deid = (deid / 3 * 3) + ((deid + 1) % 3);
                            if (deid == startDeid)
                            {
                                break;
                            }

                            l.AddFirst(deid);
                        }
                    }

                    var dedges = new List<int>(l);
                    var loc = new Dictionary<(int, int), int>();
                    var deidColors = new int[dedges.Count];
                    Array.Fill(deidColors, numColor);
                    numColor++;
                    for (int jj = 0; jj < dedges.Count; jj++)
                    {
                        int d = dedges[jj];
                        colors[d] = 0;
                        int v1 = tv[d / 3, d % 3];
                        int v2 = tv[d / 3, (d + 1) % 3];
                        var pt = (v1, v2);
                        if (loc.TryGetValue(pt, out int s))
                        {
                            for (int k = s; k < jj; k++)
                            {
                                int d1 = dedges[k];
                                loc.Remove((tv[d1 / 3, d1 % 3], tv[d1 / 3, (d1 + 1) % 3]));
                                deidColors[k] = numColor;
                            }

                            numColor++;
                        }

                        loc[pt] = jj;
                    }

                    for (int jj = 0; jj < dedges.Count; jj++)
                    {
                        if (deidColors[jj] > 0)
                        {
                            int d = dedges[jj];
                            tv[d / 3, d % 3] = numV + deidColors[jj] - 1;
                        }
                    }
                }

                if (numColor > 1)
                {
                    for (int k = 0; k < numColor - 1; k++)
                    {
                        group.Add(group[vi]);
                    }

                    numV += numColor - 1;
                }
            }
        }
        while (numV != numV0);
    }

    // --- Part B: quads from (±1,±1) diagonals on the manifold-ized vertices ----------------------

    private QuadMesh BuildQuads(int[,] tv, int numV, int[] outGroup)
    {
        // Drop degenerate coarse triangles.
        int cfCount = tv.GetLength(0);
        var quads = new Dictionary<long, (int[] First, int[]? Second)>();
        for (int i = 0; i < cfCount; i++)
        {
            if (tv[i, 0] == tv[i, 1] || tv[i, 1] == tv[i, 2] || tv[i, 0] == tv[i, 2])
            {
                continue;
            }

            for (int j = 0; j < 3; j++)
            {
                Vec2i d = Lattice2D.Rshift90(_cfDiff[i, j], _cfOrient[i, j]);
                if (System.Math.Abs(d.X) != 1 || System.Math.Abs(d.Y) != 1)
                {
                    continue;
                }

                int v1 = tv[i, j];
                int v2 = tv[i, (j + 1) % 3];
                int v3 = tv[i, (j + 2) % 3];
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

        // Output vertex positions = centroid of the coarse vertex's original vertices.
        var accum = new Vector3[numV];
        var count = new int[numV];
        var coarseAccum = new Vector3[_coarseVertexCount];
        var coarseCount = new int[_coarseVertexCount];
        for (int v = 0; v < _positions.Length; v++)
        {
            coarseAccum[_coarseOf[v]] += _positions[v];
            coarseCount[_coarseOf[v]]++;
        }

        var outPos = new Vector3[numV];
        for (int i = 0; i < numV; i++)
        {
            int g = outGroup[i];
            outPos[i] = coarseCount[g] > 0 ? coarseAccum[g] / coarseCount[g] : Vector3.Zero;
        }

        var faces = new List<int[]>();
        foreach ((int[] first, int[]? second) in quads.Values)
        {
            if (second is not null && first[2] != second[2])
            {
                faces.Add(new[] { first[1], first[2], first[0], second[2] });
            }
        }

        return Compact(outPos, faces);
    }

    private static QuadMesh Compact(Vector3[] outPos, List<int[]> faces)
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
}

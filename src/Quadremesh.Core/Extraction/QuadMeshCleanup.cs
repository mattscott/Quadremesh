using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// Post-extraction quad-mesh cleanup — a port of QuadriFlow's <c>Parametrizer::FixValence</c>
/// (parametrizer-mesh.cpp) plus the quad half-edge build <c>compute_direct_graph_quad</c> (dedge.cpp).
/// Four passes over the all-quad output: remove valence-2 doublets, split pinched (multi-fan)
/// vertices, decrease valence &gt; 5, and drop now-unreferenced vertices. Operates purely on quad
/// topology + positions. Reimplemented clean-room (QuadriFlow is MIT, read for understanding only).
/// </summary>
internal static class QuadMeshCleanup
{
    private const int Invalid = -1;

    /// <summary>Runs the valence cleanup on an all-quad mesh, returning the cleaned mesh.</summary>
    public static QuadMesh FixValence(QuadMesh mesh)
    {
        var o = new List<Vector3>(mesh.Positions);
        var f = new List<int[]>(mesh.FaceCount);
        for (int i = 0; i < mesh.FaceCount; i++)
        {
            ReadOnlySpan<int> face = mesh.Face(i);
            if (face.Length != 4)
            {
                return mesh; // only defined for pure-quad input
            }

            f.Add(new[] { face[0], face[1], face[2], face[3] });
        }

        RemoveValence2(o, f);
        SplitPinchedVertices(o, f);
        DecreaseValence(o, f);
        return RemoveZeroValence(o, f);
    }

    // --- quad dedge graph (compute_direct_graph_quad) ---------------------------------------------

    private static int DedgePrev(int e) => (e / 4 * 4) + ((e + 3) % 4);

    private static int DedgeNext(int e) => (e / 4 * 4) + ((e + 1) % 4);

    private static (int[] V2E, int[] E2E, bool[] NonManifold, bool[] Boundary) BuildGraph(
        List<Vector3> o, List<int[]> f)
    {
        int vCount = o.Count;
        var v2e = new int[vCount];
        Array.Fill(v2e, Invalid);

        // tmpTarget[dedge] = the dedge's far vertex; tmpNext = linked list of dedges from same origin.
        int deCount = f.Count * 4;
        var tmpTarget = new int[deCount];
        var tmpNext = new int[deCount];
        Array.Fill(tmpNext, Invalid);

        for (int face = 0; face < f.Count; face++)
        {
            for (int i = 0; i < 4; i++)
            {
                int cur = f[face][i];
                int next = f[face][(i + 1) % 4];
                int eid = (face * 4) + i;
                if (cur == next)
                {
                    continue;
                }

                tmpTarget[eid] = next;
                if (v2e[cur] == Invalid)
                {
                    v2e[cur] = eid;
                }
                else
                {
                    int idx = v2e[cur];
                    while (tmpNext[idx] != Invalid)
                    {
                        idx = tmpNext[idx];
                    }

                    tmpNext[idx] = eid;
                }
            }
        }

        var nonManifold = new bool[vCount];
        var e2e = new int[deCount];
        Array.Fill(e2e, Invalid);

        for (int face = 0; face < f.Count; face++)
        {
            for (int i = 0; i < 4; i++)
            {
                int cur = f[face][i];
                int next = f[face][(i + 1) % 4];
                int eidCur = (face * 4) + i;
                if (cur == next)
                {
                    continue;
                }

                int it = v2e[next];
                int eidOpp = Invalid;
                while (it != Invalid)
                {
                    if (tmpTarget[it] == cur)
                    {
                        if (eidOpp == Invalid)
                        {
                            eidOpp = it;
                        }
                        else
                        {
                            nonManifold[cur] = true;
                            nonManifold[next] = true;
                            eidOpp = Invalid;
                            break;
                        }
                    }

                    it = tmpNext[it];
                }

                if (eidOpp != Invalid && eidCur < eidOpp)
                {
                    e2e[eidCur] = eidOpp;
                    e2e[eidOpp] = eidCur;
                }
            }
        }

        var boundary = new bool[vCount];
        for (int i = 0; i < vCount; i++)
        {
            int edge = v2e[i];
            if (edge == Invalid)
            {
                continue; // isolated
            }

            if (nonManifold[i])
            {
                v2e[i] = Invalid;
                continue;
            }

            int start = edge;
            int v2eVal = int.MaxValue;
            do
            {
                v2eVal = System.Math.Min(v2eVal, edge);
                int prevEdge = e2e[DedgePrev(edge)];
                if (prevEdge == Invalid)
                {
                    v2eVal = edge;
                    boundary[i] = true;
                    break;
                }

                edge = prevEdge;
            }
            while (edge != start);

            v2e[i] = v2eVal;
        }

        return (v2e, e2e, nonManifold, boundary);
    }

    // --- pass 1: remove valence-2 doublets --------------------------------------------------------

    private static void RemoveValence2(List<Vector3> o, List<int[]> f)
    {
        while (true)
        {
            (int[] v2e, int[] e2e, _, _) = BuildGraph(o, f);
            var marks = new bool[o.Count];
            var erasedF = new bool[f.Count];
            bool update = false;

            for (int i = 0; i < v2e.Length; i++)
            {
                int deid0 = v2e[i];
                if (marks[i] || deid0 == Invalid)
                {
                    continue;
                }

                var dedges = new List<int>();
                int deid = deid0;
                do
                {
                    dedges.Add(deid);
                    int deid1 = DedgePrev(deid);
                    deid = e2e[deid1];
                }
                while (deid != deid0 && deid != Invalid);

                if (dedges.Count != 2)
                {
                    continue;
                }

                int v1 = f[dedges[0] / 4][(dedges[0] + 1) % 4];
                int v2 = f[dedges[0] / 4][(dedges[0] + 2) % 4];
                int v3 = f[dedges[1] / 4][(dedges[1] + 1) % 4];
                int v4 = f[dedges[1] / 4][(dedges[1] + 2) % 4];
                if (marks[v1] || marks[v2] || marks[v3] || marks[v4])
                {
                    continue;
                }

                marks[v1] = marks[v2] = marks[v3] = marks[v4] = true;
                if (v1 == v2 || v1 == v3 || v1 == v4 || v2 == v3 || v2 == v4 || v3 == v4)
                {
                    erasedF[dedges[0] / 4] = true;
                }
                else
                {
                    f[dedges[0] / 4] = new[] { v1, v2, v3, v4 };
                }

                erasedF[dedges[1] / 4] = true;
                update = true;
            }

            if (!update)
            {
                break;
            }

            int top = 0;
            for (int i = 0; i < erasedF.Length; i++)
            {
                if (!erasedF[i])
                {
                    f[top++] = f[i];
                }
            }

            f.RemoveRange(top, f.Count - top);
        }
    }

    // --- pass 2: split pinched (multi-fan) vertices -----------------------------------------------

    private static void SplitPinchedVertices(List<Vector3> o, List<int[]> f)
    {
        (int[] _, int[] e2e, _, _) = BuildGraph(o, f);
        var vDedges = new List<int>[o.Count];
        for (int i = 0; i < o.Count; i++)
        {
            vDedges[i] = new List<int>();
        }

        for (int i = 0; i < f.Count; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                vDedges[f[i][j]].Add((i * 4) + j);
            }
        }

        int top = o.Count;
        int originalCount = o.Count;
        for (int i = 0; i < originalCount; i++)
        {
            var groups = new Dictionary<int, int>();
            int groupId = 0;
            foreach (int startDeid in vDedges[i])
            {
                if (groups.ContainsKey(startDeid))
                {
                    continue;
                }

                int deid0 = startDeid;
                int deid = startDeid;
                do
                {
                    groups[deid] = groupId;
                    deid = DedgePrev(deid);
                    deid = e2e[deid];
                }
                while (deid != deid0 && deid != Invalid);

                if (deid == Invalid)
                {
                    deid = deid0;
                    while (e2e[deid] != Invalid)
                    {
                        deid = e2e[deid];
                        deid = DedgeNext(deid);
                        groups[deid] = groupId;
                    }
                }

                groupId += 1;
            }

            if (groupId > 1)
            {
                foreach ((int dedge, int g) in groups)
                {
                    if (g >= 1)
                    {
                        f[dedge / 4][dedge % 4] = top - 1 + g;
                    }
                }

                for (int j = 1; j < groupId; j++)
                {
                    o.Add(o[i]);
                }

                top = o.Count;
            }
        }
    }

    // --- pass 3: decrease valence > 5 -------------------------------------------------------------

    private static void DecreaseValence(List<Vector3> o, List<int[]> f)
    {
        while (true)
        {
            (int[] v2e, int[] e2e, _, _) = BuildGraph(o, f);
            var marks = new bool[o.Count];
            var valences = new int[o.Count];
            for (int i = 0; i < v2e.Length; i++)
            {
                int deid0 = v2e[i];
                if (deid0 == Invalid)
                {
                    continue;
                }

                int deid = deid0;
                int count = 0;
                do
                {
                    count += 1;
                    int deid1 = e2e[deid];
                    if (deid1 == Invalid)
                    {
                        count += 1;
                        break;
                    }

                    deid = DedgeNext(deid1);
                }
                while (deid != deid0 && deid != Invalid);

                if (deid == Invalid)
                {
                    count += 1;
                }

                valences[i] = count;
            }

            // Max-heap on valence: split the highest-valence vertices first.
            var queue = new PriorityQueue<int, int>();
            for (int i = 0; i < valences.Length; i++)
            {
                if (valences[i] > 5)
                {
                    queue.Enqueue(i, -valences[i]);
                }
            }

            bool update = false;
            while (queue.Count > 0)
            {
                int vertex = queue.Dequeue();
                if (marks[vertex])
                {
                    continue;
                }

                int valence = valences[vertex];
                int deid0 = v2e[vertex];
                if (deid0 == Invalid)
                {
                    continue;
                }

                var loopVertices = new List<int>();
                var loopDedges = new List<int>();
                int deid = deid0;
                bool marked = false;
                do
                {
                    int v = f[deid / 4][(deid + 1) % 4];
                    loopDedges.Add(deid);
                    loopVertices.Add(v);
                    if (marks[v])
                    {
                        marked = true;
                    }

                    int deid1 = e2e[deid];
                    if (deid1 == Invalid)
                    {
                        break;
                    }

                    deid = DedgeNext(deid1);
                }
                while (deid != deid0 && deid != Invalid);

                if (marked)
                {
                    continue;
                }

                if (deid != Invalid)
                {
                    int step = (valence + 1) / 2;
                    (int A, int B) minVal = (int.MaxValue, int.MaxValue);
                    int splitIdx = -1;
                    for (int i = 0; i < loopVertices.Count; i++)
                    {
                        if (i + step >= loopVertices.Count)
                        {
                            continue;
                        }

                        int a = valences[loopVertices[i]];
                        int b = valences[loopVertices[i + step]];
                        if (a < b)
                        {
                            (a, b) = (b, a);
                        }

                        if (a < minVal.A || (a == minVal.A && b < minVal.B))
                        {
                            minVal = (a, b);
                            splitIdx = i + 1;
                        }
                    }

                    if (minVal.A >= valence)
                    {
                        continue;
                    }

                    update = true;
                    for (int id = splitIdx; id < splitIdx + step; id++)
                    {
                        f[loopDedges[id] / 4][loopDedges[id] % 4] = o.Count;
                    }

                    int n = loopVertices.Count;
                    f.Add(new[]
                    {
                        o.Count,
                        loopVertices[(splitIdx + n - 1) % n],
                        vertex,
                        loopVertices[(splitIdx + step - 1 + n) % n],
                    });
                }
                else
                {
                    for (int id = loopVertices.Count / 2; id < loopVertices.Count; id++)
                    {
                        f[loopDedges[id] / 4][loopDedges[id] % 4] = o.Count;
                    }

                    update = true;
                }

                marks[vertex] = true;
                for (int i = 0; i < loopVertices.Count; i++)
                {
                    marks[loopVertices[i]] = true;
                }

                o.Add(o[vertex]);
            }

            if (!update)
            {
                break;
            }
        }
    }

    // --- pass 4: drop now-unreferenced vertices ---------------------------------------------------

    private static QuadMesh RemoveZeroValence(List<Vector3> o, List<int[]> f)
    {
        var used = new bool[o.Count];
        foreach (int[] face in f)
        {
            for (int j = 0; j < 4; j++)
            {
                used[face[j]] = true;
            }
        }

        var remap = new int[o.Count];
        var positions = new List<Vector3>(o.Count);
        for (int i = 0; i < o.Count; i++)
        {
            if (!used[i])
            {
                continue;
            }

            remap[i] = positions.Count;
            positions.Add(o[i]);
        }

        var faces = new List<int[]>(f.Count);
        foreach (int[] face in f)
        {
            faces.Add(new[] { remap[face[0]], remap[face[1]], remap[face[2]], remap[face[3]] });
        }

        return new QuadMesh(positions.ToArray(), faces);
    }
}

using Quadremesh.Core.Math;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// Port of QuadriFlow's edge-graph flip machinery (hierarchy.cpp): collapse all zero-offset edges into
/// a coarse edge graph (<c>DownsampleEdgeGraph</c>), remove the integer-parametrization foldovers
/// (negative-area faces) by shrinking edge rings (<c>FixFlip</c>), then push the corrected offsets back
/// down to the fine level (<c>PropagateEdge</c>). Operates purely on the edge graph — face→edge ids,
/// face→edge orients, and per-edge integer offsets — with no geometry. This is the stage that aligns
/// the triangulation with the integer lattice so every quad cell yields a clean (±1,±1) diagonal,
/// killing the holes left by raw collapse. Reimplemented clean-room (QuadriFlow is MIT).
/// </summary>
internal sealed class EdgeGraphHierarchy
{
    // Per-level arrays, mirroring the reference's mFQ / mF2E / mE2F / mEdgeDiff / mAllowChanges / mSing.
    private readonly List<List<int[]>> _fq = new();        // [level][face] = 3 edge orients
    private readonly List<List<int[]>> _f2e = new();       // [level][face] = 3 edge ids
    private readonly List<List<int[]>> _e2f = new();       // [level][edge] = (≤2 incident faces)
    private readonly List<List<Vec2i>> _edgeDiff = new();  // [level][edge] = integer offset
    private readonly List<List<int>> _allow = new();       // [level][edge*2+dim] = 0/1/2
    private readonly List<List<int>> _sing = new();        // [level] = fixed (singular) faces

    private readonly List<int[]> _toUpperEdges = new();    // [level][edge] -> coarse edge id (or <0 code)
    private readonly List<int[]> _toUpperOrients = new();  // [level][edge] -> relative rotation
    private readonly List<int[]> _toUpperFaces = new();    // [level][face] -> coarse face id (or -1)

    /// <summary>
    /// Runs the full flip-fix on a fine edge graph in place. <paramref name="fq"/>/<paramref name="f2e"/>
    /// are per-face triples; <paramref name="edgeDiff"/> is per-edge. All arrays are updated to the
    /// foldover-free assignment. <paramref name="allow"/> is per-edge-component change permission (all 1
    /// when there are no sharp constraints).
    /// </summary>
    public static void Run(int[][] fq, int[][] f2e, Vec2i[] edgeDiff, int[] allow, int maxShrinkLen = 2)
    {
        var h = new EdgeGraphHierarchy();
        h.DownsampleEdgeGraph(
            new List<int[]>(fq), new List<int[]>(f2e), new List<Vec2i>(edgeDiff), new List<int>(allow), -1);
        h.FixFlip(maxShrinkLen);

        // UpdateGraphValue: level 0 now holds the corrected graph; copy it back out.
        List<int[]> nfq = h._fq[0];
        List<Vec2i> ndiff = h._edgeDiff[0];
        for (int i = 0; i < fq.Length; i++)
        {
            fq[i][0] = nfq[i][0];
            fq[i][1] = nfq[i][1];
            fq[i][2] = nfq[i][2];
        }

        for (int e = 0; e < edgeDiff.Length; e++)
        {
            edgeDiff[e] = ndiff[e];
        }
    }

    /// <summary>
    /// SAT flip-fixer (QuadriFlow's <c>FixFlipSat</c>): like <see cref="Run"/> but clears the residual
    /// foldovers <see cref="FixFlip"/> cannot, by collapsing to the coarse edge graph (where zero-offset
    /// edges are gone, so the SAT problems stay small) and solving a per-cluster integer feasibility
    /// problem there, then propagating the corrected offsets back down. Returns the foldover count left.
    /// </summary>
    public static int RunSat(int[][] fq, int[][] f2e, Vec2i[] edgeDiff, int[] allow)
    {
        var h = new EdgeGraphHierarchy();
        h.DownsampleEdgeGraph(
            new List<int[]>(fq), new List<int[]>(f2e), new List<Vec2i>(edgeDiff), new List<int>(allow), -1);

        int l = h._f2e.Count - 1;
        int[][] cfq = h._fq[l].ToArray();
        int[][] cf2e = h._f2e[l].ToArray();
        Vec2i[] cdiff = h._edgeDiff[l].ToArray();
        int[] callow = h._allow[l].ToArray();

        int remaining = FlipSat.Run(cfq, cf2e, cdiff, callow);

        // FlipSat only changes offsets (not orients); write them back and push down to level 0.
        for (int e = 0; e < cdiff.Length; e++)
        {
            h._edgeDiff[l][e] = cdiff[e];
        }

        h.PropagateEdge();

        List<int[]> nfq = h._fq[0];
        List<Vec2i> ndiff = h._edgeDiff[0];
        for (int i = 0; i < fq.Length; i++)
        {
            fq[i][0] = nfq[i][0];
            fq[i][1] = nfq[i][1];
            fq[i][2] = nfq[i][2];
        }

        for (int e = 0; e < edgeDiff.Length; e++)
        {
            edgeDiff[e] = ndiff[e];
        }

        return remaining;
    }

    private void DownsampleEdgeGraph(
        List<int[]> fq, List<int[]> f2e, List<Vec2i> edgeDiff, List<int> allow, int level)
    {
        var e2f = new List<int[]>(edgeDiff.Count);
        for (int i = 0; i < edgeDiff.Count; i++)
        {
            e2f.Add(new[] { -1, -1 });
        }

        for (int i = 0; i < f2e.Count; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int e = f2e[i][j];
                if (e2f[e][0] == -1)
                {
                    e2f[e][0] = i;
                }
                else
                {
                    e2f[e][1] = i;
                }
            }
        }

        int levels = level == -1 ? 100 : level;

        var sing0 = new List<int>();
        for (int i = 0; i < fq.Count; i++)
        {
            var diff = new Vec2i(0, 0);
            for (int j = 0; j < 3; j++)
            {
                diff += Lattice2D.Rshift90(edgeDiff[f2e[i][j]], fq[i][j]);
            }

            if (!diff.Equals(new Vec2i(0, 0)))
            {
                sing0.Add(i);
            }
        }

        _fq.Add(fq);
        _f2e.Add(f2e);
        _e2f.Add(e2f);
        _edgeDiff.Add(edgeDiff);
        _allow.Add(allow);
        _sing.Add(sing0);

        for (int l = 0; l < levels - 1; l++)
        {
            List<int[]> lFQ = _fq[l];
            List<int[]> lF2E = _f2e[l];
            List<int[]> lE2F = _e2f[l];
            List<Vec2i> lEdgeDiff = _edgeDiff[l];
            List<int> lAllow = _allow[l];
            List<int> lSing = _sing[l];

            var fixedFaces = new int[lF2E.Count];
            foreach (int s in lSing)
            {
                fixedFaces[s] = 1;
            }

            var toUpper = new int[lE2F.Count];
            var toUpperOrients = new int[lE2F.Count];
            Array.Fill(toUpper, -1);

            var nFQ = new List<int[]>();
            var nE2F = new List<int[]>();
            var nF2E = new List<int[]>();
            var nAllow = new List<int>();
            var nEdgeDiff = new List<Vec2i>();
            var nSing = new List<int>();

            // Mark collapsible (zero-diff, away from fixed faces) edges, and protect their neighbours.
            for (int i = 0; i < lE2F.Count; i++)
            {
                if (!lEdgeDiff[i].Equals(new Vec2i(0, 0)))
                {
                    continue;
                }

                if ((lE2F[i][0] >= 0 && fixedFaces[lE2F[i][0]] != 0) ||
                    (lE2F[i][1] >= 0 && fixedFaces[lE2F[i][1]] != 0))
                {
                    continue;
                }

                for (int j = 0; j < 2; j++)
                {
                    int f = lE2F[i][j];
                    if (f < 0)
                    {
                        continue;
                    }

                    for (int k = 0; k < 3; k++)
                    {
                        int neighborE = lF2E[f][k];
                        for (int m = 0; m < 2; m++)
                        {
                            int neighborF = lE2F[neighborE][m];
                            if (neighborF < 0)
                            {
                                continue;
                            }

                            if (fixedFaces[neighborF] == 0)
                            {
                                fixedFaces[neighborF] = 1;
                            }
                        }
                    }
                }

                if (lE2F[i][0] >= 0)
                {
                    fixedFaces[lE2F[i][0]] = 2;
                }

                if (lE2F[i][1] >= 0)
                {
                    fixedFaces[lE2F[i][1]] = 2;
                }

                toUpper[i] = -2;
            }

            for (int i = 0; i < lE2F.Count; i++)
            {
                if (toUpper[i] == -2)
                {
                    continue;
                }

                if ((lE2F[i][0] < 0 || fixedFaces[lE2F[i][0]] == 2) &&
                    (lE2F[i][1] < 0 || fixedFaces[lE2F[i][1]] == 2))
                {
                    toUpper[i] = -3;
                }
            }

            int numE = 0;
            for (int i = 0; i < toUpper.Length; i++)
            {
                if (toUpper[i] != -1)
                {
                    continue;
                }

                if ((lE2F[i][0] < 0 || fixedFaces[lE2F[i][0]] < 2) &&
                    (lE2F[i][1] < 0 || fixedFaces[lE2F[i][1]] < 2))
                {
                    nE2F.Add(new[] { lE2F[i][0], lE2F[i][1] });
                    toUpperOrients[i] = 0;
                    toUpper[i] = numE++;
                    continue;
                }

                int f0 = (lE2F[i][1] < 0 || fixedFaces[lE2F[i][0]] < 2) ? lE2F[i][0] : lE2F[i][1];
                int e = i;
                int f = f0;
                var paths = new List<(int Edge, int Orient)> { (i, 0) };
                while (true)
                {
                    if (lE2F[e][0] == f)
                    {
                        f = lE2F[e][1];
                    }
                    else if (lE2F[e][1] == f)
                    {
                        f = lE2F[e][0];
                    }

                    if (f < 0 || fixedFaces[f] < 2)
                    {
                        for (int j = 0; j < paths.Count; j++)
                        {
                            (int pEdge, int pOrient) = paths[j];
                            toUpper[pEdge] = numE;
                            int orient = pOrient;
                            if (j > 0)
                            {
                                orient = (orient + toUpperOrients[paths[j - 1].Edge]) % 4;
                            }

                            toUpperOrients[pEdge] = orient;
                        }

                        nE2F.Add(new[] { f0, f });
                        numE += 1;
                        break;
                    }

                    int ind0 = -1, ind1 = -1;
                    int e0 = e;
                    for (int j = 0; j < 3; j++)
                    {
                        if (lF2E[f][j] == e)
                        {
                            ind0 = j;
                            break;
                        }
                    }

                    for (int j = 0; j < 3; j++)
                    {
                        int e1 = lF2E[f][j];
                        if (e1 != e && toUpper[e1] != -2)
                        {
                            e = e1;
                            ind1 = j;
                            break;
                        }
                    }

                    if (ind1 != -1)
                    {
                        paths.Add((e, (lFQ[f][ind1] - lFQ[f][ind0] + 6) % 4));
                    }
                    else
                    {
                        if (!lEdgeDiff[e].Equals(new Vec2i(0, 0)))
                        {
                            throw new InvalidOperationException(
                                $"DownsampleEdgeGraph: unsatisfied collapse at face {f} ({e0}->{e}).");
                        }

                        foreach ((int pEdge, _) in paths)
                        {
                            toUpper[pEdge] = numE;
                            toUpperOrients[pEdge] = 0;
                        }

                        numE += 1;
                        nE2F.Add(new[] { f0, f0 });
                        break;
                    }
                }
            }

            for (int i = 0; i < numE; i++)
            {
                nEdgeDiff.Add(new Vec2i(0, 0));
                nAllow.Add(1);
                nAllow.Add(1);
            }

            for (int i = 0; i < toUpper.Length; i++)
            {
                if (toUpper[i] >= 0 && toUpperOrients[i] == 0)
                {
                    nEdgeDiff[toUpper[i]] = lEdgeDiff[i];
                }

                if (toUpper[i] >= 0)
                {
                    int dimension = toUpperOrients[i] % 2;
                    if (lAllow[(i * 2) + dimension] == 0)
                    {
                        nAllow[toUpper[i] * 2] = 0;
                    }
                    else if (lAllow[(i * 2) + dimension] == 2)
                    {
                        nAllow[toUpper[i] * 2] = 2;
                    }

                    if (lAllow[(i * 2) + 1 - dimension] == 0)
                    {
                        nAllow[(toUpper[i] * 2) + 1] = 0;
                    }
                    else if (lAllow[(i * 2) + 1 - dimension] == 2)
                    {
                        nAllow[(toUpper[i] * 2) + 1] = 2;
                    }
                }
            }

            var upperFace = new int[lF2E.Count];
            Array.Fill(upperFace, -1);
            for (int i = 0; i < lF2E.Count; i++)
            {
                var eid = new int[3];
                for (int j = 0; j < 3; j++)
                {
                    eid[j] = toUpper[lF2E[i][j]];
                }

                if (eid[0] >= 0 && eid[1] >= 0 && eid[2] >= 0)
                {
                    var eidOrient = new int[3];
                    for (int j = 0; j < 3; j++)
                    {
                        eidOrient[j] = (lFQ[i][j] + 4 - toUpperOrients[lF2E[i][j]]) % 4;
                    }

                    upperFace[i] = nF2E.Count;
                    nF2E.Add(eid);
                    nFQ.Add(eidOrient);
                }
            }

            for (int i = 0; i < nE2F.Count; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    if (nE2F[i][j] >= 0)
                    {
                        nE2F[i][j] = upperFace[nE2F[i][j]];
                    }
                }
            }

            foreach (int s in lSing)
            {
                if (upperFace[s] >= 0)
                {
                    nSing.Add(upperFace[s]);
                }
            }

            _toUpperEdges.Add(toUpper);
            _toUpperOrients.Add(toUpperOrients);
            _toUpperFaces.Add(upperFace);

            _fq.Add(nFQ);
            _f2e.Add(nF2E);
            _e2f.Add(nE2F);
            _edgeDiff.Add(nEdgeDiff);
            _allow.Add(nAllow);
            _sing.Add(nSing);

            if (nEdgeDiff.Count == lEdgeDiff.Count)
            {
                break;
            }
        }
    }

    private void FixFlip(int maxShrinkLen = 2)
    {
        int l = _f2e.Count - 1;
        List<int[]> f2e = _f2e[l];
        List<int[]> e2f = _e2f[l];
        List<int[]> fq = _fq[l];
        List<Vec2i> edgeDiff = _edgeDiff[l];
        List<int> allow = _allow[l];

        // Build the dedge twin map E2E from E2F / F2E.
        var e2e = new int[f2e.Count * 3];
        Array.Fill(e2e, -1);
        for (int i = 0; i < e2f.Count; i++)
        {
            int v1 = e2f[i][0];
            int v2 = e2f[i][1];
            int t1 = 0;
            int t2 = 2;
            if (v1 != -1)
            {
                while (f2e[v1][t1] != i)
                {
                    t1 += 1;
                }
            }

            if (v2 != -1)
            {
                while (f2e[v2][t2] != i)
                {
                    t2 -= 1;
                }
            }

            t1 += v1 * 3;
            t2 += v2 * 3;
            if (v1 != -1)
            {
                e2e[t1] = v2 == -1 ? -1 : t2;
            }

            if (v2 != -1)
            {
                e2e[t2] = v1 == -1 ? -1 : t1;
            }
        }

        int Area(int f)
        {
            Vec2i diff1 = Lattice2D.Rshift90(edgeDiff[f2e[f][0]], fq[f][0]);
            Vec2i diff2 = Lattice2D.Rshift90(edgeDiff[f2e[f][1]], fq[f][1]);
            return (diff1.X * diff2.Y) - (diff1.Y * diff2.X);
        }

        bool CheckShrink(int deid, int allowedEdgeLength)
        {
            if (deid == -1)
            {
                return false;
            }

            var correspondingEdges = new List<int>();
            var correspondingFaces = new List<int>();
            var correspondingDiff = new List<Vec2i>();
            int deid0 = deid;
            while (deid != -1)
            {
                deid = (deid / 3 * 3) + ((deid + 2) % 3);
                if (e2e[deid] == -1)
                {
                    break;
                }

                deid = e2e[deid];
                if (deid == deid0)
                {
                    break;
                }
            }

            Vec2i diff = edgeDiff[f2e[deid / 3][deid % 3]];
            do
            {
                correspondingDiff.Add(diff);
                correspondingEdges.Add(deid);
                correspondingFaces.Add(deid / 3);

                deid = e2e[deid];
                if (deid == -1)
                {
                    return false;
                }

                diff = -Lattice2D.Rshift90(diff, fq[deid / 3][deid % 3]);
                deid = (deid / 3 * 3) + ((deid + 1) % 3);
                diff = Lattice2D.Rshift90(diff, (4 - fq[deid / 3][deid % 3]) % 4);
            }
            while (deid != correspondingEdges[0]);

            if (deid != -1 && !diff.Equals(correspondingDiff[0]))
            {
                return false;
            }

            var original = new Dictionary<int, Vec2i>();
            var shrunk = new Dictionary<int, Vec2i>();
            for (int i = 0; i < correspondingDiff.Count; i++)
            {
                int de = correspondingEdges[i];
                int eid = f2e[de / 3][de % 3];
                original[eid] = edgeDiff[eid];
                shrunk[eid] = edgeDiff[eid];
            }

            for (int i = 0; i < correspondingDiff.Count; i++)
            {
                int de = correspondingEdges[i];
                int eid = f2e[de / 3][de % 3];
                for (int j = 0; j < 2; j++)
                {
                    if (correspondingDiff[i][j] != 0 && allow[(eid * 2) + j] == 0)
                    {
                        return false;
                    }
                }

                Vec2i res = shrunk[eid] - correspondingDiff[i];
                shrunk[eid] = res;
                if (System.Math.Abs(res.X) > allowedEdgeLength || System.Math.Abs(res.Y) > allowedEdgeLength)
                {
                    return false;
                }

                if ((System.Math.Abs(res.X) > 1 && System.Math.Abs(res.Y) != 0) ||
                    (System.Math.Abs(res.Y) > 1 && System.Math.Abs(res.X) != 0))
                {
                    return false;
                }
            }

            int prevArea = 0;
            foreach (int f in correspondingFaces)
            {
                if (Area(f) < 0)
                {
                    prevArea += 1;
                }
            }

            foreach ((int eid, Vec2i v) in shrunk)
            {
                edgeDiff[eid] = v;
            }

            int currentArea = 0;
            foreach (int f in correspondingFaces)
            {
                if (Area(f) < 0)
                {
                    currentArea += 1;
                }
            }

            if (currentArea < prevArea)
            {
                return true;
            }

            foreach ((int eid, Vec2i v) in original)
            {
                edgeDiff[eid] = v;
            }

            return false;
        }

        var flipped = new Queue<int>();
        for (int i = 0; i < f2e.Count; i++)
        {
            if (Area(i) < 0)
            {
                flipped.Enqueue(i);
            }
        }

        bool update = false;
        int maxLen = 1;
        while (!update && maxLen <= maxShrinkLen)
        {
            while (flipped.Count > 0)
            {
                int f = flipped.Peek();
                if (Area(f) >= 0)
                {
                    flipped.Dequeue();
                    continue;
                }

                for (int i = 0; i < 3; i++)
                {
                    if (CheckShrink((f * 3) + i, maxLen) || CheckShrink(e2e[(f * 3) + i], maxLen))
                    {
                        update = true;
                        break;
                    }
                }

                flipped.Dequeue();
            }

            maxLen += 1;
        }

        if (update)
        {
            var sub = new EdgeGraphHierarchy();
            sub.DownsampleEdgeGraph(
                new List<int[]>(_fq[l]), new List<int[]>(_f2e[l]), new List<Vec2i>(_edgeDiff[l]),
                new List<int>(_allow[l]), -1);
            sub.FixFlip(maxShrinkLen);

            // UpdateGraphValue into this level's coarsest arrays.
            List<int[]> subFQ = sub._fq[0];
            List<Vec2i> subDiff = sub._edgeDiff[0];
            for (int i = 0; i < fq.Count; i++)
            {
                fq[i][0] = subFQ[i][0];
                fq[i][1] = subFQ[i][1];
                fq[i][2] = subFQ[i][2];
            }

            for (int e = 0; e < edgeDiff.Count; e++)
            {
                edgeDiff[e] = subDiff[e];
            }
        }

        PropagateEdge();
    }

    private void PropagateEdge()
    {
        for (int level = _toUpperEdges.Count; level > 0; level--)
        {
            List<Vec2i> edgeDiff = _edgeDiff[level];
            List<Vec2i> nEdgeDiff = _edgeDiff[level - 1];
            List<int[]> fq = _fq[level];
            List<int[]> nFQ = _fq[level - 1];
            List<int[]> f2e = _f2e[level - 1];
            int[] toUpper = _toUpperEdges[level - 1];
            int[] toUpperFace = _toUpperFaces[level - 1];
            int[] toUpperOrients = _toUpperOrients[level - 1];

            for (int i = 0; i < toUpper.Length; i++)
            {
                if (toUpper[i] >= 0)
                {
                    int orient = (4 - toUpperOrients[i]) % 4;
                    nEdgeDiff[i] = Lattice2D.Rshift90(edgeDiff[toUpper[i]], orient);
                }
                else
                {
                    nEdgeDiff[i] = new Vec2i(0, 0);
                }
            }

            for (int i = 0; i < toUpperFace.Length; i++)
            {
                if (toUpperFace[i] == -1)
                {
                    continue;
                }

                int[] eidOrient = fq[toUpperFace[i]];
                for (int j = 0; j < 3; j++)
                {
                    nFQ[i][j] = (eidOrient[j] + toUpperOrients[f2e[i][j]]) % 4;
                }
            }
        }
    }
}

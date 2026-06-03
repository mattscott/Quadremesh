namespace Quadremesh.Core.Extraction;

/// <summary>
/// Port of QuadriFlow's <c>Hierarchy::FixFlipSat</c> (hierarchy.cpp:640) + its <c>Parametrizer::FixFlipSat</c>
/// driver (parametrizer-flip.cpp:175), operating on the finest edge graph. After FixFlip leaves a few
/// stubborn parametrization foldovers (negative-area lattice cells that a single edge-ring shrink cannot
/// clear — they need a *coordinated* multi-edge change), this marks the region around each foldover,
/// groups the changeable edges, and asks <see cref="LocalSat"/> for an assignment that makes every
/// covered face's divergence zero and area non-negative. It clears the foldovers FixFlip cannot, which
/// is what otherwise leaves triangular holes in extraction. Reimplemented clean-room (QuadriFlow is MIT);
/// the original calls minisat, this uses the in-process DPLL solver instead.
/// </summary>
internal static class FlipSat
{
    /// <summary>
    /// Skip groups with more changeable edges than this (keeps the local SAT tractable). Overridable via
    /// QUADREMESH_FLIPSAT_MAXGROUP for the M0 de-risk experiment (set huge to attempt every group).
    /// </summary>
    private static readonly int MaxGroupEdges =
        int.TryParse(Environment.GetEnvironmentVariable("QUADREMESH_FLIPSAT_MAXGROUP"), out int g) ? g : 60;

    /// <summary>DPLL conflict budget per group; overridable via QUADREMESH_FLIPSAT_BUDGET.</summary>
    private static readonly long ConflictBudget =
        long.TryParse(Environment.GetEnvironmentVariable("QUADREMESH_FLIPSAT_BUDGET"), out long b) ? b : 200_000;

    // Diagnostics for the foldover/hole investigation.
    internal static int LastGroups { get; private set; }

    internal static int LastSolvedSat { get; private set; }

    internal static int LastUnsat { get; private set; }

    internal static int LastTimeout { get; private set; }

    internal static int LastSkippedLarge { get; private set; }

    internal static int LastMaxGroupEdges { get; private set; }

    /// <summary>
    /// Driver: escalate the marking threshold 1→4, running one SAT pass each, until no foldovers remain.
    /// Mutates <paramref name="fq"/>/<paramref name="edgeDiff"/> in place. Returns the foldover count left.
    /// </summary>
    public static int Run(int[][] fq, int[][] f2e, Vec2i[] edgeDiff, int[] allow)
    {
        int nflip = CountFlips(fq, f2e, edgeDiff);
        for (int threshold = 1; threshold <= 4 && nflip > 0; threshold++)
        {
            nflip = RunSat(fq, f2e, edgeDiff, allow, threshold);
        }

        return nflip;
    }

    private static int CountFlips(int[][] fq, int[][] f2e, Vec2i[] edgeDiff)
    {
        int n = 0;
        for (int f = 0; f < f2e.Length; f++)
        {
            if (Area(fq, f2e, edgeDiff, f) < 0)
            {
                n++;
            }
        }

        return n;
    }

    private static int Area(int[][] fq, int[][] f2e, Vec2i[] edgeDiff, int f)
    {
        Vec2i d1 = Lattice2D.Rshift90(edgeDiff[f2e[f][0]], fq[f][0]);
        Vec2i d2 = Lattice2D.Rshift90(edgeDiff[f2e[f][1]], fq[f][1]);
        return (d1.X * d2.Y) - (d1.Y * d2.X);
    }

    private static int RunSat(int[][] fq, int[][] f2e, Vec2i[] edgeDiff, int[] allow, int threshold)
    {
        int nFaces = f2e.Length;
        int nEdges = edgeDiff.Length;

        // E2F: each edge's (≤2) incident faces, then E2E: dedge twin map.
        var e2f = new int[nEdges][];
        for (int e = 0; e < nEdges; e++)
        {
            e2f[e] = new[] { -1, -1 };
        }

        for (int f = 0; f < nFaces; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                int e = f2e[f][j];
                if (e2f[e][0] == -1)
                {
                    e2f[e][0] = f;
                }
                else
                {
                    e2f[e][1] = f;
                }
            }
        }

        var e2e = new int[nFaces * 3];
        Array.Fill(e2e, -1);
        for (int i = 0; i < nEdges; i++)
        {
            int f1 = e2f[i][0];
            int f2 = e2f[i][1];
            int t1 = 0, t2 = 2;
            if (f1 != -1)
            {
                while (f2e[f1][t1] != i)
                {
                    t1++;
                }
            }

            if (f2 != -1)
            {
                while (f2e[f2][t2] != i)
                {
                    t2--;
                }
            }

            t1 += f1 * 3;
            t2 += f2 * 3;
            if (f1 != -1)
            {
                e2e[t1] = f2 == -1 ? -1 : t2;
            }

            if (f2 != -1)
            {
                e2e[t2] = f1 == -1 ? -1 : t1;
            }
        }

        int Len(int dedge) => System.Math.Abs(edgeDiff[f2e[dedge / 3][dedge % 3]].X) +
                              System.Math.Abs(edgeDiff[f2e[dedge / 3][dedge % 3]].Y);

        // --- Marking BFS: flag dedges in the foldover region, bounded by accumulated edge length. -----
        var mark = new bool[nFaces * 3];
        var q = new LinkedList<(int Dedge, int Depth)>();
        for (int f = 0; f < nFaces; f++)
        {
            if (Area(fq, f2e, edgeDiff, f) < 0)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (!mark[(f * 3) + j])
                    {
                        mark[(f * 3) + j] = true;
                        q.AddLast(((f * 3) + j, 0));
                    }
                }
            }
        }

        while (q.Count > 0)
        {
            (int e0, int depth) = q.First!.Value;
            q.RemoveFirst();

            int e = e0, e1 = -1;
            do
            {
                e1 = e2e[e];
                if (e1 == -1)
                {
                    break;
                }

                if (Len(e1) == 0 && !mark[e1])
                {
                    mark[e1] = true;
                    q.AddFirst((e1, depth));
                }

                e = (e1 / 3 * 3) + ((e1 + 1) % 3);
                mark[e] = true;
            }
            while (e != e0);

            if (e1 == -1)
            {
                do
                {
                    e1 = e2e[e];
                    if (e1 == -1)
                    {
                        break;
                    }

                    if (Len(e1) == 0 && !mark[e1])
                    {
                        mark[e1] = true;
                        q.AddFirst((e1, depth));
                    }

                    e = (e1 / 3 * 3) + ((e1 + 2) % 3);
                    mark[e] = true;
                }
                while (e != e0);
            }

            do
            {
                e1 = e2e[e];
                if (e1 == -1)
                {
                    break;
                }

                int length = Len(e1);
                if (length > 0 && depth + length <= threshold && !mark[e1])
                {
                    mark[e1] = true;
                    q.AddLast((e1, depth + length));
                }

                e = (e1 / 3 * 3) + ((e1 + 1) % 3);
                mark[e] = true;
            }
            while (e != e0);

            if (e1 == -1)
            {
                do
                {
                    e1 = e2e[e];
                    if (e1 == -1)
                    {
                        break;
                    }

                    int length = Len(e1);
                    if (length > 0 && depth + length <= threshold && !mark[e1])
                    {
                        mark[e1] = true;
                        q.AddLast((e1, depth + length));
                    }

                    e = (e1 / 3 * 3) + ((e1 + 2) % 3);
                    mark[e] = true;
                }
                while (e != e0);
            }
        }

        // --- Flexible edges: marked, not a slit, not sharp-locked. ------------------------------------
        var flexible = new bool[nEdges];
        for (int f = 0; f < nFaces; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (mark[(f * 3) + j])
                {
                    flexible[f2e[f][j]] = true;
                }
            }
        }

        for (int i = 0; i < nEdges; i++)
        {
            if (e2f[i][0] == e2f[i][1])
            {
                flexible[i] = false;
            }

            if (allow[i] == 0)
            {
                flexible[i] = false;
            }
        }

        // --- Group connected flexible edges (share a face). ------------------------------------------
        var groups = new int[nEdges];
        Array.Fill(groups, -1);
        int numGroup = 0;
        for (int i = 0; i < nEdges; i++)
        {
            if (groups[i] != -1 || !flexible[i])
            {
                continue;
            }

            var bfs = new Queue<int>();
            bfs.Enqueue(i);
            groups[i] = numGroup;
            while (bfs.Count > 0)
            {
                int e = bfs.Dequeue();
                for (int j = 0; j < 2; j++)
                {
                    int f = e2f[e][j];
                    if (f == -1)
                    {
                        continue;
                    }

                    for (int k = 0; k < 3; k++)
                    {
                        int e1 = f2e[f][k];
                        if (flexible[e1] && groups[e1] == -1)
                        {
                            groups[e1] = numGroup;
                            bfs.Enqueue(e1);
                        }
                    }
                }
            }

            numGroup++;
        }

        if (numGroup == 0)
        {
            return CountFlips(fq, f2e, edgeDiff);
        }

        // --- Build per-group constraints. ------------------------------------------------------------
        var indices = new int[nEdges];
        Array.Fill(indices, -1);
        var numEdges = new int[numGroup];
        var values = new List<int>[numGroup];
        var variableEq = new List<(int, int, int)>[numGroup];
        var constantEq = new List<(int, int, int)>[numGroup];
        var variableGe = new List<(int, int, int, int)>[numGroup];
        var constantGe = new List<(int, int)>[numGroup];
        for (int g = 0; g < numGroup; g++)
        {
            values[g] = new List<int>();
            variableEq[g] = new List<(int, int, int)>();
            constantEq[g] = new List<(int, int, int)>();
            variableGe[g] = new List<(int, int, int, int)>();
            constantGe[g] = new List<(int, int)>();
        }

        for (int i = 0; i < nEdges; i++)
        {
            if (groups[i] != -1)
            {
                indices[i] = numEdges[groups[i]]++;
                values[groups[i]].Add(edgeDiff[i].X);
                values[groups[i]].Add(edgeDiff[i].Y);
            }
        }

        var numEdgesFlexible = (int[])numEdges.Clone();
        var fixedVariables = new Dictionary<(int, int), int>();
        for (int i = 0; i < nFaces; i++)
        {
            int gind = 0;
            while (gind < 3 && groups[f2e[i][gind]] == -1)
            {
                gind++;
            }

            if (gind == 3)
            {
                continue;
            }

            int group = groups[f2e[i][gind]];
            var ind = new[] { -1, -1, -1 };
            for (int j = 0; j < 3; j++)
            {
                int g = groups[f2e[i][j]];
                if (g != group)
                {
                    if (g == -1)
                    {
                        (int, int) key = (f2e[i][j], group);
                        if (!fixedVariables.TryGetValue(key, out int fixedId))
                        {
                            ind[j] = numEdges[group];
                            values[group].Add(edgeDiff[f2e[i][j]].X);
                            values[group].Add(edgeDiff[f2e[i][j]].Y);
                            fixedVariables[key] = numEdges[group]++;
                        }
                        else
                        {
                            ind[j] = fixedId;
                        }
                    }
                }
                else
                {
                    ind[j] = indices[f2e[i][j]];
                }
            }

            var varX = new int[3];
            var varY = new int[3];
            var cstX = new int[3];
            var cstY = new int[3];
            for (int j = 0; j < 3; j++)
            {
                Vec2i v = Lattice2D.Rshift90(new Vec2i((ind[j] * 2) + 1, (ind[j] * 2) + 2), fq[i][j]);
                cstX[j] = System.Math.Sign(v.X);
                cstY[j] = System.Math.Sign(v.Y);
                varX[j] = System.Math.Abs(v.X) - 1;
                varY[j] = System.Math.Abs(v.Y) - 1;
            }

            variableEq[group].Add((varX[0], varX[1], varX[2]));
            constantEq[group].Add((cstX[0], cstX[1], cstX[2]));
            variableEq[group].Add((varY[0], varY[1], varY[2]));
            constantEq[group].Add((cstY[0], cstY[1], cstY[2]));

            variableGe[group].Add((varX[0], varY[1], varY[0], varX[1]));
            constantGe[group].Add((cstX[0] * cstY[1], cstY[0] * cstX[1]));
        }

        // --- Solve each group; write back only on success. ------------------------------------------
        // Cap the per-group variable count: our in-process DPLL (no hierarchy descent like the reference,
        // which keeps each level's SAT tiny) is exponential in the worst case, so a large tangled
        // foldover region is left for the hole-filler rather than risking a multi-minute solve. Small
        // isolated clusters — the common residual case — solve in milliseconds.
        LastGroups = numGroup;
        LastSolvedSat = 0;
        LastUnsat = 0;
        LastTimeout = 0;
        LastSkippedLarge = 0;
        LastMaxGroupEdges = 0;
        for (int g = 0; g < numGroup; g++)
        {
            LastMaxGroupEdges = System.Math.Max(LastMaxGroupEdges, numEdgesFlexible[g]);
            if (numEdgesFlexible[g] > MaxGroupEdges)
            {
                LastSkippedLarge++;
                continue;
            }

            var flexLocal = new bool[values[g].Count];
            for (int j = 0; j < flexLocal.Length; j++)
            {
                flexLocal[j] = j < numEdgesFlexible[g] * 2;
            }

            int[] val = values[g].ToArray();
            LocalSat.Status status = LocalSat.Solve(
                val.Length, val, flexLocal, variableEq[g], constantEq[g], variableGe[g], constantGe[g],
                ConflictBudget);
            if (status == LocalSat.Status.Sat)
            {
                LastSolvedSat++;
                for (int j = 0; j < val.Length; j++)
                {
                    values[g][j] = val[j];
                }
            }
            else if (status == LocalSat.Status.Unsat)
            {
                LastUnsat++;
            }
            else
            {
                LastTimeout++;
            }
        }

        for (int i = 0; i < nEdges; i++)
        {
            int group = groups[i];
            if (group == -1)
            {
                continue;
            }

            edgeDiff[i] = new Vec2i(values[group][2 * indices[i]], values[group][(2 * indices[i]) + 1]);
        }

        return CountFlips(fq, f2e, edgeDiff);
    }
}

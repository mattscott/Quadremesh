using Quadremesh.Core.Math;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// Faithful re-port (plan 07, Phase 1c) of QuadriFlow's multiresolution coarsening
/// (hierarchy.cpp DownsampleGraph, driven by Initialize). Replaces our first-unmatched-neighbour match
/// with the reference's priority match: rank every directed link by <c>normalDot × areaRatio</c>, stable-
/// sort descending, greedily collapse the heaviest non-conflicting pairs, then area-weight the merged
/// position/normal and rebuild the coarse adjacency by the per-parent scratch-sort+dedupe scheme. Stores
/// both <c>mToLower</c> and <c>mToUpper</c> (the latter was missing in our version). Each coarser level is
/// validated against the reference state dump.
/// </summary>
internal sealed class FaithfulHierarchy
{
    private const double Rcpoverflow = 2.93873587705571876e-39;
    private const int MaxDepth = 25;

    public List<Vector3[]> V { get; } = new();

    public List<Vector3[]> N { get; } = new();

    public List<double[]> A { get; } = new();

    public List<int[][]> Adj { get; } = new();

    public List<int[]> ToLower { get; } = new();   // [downsample] node -> coarse parent

    public List<int[,]> ToUpper { get; } = new();  // [downsample] 2 × nCoarse (child0, child1|-1)

    public List<double[][]> AdjWeight { get; } = new(); // [level][node] = neighbour weights (1 at finest)

    public int LevelCount => V.Count;

    /// <summary>
    /// Builds the coarsening from a finest level (positions, angle-weighted normals, Voronoi areas, and
    /// angular adjacency). Continues until a single node, mirroring Initialize (MAX_DEPTH cap).
    /// </summary>
    public static FaithfulHierarchy Build(Vector3[] v0, Vector3[] n0, double[] a0, int[][] adj0)
    {
        var h = new FaithfulHierarchy();
        h.V.Add(v0);
        h.N.Add(n0);
        h.A.Add(a0);
        h.Adj.Add(adj0);
        var w0 = new double[adj0.Length][];
        for (int i = 0; i < adj0.Length; i++)
        {
            w0[i] = new double[adj0[i].Length];
            Array.Fill(w0[i], 1.0); // uniform weight 1 at the finest level
        }

        h.AdjWeight.Add(w0);

        for (int i = 0; i < MaxDepth; i++)
        {
            h.DownsampleGraph(i);
            if (h.V[i + 1].Length == 1)
            {
                break;
            }
        }

        return h;
    }

    /// <summary>
    /// Deterministic graph coloring for the parallel Gauss-Seidel field sweeps
    /// (hierarchy.cpp generate_graph_coloring_deterministic, non-TBB path). Visits vertices in natural
    /// order — the reference's <c>pcg32::shuffle</c> is a no-op for forward ranges (its <c>begin &lt;= end</c>
    /// guard returns immediately), so no PRNG is involved in this build. Greedy lowest-available colour;
    /// each phase lists its vertices in ascending id order.
    /// </summary>
    public static int[][] GenerateColoring(int[][] adj)
    {
        int size = adj.Length;
        var color = new int[size];
        Array.Fill(color, -1);
        var possible = new List<bool>();
        var sizePerColor = new List<int>();
        int ncolors = 0;

        for (int ip = 0; ip < size; ip++) // perm is identity (shuffle is a no-op)
        {
            for (int j = 0; j < possible.Count; j++)
            {
                possible[j] = true;
            }

            foreach (int k in adj[ip])
            {
                int c = color[k];
                if (c >= 0)
                {
                    possible[c] = false;
                }
            }

            int chosen = -1;
            for (int j = 0; j < possible.Count; j++)
            {
                if (possible[j])
                {
                    chosen = j;
                    break;
                }
            }

            if (chosen < 0)
            {
                chosen = ncolors++;
                possible.Add(false);
                sizePerColor.Add(0);
            }

            color[ip] = chosen;
            sizePerColor[chosen]++;
        }

        var phases = new int[ncolors][];
        var fill = new int[ncolors];
        for (int c = 0; c < ncolors; c++)
        {
            phases[c] = new int[sizePerColor[c]];
        }

        for (int i = 0; i < size; i++) // ascending id order within each colour
        {
            int c = color[i];
            phases[c][fill[c]++] = i;
        }

        return phases;
    }

    private readonly struct Entry
    {
        public readonly int I;
        public readonly int J;
        public readonly double Order;

        public Entry(int i, int j, double order)
        {
            I = i;
            J = j;
            Order = order;
        }
    }

    private void DownsampleGraph(int level)
    {
        Vector3[] v = V[level];
        Vector3[] n = N[level];
        double[] a = A[level];
        int[][] adj = Adj[level];
        int nv = v.Length;

        // 1. One Entry per directed link, ranked by dp*ratio, laid out vertex-ascending × neighbour-order.
        int nLinks = 0;
        for (int i = 0; i < nv; i++)
        {
            nLinks += adj[i].Length;
        }

        var entries = new Entry[nLinks];
        int idx = 0;
        for (int i = 0; i < nv; i++)
        {
            foreach (int k in adj[i])
            {
                double dp = Vector3.Dot(n[i], n[k]);
                double ratio = a[i] > a[k] ? a[i] / a[k] : a[k] / a[i];
                entries[idx++] = new Entry(i, k, dp * ratio);
            }
        }

        // 2. std::stable_sort with operator< == (order > e.order): descending by order, ties keep input order.
        var sortIdx = new int[nLinks];
        for (int i = 0; i < nLinks; i++)
        {
            sortIdx[i] = i;
        }

        Array.Sort(sortIdx, (x, y) =>
        {
            double ox = entries[x].Order;
            double oy = entries[y].Order;
            if (ox > oy)
            {
                return -1;
            }

            if (ox < oy)
            {
                return 1;
            }

            return x.CompareTo(y); // stable: preserve original (vertex-ascending × neighbour) order
        });

        // 3. Greedy maximal match over the sorted links; compact the collapsed pairs (in sorted order).
        var mergeFlag = new bool[nv];
        var collapsed = new List<Entry>();
        for (int s = 0; s < nLinks; s++)
        {
            Entry e = entries[sortIdx[s]];
            if (mergeFlag[e.I] || mergeFlag[e.J])
            {
                continue;
            }

            mergeFlag[e.I] = mergeFlag[e.J] = true;
            collapsed.Add(e);
        }

        int nCollapsed = collapsed.Count;
        int vertexCount = nv - nCollapsed;

        var vp = new Vector3[vertexCount];
        var np = new Vector3[vertexCount];
        var ap = new double[vertexCount];
        var toUpper = new int[2, vertexCount];
        var toLower = new int[nv];

        // 4. Merged pairs: area-weighted position + normal, summed area.
        for (int i = 0; i < nCollapsed; i++)
        {
            Entry e = collapsed[i];
            double area1 = a[e.I];
            double area2 = a[e.J];
            double surfaceArea = area1 + area2;
            vp[i] = surfaceArea > Rcpoverflow
                ? ((v[e.I] * area1) + (v[e.J] * area2)) / surfaceArea
                : (v[e.I] + v[e.J]) * 0.5;

            Vector3 normal = (n[e.I] * area1) + (n[e.J] * area2);
            double norm = normal.Length;
            np[i] = norm > Rcpoverflow ? normal / norm : Vector3.UnitX;
            ap[i] = surfaceArea;
            toUpper[0, i] = e.I;
            toUpper[1, i] = e.J;
            toLower[e.I] = i;
            toLower[e.J] = i;
        }

        // 5. Singletons appended in vertex-ascending order.
        int offset = nCollapsed;
        for (int i = 0; i < nv; i++)
        {
            if (!mergeFlag[i])
            {
                int id = offset++;
                vp[id] = v[i];
                np[id] = n[i];
                ap[id] = a[i];
                toUpper[0, id] = i;
                toUpper[1, id] = -1;
                toLower[i] = id;
            }
        }

        // 6. Coarse adjacency: gather children's links (remapped via to_lower), sort by id, dedupe summing weights.
        double[][] adjWlevel = AdjWeight[level];
        var adjP = new int[vertexCount][];
        var adjPW = new double[vertexCount][];
        var scratch = new List<(int Id, double W)>();
        for (int i = 0; i < vertexCount; i++)
        {
            scratch.Clear();
            for (int j = 0; j < 2; j++)
            {
                int upper = toUpper[j, i];
                if (upper == -1)
                {
                    continue;
                }

                int[] ua = adj[upper];
                double[] uw = adjWlevel[upper];
                for (int k = 0; k < ua.Length; k++)
                {
                    scratch.Add((toLower[ua[k]], uw[k]));
                }
            }

            scratch.Sort((a, b) => a.Id.CompareTo(b.Id));
            var adId = new List<int>(scratch.Count);
            var adW = new List<double>(scratch.Count);
            int last = -1;
            foreach ((int id, double w) in scratch)
            {
                if (id == i)
                {
                    continue;
                }

                if (id != last)
                {
                    adId.Add(id);
                    adW.Add(w);
                    last = id;
                }
                else
                {
                    adW[^1] += w;
                }
            }

            adjP[i] = adId.ToArray();
            adjPW[i] = adW.ToArray();
        }

        V.Add(vp);
        N.Add(np);
        A.Add(ap);
        Adj.Add(adjP);
        AdjWeight.Add(adjPW);
        ToLower.Add(toLower);
        ToUpper.Add(toUpper);
    }
}

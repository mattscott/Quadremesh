using Quadremesh.Core.Math;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// Faithful re-port (plan 07, Phase 1) of QuadriFlow's directed-edge mesh and its per-vertex attributes.
/// Builds the <c>V2E</c>/<c>E2E</c> directed-edge tables exactly as <c>dedge.cpp compute_direct_graph</c>
/// (so neighbour enumeration follows the same fan order the coarsening's stable-sort match depends on),
/// and computes the angle-weighted smooth normal (Thürmer–Wüthrich, <c>parametrizer-mesh.cpp
/// ComputeSmoothNormal</c>) and the mixed-Voronoi vertex area (<c>ComputeVertexArea</c>). These replace
/// our Newell-area-weighted normal + barycentric area, which diverged from the reference and fed the
/// coarsening priority with the wrong values. Validated per-vertex against the reference state dump.
/// </summary>
internal sealed class DirectedEdgeMesh
{
    private const double Rcpoverflow = 2.93873587705571876e-39;

    private DirectedEdgeMesh(Vector3[] v, int[][] f)
    {
        V = v;
        F = f;
    }

    public Vector3[] V { get; }

    public int[][] F { get; }                  // each face = 3 vertex ids

    public int[] V2E { get; private set; } = Array.Empty<int>();

    public int[] E2E { get; private set; } = Array.Empty<int>();

    public bool[] Boundary { get; private set; } = Array.Empty<bool>();

    public bool[] NonManifold { get; private set; } = Array.Empty<bool>();

    /// <summary>Per directed-edge sharp flag (breaks the normal fan at creases). All-false until features.</summary>
    public bool[] SharpEdges { get; private set; } = Array.Empty<bool>();

    private static int DedgePrev3(int e) => e % 3 == 0 ? e + 2 : e - 1;

    private static int DedgeNext3(int e) => e % 3 == 2 ? e - 2 : e + 1;

    /// <summary>
    /// QuadriFlow's <c>fast_acos</c> polynomial — used verbatim. The constants are FLOAT literals (as in
    /// field-math.hpp), promoted to double exactly as the reference does, so angle weights are bit-identical
    /// (a double-literal version differs at ~1e-7 and can flip a coarsening tie downstream).
    /// </summary>
    private static double FastAcos(double x)
    {
        double negate = x < 0.0 ? 1.0 : 0.0;
        x = System.Math.Abs(x);
        double ret = -0.0187293f;
        ret *= x;
        ret += 0.0742610f;
        ret *= x;
        ret -= 0.2121144f;
        ret *= x;
        ret += 1.5707288f;
        ret *= System.Math.Sqrt(1.0 - x);
        ret -= 2.0 * negate * ret;
        return (negate * System.Math.PI) + ret;
    }

    /// <summary>Builds the directed-edge tables from positions + triangle faces (dedge.cpp compute_direct_graph).</summary>
    public static DirectedEdgeMesh Build(Vector3[] v, int[][] f)
    {
        var m = new DirectedEdgeMesh(v, f);
        m.ComputeDirectGraph();
        return m;
    }

    private void ComputeDirectGraph()
    {
        const int deg = 3;
        int n = V.Length;
        int nf = F.Length;
        var v2e = new int[n];
        Array.Fill(v2e, -1);

        // tmp[edge] = (idx_next, nextEdgeInChainStartingAtIdxCur)
        var tmpFirst = new int[nf * deg];
        var tmpSecond = new int[nf * deg];
        Array.Fill(tmpSecond, -1);

        for (int fa = 0; fa < nf; fa++)
        {
            for (int i = 0; i < deg; i++)
            {
                int idxCur = F[fa][i];
                int idxNext = F[fa][(i + 1) % deg];
                int edgeId = (deg * fa) + i;
                if (idxCur == idxNext)
                {
                    continue;
                }

                tmpFirst[edgeId] = idxNext;
                if (v2e[idxCur] == -1)
                {
                    v2e[idxCur] = edgeId;
                }
                else
                {
                    int idx = v2e[idxCur];
                    while (tmpSecond[idx] != -1)
                    {
                        idx = tmpSecond[idx];
                    }

                    tmpSecond[idx] = edgeId;
                }
            }
        }

        var nonManifold = new bool[n];
        var e2e = new int[nf * deg];
        Array.Fill(e2e, -1);

        for (int fa = 0; fa < nf; fa++)
        {
            for (int i = 0; i < deg; i++)
            {
                int idxCur = F[fa][i];
                int idxNext = F[fa][(i + 1) % deg];
                int edgeIdCur = (deg * fa) + i;
                if (idxCur == idxNext)
                {
                    continue;
                }

                int it = v2e[idxNext];
                int edgeIdOpp = -1;
                while (it != -1)
                {
                    if (tmpFirst[it] == idxCur)
                    {
                        if (edgeIdOpp == -1)
                        {
                            edgeIdOpp = it;
                        }
                        else
                        {
                            nonManifold[idxCur] = true;
                            nonManifold[idxNext] = true;
                            edgeIdOpp = -1;
                            break;
                        }
                    }

                    it = tmpSecond[it];
                }

                if (edgeIdOpp != -1 && edgeIdCur < edgeIdOpp)
                {
                    e2e[edgeIdCur] = edgeIdOpp;
                    e2e[edgeIdOpp] = edgeIdCur;
                }
            }
        }

        var boundary = new bool[n];
        for (int i = 0; i < n; i++)
        {
            int edge = v2e[i];
            if (edge == -1)
            {
                continue;
            }

            if (nonManifold[i])
            {
                v2e[i] = -1;
                continue;
            }

            // Walk backwards to the first boundary edge (if any); otherwise take the min edge id (unsigned).
            int start = edge;
            long minEdge = long.MaxValue;
            do
            {
                minEdge = System.Math.Min(minEdge, edge);
                int prevEdge = e2e[DedgePrev3(edge)];
                if (prevEdge == -1)
                {
                    minEdge = edge;
                    boundary[i] = true;
                    break;
                }

                edge = prevEdge;
            }
            while (edge != start);

            v2e[i] = (int)minEdge;
        }

        V2E = v2e;
        E2E = e2e;
        Boundary = boundary;
        NonManifold = nonManifold;
        SharpEdges = new bool[nf * deg];
    }

    /// <summary>Angle-weighted smooth vertex normals (parametrizer-mesh.cpp ComputeSmoothNormal).</summary>
    public Vector3[] ComputeSmoothNormal()
    {
        int nf = F.Length;
        var nfNormals = new Vector3[nf];
        for (int f = 0; f < nf; f++)
        {
            Vector3 v0 = V[F[f][0]];
            Vector3 v1 = V[F[f][1]];
            Vector3 v2 = V[F[f][2]];
            Vector3 cr = Vector3.Cross(v1 - v0, v2 - v0);
            double norm = cr.Length;
            nfNormals[f] = norm < Rcpoverflow ? Vector3.UnitX : cr / norm;
        }

        int n = V.Length;
        var normals = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            int edge = V2E[i];
            if (NonManifold[i] || edge == -1)
            {
                normals[i] = Vector3.UnitX;
                continue;
            }

            int stop = edge;
            do
            {
                if (SharpEdges[edge])
                {
                    break;
                }

                edge = E2E[edge];
                if (edge != -1)
                {
                    edge = DedgeNext3(edge);
                }
            }
            while (edge != stop && edge != -1);

            if (edge == -1)
            {
                edge = stop;
            }
            else
            {
                stop = edge;
            }

            Vector3 normal = Vector3.Zero;
            do
            {
                int idx = edge % 3;
                int face = edge / 3;
                Vector3 d0 = V[F[face][(idx + 1) % 3]] - V[i];
                Vector3 d1 = V[F[face][(idx + 2) % 3]] - V[i];
                double angle = FastAcos(Vector3.Dot(d0, d1) / System.Math.Sqrt(d0.LengthSquared * d1.LengthSquared));
                if (double.IsFinite(angle))
                {
                    normal += nfNormals[face] * angle;
                }

                int opp = E2E[edge];
                if (opp == -1)
                {
                    break;
                }

                edge = DedgeNext3(opp);
                if (SharpEdges[edge])
                {
                    break;
                }
            }
            while (edge != stop);

            double m = normal.Length;
            normals[i] = m > Rcpoverflow ? normal / m : Vector3.UnitX;
        }

        return normals;
    }

    /// <summary>
    /// Angular-order 1-ring adjacency (adjacent-matrix.cpp generate_adjacency_matrix_uniform): neighbours
    /// in dedge fan order seeded at <c>V2E[i]</c> (boundary-first on boundaries). This ORDER feeds the
    /// coarsening's stable-sort match, so it must match the reference — our old CSR-by-edge-id order did not.
    /// Weights are uniform 1 at the finest level, so only the neighbour ids/order are returned.
    /// </summary>
    public int[][] GenerateAdjacency()
    {
        int n = V.Length;
        var adj = new int[n][];
        var list = new List<int>();
        for (int i = 0; i < n; i++)
        {
            int start = V2E[i];
            if (start == -1)
            {
                adj[i] = Array.Empty<int>();
                continue;
            }

            list.Clear();
            int edge = start;
            do
            {
                int bas = edge % 3;
                int f = edge / 3;
                int opp = E2E[edge];
                int next = DedgeNext3(opp);
                if (list.Count == 0)
                {
                    list.Add(F[f][(bas + 2) % 3]);
                }

                if (opp == -1 || next != start)
                {
                    list.Add(F[f][(bas + 1) % 3]);
                    if (opp == -1)
                    {
                        break;
                    }
                }

                edge = next;
            }
            while (edge != start);

            adj[i] = list.ToArray();
        }

        return adj;
    }

    /// <summary>Mixed-Voronoi vertex areas (parametrizer-mesh.cpp ComputeVertexArea).</summary>
    public double[] ComputeVertexArea()
    {
        const double third = 1.0f / 3.0f; // float division promoted to double, matching ComputeVertexArea
        int n = V.Length;
        var area = new double[n];
        for (int i = 0; i < n; i++)
        {
            int edge = V2E[i];
            int stop = edge;
            if (NonManifold[i] || edge == -1)
            {
                continue;
            }

            double vertexArea = 0.0;
            do
            {
                int ep = DedgePrev3(edge);
                int en = DedgeNext3(edge);
                Vector3 v = V[F[edge / 3][edge % 3]];
                Vector3 vn = V[F[en / 3][en % 3]];
                Vector3 vp = V[F[ep / 3][ep % 3]];

                Vector3 faceCenter = (v + vp + vn) * third;
                Vector3 prev = (v + vp) * 0.5;
                Vector3 next = (v + vn) * 0.5;

                vertexArea += 0.5 * (Vector3.Cross(v - prev, v - faceCenter).Length +
                                     Vector3.Cross(v - next, v - faceCenter).Length);

                int opp = E2E[edge];
                if (opp == -1)
                {
                    break;
                }

                edge = DedgeNext3(opp);
            }
            while (edge != stop);

            area[i] = vertexArea;
        }

        return area;
    }
}

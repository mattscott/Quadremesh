using Quadremesh.Core.Extraction;
using Quadremesh.Core.Math;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// Faithful re-port (plan 07, Phase 4) of QuadriFlow's integer-grid-map setup: the geometric
/// <c>edge_diff</c> construction from the converged field (parametrizer-sing.cpp
/// ComputePositionSingularities → parametrizer-int.cpp BuildEdgeInfo), before the Balance + hierarchical
/// flow. Scale is isotropic mScale (the adaptive per-axis swap is a no-op when mS=(1,1)). Reuses the
/// validated <see cref="Lattice2D.Rshift90"/>. The Balance + min-cost flow are a later sub-step.
/// </summary>
internal static class FaithfulIntegerSolve
{
    /// <summary>The geometric edge graph: per-edge integer offset, face→edge ids, and edge→(v0,v1).</summary>
    public sealed class EdgeInfo
    {
        public EdgeInfo(Vec2i[] edgeDiff, int[][] faceEdgeIds, Vec2i[] edgeValues)
        {
            EdgeDiff = edgeDiff;
            FaceEdgeIds = faceEdgeIds;
            EdgeValues = edgeValues;
        }

        public Vec2i[] EdgeDiff { get; }

        public int[][] FaceEdgeIds { get; }

        public Vec2i[] EdgeValues { get; }
    }

    private static Vector3 Rotate90By(Vector3 q, Vector3 n, int amount) =>
        ((amount & 1) != 0 ? Vector3.Cross(n, q) : q) * (amount < 2 ? 1.0 : -1.0);

    private static Vector3 MiddlePoint(Vector3 p0, Vector3 n0, Vector3 p1, Vector3 n1)
    {
        double n0p0 = Vector3.Dot(n0, p0);
        double n0p1 = Vector3.Dot(n0, p1);
        double n1p0 = Vector3.Dot(n1, p0);
        double n1p1 = Vector3.Dot(n1, p1);
        double n0n1 = Vector3.Dot(n0, n1);
        double denom = 1.0 / (1.0 - (n0n1 * n0n1) + 1e-4f);
        double lambda0 = 2.0 * (n0p1 - n0p0 - (n0n1 * (n1p0 - n1p1))) * denom;
        double lambda1 = 2.0 * (n1p0 - n1p1 - (n0n1 * (n0p1 - n0p0))) * denom;
        return (0.5 * (p0 + p1)) - (0.25 * ((n0 * lambda0) + (n1 * lambda1)));
    }

    private static (int X, int Y) PositionFloorIndex4(
        Vector3 o, Vector3 q, Vector3 n, Vector3 p, double invScaleX, double invScaleY)
    {
        Vector3 t = Vector3.Cross(n, q);
        Vector3 d = p - o;
        return ((int)System.Math.Floor(Vector3.Dot(q, d) * invScaleX),
                (int)System.Math.Floor(Vector3.Dot(t, d) * invScaleY));
    }

    // compat_position_extrinsic_index_4: integer lattice indices for the closest pair of grid nodes.
    private static (Vec2i First, Vec2i Second) CompatPositionIndex(
        Vector3 p0, Vector3 n0, Vector3 q0, Vector3 o0,
        Vector3 p1, Vector3 n1, Vector3 q1, Vector3 o1, double scale)
    {
        double inv = 1.0 / scale;
        Vector3 t0 = Vector3.Cross(n0, q0);
        Vector3 t1 = Vector3.Cross(n1, q1);
        Vector3 middle = MiddlePoint(p0, n0, p1, n1);
        (int X, int Y) o0p = PositionFloorIndex4(o0, q0, n0, middle, inv, inv);
        (int X, int Y) o1p = PositionFloorIndex4(o1, q1, n1, middle, inv, inv);

        double bestCost = double.PositiveInfinity;
        int bestI = -1, bestJ = -1;
        for (int i = 0; i < 4; i++)
        {
            Vector3 o0t = o0 + (q0 * (((i & 1) + o0p.X) * scale)) + (t0 * ((((i & 2) >> 1) + o0p.Y) * scale));
            for (int j = 0; j < 4; j++)
            {
                Vector3 o1t = o1 + (q1 * (((j & 1) + o1p.X) * scale)) + (t1 * ((((j & 2) >> 1) + o1p.Y) * scale));
                double cost = (o0t - o1t).LengthSquared;
                if (cost < bestCost)
                {
                    bestI = i;
                    bestJ = j;
                    bestCost = cost;
                }
            }
        }

        return (new Vec2i((bestI & 1) + o0p.X, ((bestI & 2) >> 1) + o0p.Y),
                new Vec2i((bestJ & 1) + o1p.X, ((bestJ & 2) >> 1) + o1p.Y));
    }

    private static Vector3 NormalizeOrSelf(Vector3 v)
    {
        double len = v.Length;
        return len > 1e-300 ? v / len : v;
    }

    /// <summary>
    /// Builds the geometric edge graph from the converged field. <paramref name="orientationSingular"/> is
    /// the orientation-singularity face set (from Phase 2) — BuildEdgeInfo doesn't overwrite an interior
    /// edge's diff from a singular face.
    /// </summary>
    public static EdgeInfo BuildEdgeDiff(
        Vector3[] v, Vector3[] n, Vector3[] q, Vector3[] o, int[][] faces, int[] e2e, double scale,
        ISet<int> orientationSingular)
    {
        int nf = faces.Length;

        // --- ComputePositionSingularities: pos_rank (3/face) + pos_index (6/face).
        var posRank = new int[nf][];
        var posIndex = new int[nf][];
        for (int f = 0; f < nf; f++)
        {
            int i0 = faces[f][0], i1 = faces[f][1], i2 = faces[f][2];
            var qq = new[] { NormalizeOrSelf(q[i0]), NormalizeOrSelf(q[i1]), NormalizeOrSelf(q[i2]) };
            var nn = new[] { n[i0], n[i1], n[i2] };
            var oo = new[] { o[i0], o[i1], o[i2] };
            var vv = new[] { v[i0], v[i1], v[i2] };

            var best = new int[3];
            double bestDp = double.NegativeInfinity;
            for (int a = 0; a < 4; a++)
            {
                Vector3 v0 = Rotate90By(qq[0], nn[0], a);
                for (int b = 0; b < 4; b++)
                {
                    Vector3 v1 = Rotate90By(qq[1], nn[1], b);
                    for (int c = 0; c < 4; c++)
                    {
                        Vector3 v2 = Rotate90By(qq[2], nn[2], c);
                        double dp = System.Math.Min(
                            System.Math.Min(Vector3.Dot(v0, v1), Vector3.Dot(v1, v2)), Vector3.Dot(v2, v0));
                        if (dp > bestDp)
                        {
                            bestDp = dp;
                            best[0] = a;
                            best[1] = b;
                            best[2] = c;
                        }
                    }
                }
            }

            posRank[f] = new[] { best[0], best[1], best[2] };
            for (int k = 0; k < 3; k++)
            {
                qq[k] = Rotate90By(qq[k], nn[k], best[k]);
            }

            posIndex[f] = new int[6];
            for (int k = 0; k < 3; k++)
            {
                int kn = k == 2 ? 0 : k + 1;
                (Vec2i first, Vec2i second) = CompatPositionIndex(
                    vv[k], nn[k], qq[k], oo[k], vv[kn], nn[kn], qq[kn], oo[kn], scale);
                Vec2i diff = first - second;
                posIndex[f][k * 2] = diff.X;
                posIndex[f][(k * 2) + 1] = diff.Y;
            }
        }

        // --- BuildEdgeInfo: assign edge ids in traversal order; per-edge diff via rshift90(pos_index, rank).
        var edgeDiff = new List<Vec2i>();
        var edgeValues = new List<Vec2i>();
        var faceEdgeIds = new int[nf][];
        for (int i = 0; i < nf; i++)
        {
            faceEdgeIds[i] = new[] { -1, -1, -1 };
        }

        for (int i = 0; i < nf; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int k1 = j, k2 = (j + 1) % 3;
                int v1 = faces[i][k1];
                int v2 = faces[i][k2];
                var e2 = new Vec2i(System.Math.Min(v1, v2), System.Math.Max(v1, v2));

                Vec2i diff2;
                if (v1 > v2)
                {
                    int rank2 = posRank[i][k2];
                    diff2 = Lattice2D.Rshift90(new Vec2i(-posIndex[i][k1 * 2], -posIndex[i][(k1 * 2) + 1]), rank2);
                }
                else
                {
                    int rank2 = posRank[i][k1];
                    diff2 = Lattice2D.Rshift90(new Vec2i(posIndex[i][k1 * 2], posIndex[i][(k1 * 2) + 1]), rank2);
                }

                int currentEid = (i * 3) + k1;
                int eid = e2e[currentEid];
                int eID1 = faceEdgeIds[i][k1];
                if (eID1 == -1)
                {
                    int eID2 = edgeValues.Count;
                    edgeValues.Add(e2);
                    edgeDiff.Add(diff2);
                    faceEdgeIds[i][k1] = eID2;
                    if (eid != -1)
                    {
                        faceEdgeIds[eid / 3][eid % 3] = eID2;
                    }
                }
                else if (!orientationSingular.Contains(i))
                {
                    int eID2 = faceEdgeIds[eid / 3][eid % 3];
                    edgeDiff[eID2] = diff2;
                }
            }
        }

        return new EdgeInfo(edgeDiff.ToArray(), faceEdgeIds, edgeValues.ToArray());
    }
}

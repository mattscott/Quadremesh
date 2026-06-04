using Quadremesh.Core.Math;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// Faithful re-port (plan 07, Phase 3) of QuadriFlow's 4-PoSy position field
/// (optimizer.cpp optimize_positions + the field-math.hpp lattice primitives middle_point,
/// position_floor_4, compat_position_extrinsic_4, position_round_4). Replaces our fixed-reference
/// round-wrap-to-v_i smoother — which lacked middle_point / floor+4×4-search and is the prime residual-
/// foldover generator — with the reference's running-reference update over the full hierarchy. Scale is
/// isotropic mScale (optimize_scale is non-adaptive in this build: mS=(1,1)). The field is seeded from the
/// same MSVC rand() stream as the orientation init (the x,y draws), so it lands in the reference's gauge.
/// </summary>
internal sealed class FaithfulPosition
{
    private const double Rcpoverflow = 2.93873587705571876e-39;
    private const int RandMax = 0x7fff;

    private uint _rngState;

    private void Srand(uint seed) => _rngState = seed;

    private int Rand()
    {
        _rngState = (_rngState * 214013u) + 2531011u;
        return (int)((_rngState >> 16) & 0x7fffu);
    }

    private static void CoordinateSystem(Vector3 a, out Vector3 b, out Vector3 c)
    {
        if (System.Math.Abs(a.X) > System.Math.Abs(a.Y))
        {
            double invLen = 1.0 / System.Math.Sqrt((a.X * a.X) + (a.Z * a.Z));
            c = new Vector3(a.Z * invLen, 0.0, -a.X * invLen);
        }
        else
        {
            double invLen = 1.0 / System.Math.Sqrt((a.Y * a.Y) + (a.Z * a.Z));
            c = new Vector3(0.0, a.Z * invLen, -a.Y * invLen);
        }

        b = Vector3.Cross(c, a);
    }

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

    private static Vector3 PositionFloor4(
        Vector3 o, Vector3 q, Vector3 n, Vector3 p, double scaleX, double scaleY, double invScaleX, double invScaleY)
    {
        Vector3 t = Vector3.Cross(n, q);
        Vector3 d = p - o;
        return o
            + (q * System.Math.Floor(Vector3.Dot(q, d) * invScaleX) * scaleX)
            + (t * System.Math.Floor(Vector3.Dot(t, d) * invScaleY) * scaleY);
    }

    private static Vector3 PositionRound4(
        Vector3 o, Vector3 q, Vector3 n, Vector3 p, double scaleX, double scaleY, double invScaleX, double invScaleY)
    {
        Vector3 t = Vector3.Cross(n, q);
        Vector3 d = p - o;
        // std::round rounds halfway cases AWAY from zero (not banker's rounding).
        return o
            + (q * System.Math.Round(Vector3.Dot(q, d) * invScaleX, MidpointRounding.AwayFromZero) * scaleX)
            + (t * System.Math.Round(Vector3.Dot(t, d) * invScaleY, MidpointRounding.AwayFromZero) * scaleY);
    }

    private static (Vector3 First, Vector3 Second) CompatPosition(
        Vector3 p0, Vector3 n0, Vector3 q0, Vector3 o0,
        Vector3 p1, Vector3 n1, Vector3 q1, Vector3 o1,
        double scaleX, double scaleY, double invScaleX, double invScaleY,
        double scaleX1, double scaleY1, double invScaleX1, double invScaleY1)
    {
        Vector3 t0 = Vector3.Cross(n0, q0);
        Vector3 t1 = Vector3.Cross(n1, q1);
        Vector3 middle = MiddlePoint(p0, n0, p1, n1);
        Vector3 o0p = PositionFloor4(o0, q0, n0, middle, scaleX, scaleY, invScaleX, invScaleY);
        Vector3 o1p = PositionFloor4(o1, q1, n1, middle, scaleX1, scaleY1, invScaleX1, invScaleY1);

        double bestCost = double.PositiveInfinity;
        int bestI = -1, bestJ = -1;
        for (int i = 0; i < 4; i++)
        {
            Vector3 o0t = o0p + ((q0 * (i & 1) * scaleX) + (t0 * ((i & 2) >> 1) * scaleY));
            for (int j = 0; j < 4; j++)
            {
                Vector3 o1t = o1p + ((q1 * (j & 1) * scaleX1) + (t1 * ((j & 2) >> 1) * scaleY1));
                double cost = (o0t - o1t).LengthSquared;
                if (cost < bestCost)
                {
                    bestI = i;
                    bestJ = j;
                    bestCost = cost;
                }
            }
        }

        Vector3 first = o0p + ((q0 * (bestI & 1) * scaleX) + (t0 * ((bestI & 2) >> 1) * scaleY));
        Vector3 second = o1p + ((q1 * (bestJ & 1) * scaleX1) + (t1 * ((bestJ & 2) >> 1) * scaleY1));
        return (first, second);
    }

    /// <summary>
    /// Solves the position field over the (reference) hierarchy. Q is the converged orientation field
    /// (per level). Scale is isotropic mScale. Returns the per-vertex anchors at level 0.
    /// </summary>
    public static Vector3[] Solve(
        Vector3[][] v, Vector3[][] n, Vector3[][] q, int[][][] adjId, double[][][] adjW,
        int[][][] phases, int[][,] toUpper, double scale, uint seed)
    {
        var solver = new FaithfulPosition();
        int levels = v.Length;

        // --- Init mO (hierarchy.cpp Initialize): O = V + (s*x + t*y)*scale; consume the angle draw too.
        var o = new Vector3[levels][];
        solver.Srand(seed);
        for (int l = 0; l < levels; l++)
        {
            int cols = v[l].Length;
            o[l] = new Vector3[cols];
            for (int j = 0; j < cols; j++)
            {
                CoordinateSystem(n[l][j], out Vector3 s, out Vector3 t);
                _ = solver.Rand(); // angle (consumed by mQ init; advances the stream)
                double x = ((double)solver.Rand() / RandMax * 2.0) - 1.0;
                double y = ((double)solver.Rand() / RandMax * 2.0) - 1.0;
                o[l][j] = v[l][j] + (((s * x) + (t * y)) * scale);
            }
        }

        const int levelIterations = 6;
        for (int level = levels - 1; level >= 0; level--)
        {
            Vector3[] vL = v[level];
            Vector3[] nL = n[level];
            Vector3[] qL = q[level];
            Vector3[] oL = o[level];
            int[][] adjI = adjId[level];
            double[][] adjWl = adjW[level];
            int[][] ph = phases[level];

            double scaleX = scale, scaleY = scale;
            double invScaleX = 1.0 / scaleX;
            double invScaleY = 1.0 / scaleY;

            for (int iter = 0; iter < levelIterations; iter++)
            {
                foreach (int[] group in ph)
                {
                    foreach (int i in group)
                    {
                        Vector3 nI = nL[i];
                        Vector3 vI = vL[i];
                        Vector3 qI = NormalizeOrSelf(qL[i]);
                        Vector3 sum = oL[i];
                        double weightSum = 0.0;
                        int[] ids = adjI[i];
                        double[] ws = adjWl[i];
                        for (int kk = 0; kk < ids.Length; kk++)
                        {
                            double weight = ws[kk];
                            if (weight == 0)
                            {
                                continue;
                            }

                            int j = ids[kk];
                            Vector3 qJ = NormalizeOrSelf(qL[j]);
                            (Vector3 first, Vector3 second) = CompatPosition(
                                vI, nI, qI, sum, vL[j], nL[j], qJ, oL[j],
                                scaleX, scaleY, invScaleX, invScaleY, scaleX, scaleY, invScaleX, invScaleY);

                            sum = (first * weightSum) + (second * weight);
                            weightSum += weight;
                            if (weightSum > Rcpoverflow)
                            {
                                sum /= weightSum;
                            }

                            sum -= Vector3.Dot(nI, sum - vI) * nI;
                        }

                        if (weightSum > 0)
                        {
                            oL[i] = PositionRound4(sum, qI, nI, vI, scaleX, scaleY, invScaleX, invScaleY);
                        }
                    }
                }
            }

            if (level > 0)
            {
                int[,] tu = toUpper[level - 1];
                Vector3[] dest = o[level - 1];
                Vector3[] destN = n[level - 1];
                Vector3[] destV = v[level - 1];
                for (int i = 0; i < oL.Length; i++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        int d = tu[k, i];
                        if (d == -1)
                        {
                            continue;
                        }

                        Vector3 oo = oL[i];
                        Vector3 nn = destN[d];
                        Vector3 vv = destV[d];
                        oo -= nn * Vector3.Dot(nn, oo - vv);
                        dest[d] = oo;
                    }
                }
            }
        }

        return o[0];
    }

    private static Vector3 NormalizeOrSelf(Vector3 v)
    {
        double len = v.Length;
        return len > 1e-300 ? v / len : v;
    }
}

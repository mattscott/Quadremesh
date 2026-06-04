using Quadremesh.Core.Math;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// Faithful re-port (plan 07, Phase 2) of QuadriFlow's 4-RoSy orientation field
/// (optimizer.cpp optimize_orientations) + singularity detection (parametrizer-sing.cpp
/// ComputeOrientationSingularities). Replaces our flat-average smoother with the reference's
/// running-reference update (self seeds the reference at weight 0; per-neighbour project+normalize;
/// compat_orientation_extrinsic_4 with the 1e-6 tie hysteresis), 6 iterations/level coarse-to-fine with
/// prolongation, then the final fine→coarse rebuild. The field is seeded from a single root cross via the
/// MSVC <c>rand()</c> LCG (the prebuilt oracle's PRNG) + <c>coordinate_system</c>, so the converged field
/// matches the reference's gauge and the singularity set is reproduced exactly.
/// </summary>
internal sealed class FaithfulOrientation
{
    private const double Rcpoverflow = 2.93873587705571876e-39;
    private const int RandMax = 0x7fff;

    private uint _rngState;

    private void Srand(uint seed) => _rngState = seed;

    // MSVC rand(): state = state*214013 + 2531011; return (state >> 16) & 0x7fff.
    private int Rand()
    {
        _rngState = (_rngState * 214013u) + 2531011u;
        return (int)((_rngState >> 16) & 0x7fffu);
    }

    /// <summary>Result of the orientation solve: the per-level field (Q[0] is finest) and the singularity map.</summary>
    public sealed class Result
    {
        public Result(Vector3[][] q, Dictionary<int, int> singularities)
        {
            Q = q;
            Singularities = singularities;
        }

        public Vector3[][] Q { get; }

        public Vector3[] Q0 => Q[0];

        public Dictionary<int, int> Singularities { get; }
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

    // compat_orientation_extrinsic_4: align q1's cross to q0's, with the 1e-6 tie hysteresis.
    private static (Vector3 First, Vector3 Second) CompatOrient(Vector3 q0, Vector3 n0, Vector3 q1, Vector3 n1)
    {
        Vector3 a0 = q0;
        Vector3 a1 = Vector3.Cross(n0, q0);
        Vector3 b0 = q1;
        Vector3 b1 = Vector3.Cross(n1, q1);
        double best = double.NegativeInfinity;
        int ba = 0, bb = 0;
        for (int i = 0; i < 2; i++)
        {
            Vector3 ai = i == 0 ? a0 : a1;
            for (int j = 0; j < 2; j++)
            {
                Vector3 bj = j == 0 ? b0 : b1;
                double score = System.Math.Abs(Vector3.Dot(ai, bj));
                if (score > best + 1e-6)
                {
                    ba = i;
                    bb = j;
                    best = score;
                }
            }
        }

        Vector3 selA = ba == 0 ? a0 : a1;
        Vector3 selB = bb == 0 ? b0 : b1;
        double dp = Vector3.Dot(selA, selB);
        return (selA, selB * System.Math.CopySign(1.0, dp));
    }

    // compat_orientation_extrinsic_index_4: integer rotation indices (strict argmax; +2 if dp<0).
    private static (int First, int Second) CompatIndex(Vector3 q0, Vector3 n0, Vector3 q1, Vector3 n1)
    {
        Vector3 a0 = q0;
        Vector3 a1 = Vector3.Cross(n0, q0);
        Vector3 b0 = q1;
        Vector3 b1 = Vector3.Cross(n1, q1);
        double best = double.NegativeInfinity;
        int ba = 0, bb = 0;
        for (int i = 0; i < 2; i++)
        {
            Vector3 ai = i == 0 ? a0 : a1;
            for (int j = 0; j < 2; j++)
            {
                Vector3 bj = j == 0 ? b0 : b1;
                double score = System.Math.Abs(Vector3.Dot(ai, bj));
                if (score > best)
                {
                    ba = i;
                    bb = j;
                    best = score;
                }
            }
        }

        Vector3 sa = ba == 0 ? a0 : a1;
        Vector3 sb = bb == 0 ? b0 : b1;
        if (Vector3.Dot(sa, sb) < 0)
        {
            bb += 2;
        }

        return (ba, bb);
    }

    private static int Modulo(int a, int b)
    {
        int r = a % b;
        return r < 0 ? r + b : r;
    }

    /// <summary>
    /// Solves the orientation field over the (reference) hierarchy and detects singularities.
    /// Inputs mirror the reference per-level arrays; <paramref name="seed"/> is the MSVC rng_seed (-seed 0).
    /// </summary>
    public static Result Solve(
        Vector3[][] n, int[][][] adjId, double[][][] adjW, int[][][] phases, int[][,] toUpper,
        double scale, int[][] faces, uint seed)
    {
        var solver = new FaithfulOrientation();
        int levels = n.Length;

        // --- Init (hierarchy.cpp Initialize): seed every node's cross from MSVC rand() + coordinate_system.
        var q = new Vector3[levels][];
        solver.Srand(seed);
        for (int l = 0; l < levels; l++)
        {
            int cols = n[l].Length;
            q[l] = new Vector3[cols];
            for (int j = 0; j < cols; j++)
            {
                CoordinateSystem(n[l][j], out Vector3 s, out Vector3 t);
                double angle = (double)solver.Rand() / RandMax * 2.0 * System.Math.PI;
                _ = solver.Rand(); // x (consumed by mO init; advances the stream)
                _ = solver.Rand(); // y
                q[l][j] = (s * System.Math.Cos(angle)) + (t * System.Math.Sin(angle));
            }
        }

        // --- optimize_orientations: coarse -> fine, 6 iters/level, then prolong.
        const int levelIterations = 6;
        for (int level = levels - 1; level >= 0; level--)
        {
            Vector3[] nLevel = n[level];
            int[][] adjI = adjId[level];
            double[][] adjWl = adjW[level];
            Vector3[] qLevel = q[level];
            int[][] ph = phases[level];

            for (int iter = 0; iter < levelIterations; iter++)
            {
                foreach (int[] group in ph)
                {
                    foreach (int i in group)
                    {
                        Vector3 nI = nLevel[i];
                        double weightSum = 0.0;
                        Vector3 sum = qLevel[i];
                        int[] ids = adjI[i];
                        double[] ws = adjWl[i];
                        for (int k = 0; k < ids.Length; k++)
                        {
                            double weight = ws[k];
                            if (weight == 0)
                            {
                                continue;
                            }

                            int j = ids[k];
                            (Vector3 first, Vector3 second) = CompatOrient(sum, nI, qLevel[j], nLevel[j]);
                            sum = (first * weightSum) + (second * weight);
                            sum -= nI * Vector3.Dot(nI, sum);
                            weightSum += weight;
                            double norm = sum.Length;
                            if (norm > Rcpoverflow)
                            {
                                sum /= norm;
                            }
                        }

                        if (weightSum > 0)
                        {
                            qLevel[i] = sum;
                        }
                    }
                }
            }

            if (level > 0)
            {
                int[,] tu = toUpper[level - 1];
                Vector3[] dest = q[level - 1];
                Vector3[] destN = n[level - 1];
                for (int i = 0; i < qLevel.Length; i++)
                {
                    for (int kk = 0; kk < 2; kk++)
                    {
                        int d = tu[kk, i];
                        if (d == -1)
                        {
                            continue;
                        }

                        Vector3 qq = qLevel[i];
                        Vector3 nn = destN[d];
                        dest[d] = qq - (nn * Vector3.Dot(nn, qq));
                    }
                }
            }
        }

        // --- final fine -> coarse rebuild (optimizer.cpp:113-142) — keeps coarse fields consistent.
        for (int l = 0; l < levels - 1; l++)
        {
            int[,] tu = toUpper[l];
            Vector3[] qL = q[l];
            Vector3[] nL = n[l];
            Vector3[] qNext = q[l + 1];
            Vector3[] nNext = n[l + 1];
            int cols = tu.GetLength(1);
            for (int i = 0; i < cols; i++)
            {
                int u0 = tu[0, i];
                int u1 = tu[1, i];
                Vector3 q0 = qL[u0];
                Vector3 n0 = nL[u0];
                Vector3 qq;
                if (u1 != -1)
                {
                    (Vector3 first, Vector3 second) = CompatOrient(q0, n0, qL[u1], nL[u1]);
                    qq = first + second;
                }
                else
                {
                    qq = q0;
                }

                Vector3 nn = nNext[i];
                qq -= Vector3.Dot(nn, qq) * nn;
                if (qq.LengthSquared > Rcpoverflow)
                {
                    qq /= qq.Length;
                }

                qNext[i] = qq;
            }
        }

        // --- ComputeOrientationSingularities on level 0.
        Vector3[] q0Field = q[0];
        Vector3[] n0Field = n[0];
        var singularities = new Dictionary<int, int>();
        for (int f = 0; f < faces.Length; f++)
        {
            int index = 0;
            for (int k = 0; k < 3; k++)
            {
                int i = faces[f][k];
                int jj = faces[f][k == 2 ? 0 : k + 1];
                (int first, int second) = CompatIndex(q0Field[i], n0Field[i], q0Field[jj], n0Field[jj]);
                index += second - first;
            }

            int indexMod = Modulo(index, 4);
            if (indexMod == 1 || indexMod == 3)
            {
                if (index >= 4 || index < 0)
                {
                    q0Field[faces[f][0]] = -q0Field[faces[f][0]];
                }

                singularities[f] = indexMod;
            }
        }

        return new Result(q, singularities);
    }
}

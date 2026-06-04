using Quadremesh.Core.Math;

namespace Quadremesh.Tests;

/// <summary>
/// Loads an intermediate field-stack dump written by the instrumented QuadriFlow reference (plan 07,
/// Phase 0 harness). The byte format is exactly <c>Hierarchy::SaveToFile</c> followed by the
/// orientation-singularity map, using the reference's <c>serialize.hpp</c> raw-<c>fwrite</c> primitives:
/// <list type="bullet">
///   <item>double: 8 bytes.</item>
///   <item>Eigen matrix: <c>int rows, int cols</c>, then rows*cols values ROW-major.</item>
///   <item><c>vector&lt;T&gt;</c>: <c>int count</c>, then count elements (each via its own Save).</item>
///   <item><c>Link</c> (in mAdj): 16 bytes — <c>int id</c>, 4 pad, <c>double weight</c> (MSVC x64 layout).</item>
///   <item><c>map&lt;int,int&gt;</c>: <c>int count</c>, then count (int key, int val) pairs.</item>
/// </list>
/// SaveToFile order: mScale, mF, mE2E, mAdj, mV, mN, mA, mToLower, mToUpper, mQ, mO, mS, mK, mPhases.
/// QuadriFlow stores per-vertex fields as 3×N column-per-vertex matrices, so the V/N/Q/O accessors
/// transpose to one <see cref="Vector3"/> per vertex. Test-only research tooling.
/// </summary>
internal sealed class RefHierarchyDump
{
    public double Scale { get; private set; }

    public int[,] F { get; private set; } = new int[0, 0];          // mF: 3 × nFaces

    public int[] E2E { get; private set; } = Array.Empty<int>();    // mE2E: 3*nFaces

    public int LevelCount { get; private set; }

    public int[][][] Adj { get; private set; } = Array.Empty<int[][]>();   // [level][node] = neighbour ids (order preserved)

    public double[][][] AdjWeight { get; private set; } = Array.Empty<double[][]>(); // [level][node] = neighbour weights

    public Vector3[][] V { get; private set; } = Array.Empty<Vector3[]>(); // [level][vertex]

    public Vector3[][] N { get; private set; } = Array.Empty<Vector3[]>();

    public double[][] A { get; private set; } = Array.Empty<double[]>();   // [level][vertex] area

    public int[][] ToLower { get; private set; } = Array.Empty<int[]>();   // [level][vertex] -> parent

    public int[][,] ToUpper { get; private set; } = Array.Empty<int[,]>(); // [level] = 2 × nCoarse

    public Vector3[][] Q { get; private set; } = Array.Empty<Vector3[]>(); // mQ orientation field

    public Vector3[][] O { get; private set; } = Array.Empty<Vector3[]>(); // mO position field

    public int[][][] Phases { get; private set; } = Array.Empty<int[][]>(); // mPhases: [level][color] = vertex ids

    public Dictionary<int, int> Singularities { get; private set; } = new();

    /// <summary>Converts mF (3 × nFaces, column = face) into one 3-vertex array per face.</summary>
    public int[][] Faces()
    {
        int nf = F.GetLength(1);
        var faces = new int[nf][];
        for (int f = 0; f < nf; f++)
        {
            faces[f] = new[] { F[0, f], F[1, f], F[2, f] };
        }

        return faces;
    }

    public static RefHierarchyDump Load(string path)
    {
        using var r = new BinaryReader(File.OpenRead(path));
        var d = new RefHierarchyDump
        {
            Scale = r.ReadDouble(),
            F = ReadMatrixInt(r),
            E2E = ReadMatrixIntFlat(r),
        };

        (d.Adj, d.AdjWeight) = ReadAdj(r);
        d.V = ReadMatrixListAsVectors3(r);
        d.N = ReadMatrixListAsVectors3(r);
        d.A = ReadMatrixListAsFlatDouble(r);
        d.ToLower = ReadMatrixListAsFlatInt(r);
        d.ToUpper = ReadMatrixListAsInt(r);
        d.Q = ReadMatrixListAsVectors3(r);
        d.O = ReadMatrixListAsVectors3(r);
        _ = ReadMatrixListAsVectors3OrEmpty(r); // mS (often 2×N or empty in non-adaptive runs) — skip
        _ = ReadMatrixListAsVectors3OrEmpty(r); // mK — skip
        d.LevelCount = d.V.Length;
        d.Phases = ReadPhases(r); // mPhases: per-level graph coloring
        d.Singularities = ReadIntMap(r);
        return d;
    }

    // --- primitives -----------------------------------------------------------------------------

    private static int[,] ReadMatrixInt(BinaryReader r)
    {
        int rows = r.ReadInt32();
        int cols = r.ReadInt32();
        var m = new int[rows, cols];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                m[i, j] = r.ReadInt32();
            }
        }

        return m;
    }

    private static int[] ReadMatrixIntFlat(BinaryReader r)
    {
        int rows = r.ReadInt32();
        int cols = r.ReadInt32();
        var a = new int[rows * cols];
        for (int i = 0; i < a.Length; i++)
        {
            a[i] = r.ReadInt32();
        }

        return a;
    }

    /// <summary>Reads a row-major Eigen double matrix as raw [rows, cols] doubles.</summary>
    private static double[,] ReadMatrixDoubleRaw(BinaryReader r, out int rows, out int cols)
    {
        rows = r.ReadInt32();
        cols = r.ReadInt32();
        var m = new double[rows, cols];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                m[i, j] = r.ReadDouble();
            }
        }

        return m;
    }

    // --- vector<...> readers (each consumes the leading int count) -------------------------------

    private static (int[][][] Ids, double[][][] Weights) ReadAdj(BinaryReader r)
    {
        int levels = r.ReadInt32();
        var adj = new int[levels][][];
        var w = new double[levels][][];
        for (int l = 0; l < levels; l++)
        {
            int nodes = r.ReadInt32();
            adj[l] = new int[nodes][];
            w[l] = new double[nodes][];
            for (int n = 0; n < nodes; n++)
            {
                int links = r.ReadInt32();
                var ids = new int[links];
                var weights = new double[links];
                for (int k = 0; k < links; k++)
                {
                    ids[k] = r.ReadInt32();    // Link.id
                    r.ReadInt32();             // 4 bytes padding (MSVC x64 alignment of double)
                    weights[k] = r.ReadDouble(); // Link.weight (1 at finest; accumulated at coarse levels)
                }

                adj[l][n] = ids;
                w[l][n] = weights;
            }
        }

        return (adj, w);
    }

    private static Vector3[][] ReadMatrixListAsVectors3(BinaryReader r)
    {
        int levels = r.ReadInt32();
        var outp = new Vector3[levels][];
        for (int l = 0; l < levels; l++)
        {
            double[,] m = ReadMatrixDoubleRaw(r, out int rows, out int cols);
            if (rows != 3)
            {
                throw new InvalidDataException($"Expected 3×N for a per-vertex field, got {rows}×{cols}.");
            }

            var v = new Vector3[cols];
            for (int j = 0; j < cols; j++)
            {
                v[j] = new Vector3(m[0, j], m[1, j], m[2, j]);
            }

            outp[l] = v;
        }

        return outp;
    }

    /// <summary>Like <see cref="ReadMatrixListAsVectors3"/> but tolerates non-3-row / empty matrices (mS/mK).</summary>
    private static int ReadMatrixListAsVectors3OrEmpty(BinaryReader r)
    {
        int levels = r.ReadInt32();
        for (int l = 0; l < levels; l++)
        {
            ReadMatrixDoubleRaw(r, out _, out _);
        }

        return levels;
    }

    private static double[][] ReadMatrixListAsFlatDouble(BinaryReader r)
    {
        int levels = r.ReadInt32();
        var outp = new double[levels][];
        for (int l = 0; l < levels; l++)
        {
            double[,] m = ReadMatrixDoubleRaw(r, out int rows, out int cols);
            var a = new double[rows * cols];
            int idx = 0;
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    a[idx++] = m[i, j];
                }
            }

            outp[l] = a;
        }

        return outp;
    }

    private static int[][] ReadMatrixListAsFlatInt(BinaryReader r)
    {
        int levels = r.ReadInt32();
        var outp = new int[levels][];
        for (int l = 0; l < levels; l++)
        {
            outp[l] = ReadMatrixIntFlat(r);
        }

        return outp;
    }

    private static int[][,] ReadMatrixListAsInt(BinaryReader r)
    {
        int levels = r.ReadInt32();
        var outp = new int[levels][,];
        for (int l = 0; l < levels; l++)
        {
            outp[l] = ReadMatrixInt(r);
        }

        return outp;
    }

    private static int[][][] ReadPhases(BinaryReader r)
    {
        int levels = r.ReadInt32();
        var outp = new int[levels][][];
        for (int l = 0; l < levels; l++)
        {
            int phases = r.ReadInt32();
            var byColor = new int[phases][];
            for (int p = 0; p < phases; p++)
            {
                byColor[p] = ReadVectorIntBody(r);
            }

            outp[l] = byColor;
        }

        return outp;
    }

    private static int[] ReadVectorIntBody(BinaryReader r)
    {
        int count = r.ReadInt32();
        var a = new int[count];
        for (int i = 0; i < count; i++)
        {
            a[i] = r.ReadInt32();
        }

        return a;
    }

    private static Dictionary<int, int> ReadIntMap(BinaryReader r)
    {
        int count = r.ReadInt32();
        var m = new Dictionary<int, int>(count);
        for (int i = 0; i < count; i++)
        {
            int key = r.ReadInt32();
            int val = r.ReadInt32();
            m[key] = val;
        }

        return m;
    }
}

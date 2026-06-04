using Quadremesh.Core.Math;

namespace Quadremesh.Tests;

/// <summary>
/// Reads intermediate-state dumps written by the instrumented QuadriFlow reference oracle (plan 07,
/// Phase 0 harness). The byte format is exactly the reference's <c>serialize.hpp</c> primitives, which
/// are raw little-endian <c>fwrite</c>s — so this mirrors them 1:1:
/// <list type="bullet">
///   <item>Eigen matrix: <c>int rows, int cols</c>, then <c>rows*cols</c> values in ROW-major order.</item>
///   <item><c>std::vector&lt;T&gt;</c>: <c>int count</c>, then <c>count</c> elements (each via its own Save).</item>
/// </list>
/// QuadriFlow stores per-vertex fields as 3×N matrices (column = vertex), so <see cref="ReadVectors3"/>
/// transposes a 3×N double matrix into one <see cref="Vector3"/> per column (vertex). This reader is the
/// CONTRACT the reference dump must satisfy; the matching dump calls live in the oracle build (the other
/// half of Phase 0). Test-only research tooling — not part of the shipping library.
/// </summary>
internal static class RefStateReader
{
    /// <summary>Reads an Eigen double matrix saved as [rows][cols][row-major doubles]; returns [r,c].</summary>
    public static double[,] ReadMatrixDouble(BinaryReader r)
    {
        int rows = r.ReadInt32();
        int cols = r.ReadInt32();
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

    /// <summary>Reads an Eigen int matrix saved as [rows][cols][row-major ints]; returns [r,c].</summary>
    public static int[,] ReadMatrixInt(BinaryReader r)
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

    /// <summary>
    /// Reads a 3×N double matrix (QuadriFlow's per-vertex layout: column j = vertex j) into N vectors.
    /// </summary>
    public static Vector3[] ReadVectors3(BinaryReader r)
    {
        int rows = r.ReadInt32();
        int cols = r.ReadInt32();
        if (rows != 3)
        {
            throw new InvalidDataException($"Expected a 3xN matrix, got {rows}x{cols}.");
        }

        // Stored row-major: all of row 0, then row 1, then row 2.
        var x = new double[cols];
        var y = new double[cols];
        var z = new double[cols];
        for (int j = 0; j < cols; j++)
        {
            x[j] = r.ReadDouble();
        }

        for (int j = 0; j < cols; j++)
        {
            y[j] = r.ReadDouble();
        }

        for (int j = 0; j < cols; j++)
        {
            z[j] = r.ReadDouble();
        }

        var v = new Vector3[cols];
        for (int j = 0; j < cols; j++)
        {
            v[j] = new Vector3(x[j], y[j], z[j]);
        }

        return v;
    }

    /// <summary>Reads a <c>std::vector&lt;double&gt;</c> ([count][doubles]).</summary>
    public static double[] ReadVectorDouble(BinaryReader r)
    {
        int n = r.ReadInt32();
        var a = new double[n];
        for (int i = 0; i < n; i++)
        {
            a[i] = r.ReadDouble();
        }

        return a;
    }

    /// <summary>Reads a <c>std::vector&lt;int&gt;</c> ([count][ints]).</summary>
    public static int[] ReadVectorInt(BinaryReader r)
    {
        int n = r.ReadInt32();
        var a = new int[n];
        for (int i = 0; i < n; i++)
        {
            a[i] = r.ReadInt32();
        }

        return a;
    }
}

using Quadremesh.Core.Math;

namespace Quadremesh.Core.Extraction;

/// <summary>A 2D integer lattice offset (one quad-grid step per unit). The currency of the integer solve.</summary>
public readonly struct Vec2i : IEquatable<Vec2i>
{
    /// <summary>First lattice coordinate.</summary>
    public readonly int X;

    /// <summary>Second lattice coordinate.</summary>
    public readonly int Y;

    /// <summary>Constructs a 2D integer vector.</summary>
    public Vec2i(int x, int y)
    {
        X = x;
        Y = y;
    }

    /// <summary>Component-wise sum.</summary>
    public static Vec2i operator +(Vec2i a, Vec2i b) => new(a.X + b.X, a.Y + b.Y);

    /// <summary>Negation.</summary>
    public static Vec2i operator -(Vec2i a) => new(-a.X, -a.Y);

    /// <inheritdoc/>
    public bool Equals(Vec2i other) => X == other.X && Y == other.Y;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Vec2i v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(X, Y);

    /// <inheritdoc/>
    public override string ToString() => $"({X}, {Y})";
}

/// <summary>
/// The 4-fold (90°) lattice symmetry arithmetic QuadriFlow's integer solve runs on: rotating integer
/// offsets, and the rank of the cross-field transition across an edge. Mirrors <c>rshift90</c> and
/// <c>compat_orientation_extrinsic_index_4</c> from the reference (read for understanding only).
/// </summary>
public static class Lattice2D
{
    /// <summary>Rotates an integer lattice offset by <paramref name="amount"/>·90° (the lattice's symmetry group).</summary>
    public static Vec2i Rshift90(Vec2i shift, int amount)
    {
        amount &= 3;
        if ((amount & 1) != 0)
        {
            shift = new Vec2i(-shift.Y, shift.X);
        }

        if (amount >= 2)
        {
            shift = -shift;
        }

        return shift;
    }

    /// <summary>
    /// The matching of two cross fields as a pair (a, b), a∈{0,1}, b∈{0,1,2,3}: the axis of cross 0
    /// and the (signed) axis of cross 1 that best align. The transition rank is (a − b) mod 4.
    /// </summary>
    public static (int A, int B) Match(Vector3 q0, Vector3 n0, Vector3 q1, Vector3 n1)
    {
        Span<Vector3> a = stackalloc Vector3[2] { q0, Vector3.Cross(n0, q0) };
        Span<Vector3> b = stackalloc Vector3[2] { q1, Vector3.Cross(n1, q1) };

        double best = double.NegativeInfinity;
        int bestA = 0;
        int bestB = 0;
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                double score = System.Math.Abs(Vector3.Dot(a[i], b[j]));
                if (score > best)
                {
                    best = score;
                    bestA = i;
                    bestB = j;
                }
            }
        }

        if (Vector3.Dot(a[bestA], b[bestB]) < 0)
        {
            bestB += 2;
        }

        return (bestA, bestB);
    }

    /// <summary>The cross-field transition rank across an edge from <paramref name="q0"/> to <paramref name="q1"/> (0–3).</summary>
    public static int Rank(Vector3 q0, Vector3 n0, Vector3 q1, Vector3 n1)
    {
        (int a, int b) = Match(q0, n0, q1, n1);
        return ((a - b) % 4 + 4) % 4;
    }
}

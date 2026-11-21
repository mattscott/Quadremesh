using System.Globalization;
using static System.Math;

namespace Quadremesh.Core.Math;

/// <summary>
/// Immutable double-precision 3-vector. The single geometric value type used across the library;
/// double precision is deliberate (field solvers accumulate many small averages — see docs/design/06).
/// </summary>
public readonly struct Vector3 : IEquatable<Vector3>
{
    /// <summary>X component.</summary>
    public readonly double X;

    /// <summary>Y component.</summary>
    public readonly double Y;

    /// <summary>Z component.</summary>
    public readonly double Z;

    /// <summary>Constructs a vector from its components.</summary>
    public Vector3(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    /// <summary>The zero vector.</summary>
    public static Vector3 Zero => new(0, 0, 0);

    /// <summary>The unit vector along +X.</summary>
    public static Vector3 UnitX => new(1, 0, 0);

    /// <summary>The unit vector along +Y.</summary>
    public static Vector3 UnitY => new(0, 1, 0);

    /// <summary>The unit vector along +Z.</summary>
    public static Vector3 UnitZ => new(0, 0, 1);

    /// <summary>Component-wise addition.</summary>
    public static Vector3 operator +(Vector3 a, Vector3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    /// <summary>Component-wise subtraction.</summary>
    public static Vector3 operator -(Vector3 a, Vector3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    /// <summary>Negation.</summary>
    public static Vector3 operator -(Vector3 a) => new(-a.X, -a.Y, -a.Z);

    /// <summary>Scalar multiplication.</summary>
    public static Vector3 operator *(Vector3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);

    /// <summary>Scalar multiplication.</summary>
    public static Vector3 operator *(double s, Vector3 a) => new(a.X * s, a.Y * s, a.Z * s);

    /// <summary>Scalar division.</summary>
    public static Vector3 operator /(Vector3 a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    /// <summary>Dot product.</summary>
    public static double Dot(Vector3 a, Vector3 b) => (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);

    /// <summary>Cross product (right-handed).</summary>
    public static Vector3 Cross(Vector3 a, Vector3 b) => new(
        (a.Y * b.Z) - (a.Z * b.Y),
        (a.Z * b.X) - (a.X * b.Z),
        (a.X * b.Y) - (a.Y * b.X));

    /// <summary>Squared Euclidean length (no square root).</summary>
    public double LengthSquared => (X * X) + (Y * Y) + (Z * Z);

    /// <summary>Euclidean length.</summary>
    public double Length => Sqrt(LengthSquared);

    /// <summary>Squared distance between two points.</summary>
    public static double DistanceSquared(Vector3 a, Vector3 b) => (a - b).LengthSquared;

    /// <summary>Distance between two points.</summary>
    public static double Distance(Vector3 a, Vector3 b) => (a - b).Length;

    /// <summary>
    /// Returns this vector scaled to unit length. Throws if the vector is shorter than
    /// <paramref name="epsilon"/>; use <see cref="TryNormalize"/> for the non-throwing variant.
    /// </summary>
    public Vector3 Normalized(double epsilon = 1e-12)
    {
        double len = Length;
        if (len < epsilon)
        {
            throw new InvalidOperationException("Cannot normalize a zero-length vector.");
        }

        return this / len;
    }

    /// <summary>
    /// Attempts to scale this vector to unit length. Returns <c>false</c> (and <see cref="Zero"/>)
    /// if the vector is degenerate, so callers can degrade gracefully instead of throwing.
    /// </summary>
    public bool TryNormalize(out Vector3 result, double epsilon = 1e-12)
    {
        double len = Length;
        if (len < epsilon)
        {
            result = Zero;
            return false;
        }

        result = this / len;
        return true;
    }

    /// <summary>Linear interpolation between <paramref name="a"/> and <paramref name="b"/>.</summary>
    public static Vector3 Lerp(Vector3 a, Vector3 b, double t) => a + ((b - a) * t);

    /// <summary>Component-wise minimum.</summary>
    public static Vector3 Min(Vector3 a, Vector3 b) =>
        new(System.Math.Min(a.X, b.X), System.Math.Min(a.Y, b.Y), System.Math.Min(a.Z, b.Z));

    /// <summary>Component-wise maximum.</summary>
    public static Vector3 Max(Vector3 a, Vector3 b) =>
        new(System.Math.Max(a.X, b.X), System.Math.Max(a.Y, b.Y), System.Math.Max(a.Z, b.Z));

    /// <inheritdoc/>
    public bool Equals(Vector3 other) => X.Equals(other.X) && Y.Equals(other.Y) && Z.Equals(other.Z);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Vector3 v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    /// <summary>Exact equality (component-wise). For tolerant comparison use a dedicated epsilon test.</summary>
    public static bool operator ==(Vector3 a, Vector3 b) => a.Equals(b);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(Vector3 a, Vector3 b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() =>
        string.Create(CultureInfo.InvariantCulture, $"({X:R}, {Y:R}, {Z:R})");
}

using Quadremesh.Core.Math;

namespace Quadremesh.Core.Acceleration;

/// <summary>An axis-aligned bounding box. Used by <see cref="Bvh"/> nodes.</summary>
public readonly struct Aabb
{
    /// <summary>Minimum corner.</summary>
    public Vector3 Min { get; }

    /// <summary>Maximum corner.</summary>
    public Vector3 Max { get; }

    /// <summary>Constructs a box from explicit corners.</summary>
    public Aabb(Vector3 min, Vector3 max)
    {
        Min = min;
        Max = max;
    }

    /// <summary>The empty (inverted-infinite) box; the identity for <see cref="Union"/> / <see cref="Include"/>.</summary>
    public static Aabb Empty => new(
        new Vector3(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity),
        new Vector3(double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity));

    /// <summary>The tight box around a single triangle.</summary>
    public static Aabb FromTriangle(Vector3 a, Vector3 b, Vector3 c) =>
        new(Vector3.Min(Vector3.Min(a, b), c), Vector3.Max(Vector3.Max(a, b), c));

    /// <summary>Returns this box enlarged to contain <paramref name="other"/>.</summary>
    public Aabb Union(Aabb other) => new(Vector3.Min(Min, other.Min), Vector3.Max(Max, other.Max));

    /// <summary>Returns this box enlarged to contain point <paramref name="p"/>.</summary>
    public Aabb Include(Vector3 p) => new(Vector3.Min(Min, p), Vector3.Max(Max, p));

    /// <summary>Squared distance from <paramref name="p"/> to the box (0 if inside).</summary>
    public double DistanceSquared(Vector3 p)
    {
        double dx = AxisDistance(p.X, Min.X, Max.X);
        double dy = AxisDistance(p.Y, Min.Y, Max.Y);
        double dz = AxisDistance(p.Z, Min.Z, Max.Z);
        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    /// <summary>Index (0/1/2) of the axis along which the box is widest.</summary>
    public int LargestAxis()
    {
        double ex = Max.X - Min.X;
        double ey = Max.Y - Min.Y;
        double ez = Max.Z - Min.Z;
        if (ex >= ey && ex >= ez)
        {
            return 0;
        }

        return ey >= ez ? 1 : 2;
    }

    private static double AxisDistance(double v, double lo, double hi)
    {
        if (v < lo)
        {
            return lo - v;
        }

        return v > hi ? v - hi : 0.0;
    }
}

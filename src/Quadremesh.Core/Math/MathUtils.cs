using static System.Math;

namespace Quadremesh.Core.Math;

/// <summary>Small geometric helpers shared across the mesh and field code.</summary>
public static class MathUtils
{
    /// <summary>A small absolute tolerance used for degeneracy checks.</summary>
    public const double Epsilon = 1e-12;

    /// <summary>Clamps <paramref name="value"/> to the inclusive range [<paramref name="min"/>, <paramref name="max"/>].</summary>
    public static double Clamp(double value, double min, double max) =>
        value < min ? min : value > max ? max : value;

    /// <summary>
    /// Unsigned area of the triangle (a, b, c).
    /// </summary>
    public static double TriangleArea(Vector3 a, Vector3 b, Vector3 c) =>
        0.5 * Vector3.Cross(b - a, c - a).Length;

    /// <summary>
    /// Unnormalized face normal of the triangle (a, b, c). Its length is twice the triangle area,
    /// which makes it a convenient area weight when accumulating per-vertex normals.
    /// </summary>
    public static Vector3 TriangleNormalAreaWeighted(Vector3 a, Vector3 b, Vector3 c) =>
        Vector3.Cross(b - a, c - a);

    /// <summary>
    /// Cotangent of the angle at vertex <paramref name="apex"/> in the triangle
    /// (<paramref name="apex"/>, <paramref name="u"/>, <paramref name="v"/>). Used for cotangent
    /// edge weights. Clamped away from degenerate (near-zero-area) configurations.
    /// </summary>
    public static double Cotangent(Vector3 apex, Vector3 u, Vector3 v)
    {
        Vector3 e0 = u - apex;
        Vector3 e1 = v - apex;
        double dot = Vector3.Dot(e0, e1);
        double crossLen = Vector3.Cross(e0, e1).Length;
        if (crossLen < Epsilon)
        {
            // Degenerate triangle: fall back to zero weight rather than dividing by ~0.
            return 0.0;
        }

        return dot / crossLen;
    }

    /// <summary>Interior angle (radians) at <paramref name="apex"/> between edges to <paramref name="u"/> and <paramref name="v"/>.</summary>
    public static double Angle(Vector3 apex, Vector3 u, Vector3 v)
    {
        if (!(u - apex).TryNormalize(out Vector3 d0) || !(v - apex).TryNormalize(out Vector3 d1))
        {
            return 0.0;
        }

        return Acos(Clamp(Vector3.Dot(d0, d1), -1.0, 1.0));
    }
}

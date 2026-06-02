using Quadremesh.Core.Math;

namespace Quadremesh.Core.Fields;

/// <summary>
/// Helpers for working in a vertex's tangent plane. Directions in the field solvers are kept as
/// extrinsic unit 3-vectors lying in the tangent plane of their vertex's normal (the Instant Meshes
/// convention), so a frame is only needed to seed an initial direction and to rotate by 90°.
/// </summary>
public static class TangentFrame
{
    /// <summary>An arbitrary unit tangent direction orthogonal to <paramref name="normal"/>.</summary>
    public static Vector3 AnyTangent(Vector3 normal)
    {
        // Pick a reference axis least aligned with the normal to avoid a near-degenerate projection.
        Vector3 reference = System.Math.Abs(normal.X) < 0.9 ? Vector3.UnitX : Vector3.UnitY;
        Vector3 t = reference - (Vector3.Dot(reference, normal) * normal);
        return t.TryNormalize(out Vector3 unit) ? unit : Vector3.UnitZ;
    }

    /// <summary>Removes the component of <paramref name="v"/> along <paramref name="normal"/> (projects into the tangent plane).</summary>
    public static Vector3 ProjectToTangent(Vector3 v, Vector3 normal) =>
        v - (Vector3.Dot(v, normal) * normal);

    /// <summary>Rotates tangent direction <paramref name="d"/> by +90° about <paramref name="normal"/>.</summary>
    public static Vector3 Rotate90(Vector3 d, Vector3 normal) => Vector3.Cross(normal, d);
}

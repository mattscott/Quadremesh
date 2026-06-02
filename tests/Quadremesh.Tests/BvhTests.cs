using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.CorpusGen;
using Xunit;

namespace Quadremesh.Tests;

public class BvhTests
{
    [Fact]
    public void Closest_point_on_triangle_handles_interior_and_corners()
    {
        Vector3 a = Vector3.Zero;
        Vector3 b = Vector3.UnitX;
        Vector3 c = Vector3.UnitY;

        // Above the centroid -> projects into the interior (z drops to 0).
        Vector3 interior = Bvh.ClosestPointOnTriangle(new Vector3(0.25, 0.25, 1.0), a, b, c);
        Assert.Equal(new Vector3(0.25, 0.25, 0.0), interior);

        // Beyond vertex b -> clamps to b.
        Vector3 corner = Bvh.ClosestPointOnTriangle(new Vector3(2.0, -1.0, 0.0), a, b, c);
        Assert.Equal(b, corner);
    }

    [Fact]
    public void Query_point_on_surface_has_zero_distance()
    {
        TriangleMesh sphere = Primitives.Icosphere();
        Bvh bvh = Bvh.Build(sphere);

        Vector3 onSurface = sphere.Positions[10];
        bvh.ClosestPoint(onSurface, out double dSq);
        Assert.Equal(0.0, dSq, 12);
    }

    [Fact]
    public void Query_outside_sphere_projects_near_unit_radius()
    {
        TriangleMesh sphere = Primitives.Icosphere(subdivisions: 3);
        Bvh bvh = Bvh.Build(sphere);

        Vector3 closest = bvh.ClosestPoint(new Vector3(3.0, 0.0, 0.0), out double dSq);
        // The unit icosphere surface sits at radius ~1, so the nearest point is ~2 away.
        Assert.InRange(System.Math.Sqrt(dSq), 1.95, 2.05);
        Assert.InRange(closest.Length, 0.98, 1.0);
    }
}

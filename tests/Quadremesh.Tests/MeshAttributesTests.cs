using Quadremesh.CorpusGen;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Xunit;

namespace Quadremesh.Tests;

public class MeshAttributesTests
{
    [Fact]
    public void Sphere_normals_point_outward()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere());
        Vector3[] normals = MeshAttributes.VertexNormals(mesh);

        for (int v = 0; v < mesh.VertexCount; v++)
        {
            // For a sphere centred at the origin the outward normal aligns with the position.
            double alignment = Vector3.Dot(normals[v], mesh.Positions[v].Normalized());
            Assert.True(alignment > 0.9, $"normal at {v} not outward (alignment {alignment})");
        }
    }

    [Fact]
    public void Vertex_areas_sum_to_total_surface_area()
    {
        TriangleMesh tri = Primitives.Icosphere();
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(tri);

        double total = 0.0;
        for (int t = 0; t < tri.TriangleCount; t++)
        {
            (int a, int b, int c) = tri.Triangle(t);
            total += MathUtils.TriangleArea(tri.Positions[a], tri.Positions[b], tri.Positions[c]);
        }

        double summed = 0.0;
        foreach (double area in MeshAttributes.VertexAreas(mesh))
        {
            summed += area;
        }

        Assert.True(total > 0.0);
        Assert.Equal(total, summed, 9);
    }

    [Fact]
    public void Cotangent_weight_of_equilateral_triangle_edges()
    {
        // Single equilateral triangle: each edge is a boundary edge, so its weight is ½·cot(60°).
        var positions = new[]
        {
            Vector3.Zero,
            Vector3.UnitX,
            new Vector3(0.5, System.Math.Sqrt(3.0) / 2.0, 0.0),
        };
        var tri = new TriangleMesh(positions, new[] { 0, 1, 2 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(tri);

        double[] weights = MeshAttributes.CotangentEdgeWeights(mesh);
        double expected = 0.5 / System.Math.Tan(System.Math.PI / 3.0); // ½·cot(60°)

        Assert.Equal(3, weights.Length);
        foreach (double w in weights)
        {
            Assert.Equal(expected, w, 12);
        }
    }

    [Fact]
    public void Sphere_cotangent_weights_are_finite()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere());

        foreach (double w in MeshAttributes.CotangentEdgeWeights(mesh))
        {
            Assert.True(double.IsFinite(w));
        }
    }
}

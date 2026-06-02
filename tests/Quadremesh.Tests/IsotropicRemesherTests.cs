using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.Core.Remeshing;
using Quadremesh.CorpusGen;
using Xunit;

namespace Quadremesh.Tests;

public class IsotropicRemesherTests
{
    private static (double Mean, double Min, double Max, double FractionInBand) EdgeStats(
        TriangleMesh mesh, double low, double high)
    {
        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(mesh);
        double sum = 0, min = double.MaxValue, max = 0;
        int inBand = 0;
        for (int e = 0; e < he.EdgeCount; e++)
        {
            (int a, int b) = he.Edge(e);
            double len = Vector3.Distance(mesh.Positions[a], mesh.Positions[b]);
            sum += len;
            min = System.Math.Min(min, len);
            max = System.Math.Max(max, len);
            if (len >= low && len <= high)
            {
                inBand++;
            }
        }

        return (sum / he.EdgeCount, min, max, (double)inBand / he.EdgeCount);
    }

    [Fact]
    public void Sphere_remesh_is_near_uniform_at_target_length()
    {
        TriangleMesh sphere = Primitives.Icosphere(subdivisions: 2);
        const double target = 0.3;
        var options = new IsotropicRemeshOptions { TargetEdgeLength = target, Iterations = 10 };

        TriangleMesh result = IsotropicRemesher.Run(sphere, options);

        (double mean, _, _, double fractionInBand) = EdgeStats(result, 0.8 * target, 4.0 / 3.0 * target);
        Assert.InRange(mean, 0.8 * target, 1.25 * target);
        Assert.True(fractionInBand > 0.8, $"only {fractionInBand:P0} of edges in the target band");
    }

    [Fact]
    public void Remesh_preserves_closed_manifold_topology()
    {
        TriangleMesh sphere = Primitives.Icosphere(subdivisions: 2);
        var options = new IsotropicRemeshOptions { TargetEdgeLength = 0.25, Iterations = 8 };

        TriangleMesh result = IsotropicRemesher.Run(sphere, options);
        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(result);

        Assert.True(he.Defects.IsEdgeManifold);
        Assert.True(he.Defects.IsClosed);
        Assert.Equal(2, he.VertexCount - he.EdgeCount + he.FaceCount);
    }

    [Fact]
    public void Remesh_output_has_no_degenerate_triangles()
    {
        TriangleMesh sphere = Primitives.Icosphere(subdivisions: 2);
        var options = new IsotropicRemeshOptions { TargetEdgeLength = 0.25, Iterations = 8 };

        TriangleMesh result = IsotropicRemesher.Run(sphere, options);

        for (int t = 0; t < result.TriangleCount; t++)
        {
            (int a, int b, int c) = result.Triangle(t);
            double area = MathUtils.TriangleArea(result.Positions[a], result.Positions[b], result.Positions[c]);
            Assert.True(area > 1e-12, $"degenerate triangle {t} with area {area}");
        }
    }

    [Fact]
    public void Remesh_keeps_vertices_on_the_original_surface()
    {
        TriangleMesh sphere = Primitives.Icosphere(subdivisions: 2);
        Bvh original = Bvh.Build(sphere);
        var options = new IsotropicRemeshOptions { TargetEdgeLength = 0.25, Iterations = 8 };

        TriangleMesh result = IsotropicRemesher.Run(sphere, options);

        // One-sided Hausdorff: every output vertex was projected onto the original surface.
        double maxDeviation = 0.0;
        foreach (Vector3 p in result.Positions)
        {
            original.ClosestPoint(p, out double dSq);
            maxDeviation = System.Math.Max(maxDeviation, System.Math.Sqrt(dSq));
        }

        Assert.True(maxDeviation < 1e-9, $"max surface deviation {maxDeviation}");
    }

    [Fact]
    public void Refining_increases_vertex_count()
    {
        TriangleMesh coarse = Primitives.Icosphere(subdivisions: 1); // 42 vertices
        var options = new IsotropicRemeshOptions { TargetEdgeLength = 0.2, Iterations = 8 };

        TriangleMesh result = IsotropicRemesher.Run(coarse, options);

        Assert.True(result.VertexCount > coarse.VertexCount);
    }

    [Fact]
    public void Target_quad_count_controls_resolution()
    {
        TriangleMesh sphere = Primitives.Icosphere(subdivisions: 3);
        var dense = IsotropicRemesher.Run(sphere, new IsotropicRemeshOptions { TargetQuadCount = 2000, Iterations = 6 });
        var coarse = IsotropicRemesher.Run(sphere, new IsotropicRemeshOptions { TargetQuadCount = 300, Iterations = 6 });

        Assert.True(dense.VertexCount > coarse.VertexCount);
    }

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("cylinder")]
    [InlineData("hard-surface")]
    [InlineData("head")]
    [InlineData("non-manifold")]
    public void Runs_on_full_corpus_without_throwing(string name)
    {
        TriangleMesh input = Primitives.All()[name];
        var options = new IsotropicRemeshOptions { TargetQuadCount = 400, Iterations = 5 };

        TriangleMesh result = IsotropicRemesher.Run(input, options);

        Assert.True(result.TriangleCount > 0);
        for (int t = 0; t < result.TriangleCount; t++)
        {
            (int a, int b, int c) = result.Triangle(t);
            double area = MathUtils.TriangleArea(result.Positions[a], result.Positions[b], result.Positions[c]);
            Assert.True(area > 1e-14);
        }
    }
}

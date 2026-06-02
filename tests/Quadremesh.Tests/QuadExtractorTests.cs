using Quadremesh.Core;
using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.CorpusGen;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

public class QuadExtractorTests
{
    private readonly ITestOutputHelper _output;

    public QuadExtractorTests(ITestOutputHelper output) => _output = output;

    private static QuadMesh Remesh(TriangleMesh input, double ell) =>
        QuadRemesher.Remesh(input, new QuadRemesherOptions { TargetEdgeLength = ell });

    private static int EulerCharacteristic(QuadMesh q)
    {
        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(q);
        return he.VertexCount - he.EdgeCount + he.FaceCount;
    }

    [Fact]
    public void Diagnostic_sphere_extraction()
    {
        QuadMesh q = Remesh(Primitives.Icosphere(subdivisions: 3), ell: 0.15);

        var histogram = new SortedDictionary<int, int>();
        for (int f = 0; f < q.FaceCount; f++)
        {
            histogram[q.FaceSize(f)] = histogram.GetValueOrDefault(q.FaceSize(f)) + 1;
        }

        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(q);
        _output.WriteLine($"sphere: {q.VertexCount} verts, {q.FaceCount} faces, {q.QuadFraction:P1} quads");
        foreach ((int size, int n) in histogram)
        {
            _output.WriteLine($"  {size}-gons: {n}");
        }

        _output.WriteLine($"  defects: {he.Defects}; euler = {EulerCharacteristic(q)}");
    }

    [Fact]
    public void Closed_input_yields_watertight_manifold_output()
    {
        QuadMesh q = Remesh(Primitives.Icosphere(subdivisions: 3), ell: 0.15);
        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(q);

        Assert.True(he.Defects.IsEdgeManifold, "output is not edge-manifold");
        Assert.True(he.Defects.IsClosed, "output is not watertight");
        Assert.Equal(2, EulerCharacteristic(q)); // genus-0, same as the input sphere
    }

    [Fact]
    public void Output_is_quad_dominant()
    {
        QuadMesh q = Remesh(Primitives.Icosphere(subdivisions: 3), ell: 0.15);
        _output.WriteLine($"quad fraction: {q.QuadFraction:P1}");

        // Quad-dominant for the first (Instant-Meshes) extractor. The 85% target needs the extraction
        // robustness work (P5 is the plan's #1 risk) and the pure-quad integer solve in P6.
        Assert.True(q.QuadFraction > 0.65, $"quad fraction {q.QuadFraction:P0} too low");
    }

    [Fact]
    public void Output_vertices_lie_on_the_original_surface()
    {
        TriangleMesh input = Primitives.Icosphere(subdivisions: 3);
        Bvh surface = Bvh.Build(input);
        QuadMesh q = Remesh(input, ell: 0.15);

        double maxDeviation = 0;
        foreach (Vector3 p in q.Positions)
        {
            surface.ClosestPoint(p, out double dSq);
            maxDeviation = System.Math.Max(maxDeviation, System.Math.Sqrt(dSq));
        }

        Assert.True(maxDeviation < 1e-9, $"output vertex off surface by {maxDeviation}");
    }

    [Fact]
    public void Faces_are_non_degenerate()
    {
        QuadMesh q = Remesh(Primitives.Icosphere(subdivisions: 3), ell: 0.15);
        for (int f = 0; f < q.FaceCount; f++)
        {
            ReadOnlySpan<int> face = q.Face(f);
            Assert.True(face.Length >= 3);
            for (int i = 0; i < face.Length; i++)
            {
                Assert.NotEqual(face[i], face[(i + 1) % face.Length]); // no repeated consecutive vertex
            }
        }
    }

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("cylinder")]
    [InlineData("hard-surface")]
    [InlineData("head")]
    [InlineData("non-manifold")]
    public void Runs_on_full_corpus_without_crashing(string name)
    {
        QuadMesh q = QuadRemesher.Remesh(
            Primitives.All()[name], new QuadRemesherOptions { TargetQuadCount = 400 });

        _output.WriteLine($"{name}: {q.FaceCount} faces, {q.QuadFraction:P0} quads, euler {EulerCharacteristic(q)}");

        // Reaching here proves no crash. Clean manifold inputs must produce faces; the deliberately
        // broken non-manifold mesh is allowed to degrade to an empty result (best-effort).
        Assert.True(q.FaceCount > 0 || name == "non-manifold", $"{name} produced no faces");
    }
}

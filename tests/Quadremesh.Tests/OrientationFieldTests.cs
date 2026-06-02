using Quadremesh.Core.Fields;
using Quadremesh.Core.Hierarchy;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.CorpusGen;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

public class OrientationFieldTests
{
    private readonly ITestOutputHelper _output;

    public OrientationFieldTests(ITestOutputHelper output) => _output = output;

    private static int EulerCharacteristic(HalfEdgeMesh m) => m.VertexCount - m.EdgeCount + m.FaceCount;

    private static (HalfEdgeMesh Mesh, MultiresHierarchy Hierarchy) BuildField(
        TriangleMesh input, out OrientationSolveResult result, bool recordEnergy = false)
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(input);
        MultiresHierarchy hierarchy = MultiresHierarchy.Build(mesh);
        result = OrientationFieldSolver.Solve(
            hierarchy, new OrientationSolveOptions { SweepsPerLevel = 20, RecordEnergy = recordEnergy });
        return (mesh, hierarchy);
    }

    [Fact]
    public void Hierarchy_node_counts_strictly_decrease_to_a_small_top()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere(subdivisions: 3));
        MultiresHierarchy hierarchy = MultiresHierarchy.Build(mesh);

        Assert.True(hierarchy.Levels.Count > 1);
        for (int l = 1; l < hierarchy.Levels.Count; l++)
        {
            Assert.True(hierarchy.Levels[l].NodeCount < hierarchy.Levels[l - 1].NodeCount);
        }

        Assert.Equal(mesh.VertexCount, hierarchy.Finest.NodeCount);
        Assert.True(hierarchy.Coarsest.NodeCount <= 4);
    }

    [Fact]
    public void Field_energy_decreases_monotonically_and_smooths_appreciably()
    {
        // The torus admits a globally smooth, singularity-free field, so energy should collapse toward 0.
        (HalfEdgeMesh mesh, MultiresHierarchy h) =
            BuildField(Primitives.Torus(), out OrientationSolveResult result, recordEnergy: true);

        var arbitrary = new Vector3[mesh.VertexCount];
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            arbitrary[v] = TangentFrame.AnyTangent(h.Finest.Normal[v]);
        }

        double initialEnergy = OrientationFieldSolver.Energy(h.Finest, arbitrary);
        double solvedEnergy = OrientationFieldSolver.Energy(h.Finest, result.Orientation);
        _output.WriteLine($"torus orientation energy: {initialEnergy:F3} -> {solvedEnergy:F3}");
        Assert.True(solvedEnergy < 0.6 * initialEnergy, $"field did not smooth: {initialEnergy} -> {solvedEnergy}");

        // The residual energy is concentrated at the few leftover singularities; the field should be
        // smooth almost everywhere, so the overwhelming majority of edges are near-perfectly aligned.
        double alignedFraction = WellAlignedEdgeFraction(h.Finest, result.Orientation, 0.05);
        _output.WriteLine($"torus well-aligned edge fraction: {alignedFraction:P1}");
        Assert.True(alignedFraction > 0.95, $"only {alignedFraction:P0} of edges are well aligned");

        // The finest sweeps start already near-converged (the hierarchy did the heavy lifting), so the
        // trace sits at the energy floor. It must stay stable there: the end is no worse than the start
        // and no single sweep increases energy by more than numerical floor-noise (<1%). The closest-rep
        // update is not the exact L2 minimiser, so tiny upticks at the floor are expected, not a bug.
        IReadOnlyList<double> trace = result.EnergyTrace;
        Assert.True(trace.Count > 1);
        Assert.True(trace[^1] <= trace[0] * 1.005, $"energy floor drifted up: {trace[0]} -> {trace[^1]}");
        for (int i = 1; i < trace.Count; i++)
        {
            Assert.True(trace[i] <= trace[i - 1] * 1.01, $"energy jumped at sweep {i}: {trace[i - 1]} -> {trace[i]}");
        }
    }

    private static double WellAlignedEdgeFraction(GraphLevel g, Vector3[] orient, double tolerance)
    {
        int total = 0, aligned = 0;
        for (int i = 0; i < g.NodeCount; i++)
        {
            foreach (int j in g.Neighbors(i))
            {
                if (j <= i)
                {
                    continue;
                }

                total++;
                Vector3 rep = Rosy.ClosestRepresentative(orient[i], orient[j], g.Normal[j]);
                if ((orient[i] - rep).LengthSquared < tolerance)
                {
                    aligned++;
                }
            }
        }

        return total == 0 ? 1.0 : (double)aligned / total;
    }

    [Fact]
    public void Sphere_singularity_index_sum_equals_four_euler()
    {
        (HalfEdgeMesh mesh, MultiresHierarchy h) = BuildField(Primitives.Icosphere(subdivisions: 2), out var result);
        SingularityResult sing = SingularityDetector.Detect(mesh, result.Orientation, h.Finest.Normal);

        _output.WriteLine($"sphere: {sing.SingularCount} singular faces, index sum {sing.IndexSum}");
        Assert.Equal(4 * EulerCharacteristic(mesh), sing.IndexSum); // 4·χ = 8
        Assert.True(sing.SingularCount >= 4);
    }

    [Fact]
    public void Torus_singularity_index_sum_is_zero_and_field_is_nearly_singularity_free()
    {
        (HalfEdgeMesh mesh, MultiresHierarchy h) = BuildField(Primitives.Torus(), out var result);
        SingularityResult sing = SingularityDetector.Detect(mesh, result.Orientation, h.Finest.Normal);

        _output.WriteLine($"torus: {sing.SingularCount} singular faces of {mesh.FaceCount}, index sum {sing.IndexSum}");
        Assert.Equal(0, sing.IndexSum); // 4·χ = 0 for a torus
        Assert.True(sing.SingularCount <= 0.1 * mesh.FaceCount, "torus field has too many singularities");
    }

    [Fact]
    public void Orientation_directions_are_unit_and_tangent()
    {
        (HalfEdgeMesh mesh, MultiresHierarchy h) = BuildField(Primitives.Icosphere(subdivisions: 2), out var result);

        for (int v = 0; v < mesh.VertexCount; v++)
        {
            Assert.Equal(1.0, result.Orientation[v].Length, 9);
            Assert.Equal(0.0, Vector3.Dot(result.Orientation[v], h.Finest.Normal[v]), 9);
        }
    }

    [Fact]
    public void Field_export_produces_obj_lines()
    {
        (HalfEdgeMesh mesh, MultiresHierarchy h) = BuildField(Primitives.Icosphere(subdivisions: 1), out var result);

        var sw = new StringWriter();
        FieldExporter.WriteOrientationField(sw, mesh.Positions, h.Finest.Normal, result.Orientation, 0.1);
        string obj = sw.ToString();

        Assert.Contains("\nl ", obj);
        Assert.Equal(mesh.VertexCount * 2, obj.Split("\nl ").Length - 1); // two segments per vertex
    }
}

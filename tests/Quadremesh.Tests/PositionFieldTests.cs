using Quadremesh.Core.Fields;
using Quadremesh.Core.Hierarchy;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.Core.Remeshing;
using Quadremesh.CorpusGen;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

public class PositionFieldTests
{
    private readonly ITestOutputHelper _output;

    public PositionFieldTests(ITestOutputHelper output) => _output = output;

    private sealed record Pipeline(
        HalfEdgeMesh Mesh, MultiresHierarchy Hierarchy, OrientationSolveResult Orientation,
        PositionSolveResult Position, double Ell);

    // Run the realistic pipeline: isotropic remesh at edge length ell, then solve both fields with the
    // lattice spacing matched to that resolution (this is what the orchestrator will actually do).
    private static Pipeline Run(TriangleMesh input, double ell, bool recordEnergy = false)
    {
        TriangleMesh iso = IsotropicRemesher.Run(input, new IsotropicRemeshOptions { TargetEdgeLength = ell, Iterations = 8 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(iso);
        MultiresHierarchy h = MultiresHierarchy.Build(mesh);
        OrientationSolveResult ori = OrientationFieldSolver.Solve(h, new OrientationSolveOptions { SweepsPerLevel = 20 });
        PositionSolveResult pos = PositionFieldSolver.Solve(
            h, ori.OrientationByLevel, new UniformSizingField(ell),
            new PositionSolveOptions { SweepsPerLevel = 20, RecordEnergy = recordEnergy });
        return new Pipeline(mesh, h, ori, pos, ell);
    }

    /// <summary>Mean fractional lattice offset across edges for the given anchors (0 = exact integer cells).</summary>
    private static double MeanLatticeResidual(Pipeline p, Vector3[] anchor)
    {
        Vector3[] normal = p.Hierarchy.Finest.Normal;
        Vector3[] orient = p.Orientation.Orientation;
        double total = 0;
        int count = 0;
        for (int e = 0; e < p.Mesh.EdgeCount; e++)
        {
            (int a, int b) = p.Mesh.Edge(e);
            Vector3 e1 = orient[a];
            Vector3 e2 = Vector3.Cross(normal[a], e1);
            Vector3 d = anchor[b] - anchor[a];
            double f1 = Frac(Vector3.Dot(d, e1) / p.Ell);
            double f2 = Frac(Vector3.Dot(d, e2) / p.Ell);
            total += System.Math.Sqrt((f1 * f1) + (f2 * f2));
            count++;
        }

        return total / count;
    }

    private static double Frac(double x) => x - System.Math.Round(x);

    [Fact]
    public void Anchors_snap_to_a_coherent_lattice()
    {
        Pipeline p = Run(Primitives.Icosphere(subdivisions: 3), ell: 0.15);
        double baseline = MeanLatticeResidual(p, p.Hierarchy.Finest.Position); // anchors left at vertices
        double solved = MeanLatticeResidual(p, p.Position.Position);
        _output.WriteLine($"sphere lattice residual: baseline {baseline:F4} -> solved {solved:F4} cells");
        Assert.True(solved < 0.7 * baseline, "position solve did not improve lattice alignment enough");
        Assert.True(solved < 0.25, $"anchors not on lattice: residual {solved}");
    }

    [Fact]
    public void Anchors_are_finite_and_near_their_vertices()
    {
        Pipeline p = Run(Primitives.Icosphere(subdivisions: 2), ell: 0.2);
        double maxDrift = 0;
        for (int v = 0; v < p.Mesh.VertexCount; v++)
        {
            Vector3 a = p.Position.Position[v];
            Assert.True(double.IsFinite(a.X) && double.IsFinite(a.Y) && double.IsFinite(a.Z));
            maxDrift = System.Math.Max(maxDrift, Vector3.Distance(a, p.Mesh.Positions[v]) / p.Ell);
        }

        _output.WriteLine($"max anchor drift: {maxDrift:F3} cells");
        // A lattice anchor is the nearest grid corner to its vertex, so it stays within ~half a cell.
        Assert.True(maxDrift < 0.75, $"anchor drifted {maxDrift:F2} cells from its vertex");
    }

    [Fact]
    public void Position_energy_decreases_and_is_stable()
    {
        Pipeline p = Run(Primitives.Torus(), ell: 0.15, recordEnergy: true);

        GraphLevel g = p.Hierarchy.Finest;
        var atVertices = (Vector3[])g.Position.Clone(); // initial anchors p_i = v_i
        double[] scale = new double[g.NodeCount];
        Array.Fill(scale, p.Ell);
        double initial = PositionFieldSolver.Energy(g, atVertices, p.Orientation.Orientation, scale);
        double solved = PositionFieldSolver.Energy(g, p.Position.Position, p.Orientation.Orientation, scale);
        _output.WriteLine($"torus position energy: {initial:F3} -> {solved:F3}");
        Assert.True(solved < initial, "position field did not reduce energy");

        IReadOnlyList<double> trace = p.Position.EnergyTrace;
        Assert.True(trace.Count > 1);
        Assert.True(trace[^1] <= trace[0] * 1.005, $"energy floor drifted up: {trace[0]} -> {trace[^1]}");
        for (int i = 1; i < trace.Count; i++)
        {
            Assert.True(trace[i] <= trace[i - 1] * 1.01, $"energy jumped at sweep {i}");
        }
    }

    [Fact]
    public void Anchor_export_produces_obj_points()
    {
        Pipeline p = Run(Primitives.Icosphere(subdivisions: 1), ell: 0.3);
        var sw = new StringWriter();
        FieldExporter.WritePositionAnchors(sw, p.Position.Position);
        string obj = sw.ToString();
        Assert.Contains("\np ", obj);
    }

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("cylinder")]
    [InlineData("hard-surface")]
    [InlineData("head")]
    [InlineData("non-manifold")]
    public void Converges_on_full_corpus_without_nans(string name)
    {
        Pipeline p = Run(Primitives.All()[name], ell: 0.2);
        foreach (Vector3 a in p.Position.Position)
        {
            Assert.True(double.IsFinite(a.X) && double.IsFinite(a.Y) && double.IsFinite(a.Z));
        }
    }
}

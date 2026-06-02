using Quadremesh.Core;
using Quadremesh.Core.Extraction;
using Quadremesh.Core.Fields;
using Quadremesh.Core.Hierarchy;
using Quadremesh.Core.Mesh;
using Quadremesh.Core.Remeshing;
using Quadremesh.CorpusGen;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

public class IntegerSolveTests
{
    private readonly ITestOutputHelper _output;

    public IntegerSolveTests(ITestOutputHelper output) => _output = output;

    private static IntegerOffsetSolver SolveFor(TriangleMesh input, double ell)
    {
        TriangleMesh iso = IsotropicRemesher.Run(input, new IsotropicRemeshOptions { TargetEdgeLength = ell, Iterations = 8 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(iso);
        MultiresHierarchy h = MultiresHierarchy.Build(mesh);
        OrientationSolveResult ori = OrientationFieldSolver.Solve(h, new OrientationSolveOptions { SweepsPerLevel = 20 });
        PositionSolveResult pos = PositionFieldSolver.Solve(
            h, ori.OrientationByLevel, new UniformSizingField(ell), new PositionSolveOptions { SweepsPerLevel = 20 });

        var scale = new double[mesh.VertexCount];
        Array.Fill(scale, ell);
        return IntegerOffsetSolver.Solve(mesh, ori.Orientation, h.Finest.Normal, pos.Position, scale);
    }

    [Fact]
    public void Integer_solve_cancels_face_divergence()
    {
        IntegerOffsetSolver solver = SolveFor(Primitives.Icosphere(subdivisions: 3), ell: 0.15);

        (int[] roundedK1, int[] roundedK2) = solver.RoundedOffsets();
        int before = solver.MaxFaceDivergence(roundedK1, roundedK2);
        int after = solver.MaxFaceDivergence(solver.EdgeK1, solver.EdgeK2);

        _output.WriteLine($"max face divergence: rounded {before} -> solved {after}");
        Assert.True(before > 0, "expected some inconsistency in the raw rounding");
        Assert.Equal(0, after); // min-cost flow makes every face close up
    }

    [Fact]
    public void Pure_quad_mode_raises_quad_fraction_and_removes_odd_faces()
    {
        TriangleMesh input = Primitives.Icosphere(subdivisions: 3);
        QuadMesh dominant = QuadRemesher.Remesh(
            input, new QuadRemesherOptions { TargetEdgeLength = 0.15, Extraction = ExtractionMode.QuadDominant });
        QuadMesh pure = QuadRemesher.Remesh(
            input, new QuadRemesherOptions { TargetEdgeLength = 0.15, Extraction = ExtractionMode.PureQuad });

        _output.WriteLine($"quad fraction: dominant {dominant.QuadFraction:P1} -> pure {pure.QuadFraction:P1}");
        Assert.True(pure.QuadFraction > dominant.QuadFraction + 0.10, "integer solve should raise quad fraction markedly");

        // A globally consistent integer lattice yields only even-sided faces; the greedy extractor's
        // stray triangles/pentagons vanish. (Zero odd faces is a strong signal the offsets are consistent.)
        int oddFaces = 0;
        for (int f = 0; f < pure.FaceCount; f++)
        {
            if (pure.FaceSize(f) % 2 == 1)
            {
                oddFaces++;
            }
        }

        Assert.Equal(0, oddFaces);
    }
}

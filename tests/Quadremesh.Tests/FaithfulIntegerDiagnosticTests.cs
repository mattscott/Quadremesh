using Quadremesh.Core.Extraction;
using Quadremesh.Core.Faithful;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

/// <summary>
/// Diagnostic (QUADREMESH_INTDIAG=1): compares the raw and post-flow |edge_diff| produced by the two
/// integer-solve front ends on the same faithful field — QuadriFlowSolver's own naive-round
/// <c>ComputeEdgeDiff</c> vs the faithful <c>FaithfulIntegerSolve.BuildEdgeDiff</c> (4×4 midpoint search).
/// The thesis under test: the reference keeps |edge_diff| raw≤1 / post≤2 because of the faithful
/// construction, not a hierarchical flow (the reference flow is single-level, DownsampleEdgeGraph(...,1)).
/// </summary>
public class FaithfulIntegerDiagnosticTests
{
    private readonly ITestOutputHelper _output;

    public FaithfulIntegerDiagnosticTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("fandisk")]
    [InlineData("happy")]
    public void MaxDiffComparison(string name)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_INTDIAG") != "1")
        {
            _output.WriteLine($"SKIP {name}: set QUADREMESH_INTDIAG=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string inputPath = Path.Combine(root!, "tests", "corpus", "external", name + ".obj");
        string goldenPath = Path.Combine(root!, "tests", "golden", "quadriflow", name + ".obj");
        int targetFaces = File.Exists(goldenPath) ? ObjReader.ReadPolygons(goldenPath).FaceCount : 5000;

        TriangleMesh input = ObjReader.ReadTriangles(inputPath);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        FaithfulPipeline.FaithfulField field = FaithfulPipeline.ComputeField(input, targetFaces);
        _output.WriteLine($"{name}: field computed in {sw.ElapsedMilliseconds}ms, {field.V.Length} verts, {field.F.Length} faces, sing={field.OrientationSingular.Count}");

        // --- Front end A: the existing QuadriFlowSolver (naive-round edge_diff + single-level MCF). ---
        var flatF = new int[field.F.Length * 3];
        for (int f = 0; f < field.F.Length; f++)
        {
            flatF[(f * 3) + 0] = field.F[f][0];
            flatF[(f * 3) + 1] = field.F[f][1];
            flatF[(f * 3) + 2] = field.F[f][2];
        }

        var mesh = new TriangleMesh(field.V, flatF);
        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(mesh);
        var scaleArr = new double[field.V.Length];
        Array.Fill(scaleArr, field.Scale);

        var solver = new QuadriFlowSolver(he, field.Q0, field.Normal, field.O, scaleArr);
        int rawA = MaxAbs(EnumerateEdgeDiff(solver));
        int divA0 = DivergentFaces(solver);
        solver.Solve();
        int postA = MaxAbs(EnumerateEdgeDiff(solver));
        int divA1 = DivergentFaces(solver);
        _output.WriteLine($"  [QuadriFlowSolver round]  rawMaxDiff={rawA} postMaxDiff={postA} divergentFaces {divA0}->{divA1}");

        // --- Front end B: the faithful BuildEdgeDiff (bit-exact to reference) raw |edge_diff|. ---
        int[] e2e = solver.BuildDedgeTwins();
        FaithfulIntegerSolve.EdgeInfo info = FaithfulIntegerSolve.BuildEdgeDiff(
            field.V, field.Normal, field.Q0, field.O, field.F, e2e, field.Scale, field.OrientationSingular);
        int rawB = MaxAbs(info.EdgeDiff);
        _output.WriteLine($"  [Faithful BuildEdgeDiff]  rawMaxDiff={rawB} edges={info.EdgeDiff.Length}");
    }

    [Theory]
    [InlineData("happy")]
    public void ExtractionOrdering(string name)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_INTDIAG") != "1")
        {
            _output.WriteLine($"SKIP {name}: set QUADREMESH_INTDIAG=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string inputPath = Path.Combine(root!, "tests", "corpus", "external", name + ".obj");
        string goldenPath = Path.Combine(root!, "tests", "golden", "quadriflow", name + ".obj");
        int targetFaces = File.Exists(goldenPath) ? ObjReader.ReadPolygons(goldenPath).FaceCount : 5000;

        TriangleMesh input = ObjReader.ReadTriangles(inputPath);
        FaithfulPipeline.FaithfulField field = FaithfulPipeline.ComputeField(input, targetFaces);

        var flatF = new int[field.F.Length * 3];
        for (int f = 0; f < field.F.Length; f++)
        {
            flatF[(f * 3) + 0] = field.F[f][0];
            flatF[(f * 3) + 1] = field.F[f][1];
            flatF[(f * 3) + 2] = field.F[f][2];
        }

        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(new TriangleMesh(field.V, flatF));
        var scaleArr = new double[field.V.Length];
        Array.Fill(scaleArr, field.Scale);
        var solver = new QuadriFlowSolver(he, field.Q0, field.Normal, field.O, scaleArr);
        solver.Solve();

        IntegerGridMesh solved = IntegerGridMesh.FromSolver(solver);
        var snapshot = new MemoryStream();
        solved.Save(new BinaryWriter(snapshot));

        void Try(string label, Action<IntegerGridMesh> align, bool faithfulCoarsening = false)
        {
            snapshot.Position = 0;
            IntegerGridMesh g = IntegerGridMesh.Load(new BinaryReader(snapshot));
            align(g);
            int neg = g.NegativeAreaFaceCount();
            var extractor = new QuadriFlowExtractor(g, faithfulCoarsening);
            QuadMesh raw = extractor.Extract();
            HalfEdgeMesh rawHe = HalfEdgeMesh.FromQuadMesh(raw);
            int bdRaw = Boundary(rawHe);
            QuadMesh filled = QuadHoleFiller.Fill(raw);
            HalfEdgeMesh fHe = HalfEdgeMesh.FromQuadMesh(filled);
            _output.WriteLine($"  {label,-32} negArea={neg} faces={raw.FaceCount} bdRaw={bdRaw} bdFilled={Boundary(fHe)} irr={Irr(fHe)} " +
                $"unpaired={extractor.DiagUnpairedDiagonals} degen={extractor.DiagDegenerateDiagonals}");
        }

        _output.WriteLine($"{name}: solved (maxDiff={solved.MaxEdgeDiff()})");
        Try("A old: FF,S,FF,S", g => { g.FixFlip(); g.SubdivideEdgeDiff(1); g.FixFlip(); g.SubdivideEdgeDiff(1); });
        Try("B ref: S,FF,S,SAT", g => { g.SubdivideEdgeDiff(1); g.FixFlip(); g.SubdivideEdgeDiff(1); g.FixFlipSat(); });
        Try("B+faithCoarse", g => { g.SubdivideEdgeDiff(1); g.FixFlip(); g.SubdivideEdgeDiff(1); g.FixFlipSat(); }, faithfulCoarsening: true);
        Try("A+faithCoarse", g => { g.FixFlip(); g.SubdivideEdgeDiff(1); g.FixFlip(); g.SubdivideEdgeDiff(1); }, faithfulCoarsening: true);
    }

    private static int Boundary(HalfEdgeMesh he)
    {
        int n = 0;
        for (int e = 0; e < he.EdgeCount; e++)
        {
            if (he.Twin(he.EdgeHalfEdge(e)) < 0)
            {
                n++;
            }
        }

        return n;
    }

    private static int Irr(HalfEdgeMesh he)
    {
        int irr = 0;
        for (int v = 0; v < he.VertexCount; v++)
        {
            bool boundary = false;
            foreach (int e in he.NeighborEdges(v))
            {
                if (he.Twin(he.EdgeHalfEdge(e)) < 0)
                {
                    boundary = true;
                    break;
                }
            }

            if (!boundary && he.Valence(v) != 4)
            {
                irr++;
            }
        }

        return irr;
    }

    private static IEnumerable<Vec2i> EnumerateEdgeDiff(QuadriFlowSolver solver)
    {
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            yield return solver.EdgeDiff(e);
        }
    }

    private static int MaxAbs(IEnumerable<Vec2i> diffs)
    {
        int m = 0;
        foreach (Vec2i d in diffs)
        {
            m = System.Math.Max(m, System.Math.Max(System.Math.Abs(d.X), System.Math.Abs(d.Y)));
        }

        return m;
    }

    private static int DivergentFaces(QuadriFlowSolver solver)
    {
        int n = 0;
        for (int f = 0; f < solver.FaceCount; f++)
        {
            if (solver.IsSingular(f))
            {
                continue;
            }

            Vec2i d = solver.FaceDivergence(f);
            if (d.X != 0 || d.Y != 0)
            {
                n++;
            }
        }

        return n;
    }

    private static string? RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            if (File.Exists(Path.Combine(dir.FullName, "Quadremesh.sln")))
            {
                return dir.FullName;
            }

            dir = dir.Parent;
        }

        return null;
    }
}

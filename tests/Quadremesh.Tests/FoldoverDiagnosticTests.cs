using Quadremesh.Core;
using Quadremesh.Core.Extraction;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

/// <summary>
/// Investigation harness for the foldover-induced holes on dense meshes (kickoff-foldover-holes). NOT a
/// pass/fail test — it instruments the lattice-alignment stages (FixFlip / SubdivideEdgeDiff) under
/// different orderings and reports per-stage foldover count, peak |edge_diff|, split count, guard hits,
/// and final watertightness (χ, boundary edges, irregular vertices). Opt-in via QUADREMESH_DIAG=1.
///
/// Every line is flushed to tests/diag/&lt;mesh&gt;.log immediately because the worst cases crash with an
/// uncatchable OOM/SO (xUnit just reports "test host crashed" and loses buffered output).
/// </summary>
public class FoldoverDiagnosticTests
{
    private readonly ITestOutputHelper _output;

    public FoldoverDiagnosticTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void GoldenWatertightness()
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_DIAG") != "1")
        {
            _output.WriteLine("SKIP: set QUADREMESH_DIAG=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        foreach (string name in new[] { "fandisk", "rocker-arm", "happy", "armadillo", "xyzrgb_dragon", "sphere", "torus" })
        {
            string p = Path.Combine(root!, "tests", "golden", "quadriflow", name + ".obj");
            if (!File.Exists(p))
            {
                _output.WriteLine($"{name}: MISSING");
                continue;
            }

            QuadMesh m = ObjReader.ReadPolygons(p);
            HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(m);
            int chi = he.VertexCount - he.EdgeCount + he.FaceCount;
            int bd = BoundaryEdgeCount(he);
            int irr = IrregularVertexCount(he);
            _output.WriteLine($"{name}: V={he.VertexCount} F={m.FaceCount} ({m.QuadFraction:P0}quad) χ={chi} boundaryEdges={bd} irregular={irr} {he.Defects}");
        }
    }

    [Theory]
    [InlineData("fandisk")]
    [InlineData("rocker-arm")]
    [InlineData("happy")]
    [InlineData("armadillo")]
    [InlineData("xyzrgb_dragon")]
    public void Diagnose(string name)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_DIAG") != "1")
        {
            _output.WriteLine($"SKIP {name}: set QUADREMESH_DIAG=1 to run the diagnostic.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string inputPath = Path.Combine(root!, "tests", "corpus", "external", name + ".obj");
        string goldenPath = Path.Combine(root!, "tests", "golden", "quadriflow", name + ".obj");
        if (!File.Exists(inputPath))
        {
            _output.WriteLine($"SKIP {name}: missing input.");
            return;
        }

        string diagDir = Path.Combine(root!, "tests", "diag");
        Directory.CreateDirectory(diagDir);
        bool runFeatures = Environment.GetEnvironmentVariable("QUADREMESH_DIAG_FEATURES") != "0";
        string suffix = runFeatures ? "feat" : "nofeat";
        string logPath = Path.Combine(diagDir, $"{name}.{suffix}.log");
        using var log = new StreamWriter(logPath, append: false) { AutoFlush = true };

        void Line(string s)
        {
            log.WriteLine(s);
            _output.WriteLine(s);
        }

        int targetQuads = File.Exists(goldenPath) ? ObjReader.ReadPolygons(goldenPath).FaceCount : 5000;
        TriangleMesh input = ObjReader.ReadTriangles(inputPath);
        Line($"=== {name} ({input.TriangleCount} tris, target {targetQuads}q, features={runFeatures}) ===");

        var sw = System.Diagnostics.Stopwatch.StartNew();
        (QuadriFlowSolver solver, _, bool hasFeatures) = QuadRemesher.BuildSolvedPureQuad(
            input, new QuadRemesherOptions
            {
                TargetQuadCount = targetQuads,
                Extraction = ExtractionMode.PureQuad,
                PreserveSharpFeatures = runFeatures,
            });
        Line($"solved in {sw.ElapsedMilliseconds}ms; features active={hasFeatures}; " +
            $"faces={solver.FaceCount} edges={solver.EdgeCount} singular={solver.SingularCount}");

        // edge_diff histogram straight off the solve.
        var hist = new int[8];
        int maxDiff = 0;
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            Vec2i d = solver.EdgeDiff(e);
            int m = System.Math.Max(System.Math.Abs(d.X), System.Math.Abs(d.Y));
            maxDiff = System.Math.Max(maxDiff, m);
            hist[System.Math.Min(m, hist.Length - 1)]++;
        }

        Line($"post-solve |edge_diff| max={maxDiff} hist[0..7+]=[{string.Join(",", hist)}]");

        // Cache the post-solve grid so AlignExperiment can iterate alignment strategies without re-solving.
        string gridCache = Path.Combine(diagDir, $"{name}.{suffix}.grid.bin");
        using (var bw = new BinaryWriter(File.Create(gridCache)))
        {
            IntegerGridMesh.FromSolver(solver).Save(bw);
        }

        // Cache the raw extracted mesh (pre-FixHoles) so FixHolesOnCached can iterate without re-solving.
        string rawObj = Path.Combine(diagDir, $"{name}.{suffix}.raw.obj");

        // Each ordering rebuilds a fresh grid from the solver (FromSolver re-reads the immutable solve).
        RunOrdering(Line, "current (FixFlip,Sub,FixFlip,Sub)", solver, rawObj, g =>
        {
            Stage(Line, g, "FixFlip", () => g.FixFlip());
            Stage(Line, g, "Subdiv1", () => g.SubdivideEdgeDiff(1));
            Stage(Line, g, "FixFlip", () => g.FixFlip());
            Stage(Line, g, "Subdiv1", () => g.SubdivideEdgeDiff(1));
        });

        Line("done.");
    }

    [Theory]
    [InlineData("fandisk.feat")]
    [InlineData("happy.feat")]
    [InlineData("armadillo.feat")]
    [InlineData("xyzrgb_dragon.feat")]
    public void AlignExperiment(string tag)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_DIAG") != "1")
        {
            _output.WriteLine($"SKIP {tag}: set QUADREMESH_DIAG=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string gridCache = Path.Combine(root!, "tests", "diag", $"{tag}.grid.bin");
        if (!File.Exists(gridCache))
        {
            _output.WriteLine($"SKIP {tag}: no cached grid (run Diagnose first).");
            return;
        }

        _output.WriteLine($"=== {tag} ===");

        // Baseline: current production alignment.
        Experiment(tag, gridCache, "baseline FixFlip(2)x2", g => QuadRemesher.AlignLatticeForExtraction(g));

        // FlipSat: baseline, then the SAT flip-fixer to clear residual foldovers.
        Experiment(tag, gridCache, "baseline + FixFlipSat", g =>
        {
            QuadRemesher.AlignLatticeForExtraction(g);
            int before = g.NegativeAreaFaceCount();
            int after = g.FixFlipSat();
            _output.WriteLine($"    [FlipSat negArea {before} -> {after}; groups={FlipSat.LastGroups} sat={FlipSat.LastSolvedSat} unsat={FlipSat.LastUnsat} timeout={FlipSat.LastTimeout} skippedLarge={FlipSat.LastSkippedLarge} maxGroupEdges={FlipSat.LastMaxGroupEdges}]");
        });
    }

    private void Experiment(string tag, string gridCache, string label, Action<IntegerGridMesh> align)
    {
        IntegerGridMesh g;
        using (var br = new BinaryReader(File.OpenRead(gridCache)))
        {
            g = IntegerGridMesh.Load(br);
        }

        var sw = System.Diagnostics.Stopwatch.StartNew();
        align(g);
        int neg = g.NegativeAreaFaceCount();
        QuadMesh raw = new QuadriFlowExtractor(g).Extract();
        QuadMesh filled = QuadHoleFiller.Fill(raw);
        QuadMesh quads = QuadMeshCleanup.FixValence(filled);
        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(quads);
        int chi = he.VertexCount - he.EdgeCount + he.FaceCount;
        _output.WriteLine($"  {label}: negArea={neg} faces={g.FaceCount} -> {quads.FaceCount}q χ={chi} boundaryEdges={BoundaryEdgeCount(he)} irregular={IrregularVertexCount(he)} ({sw.ElapsedMilliseconds}ms)");
    }

    [Theory]
    [InlineData("fandisk.feat")]
    [InlineData("fandisk.nofeat")]
    [InlineData("happy.feat")]
    [InlineData("happy.nofeat")]
    [InlineData("armadillo.feat")]
    [InlineData("xyzrgb_dragon.feat")]
    public void FixHolesOnCached(string tag)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_DIAG") != "1")
        {
            _output.WriteLine($"SKIP {tag}: set QUADREMESH_DIAG=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string rawObj = Path.Combine(root!, "tests", "diag", $"{tag}.raw.obj");
        if (!File.Exists(rawObj))
        {
            _output.WriteLine($"SKIP {tag}: no cached raw mesh (run Diagnose first).");
            return;
        }

        QuadMesh raw = ObjReader.ReadPolygons(rawObj);
        HalfEdgeMesh rawHe = HalfEdgeMesh.FromQuadMesh(raw);
        _output.WriteLine($"{tag}: raw {raw.FaceCount}q χ={rawHe.VertexCount - rawHe.EdgeCount + rawHe.FaceCount} boundaryEdges={BoundaryEdgeCount(rawHe)}");

        QuadMesh filled = QuadHoleFiller.Fill(raw);
        _output.WriteLine($"  fixholes: loops=[{string.Join(",", QuadHoleFiller.LastLoopSizes)}] quadsAdded={QuadHoleFiller.LastQuadsAdded}");
        QuadMesh quads = QuadMeshCleanup.FixValence(filled);
        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(quads);
        _output.WriteLine($"  result: {quads.FaceCount}q {quads.QuadFraction:P0}quad χ={he.VertexCount - he.EdgeCount + he.FaceCount} boundaryEdges={BoundaryEdgeCount(he)} irregular={IrregularVertexCount(he)} {he.Defects}");
    }

    private static void RunOrdering(Action<string> line, string label, QuadriFlowSolver solver, string? rawObjPath, Action<IntegerGridMesh> stages)
    {
        line($"--- ordering: {label} ---");
        var sw = System.Diagnostics.Stopwatch.StartNew();
        IntegerGridMesh grid;
        try
        {
            grid = IntegerGridMesh.FromSolver(solver);
            line($"  init: faces={grid.FaceCount} verts={grid.VertexCount} negArea={grid.NegativeAreaFaceCount()} maxDiff={grid.MaxEdgeDiff()}");
            stages(grid);
        }
        catch (Exception ex)
        {
            line($"  EXCEPTION: {ex.GetType().Name}: {ex.Message}");
            return;
        }

        try
        {
            var extractor = new QuadriFlowExtractor(grid);
            QuadMesh raw = extractor.Extract();
            if (rawObjPath is not null)
            {
                ObjWriter.Write(rawObjPath, raw);
            }

            line($"  extractor: coarseFaces={extractor.CoarseFaceCount} diagonals={extractor.DiagDiagonalDedges} pairedQuads={extractor.DiagPairedQuads} unpaired={extractor.DiagUnpairedDiagonals} degenerate={extractor.DiagDegenerateDiagonals}");
            QuadMesh filled = QuadHoleFiller.Fill(raw);
            line($"  fixholes: loops=[{string.Join(",", QuadHoleFiller.LastLoopSizes)}] quadsAdded={QuadHoleFiller.LastQuadsAdded}");
            QuadMesh quads = QuadMeshCleanup.FixValence(filled);
            HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(quads);
            int chi = he.VertexCount - he.EdgeCount + he.FaceCount;
            int bd = BoundaryEdgeCount(he);
            int irr = IrregularVertexCount(he);
            line($"  extract+fixholes: {quads.FaceCount}q {quads.QuadFraction:P0}quad χ={chi} boundaryEdges={bd} irregular={irr} {he.Defects} in {sw.ElapsedMilliseconds}ms");
        }
        catch (Exception ex)
        {
            line($"  EXTRACT EXCEPTION: {ex.GetType().Name}: {ex.Message}");
        }
    }

    private static void Stage(Action<string> line, IntegerGridMesh g, string name, Action act)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        act();
        string extra = name.StartsWith("Subdiv", StringComparison.Ordinal)
            ? $" splits={g.LastSplitCount} peakDiff={g.LastPeakDiff} hitGuard={g.LastHitGuard}"
            : string.Empty;
        line($"  {name}: faces={g.FaceCount} verts={g.VertexCount} negArea={g.NegativeAreaFaceCount()} maxDiff={g.MaxEdgeDiff()}{extra} ({sw.ElapsedMilliseconds}ms)");
    }

    private static int BoundaryEdgeCount(HalfEdgeMesh he)
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

    private static int IrregularVertexCount(HalfEdgeMesh he)
    {
        int irregular = 0;
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
                irregular++;
            }
        }

        return irregular;
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

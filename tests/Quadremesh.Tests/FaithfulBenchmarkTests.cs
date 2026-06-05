using Quadremesh.Core.Faithful;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

/// <summary>
/// End-to-end benchmark of the faithful pipeline (plan 07 integration) across the full golden corpus.
/// The headline metric is boundary edges (0 = watertight) — the dense-mesh holes the whole port targets.
/// Reports faces / quad% / χ / boundary / irregular vs the QuadriFlow golden. Opt-in (slow on dense):
/// QUADREMESH_FAITHBENCH=1. Results are also flushed to tests/diag/faithbench.log.
/// </summary>
public class FaithfulBenchmarkTests
{
    private readonly ITestOutputHelper _output;

    public FaithfulBenchmarkTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("fandisk")]
    [InlineData("rocker-arm")]
    [InlineData("happy")]
    [InlineData("armadillo")]
    [InlineData("xyzrgb_dragon")]
    public void FaithfulBenchmark(string name)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_FAITHBENCH") != "1")
        {
            _output.WriteLine($"SKIP {name}: set QUADREMESH_FAITHBENCH=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string goldenPath = Path.Combine(root!, "tests", "golden", "quadriflow", name + ".obj");
        string inputPath = Path.Combine(root!, "tests", "corpus", name + ".obj");
        if (!File.Exists(inputPath))
        {
            inputPath = Path.Combine(root!, "tests", "corpus", "external", name + ".obj");
        }

        if (!File.Exists(inputPath))
        {
            _output.WriteLine($"SKIP {name}: no input mesh.");
            return;
        }

        string diagDir = Path.Combine(root!, "tests", "diag");
        Directory.CreateDirectory(diagDir);
        using var log = new StreamWriter(Path.Combine(diagDir, "faithbench.log"), append: true) { AutoFlush = true };
        void Line(string s)
        {
            log.WriteLine(s);
            _output.WriteLine(s);
        }

        // Every golden was generated with QuadriFlow `-f 5000` (build-references.ps1); the varied golden face
        // counts are just different OUTPUTS of that one budget. So compare at the SAME budget — targetFaces is
        // QuadriFlow's -f, where we're bit-exact-validated and match golden quality (e.g. happy genus-6
        // watertight). Targeting golden-count instead runs a coarser field than the golden used.
        const int generationBudget = 5000;
        int goldIrr = -1, goldChi = 0, targetFaces = generationBudget;
        if (File.Exists(goldenPath))
        {
            QuadMesh g = ObjReader.ReadPolygons(goldenPath);
            HalfEdgeMesh gh = HalfEdgeMesh.FromQuadMesh(g);
            goldChi = gh.VertexCount - gh.EdgeCount + gh.FaceCount;
            goldIrr = IrregularVertexCount(gh);
        }

        TriangleMesh input = ObjReader.ReadTriangles(inputPath);
        // The faithful flow is now the default; set QUADREMESH_FAITHFLOW=0 to benchmark the old SSP flow.
        bool faithfulFlow = Environment.GetEnvironmentVariable("QUADREMESH_FAITHFLOW") != "0";
        if (int.TryParse(Environment.GetEnvironmentVariable("QUADREMESH_TARGETFACES"), out int tfOverride))
        {
            targetFaces = tfOverride;
        }
        var sw = System.Diagnostics.Stopwatch.StartNew();
        QuadMesh quads;
        try
        {
            quads = FaithfulPipeline.Remesh(input, targetFaces, faithfulFlow);
        }
        catch (Exception ex)
        {
            Line($"{name}: EXCEPTION {ex.GetType().Name}: {ex.Message}");
            return;
        }

        ObjWriter.Write(Path.Combine(diagDir, $"faith_{name}.obj"), quads);

        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(quads);
        int chi = he.VertexCount - he.EdgeCount + he.FaceCount;
        int bd = BoundaryEdgeCount(he);
        int irr = IrregularVertexCount(he);
        // Patches applied to reach watertight: 0 = the raw extraction was already clean at this budget (the
        // "lowest clean target" signal); >0 = the budget was coarse enough to need hole-patching.
        int patches = Quadremesh.Core.Extraction.QuadHoleFiller.LastQuadsAdded
            + Quadremesh.Core.Extraction.QuadHoleFiller.LastOddTrianglesAdded;
        Line($"{name}: target={targetFaces} {quads.FaceCount}q {quads.QuadFraction:P0}quad χ={chi} boundaryEdges={bd} irregular={irr} " +
            $"(golden irr={goldIrr} χ={goldChi}) patches={patches} {(bd == 0 ? "WATERTIGHT" : "HOLES")} {sw.ElapsedMilliseconds}ms");
    }

    [Theory]
    [InlineData("happy", -10)]   // genus 6
    public void SearchMinBudget_demo(string name, int expectedChi)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_FAITHBENCH") != "1")
        {
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string inputPath = Path.Combine(root!, "tests", "corpus", "external", name + ".obj");
        if (!File.Exists(inputPath))
        {
            inputPath = Path.Combine(root!, "tests", "corpus", name + ".obj");
        }

        if (!File.Exists(inputPath))
        {
            _output.WriteLine($"SKIP {name}: no input.");
            return;
        }

        TriangleMesh input = ObjReader.ReadTriangles(inputPath);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        FaithfulPipeline.BudgetSearchResult r = FaithfulPipeline.SearchMinBudget(
            input, expectedChi, out IReadOnlyList<FaithfulPipeline.BudgetSearchResult> scan,
            minBudget: 2400, maxBudget: 5600, steps: 8);
        foreach (FaithfulPipeline.BudgetSearchResult p in scan)
        {
            _output.WriteLine($"  scan budget={p.Budget} → {p.OutputQuads}q χ={p.Chi} boundary={p.BoundaryEdges} patches={p.Patches} " +
                $"{(p.BoundaryEdges == 0 && p.Chi == expectedChi ? "PASS" : "fail")}");
        }

        _output.WriteLine(
            $"{name}: LOWEST budget={r.Budget} → {r.OutputQuads} output quads, χ={r.Chi}, boundary={r.BoundaryEdges}, " +
            $"patches={r.Patches}, found={r.Found}, probes={r.Probes}, {sw.ElapsedMilliseconds}ms");
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

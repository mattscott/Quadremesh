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

    /// <summary>
    /// Characterizes the residual topology of the faithful pipeline output: boundary loops (count / length /
    /// parity) before and after <see cref="QuadHoleFiller"/>, plus connected components and per-component χ.
    /// (B) happy: are the last holes odd loops QuadHoleFiller can't quad-fill? (C) dragon: is χ=4 a second
    /// detached component? Replicates FaithfulPipeline's extraction exactly (faithful integer solve + order).
    /// </summary>
    [Theory]
    [InlineData("happy")]
    [InlineData("xyzrgb_dragon")]
    public void ResidualHoles(string name)
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

        FaithfulIntegerSolve.EdgeInfo edgeInfo = FaithfulIntegerSolve.BuildEdgeDiff(
            field.V, field.Normal, field.Q0, field.O, field.F, field.E2E, field.Scale, field.OrientationSingular);
        int[][] faceEdgeOrients = FaithfulIntegerSolve.ComputeFaceEdgeOrients(
            field.Normal, field.Q0, field.F, edgeInfo.FaceEdgeIds, edgeInfo.EdgeDiff.Length, field.OrientationSingular);
        var solver = new QuadriFlowSolver(
            he, field.Q0, field.Normal, field.O, scaleArr, field.F, edgeInfo.FaceEdgeIds, edgeInfo.EdgeDiff,
            edgeInfo.EdgeValues, faceEdgeOrients, field.OrientationSingular);
        solver.Solve();

        IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
        grid.SubdivideEdgeDiff(1);
        grid.FixFlip();
        grid.SubdivideEdgeDiff(1);
        grid.FixFlipSat();
        QuadMesh raw = new QuadriFlowExtractor(grid, faithfulCoarsening: true).Extract();
        QuadMesh filled = QuadHoleFiller.Fill(raw);

        _output.WriteLine($"{name}: raw {raw.FaceCount}q  ->  filled {filled.FaceCount}q");
        ReportLoops("  raw    boundary loops", raw);
        ReportLoops("  filled boundary loops", filled);
        ReportComponents("  filled components", filled);
    }

    private void ReportLoops(string label, QuadMesh m)
    {
        List<int> loops = BoundaryLoops(m);
        loops.Sort();
        int odd = loops.Count(l => (l & 1) == 1);
        int totalEdges = loops.Sum();
        _output.WriteLine($"{label}: {loops.Count} loops, {totalEdges} edges, {odd} ODD; lengths=[{string.Join(",", loops)}]");
    }

    private void ReportComponents(string label, QuadMesh m)
    {
        List<(int V, int E, int F, int Chi)> comps = Components(m);
        comps.Sort((a, b) => b.F.CompareTo(a.F));
        _output.WriteLine($"{label}: {comps.Count} components");
        foreach ((int v, int e, int f, int chi) in comps.Take(8))
        {
            _output.WriteLine($"      V={v} E={e} F={f} χ={chi} (genus {(2 - chi) / 2})");
        }
    }

    // Boundary directed edge (a,b) = one with no opposite (b,a) among face cycles; chain into loops.
    private static List<int> BoundaryLoops(QuadMesh m)
    {
        long N = m.VertexCount;
        var present = new HashSet<long>();
        for (int f = 0; f < m.FaceCount; f++)
        {
            ReadOnlySpan<int> face = m.Face(f);
            for (int j = 0; j < face.Length; j++)
            {
                present.Add((face[j] * N) + face[(j + 1) % face.Length]);
            }
        }

        var nextFrom = new Dictionary<int, List<int>>();
        for (int f = 0; f < m.FaceCount; f++)
        {
            ReadOnlySpan<int> face = m.Face(f);
            for (int j = 0; j < face.Length; j++)
            {
                int a = face[j], b = face[(j + 1) % face.Length];
                if (!present.Contains((b * N) + a))
                {
                    if (!nextFrom.TryGetValue(a, out List<int>? list))
                    {
                        list = new List<int>();
                        nextFrom[a] = list;
                    }

                    list.Add(b);
                }
            }
        }

        var used = new HashSet<long>();
        var loops = new List<int>();
        foreach ((int start, List<int> targets) in nextFrom)
        {
            foreach (int t0 in targets)
            {
                if (used.Contains((start * N) + t0))
                {
                    continue;
                }

                int len = 0, a = start, b = t0;
                while (!used.Contains((a * N) + b))
                {
                    used.Add((a * N) + b);
                    len++;
                    int na = b;
                    if (!nextFrom.TryGetValue(na, out List<int>? outs))
                    {
                        break;
                    }

                    int nb = -1;
                    foreach (int cand in outs)
                    {
                        if (!used.Contains((na * N) + cand))
                        {
                            nb = cand;
                            break;
                        }
                    }

                    if (nb == -1)
                    {
                        break;
                    }

                    a = na;
                    b = nb;
                }

                loops.Add(len);
            }
        }

        return loops;
    }

    private static List<(int V, int E, int F, int Chi)> Components(QuadMesh m)
    {
        var uf = new int[m.VertexCount];
        for (int i = 0; i < uf.Length; i++)
        {
            uf[i] = i;
        }

        int Find(int x)
        {
            while (uf[x] != x)
            {
                uf[x] = uf[uf[x]];
                x = uf[x];
            }

            return x;
        }

        void Union(int a, int b) => uf[Find(a)] = Find(b);

        for (int f = 0; f < m.FaceCount; f++)
        {
            ReadOnlySpan<int> face = m.Face(f);
            for (int j = 1; j < face.Length; j++)
            {
                Union(face[0], face[j]);
            }
        }

        var faces = new Dictionary<int, int>();
        var verts = new Dictionary<int, HashSet<int>>();
        var edges = new Dictionary<int, HashSet<long>>();
        for (int f = 0; f < m.FaceCount; f++)
        {
            ReadOnlySpan<int> face = m.Face(f);
            int c = Find(face[0]);
            faces.TryGetValue(c, out int fc);
            faces[c] = fc + 1;
            if (!verts.TryGetValue(c, out HashSet<int>? vs))
            {
                vs = new HashSet<int>();
                verts[c] = vs;
                edges[c] = new HashSet<long>();
            }

            for (int j = 0; j < face.Length; j++)
            {
                int a = face[j], b = face[(j + 1) % face.Length];
                vs.Add(a);
                edges[c].Add(((long)System.Math.Min(a, b) * m.VertexCount) + System.Math.Max(a, b));
            }
        }

        var result = new List<(int, int, int, int)>();
        foreach ((int c, int fcount) in faces)
        {
            int v = verts[c].Count, e = edges[c].Count;
            result.Add((v, e, fcount, v - e + fcount));
        }

        return result;
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

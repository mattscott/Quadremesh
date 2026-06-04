using Quadremesh.Core.Acceleration;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

/// <summary>
/// Plan 08 kill-point isolation: feed our <see cref="NetworkSimplex"/> port the EXACT arc graph LEMON
/// received (dumped by the instrumented oracle, QF_NSDUMP → <c>state/&lt;mesh&gt;/ns_0.bin</c>) and compare
/// the resulting per-component flow delta bit-for-bit. This separates the NS algorithm from our graph
/// construction (<c>SolveFaithful</c>): if this matches but the end-to-end flow doesn't, the bug is in our
/// construction; if this differs, the bug is in the NS port itself (and the first-diff localizes it).
/// </summary>
public class NetworkSimplexTests
{
    private readonly ITestOutputHelper _output;

    public NetworkSimplexTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void MatchesReferenceArcs(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string path = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "ns_0.bin");
        if (!File.Exists(path))
        {
            _output.WriteLine($"SKIP {mesh}: no ns_0.bin (run oracle with QF_NSDUMP).");
            return;
        }

        using var r = new BinaryReader(File.OpenRead(path));
        int nodeNum = r.ReadInt32();
        int rcnt = r.ReadInt32();
        var rx = new int[rcnt];
        var ry = new int[rcnt];
        var rc = new int[rcnt];
        var rrc = new int[rcnt];
        var rv = new int[rcnt];
        var rcst = new int[rcnt];
        var rfwd = new int[rcnt];
        var rrev = new int[rcnt];
        int maxVar = -1;
        for (int i = 0; i < rcnt; i++)
        {
            rx[i] = r.ReadInt32();
            ry[i] = r.ReadInt32();
            rc[i] = r.ReadInt32();
            rrc[i] = r.ReadInt32();
            rv[i] = r.ReadInt32();
            rcst[i] = r.ReadInt32();
            rfwd[i] = r.ReadInt32();
            rrev[i] = r.ReadInt32();
            maxVar = System.Math.Max(maxVar, rv[i]);
        }

        int refMaxflow = r.ReadInt32();

        var ns = new NetworkSimplex();
        ns.Resize(nodeNum);
        for (int i = 0; i < rcnt; i++)
        {
            ns.AddEdge(rx[i], ry[i], rc[i], rrc[i], rv[i], rcst[i]);
        }

        long myMaxflow = ns.Compute();

        var myDelta = new int[maxVar + 1];
        ns.ApplyTo(myDelta);

        // Reference per-component delta from the dumped per-arc flow: applyTo does edge_diff[var] -= sgn·flow
        // (fwd sgn +1 → −flowFwd, rev sgn −1 → +flowRev).
        var refDelta = new int[maxVar + 1];
        for (int i = 0; i < rcnt; i++)
        {
            if (rv[i] >= 0)
            {
                refDelta[rv[i]] += -rfwd[i] + rrev[i];
            }
        }

        int agree = 0, firstDiff = -1;
        for (int k = 0; k <= maxVar; k++)
        {
            if (myDelta[k] == refDelta[k])
            {
                agree++;
            }
            else if (firstDiff < 0)
            {
                firstDiff = k;
            }
        }

        _output.WriteLine(
            $"{mesh}: nodes={nodeNum} arcs={rcnt} maxflow ours={myMaxflow} ref={refMaxflow} | " +
            $"componentDelta match {agree}/{maxVar + 1} ({100.0 * agree / (maxVar + 1):F2}%)" +
            (firstDiff >= 0 ? $" firstDiff@var={firstDiff} ours={myDelta[firstDiff]} ref={refDelta[firstDiff]}" : " — BIT-EXACT"));

        Assert.Equal(refMaxflow, (int)myMaxflow); // the max-flow value is unique; must match
    }

    /// <summary>
    /// Diffs OUR SolveFaithful first-iteration arc graph against the reference's dumped ns_0.bin — to find
    /// where our construction (Balance / arc order / caps) diverges from optimize_integer_constraints.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void ConstructionMatchesReferenceArcs(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string nsPath = Path.Combine(dir, "ns_0.bin");
        if (!File.Exists(nsPath) || !File.Exists(Path.Combine(dir, "state.bin")))
        {
            _output.WriteLine($"SKIP {mesh}: missing ns_0.bin or state dump.");
            return;
        }

        // Reference arcs from ns_0.bin (x, y, c, rc, v, cst).
        using var r = new BinaryReader(File.OpenRead(nsPath));
        int refNodes = r.ReadInt32();
        int rcnt = r.ReadInt32();
        var refArcs = new int[rcnt][];
        for (int i = 0; i < rcnt; i++)
        {
            refArcs[i] = new[] { r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32() };
            r.ReadInt32();
            r.ReadInt32(); // skip flowFwd/flowRev
        }

        // Our arcs from the faithful field + integer setup fed the reference's raw integer state.
        RefHierarchyDump d = RefHierarchyDump.Load(Path.Combine(dir, "state.bin"));
        RefIntegerDump rawInt = RefIntegerDump.LoadRaw(Path.Combine(dir, "state_int.bin"));
        (_, int[][] orients) = RefIntegerDump.LoadPost(Path.Combine(dir, "state_int_post.bin"));
        int[][] faces = d.Faces();
        var flat = new int[faces.Length * 3];
        for (int f = 0; f < faces.Length; f++)
        {
            flat[(f * 3) + 0] = faces[f][0];
            flat[(f * 3) + 1] = faces[f][1];
            flat[(f * 3) + 2] = faces[f][2];
        }

        var he = Quadremesh.Core.Mesh.HalfEdgeMesh.FromTriangleMesh(new Quadremesh.Core.Mesh.TriangleMesh(d.V[0], flat));
        var scale = new double[d.V[0].Length];
        Array.Fill(scale, d.Scale);
        var solver = new Quadremesh.Core.Extraction.QuadriFlowSolver(
            he, d.Q[0], d.N[0], d.O[0], scale, faces, rawInt.FaceEdgeIds, rawInt.EdgeDiff,
            rawInt.EdgeValues, orients, new HashSet<int>(d.Singularities.Keys));
        (int ourNodes, List<int[]> ourArcs) = solver.DebugFaithfulFirstArcs();

        // Reconstruct initial[c] (per-constraint divergence) from the supply/demand arcs (v == -1).
        int constraintCount = refNodes - 2;
        int sinkNode = refNodes - 1;
        int[] InitialFrom(IReadOnlyList<int[]> arcs)
        {
            var init = new int[constraintCount];
            foreach (int[] a in arcs)
            {
                if (a[4] != -1)
                {
                    continue; // conservation arc
                }

                if (a[0] == 0)
                {
                    init[a[1] - 1] = a[2];      // supply: source(0) -> c+1, cap = initial
                }
                else if (a[1] == sinkNode)
                {
                    init[a[0] - 1] = -a[2];     // demand: c+1 -> sink, cap = -initial
                }
            }

            return init;
        }

        int[] ourInit = InitialFrom(ourArcs);
        int[] refInit = InitialFrom(refArcs);
        int diffs = 0, firstC = -1;
        for (int c = 0; c < constraintCount; c++)
        {
            if (ourInit[c] != refInit[c])
            {
                diffs++;
                if (firstC < 0)
                {
                    firstC = c;
                }
            }
        }

        _output.WriteLine($"{mesh}: nodes ours={ourNodes} ref={refNodes} | arcs ours={ourArcs.Count} ref={rcnt}");
        _output.WriteLine($"  initial[] differs at {diffs} constraints" +
            (firstC >= 0 ? $"; first c={firstC} (face {firstC / 2} dim {firstC % 2}) ours={ourInit[firstC]} ref={refInit[firstC]}" : " — INITIAL BIT-EXACT"));
    }

    /// <summary>
    /// Plan 08 end-to-end: the full faithful pipeline with the bit-exact flow, at the dump-matched resolution
    /// (-f 5000). If happy is watertight genus-6 here, the bit-exact flow closes it and the benchmark's holes
    /// are purely a resolution mismatch. Gated QUADREMESH_INTDIAG=1 (slow on dense meshes).
    /// </summary>
    [Theory]
    [InlineData("happy")]
    [InlineData("armadillo")]
    [InlineData("fandisk")]
    public void FaithfulFlowEndToEnd(string mesh)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_INTDIAG") != "1")
        {
            _output.WriteLine($"SKIP {mesh}: set QUADREMESH_INTDIAG=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string inputPath = Path.Combine(root!, "tests", "corpus", mesh + ".obj");
        if (!File.Exists(inputPath))
        {
            inputPath = Path.Combine(root!, "tests", "corpus", "external", mesh + ".obj");
        }

        Quadremesh.Core.Mesh.TriangleMesh input = Quadremesh.IO.ObjReader.ReadTriangles(inputPath);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        Quadremesh.Core.Mesh.QuadMesh q;
        try
        {
            q = Quadremesh.Core.Faithful.FaithfulPipeline.Remesh(input, 5000, faithfulFlow: true);
        }
        catch (Exception ex)
        {
            _output.WriteLine($"{mesh}: EXCEPTION {ex.GetType().Name}: {ex.Message}");
            throw;
        }

        var he = Quadremesh.Core.Mesh.HalfEdgeMesh.FromQuadMesh(q);
        int bd = 0;
        for (int e = 0; e < he.EdgeCount; e++)
        {
            if (he.Twin(he.EdgeHalfEdge(e)) < 0)
            {
                bd++;
            }
        }

        int chi = he.VertexCount - he.EdgeCount + he.FaceCount;
        _output.WriteLine($"{mesh}: -f5000 faithfulFlow → {q.FaceCount}q boundaryEdges={bd} χ={chi} " +
            $"{(bd == 0 ? "WATERTIGHT" : "HOLES")} {sw.ElapsedMilliseconds}ms");
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

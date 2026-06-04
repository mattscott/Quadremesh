using Quadremesh.Core.Extraction;
using Quadremesh.Core.Faithful;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

/// <summary>
/// Phase-0 validation harness (plan 07 §4): diff our per-stage state against dumps from the instrumented
/// QuadriFlow reference. This is the gate the bottom-up field re-port (Phases 1-4) is validated against —
/// goldens alone are insufficient because intermediate states are PROVEN to regress them.
///
/// Comparisons are modulo the stage's invariance: orientation (4-RoSy) matches modulo 90° rotation about
/// the normal; positions modulo lattice translation; hierarchy modulo a node permutation recovered from
/// mToLower. Until the oracle dumps exist (the other half of Phase 0) AND the matching faithful stage is
/// ported, these SKIP with a message naming exactly what's missing. Opt-in via QUADREMESH_STATEMATCH=1.
///
/// Dump layout (written by the instrumented oracle): tests/golden/quadriflow/state/&lt;mesh&gt;/&lt;stage&gt;.bin
/// e.g. sphere/orientation.bin (a 3xN double matrix = mQ[0]).
/// </summary>
public class StateMatchTests
{
    private readonly ITestOutputHelper _output;

    public StateMatchTests(ITestOutputHelper output) => _output = output;

    /// <summary>
    /// Verifies the reader against a REAL reference dump: loads the full hierarchy state and asserts the
    /// cross-array structural invariants (per-level counts agree; coarsening maps line up; fields are
    /// per-vertex; singularities index valid faces). This proves the C# harness ingests the reference
    /// bytes correctly — the foundation every Phase 1-4 gate builds on. SKIPs if the dump is absent
    /// (regenerate via the instrumented oracle: quadriflow -mcf -seed 0 -dump &lt;dir&gt; -i &lt;mesh&gt; -f 5000).
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void ReferenceDumpLoads(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dump = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dump))
        {
            _output.WriteLine($"SKIP {mesh}: no dump at {dump}. Regenerate with the instrumented oracle (-dump).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dump);
        int faces = d.F.GetLength(1);
        _output.WriteLine(
            $"{mesh}: scale={d.Scale:F5} levels={d.LevelCount} faces={faces} finestV={d.V[0].Length} " +
            $"coarsestV={d.V[^1].Length} singularities={d.Singularities.Count}");

        Assert.True(d.LevelCount > 1, "expected a multi-level hierarchy");
        // mV/mN/mA/mQ/mO/mAdj are per-level (LevelCount); mToLower/mToUpper are inter-level maps
        // (LevelCount-1 of them — one per downsample step).
        Assert.Equal(d.LevelCount - 1, d.ToLower.Length);
        Assert.Equal(d.LevelCount - 1, d.ToUpper.Length);
        for (int l = 0; l < d.LevelCount; l++)
        {
            int n = d.V[l].Length;
            Assert.Equal(n, d.N[l].Length);
            Assert.Equal(n, d.A[l].Length);
            Assert.Equal(n, d.Q[l].Length);
            Assert.Equal(n, d.O[l].Length);
            Assert.Equal(n, d.Adj[l].Length);

            // mToLower maps each node at level l to a parent at level l+1; mToUpper is 2 × nCoarse.
            if (l + 1 < d.LevelCount)
            {
                Assert.Equal(n, d.ToLower[l].Length);
                int coarse = d.V[l + 1].Length;
                Assert.Equal(coarse, d.ToUpper[l].GetLength(1));
                foreach (int parent in d.ToLower[l])
                {
                    Assert.InRange(parent, 0, coarse - 1);
                }
            }
        }

        // Orientation field is unit-length in the tangent plane; sanity-check it's normalized.
        foreach (Vector3 q in d.Q[0])
        {
            Assert.InRange(q.Length, 0.99, 1.01);
        }

        // Singularities index real faces with QuadriFlow's valence codes (1 ⇒ valence 3, 3 ⇒ valence 5).
        foreach ((int face, int code) in d.Singularities)
        {
            Assert.InRange(face, 0, faces - 1);
            Assert.True(code == 1 || code == 3, $"unexpected singularity code {code}");
        }
    }

    /// <summary>
    /// Phase 1a gate: our faithful angle-weighted normals + mixed-Voronoi areas must reproduce the
    /// reference's mN[0]/mA[0] on the SAME finest mesh (taken straight from the dump's mV[0]+mF, so the
    /// pre-pass divergence is factored out and only the attribute math is under test).
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void AttributesMatchReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dumpPath))
        {
            _output.WriteLine($"SKIP {mesh}: no dump at {dumpPath} (regenerate via instrumented oracle).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        DirectedEdgeMesh m = DirectedEdgeMesh.Build(d.V[0], d.Faces());
        Vector3[] normals = m.ComputeSmoothNormal();
        double[] area = m.ComputeVertexArea();

        Vector3[] refN = d.N[0];
        double[] refA = d.A[0];
        Assert.Equal(refN.Length, normals.Length);
        Assert.Equal(refA.Length, area.Length);

        double worstNormalDeg = 0.0;
        double worstAreaRel = 0.0;
        for (int v = 0; v < normals.Length; v++)
        {
            double dot = System.Math.Clamp(Vector3.Dot(normals[v], refN[v]), -1.0, 1.0);
            worstNormalDeg = System.Math.Max(worstNormalDeg, System.Math.Acos(dot) * 180.0 / System.Math.PI);

            double denom = System.Math.Max(System.Math.Abs(refA[v]), 1e-12);
            worstAreaRel = System.Math.Max(worstAreaRel, System.Math.Abs(area[v] - refA[v]) / denom);
        }

        _output.WriteLine($"{mesh}: worst normal deviation={worstNormalDeg:E3} deg, worst area rel err={worstAreaRel:E3}");
        Assert.True(worstNormalDeg < 1e-3, $"normals off by {worstNormalDeg} deg");
        Assert.True(worstAreaRel < 1e-6, $"area off by {worstAreaRel} relative");
    }

    /// <summary>
    /// Phase 1b gate: our angular 1-ring adjacency (dedge fan order) must reproduce the reference's
    /// mAdj[0] neighbour lists EXACTLY (same ids, same order) on the dump's finest mesh — the order feeds
    /// the coarsening stable-sort, so a rotation or reordering would silently change the hierarchy.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void AdjacencyMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dumpPath))
        {
            _output.WriteLine($"SKIP {mesh}: no dump at {dumpPath} (regenerate via instrumented oracle).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        DirectedEdgeMesh m = DirectedEdgeMesh.Build(d.V[0], d.Faces());
        int[][] ours = m.GenerateAdjacency();
        int[][] refAdj = d.Adj[0];

        Assert.Equal(refAdj.Length, ours.Length);
        int mismatches = 0;
        int firstBad = -1;
        for (int v = 0; v < ours.Length; v++)
        {
            if (!ours[v].AsSpan().SequenceEqual(refAdj[v]))
            {
                if (firstBad < 0)
                {
                    firstBad = v;
                }

                mismatches++;
            }
        }

        if (mismatches > 0)
        {
            _output.WriteLine(
                $"{mesh}: {mismatches}/{ours.Length} vertices mismatch. v{firstBad} ours=[{string.Join(",", ours[firstBad])}] " +
                $"ref=[{string.Join(",", refAdj[firstBad])}]");
        }
        else
        {
            _output.WriteLine($"{mesh}: all {ours.Length} adjacency lists match exactly (ids + order).");
        }

        Assert.Equal(0, mismatches);
    }

    /// <summary>
    /// Phase 1c gate: our faithful priority coarsening must reproduce the reference's whole hierarchy.
    /// Fed the dump's EXACT finest mN[0]/mA[0]/mAdj[0] (so dp*ratio is computed from identical doubles and
    /// the result is bit-reproducible), every coarser level's mV/mN/mA/mToLower/mToUpper/mAdj must match.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void HierarchyMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dumpPath))
        {
            _output.WriteLine($"SKIP {mesh}: no dump at {dumpPath} (regenerate via instrumented oracle).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        FaithfulHierarchy h = FaithfulHierarchy.Build(d.V[0], d.N[0], d.A[0], d.Adj[0]);

        Assert.Equal(d.LevelCount, h.LevelCount);

        double worstV = 0.0, worstN = 0.0, worstA = 0.0;
        int adjMismatch = 0, mapMismatch = 0;
        for (int l = 0; l < h.LevelCount; l++)
        {
            Assert.Equal(d.V[l].Length, h.V[l].Length);
            for (int i = 0; i < h.V[l].Length; i++)
            {
                worstV = System.Math.Max(worstV, (h.V[l][i] - d.V[l][i]).Length);
                worstN = System.Math.Max(worstN, (h.N[l][i] - d.N[l][i]).Length);
                worstA = System.Math.Max(worstA, System.Math.Abs(h.A[l][i] - d.A[l][i]));
                if (!h.Adj[l][i].AsSpan().SequenceEqual(d.Adj[l][i]))
                {
                    adjMismatch++;
                }
            }

            if (l + 1 < h.LevelCount)
            {
                for (int i = 0; i < h.ToLower[l].Length; i++)
                {
                    if (h.ToLower[l][i] != d.ToLower[l][i])
                    {
                        mapMismatch++;
                    }
                }

                int coarse = d.V[l + 1].Length;
                for (int i = 0; i < coarse; i++)
                {
                    if (h.ToUpper[l][0, i] != d.ToUpper[l][0, i] || h.ToUpper[l][1, i] != d.ToUpper[l][1, i])
                    {
                        mapMismatch++;
                    }
                }
            }
        }

        _output.WriteLine(
            $"{mesh}: levels={h.LevelCount} worstV={worstV:E2} worstN={worstN:E2} worstA={worstA:E2} " +
            $"adjMismatch={adjMismatch} mapMismatch={mapMismatch}");

        Assert.Equal(0, mapMismatch);   // to_lower / to_upper are integer decisions — must be exact
        Assert.Equal(0, adjMismatch);   // coarse adjacency ids + order — must be exact
        Assert.True(worstV < 1e-9, $"position off by {worstV}");
        Assert.True(worstN < 1e-9, $"normal off by {worstN}");
        Assert.True(worstA < 1e-9, $"area off by {worstA}");
    }

    /// <summary>
    /// Phase 1d gate: our deterministic graph coloring must reproduce the reference's mPhases at every
    /// level (same colour count, same per-colour membership + order). Colouring runs on the dump's own
    /// adjacency to isolate it from the (separately validated) hierarchy build.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void ColoringMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dumpPath))
        {
            _output.WriteLine($"SKIP {mesh}: no dump at {dumpPath} (regenerate via instrumented oracle).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);

        // mPhases is sized MAX_DEPTH+1 (26) in the Hierarchy ctor; only the first LevelCount entries are
        // the real per-level colorings — the tail is padded empty.
        Assert.True(d.Phases.Length >= d.LevelCount);

        int colorCountMismatch = 0, memberMismatch = 0;
        for (int l = 0; l < d.LevelCount; l++)
        {
            int[][] ours = FaithfulHierarchy.GenerateColoring(d.Adj[l]);
            int[][] refPhases = d.Phases[l];
            if (ours.Length != refPhases.Length)
            {
                colorCountMismatch++;
                continue;
            }

            for (int c = 0; c < ours.Length; c++)
            {
                if (!ours[c].AsSpan().SequenceEqual(refPhases[c]))
                {
                    memberMismatch++;
                }
            }
        }

        _output.WriteLine(
            $"{mesh}: levels={d.LevelCount} colorCountMismatch={colorCountMismatch} memberMismatch={memberMismatch}");
        Assert.Equal(0, colorCountMismatch);
        Assert.Equal(0, memberMismatch);
    }

    /// <summary>
    /// Phase 1e gate: our faithful pre-pass (NormalizeMesh + split-only subdivide) run on the RAW input
    /// .obj must reproduce the reference's finest mesh mV[0]/mF exactly — the one piece that can't be
    /// validated in isolation, since it produces the mesh every other stage consumes. Closing this makes
    /// the whole faithful Initialize end-to-end faithful.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void PrepassReproducesReferenceMesh(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        string inputPath = Path.Combine(root!, "tests", "corpus", mesh + ".obj");
        if (!File.Exists(inputPath))
        {
            inputPath = Path.Combine(root!, "tests", "corpus", "external", mesh + ".obj");
        }

        if (!File.Exists(dumpPath) || !File.Exists(inputPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing dump or input .obj.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        (Vector3[] inV, int[][] inF) = LoadLikeQuadriflow(inputPath);
        FaithfulPrepass p = FaithfulPrepass.Run(inV, inF, 5000); // -f 5000, matching the dump's run

        _output.WriteLine(
            $"{mesh}: scale ours={p.Scale:E6} ref={d.Scale:E6}; verts ours={p.V.Length} ref={d.V[0].Length}; " +
            $"faces ours={p.F.Length} ref={d.F.GetLength(1)}");

        Assert.True(System.Math.Abs(p.Scale - d.Scale) < 1e-12, $"scale {p.Scale} vs {d.Scale}");
        Assert.Equal(d.V[0].Length, p.V.Length);
        int[][] refF = d.Faces();
        Assert.Equal(refF.Length, p.F.Length);

        double worstV = 0.0;
        for (int i = 0; i < p.V.Length; i++)
        {
            worstV = System.Math.Max(worstV, (p.V[i] - d.V[0][i]).Length);
        }

        int faceMismatch = 0;
        for (int f = 0; f < p.F.Length; f++)
        {
            if (!p.F[f].AsSpan().SequenceEqual(refF[f]))
            {
                faceMismatch++;
            }
        }

        _output.WriteLine($"{mesh}: worst vertex deviation={worstV:E3}, face mismatches={faceMismatch}");
        Assert.True(worstV < 1e-12, $"vertex off by {worstV}");
        Assert.Equal(0, faceMismatch);
    }

    /// <summary>
    /// Phase 1 capstone: the WHOLE faithful Initialize chained end-to-end from the raw .obj
    /// (loader → prepass → dedge attributes + adjacency → coarsening) must reproduce the reference's
    /// entire hierarchy. This is the integration check that the stages compose, not just pass in isolation.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void FullInitializeMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        string inputPath = Path.Combine(root!, "tests", "corpus", mesh + ".obj");
        if (!File.Exists(inputPath))
        {
            inputPath = Path.Combine(root!, "tests", "corpus", "external", mesh + ".obj");
        }

        if (!File.Exists(dumpPath) || !File.Exists(inputPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing dump or input .obj.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        (Vector3[] inV, int[][] inF) = LoadLikeQuadriflow(inputPath);
        FaithfulPrepass p = FaithfulPrepass.Run(inV, inF, 5000);
        DirectedEdgeMesh dm = DirectedEdgeMesh.Build(p.V, p.F);
        FaithfulHierarchy h = FaithfulHierarchy.Build(
            p.V, dm.ComputeSmoothNormal(), dm.ComputeVertexArea(), dm.GenerateAdjacency());

        Assert.Equal(d.LevelCount, h.LevelCount);
        double worstV = 0.0, worstN = 0.0, worstA = 0.0;
        int adjMismatch = 0, mapMismatch = 0;
        for (int l = 0; l < h.LevelCount; l++)
        {
            for (int i = 0; i < h.V[l].Length; i++)
            {
                worstV = System.Math.Max(worstV, (h.V[l][i] - d.V[l][i]).Length);
                worstN = System.Math.Max(worstN, (h.N[l][i] - d.N[l][i]).Length);
                worstA = System.Math.Max(worstA, System.Math.Abs(h.A[l][i] - d.A[l][i]));
                if (!h.Adj[l][i].AsSpan().SequenceEqual(d.Adj[l][i]))
                {
                    adjMismatch++;
                }
            }

            if (l + 1 < h.LevelCount)
            {
                for (int i = 0; i < h.ToLower[l].Length; i++)
                {
                    if (h.ToLower[l][i] != d.ToLower[l][i])
                    {
                        mapMismatch++;
                    }
                }
            }
        }

        _output.WriteLine(
            $"{mesh}: levels={h.LevelCount} worstV={worstV:E2} worstN={worstN:E2} worstA={worstA:E2} " +
            $"adjMismatch={adjMismatch} mapMismatch={mapMismatch}");

        Assert.Equal(0, mapMismatch);
        Assert.Equal(0, adjMismatch);
        Assert.True(worstV < 1e-9, $"position off by {worstV}");
        Assert.True(worstN < 1e-5, $"normal off by {worstN}");
    }

    /// <summary>
    /// End-to-end field gate: the FROM-SCRATCH <see cref="FaithfulPipeline.ComputeField"/> (the path the
    /// pipeline/benchmark actually use, via ObjReader + QuadriFlow re-index) must reproduce the dump's field
    /// (normal, mQ[0] mod 90°, mO[0]) on happy. If it doesn't, the pipeline's holes are an unfaithful
    /// from-scratch field, not the flow.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("happy")]
    public void ComputeFieldMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dumpPath = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        string inputPath = Path.Combine(root!, "tests", "corpus", mesh + ".obj");
        if (!File.Exists(inputPath))
        {
            inputPath = Path.Combine(root!, "tests", "corpus", "external", mesh + ".obj");
        }

        if (!File.Exists(dumpPath) || !File.Exists(inputPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing dump or input.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        Quadremesh.Core.Mesh.TriangleMesh input = Quadremesh.IO.ObjReader.ReadTriangles(inputPath);
        FaithfulPipeline.FaithfulField field = FaithfulPipeline.ComputeField(input, 5000);

        if (field.V.Length != d.V[0].Length)
        {
            _output.WriteLine($"{mesh}: VERTEX COUNT mismatch ours={field.V.Length} ref={d.V[0].Length}");
            Assert.Equal(d.V[0].Length, field.V.Length);
        }

        double worstN = 0, worstO = 0, worstV = 0;
        int qMismatch = 0;
        for (int i = 0; i < field.V.Length; i++)
        {
            worstV = System.Math.Max(worstV, (field.V[i] - d.V[0][i]).Length);
            worstN = System.Math.Max(worstN, (field.Normal[i] - d.N[0][i]).Length);
            worstO = System.Math.Max(worstO, (field.O[i] - d.O[0][i]).Length);
            // Q matches modulo 90° about the normal: best of {q, n×q, -q, -(n×q)} aligned with ref.
            Vector3 q = field.Q0[i], n = field.Normal[i], qr = d.Q[0][i];
            Vector3 qy = Vector3.Cross(n, q);
            double best = System.Math.Max(
                System.Math.Max(System.Math.Abs(Vector3.Dot(q, qr)), System.Math.Abs(Vector3.Dot(qy, qr))), 0);
            if (best < 1 - 1e-4)
            {
                qMismatch++;
            }
        }

        _output.WriteLine($"{mesh}: verts={field.V.Length} worstV={worstV:E2} worstN={worstN:E2} worstO={worstO:E2} qMismatch={qMismatch}");
    }

    /// <summary>
    /// Phase 2 gate: our faithful orientation solve (seeded by the replicated MSVC rand) must reproduce
    /// the reference's mQ[0] (modulo the 4-RoSy 90° symmetry) AND the exact singularity set. Fed the dump's
    /// per-level hierarchy so the field stage is isolated from the (separately validated) Phase 1 build.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void OrientationMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dump = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dump))
        {
            _output.WriteLine($"SKIP {mesh}: no reference dump at {dump} (regenerate via instrumented oracle).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dump);
        FaithfulOrientation.Result res = FaithfulOrientation.Solve(
            d.N, d.Adj, d.AdjWeight, d.Phases, d.ToUpper, d.Scale, d.Faces(), seed: 0);

        double maxDeg = MaxOrientationErrorDegrees(d.Q[0], res.Q0, d.N[0]);

        // Singularity set must match exactly (face id -> valence code).
        int missing = 0, extra = 0, codeMismatch = 0;
        foreach ((int face, int code) in d.Singularities)
        {
            if (!res.Singularities.TryGetValue(face, out int ours))
            {
                missing++;
            }
            else if (ours != code)
            {
                codeMismatch++;
            }
        }

        foreach (int face in res.Singularities.Keys)
        {
            if (!d.Singularities.ContainsKey(face))
            {
                extra++;
            }
        }

        _output.WriteLine(
            $"{mesh}: max orientation error={maxDeg:E3} deg; singularities ours={res.Singularities.Count} " +
            $"ref={d.Singularities.Count} missing={missing} extra={extra} codeMismatch={codeMismatch}");

        Assert.Equal(0, missing);
        Assert.Equal(0, extra);
        Assert.Equal(0, codeMismatch);
        Assert.True(maxDeg < 1e-3, $"orientation field off by {maxDeg} deg");
    }

    /// <summary>
    /// Phase 4b-i gate: our face_edgeOrients comb (BuildIntegerConstraints: local orients from the
    /// orientation ranks + DisjointOrientTree merge into one frame) must reproduce the dumped
    /// face_edgeOrients exactly. This validates the comb (and transitively our DisjointOrientTree).
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void FaceEdgeOrientsMatchReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string dumpPath = Path.Combine(dir, "state.bin");
        string intPath = Path.Combine(dir, "state_int.bin");
        string postPath = Path.Combine(dir, "state_int_post.bin");
        if (!File.Exists(dumpPath) || !File.Exists(intPath) || !File.Exists(postPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing int dumps.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        RefIntegerDump rawInt = RefIntegerDump.LoadRaw(intPath);
        (_, int[][] refOrients) = RefIntegerDump.LoadPost(postPath);

        int[][] ours = FaithfulIntegerSolve.ComputeFaceEdgeOrients(
            d.N[0], d.Q[0], d.Faces(), rawInt.FaceEdgeIds, rawInt.EdgeDiff.Length,
            new HashSet<int>(d.Singularities.Keys));

        int mismatch = 0;
        for (int f = 0; f < ours.Length; f++)
        {
            if (!ours[f].AsSpan().SequenceEqual(refOrients[f]))
            {
                mismatch++;
            }
        }

        _output.WriteLine($"{mesh}: faces={ours.Length} faceEdgeOrientMismatch={mismatch}");
        Assert.Equal(0, mismatch);
    }

    /// <summary>
    /// Phase 4b characterization: the flow's invariant. The reference's post-flow edge_diff must have ZERO
    /// divergence at every non-(orientation-)singular face — that is the property any faithful flow port
    /// must achieve (the exact integer assignment is non-unique, so this invariant, not bit-identity, is the
    /// gate). Also reports how much divergence the flow had to cancel (raw) and the maxDiff it kept bounded.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void FlowDivergenceInvariant(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string dumpPath = Path.Combine(dir, "state.bin");
        string intPath = Path.Combine(dir, "state_int.bin");
        string postPath = Path.Combine(dir, "state_int_post.bin");
        if (!File.Exists(dumpPath) || !File.Exists(intPath) || !File.Exists(postPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing int dumps.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        RefIntegerDump rawInt = RefIntegerDump.LoadRaw(intPath);
        (Vec2i[] postDiff, int[][] orients) = RefIntegerDump.LoadPost(postPath);
        int[][] faceEdgeIds = rawInt.FaceEdgeIds;

        int Divergence(Vec2i[] edgeDiff, int f, out Vec2i div)
        {
            div = new Vec2i(0, 0);
            for (int j = 0; j < 3; j++)
            {
                div += Lattice2D.Rshift90(edgeDiff[faceEdgeIds[f][j]], orients[f][j]);
            }

            return d.Singularities.ContainsKey(f) ? 0 : (div.X == 0 && div.Y == 0 ? 0 : 1);
        }

        int rawNonZero = 0, postNonZero = 0, rawMax = 0, postMax = 0;
        for (int e = 0; e < rawInt.EdgeDiff.Length; e++)
        {
            rawMax = System.Math.Max(rawMax, System.Math.Max(System.Math.Abs(rawInt.EdgeDiff[e].X), System.Math.Abs(rawInt.EdgeDiff[e].Y)));
            postMax = System.Math.Max(postMax, System.Math.Max(System.Math.Abs(postDiff[e].X), System.Math.Abs(postDiff[e].Y)));
        }

        for (int f = 0; f < faceEdgeIds.Length; f++)
        {
            rawNonZero += Divergence(rawInt.EdgeDiff, f, out _);
            postNonZero += Divergence(postDiff, f, out _);
        }

        _output.WriteLine(
            $"{mesh}: faces={faceEdgeIds.Length} rawDivergentFaces={rawNonZero} postDivergentFaces={postNonZero} " +
            $"rawMaxDiff={rawMax} postMaxDiff={postMax}");

        // The flow must leave zero divergence at every non-singular face.
        Assert.Equal(0, postNonZero);
    }

    /// <summary>
    /// Diagnostic: feed OUR flow (QuadriFlowSolver.Balance + Solve) the reference's bit-exact raw integer
    /// state (edge_diff + comb from the dump) and measure what solution it lands on — post divergence (must
    /// be 0) and postMaxDiff. The reference (-mcf NetworkSimplex) keeps postMaxDiff=2; if ours (SSP MCF) is
    /// higher, that quantifies the flow-solution gap that leaves happy genus-7 + triangular holes while the
    /// field/edge_diff/comb are all bit-exact.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void OurFlowOnReferenceInput(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string dumpPath = Path.Combine(dir, "state.bin");
        string intPath = Path.Combine(dir, "state_int.bin");
        string postPath = Path.Combine(dir, "state_int_post.bin");
        if (!File.Exists(dumpPath) || !File.Exists(intPath) || !File.Exists(postPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing int dumps.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        RefIntegerDump rawInt = RefIntegerDump.LoadRaw(intPath);
        (Vec2i[] refPost, int[][] orients) = RefIntegerDump.LoadPost(postPath);

        int[][] faces = d.Faces();
        var flat = new int[faces.Length * 3];
        for (int f = 0; f < faces.Length; f++)
        {
            flat[(f * 3) + 0] = faces[f][0];
            flat[(f * 3) + 1] = faces[f][1];
            flat[(f * 3) + 2] = faces[f][2];
        }

        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(new TriangleMesh(d.V[0], flat));
        var scale = new double[d.V[0].Length];
        Array.Fill(scale, d.Scale);

        var solver = new QuadriFlowSolver(
            he, d.Q[0], d.N[0], d.O[0], scale, faces, rawInt.FaceEdgeIds, rawInt.EdgeDiff,
            rawInt.EdgeValues, orients, new HashSet<int>(d.Singularities.Keys));
        solver.Solve();

        int ourMax = 0, ourDivergent = 0;
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            Vec2i de = solver.EdgeDiff(e);
            ourMax = System.Math.Max(ourMax, System.Math.Max(System.Math.Abs(de.X), System.Math.Abs(de.Y)));
        }

        for (int f = 0; f < solver.FaceCount; f++)
        {
            if (!solver.IsSingular(f) && !solver.FaceDivergence(f).Equals(new Vec2i(0, 0)))
            {
                ourDivergent++;
            }
        }

        int refMax = 0, agree = 0;
        long ourL1 = 0, refL1 = 0, rawL1 = 0;
        for (int e = 0; e < refPost.Length; e++)
        {
            refMax = System.Math.Max(refMax, System.Math.Max(System.Math.Abs(refPost[e].X), System.Math.Abs(refPost[e].Y)));
            Vec2i ours = solver.EdgeDiff(e);
            ourL1 += System.Math.Abs(ours.X) + System.Math.Abs(ours.Y);
            refL1 += System.Math.Abs(refPost[e].X) + System.Math.Abs(refPost[e].Y);
            rawL1 += System.Math.Abs(rawInt.EdgeDiff[e].X) + System.Math.Abs(rawInt.EdgeDiff[e].Y);
            if (ours.Equals(refPost[e]))
            {
                agree++;
            }
        }

        _output.WriteLine(
            $"{mesh}: OUR flow postMaxDiff={ourMax} postDivergent={ourDivergent} | REF postMaxDiff={refMax} | " +
            $"edges identical to ref post={agree}/{refPost.Length} ({100.0 * agree / refPost.Length:F1}%) | " +
            $"Σ|diff| raw={rawL1} ref={refL1} ours={ourL1} (ours-ref={ourL1 - refL1})");
    }

    /// <summary>
    /// Plan 08 kill-point: the FAITHFUL flow (<see cref="QuadriFlowSolver.SolveFaithful"/> via the ported
    /// LEMON NetworkSimplex) must reproduce the reference's post-flow edge_diff <b>bit-for-bit</b> — fed the
    /// reference's faithful raw integer state. sphere is the smallest/simplest case; if it matches, the
    /// NetworkSimplex port is sound and happy follows.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void FaithfulFlowMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string dumpPath = Path.Combine(dir, "state.bin");
        string intPath = Path.Combine(dir, "state_int.bin");
        string postPath = Path.Combine(dir, "state_int_post.bin");
        if (!File.Exists(dumpPath) || !File.Exists(intPath) || !File.Exists(postPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing int dumps.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        RefIntegerDump rawInt = RefIntegerDump.LoadRaw(intPath);
        (Vec2i[] refPost, int[][] orients) = RefIntegerDump.LoadPost(postPath);

        int[][] faces = d.Faces();
        var flat = new int[faces.Length * 3];
        for (int f = 0; f < faces.Length; f++)
        {
            flat[(f * 3) + 0] = faces[f][0];
            flat[(f * 3) + 1] = faces[f][1];
            flat[(f * 3) + 2] = faces[f][2];
        }

        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(new TriangleMesh(d.V[0], flat));
        var scale = new double[d.V[0].Length];
        Array.Fill(scale, d.Scale);

        var solver = new QuadriFlowSolver(
            he, d.Q[0], d.N[0], d.O[0], scale, faces, rawInt.FaceEdgeIds, rawInt.EdgeDiff,
            rawInt.EdgeValues, orients, new HashSet<int>(d.Singularities.Keys));
        solver.SolveFaithful();

        int agree = 0, firstDiff = -1, ourMax = 0, divergent = 0;
        long ourChange = 0, refChange = 0; // Σ|post − raw| ≈ the flow's actual objective (total routed change)
        for (int e = 0; e < refPost.Length; e++)
        {
            Vec2i o = solver.EdgeDiff(e);
            Vec2i raw = rawInt.EdgeDiff[e];
            ourChange += System.Math.Abs(o.X - raw.X) + System.Math.Abs(o.Y - raw.Y);
            refChange += System.Math.Abs(refPost[e].X - raw.X) + System.Math.Abs(refPost[e].Y - raw.Y);
            ourMax = System.Math.Max(ourMax, System.Math.Max(System.Math.Abs(o.X), System.Math.Abs(o.Y)));
            if (o.Equals(refPost[e]))
            {
                agree++;
            }
            else if (firstDiff < 0)
            {
                firstDiff = e;
            }
        }

        for (int f = 0; f < solver.FaceCount; f++)
        {
            if (!solver.IsSingular(f) && !solver.FaceDivergence(f).Equals(new Vec2i(0, 0)))
            {
                divergent++;
            }
        }

        _output.WriteLine(
            $"{mesh}: FAITHFUL flow vs ref post = {agree}/{refPost.Length} ({100.0 * agree / refPost.Length:F2}%) " +
            $"postMaxDiff={ourMax} divergent={divergent} Σ|change| ours={ourChange} ref={refChange} (ours-ref={ourChange - refChange})" +
            (firstDiff >= 0 ? $" firstDiff@e={firstDiff} ours={solver.EdgeDiff(firstDiff)} ref={refPost[firstDiff]}" : " — BIT-EXACT"));
    }

    /// <summary>
    /// Decisive isolation: feed the reference's EXACT post-flow edge_diff (+ comb + faithful geometry) into
    /// OUR extraction (align → faithful-coarsen extract → hole-fill) and measure the resulting boundary + χ.
    /// If this is watertight genus-6 on happy, our extraction is faithful and the residual holes come from
    /// the 4% non-unique flow difference (→ NetworkSimplex would fix it). If it still holes, our extraction
    /// (subdivide/FixFlip/extract, no post-solver) is the culprit regardless of the flow.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void ReferencePostFlowExtraction(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string dumpPath = Path.Combine(dir, "state.bin");
        string intPath = Path.Combine(dir, "state_int.bin");
        string postPath = Path.Combine(dir, "state_int_post.bin");
        if (!File.Exists(dumpPath) || !File.Exists(intPath) || !File.Exists(postPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing int dumps.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        RefIntegerDump rawInt = RefIntegerDump.LoadRaw(intPath);
        (Vec2i[] refPost, int[][] orients) = RefIntegerDump.LoadPost(postPath);

        int[][] faces = d.Faces();
        var flat = new int[faces.Length * 3];
        for (int f = 0; f < faces.Length; f++)
        {
            flat[(f * 3) + 0] = faces[f][0];
            flat[(f * 3) + 1] = faces[f][1];
            flat[(f * 3) + 2] = faces[f][2];
        }

        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(new TriangleMesh(d.V[0], flat));
        var scale = new double[d.V[0].Length];
        Array.Fill(scale, d.Scale);

        // Build the solver with the reference's ALREADY-SOLVED post-flow edge_diff; do NOT run Solve().
        var solver = new QuadriFlowSolver(
            he, d.Q[0], d.N[0], d.O[0], scale, faces, rawInt.FaceEdgeIds, refPost,
            rawInt.EdgeValues, orients, new HashSet<int>(d.Singularities.Keys));

        IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
        grid.SubdivideEdgeDiff(1);
        grid.FixFlip();
        grid.SubdivideEdgeDiff(1);
        grid.FixFlipSat();
        QuadMesh raw = new QuadriFlowExtractor(grid, faithfulCoarsening: true).Extract();
        QuadMesh filled = QuadHoleFiller.Fill(raw);

        HalfEdgeMesh fhe = HalfEdgeMesh.FromQuadMesh(filled);
        int bd = 0;
        for (int e = 0; e < fhe.EdgeCount; e++)
        {
            if (fhe.Twin(fhe.EdgeHalfEdge(e)) < 0)
            {
                bd++;
            }
        }

        int chi = fhe.VertexCount - fhe.EdgeCount + fhe.FaceCount;
        _output.WriteLine(
            $"{mesh}: ref post-flow → OUR extract: {filled.FaceCount}q boundaryEdges={bd} χ={chi} " +
            $"{(bd == 0 ? "WATERTIGHT" : "HOLES")}");
    }

    /// <summary>
    /// Phase 4a gate: our geometric edge_diff construction (ComputePositionSingularities → BuildEdgeInfo)
    /// from the converged field must reproduce the reference's raw edge graph (edge_diff, face_edgeIds,
    /// edge_values) — the input to the integer flow. Fed the dump's exact field to isolate this stage.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void EdgeDiffMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dir = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh);
        string dumpPath = Path.Combine(dir, "state.bin");
        string intPath = Path.Combine(dir, "state_int.bin");
        if (!File.Exists(dumpPath) || !File.Exists(intPath))
        {
            _output.WriteLine($"SKIP {mesh}: missing state.bin or state_int.bin.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dumpPath);
        RefIntegerDump refInt = RefIntegerDump.LoadRaw(intPath);
        DirectedEdgeMesh dm = DirectedEdgeMesh.Build(d.V[0], d.Faces());

        FaithfulIntegerSolve.EdgeInfo ours = FaithfulIntegerSolve.BuildEdgeDiff(
            d.V[0], d.N[0], d.Q[0], d.O[0], d.Faces(), dm.E2E, d.Scale,
            new HashSet<int>(d.Singularities.Keys));

        Assert.Equal(refInt.EdgeDiff.Length, ours.EdgeDiff.Length);
        Assert.Equal(refInt.FaceEdgeIds.Length, ours.FaceEdgeIds.Length);

        int diffMismatch = 0, idMismatch = 0, valMismatch = 0;
        for (int e = 0; e < ours.EdgeDiff.Length; e++)
        {
            if (!ours.EdgeDiff[e].Equals(refInt.EdgeDiff[e]))
            {
                diffMismatch++;
            }

            if (!ours.EdgeValues[e].Equals(refInt.EdgeValues[e]))
            {
                valMismatch++;
            }
        }

        for (int f = 0; f < ours.FaceEdgeIds.Length; f++)
        {
            if (!ours.FaceEdgeIds[f].AsSpan().SequenceEqual(refInt.FaceEdgeIds[f]))
            {
                idMismatch++;
            }
        }

        _output.WriteLine(
            $"{mesh}: edges={ours.EdgeDiff.Length} edgeDiffMismatch={diffMismatch} " +
            $"faceEdgeIdMismatch={idMismatch} edgeValueMismatch={valMismatch}");

        Assert.Equal(0, valMismatch);
        Assert.Equal(0, idMismatch);
        Assert.Equal(0, diffMismatch);
    }

    /// <summary>
    /// Phase 3 gate: our faithful position field (seeded by the same replicated rand stream) must reproduce
    /// the reference's mO[0], modulo integer lattice translation. The position solve has no transcendentals
    /// and is fed the dump's exact mQ, so the gauge matches and the field should be ~bit-exact.
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    [InlineData("happy")]
    public void PositionMatchesReference(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dump = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dump))
        {
            _output.WriteLine($"SKIP {mesh}: no reference dump at {dump}.");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dump);
        Vector3[] ourO = FaithfulPosition.Solve(
            d.V, d.N, d.Q, d.Adj, d.AdjWeight, d.Phases, d.ToUpper, d.Scale, seed: 0);

        Vector3[] refO = d.O[0];
        double worstDirect = 0.0, worstModLattice = 0.0;
        for (int v = 0; v < ourO.Length; v++)
        {
            Vector3 diff = ourO[v] - refO[v];
            worstDirect = System.Math.Max(worstDirect, diff.Length);

            Vector3 q = d.Q[0][v];
            double ql = q.Length;
            if (ql > 1e-300)
            {
                q /= ql;
            }

            Vector3 t = Vector3.Cross(d.N[0][v], q);
            double kx = System.Math.Round(Vector3.Dot(q, diff) / d.Scale, MidpointRounding.AwayFromZero);
            double ky = System.Math.Round(Vector3.Dot(t, diff) / d.Scale, MidpointRounding.AwayFromZero);
            Vector3 res = diff - (((q * kx) + (t * ky)) * d.Scale);
            worstModLattice = System.Math.Max(worstModLattice, res.Length);
        }

        _output.WriteLine(
            $"{mesh}: scale={d.Scale:E4} worst |O-Oref| direct={worstDirect:E3} modLattice={worstModLattice:E3} " +
            $"(modLattice/scale={worstModLattice / d.Scale:E3})");

        Assert.True(worstModLattice / d.Scale < 1e-6, $"position field off by {worstModLattice} ({worstModLattice / d.Scale} cells)");
    }

    /// <summary>
    /// Max per-vertex angular deviation (degrees) between two 4-RoSy fields, modulo 90° rotation about the
    /// normal. The four representatives of a cross at vertex v are {q, n×q, -q, -(n×q)}; the error is the
    /// angle to the best-aligned representative. 0 ⇒ identical fields up to the quad symmetry.
    /// </summary>
    internal static double MaxOrientationErrorDegrees(Vector3[] reference, Vector3[] ours, Vector3[] normals)
    {
        if (reference.Length != ours.Length || ours.Length != normals.Length)
        {
            throw new ArgumentException("Field length mismatch.");
        }

        double worst = 0.0;
        for (int v = 0; v < reference.Length; v++)
        {
            Vector3 q = ours[v];
            Vector3 perp = Vector3.Cross(normals[v], q); // +90° representative (n×q), per the audit
            double best = System.Math.Max(
                System.Math.Max(Vector3.Dot(reference[v], q), Vector3.Dot(reference[v], perp)),
                System.Math.Max(Vector3.Dot(reference[v], -q), Vector3.Dot(reference[v], -perp)));
            best = System.Math.Clamp(best, -1.0, 1.0);
            double deg = System.Math.Acos(best) * 180.0 / System.Math.PI;
            worst = System.Math.Max(worst, deg);
        }

        return worst;
    }

    /// <summary>
    /// Loads an OBJ exactly as QuadriFlow's loader.cpp does: vertices are re-indexed by first appearance
    /// in the FACE list (deduped by position index; unreferenced positions dropped), and faces reference
    /// those new indices. This labeling — not the file v-order our ObjReader uses — is what the reference
    /// subdivides, and on symmetric meshes it drives the edge-id tie-breaks that decide split order.
    /// </summary>
    private static (Vector3[] V, int[][] F) LoadLikeQuadriflow(string path)
    {
        var ci = System.Globalization.CultureInfo.InvariantCulture;
        var positions = new List<Vector3>();
        var map = new Dictionary<int, int>();  // 1-based position index -> new vertex index
        var order = new List<int>();           // new index -> 1-based position index
        var indices = new List<int>();
        char[] sep = { ' ', '\t' };

        foreach (string raw in File.ReadLines(path))
        {
            string line = raw.Trim();
            if (line.Length < 2)
            {
                continue;
            }

            if (line[0] == 'v' && line[1] == ' ')
            {
                string[] t = line.Split(sep, StringSplitOptions.RemoveEmptyEntries);
                positions.Add(new Vector3(
                    double.Parse(t[1], ci), double.Parse(t[2], ci), double.Parse(t[3], ci)));
            }
            else if (line[0] == 'f' && line[1] == ' ')
            {
                string[] t = line.Split(sep, StringSplitOptions.RemoveEmptyEntries);
                int corners = t.Length - 1;
                var p = new int[corners];
                for (int i = 0; i < corners; i++)
                {
                    p[i] = int.Parse(t[i + 1].Split('/')[0], ci); // first token before '/' = position index
                }

                int[] tri = corners == 4
                    ? new[] { p[0], p[1], p[2], p[3], p[0], p[2] } // quad -> two triangles (loader.cpp)
                    : new[] { p[0], p[1], p[2] };

                foreach (int pp in tri)
                {
                    if (!map.TryGetValue(pp, out int idx))
                    {
                        idx = order.Count;
                        map[pp] = idx;
                        order.Add(pp);
                    }

                    indices.Add(idx);
                }
            }
        }

        var v = new Vector3[order.Count];
        for (int i = 0; i < order.Count; i++)
        {
            v[i] = positions[order[i] - 1];
        }

        var f = new int[indices.Count / 3][];
        for (int i = 0; i < f.Length; i++)
        {
            f[i] = new[] { indices[(i * 3) + 0], indices[(i * 3) + 1], indices[(i * 3) + 2] };
        }

        return (v, f);
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

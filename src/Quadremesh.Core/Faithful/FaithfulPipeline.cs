using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Extraction;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// End-to-end faithful pipeline (plan 07 integration): chains the bit-exact-validated faithful field stack
/// — FaithfulPrepass → DirectedEdgeMesh (attributes + adjacency) → FaithfulHierarchy → FaithfulOrientation
/// → FaithfulPosition — from a raw triangle mesh, then feeds the resulting field (N, Q, O) into the
/// existing QuadriFlowSolver integer solve + IntegerGridMesh align + extraction. The bet: the faithful
/// field (bit-identical to QuadriFlow's, foldover-free) makes dense meshes watertight, where our old field
/// left holes. The MSVC rand seed (0) drives the field init, matching the reference's gauge.
/// </summary>
public static class FaithfulPipeline
{
    /// <summary>
    /// The converged faithful field on the subdivided, normalized field mesh: per-vertex normal,
    /// orientation (Q[0]) and position (O) plus the prepass round-trip transform. The integer solve +
    /// extraction consume this. <see cref="OrientationSingular"/> is the orientation-singularity face set
    /// (Phase 2), needed by the faithful <c>BuildEdgeDiff</c>.
    /// </summary>
    public sealed class FaithfulField
    {
        internal FaithfulField(
            Vector3[] v, int[][] f, int[] e2e, Vector3[] normal, Vector3[] q0, Vector3[] o, double scale,
            FaithfulPrepass prepass, ISet<int> orientationSingular)
        {
            V = v;
            F = f;
            E2E = e2e;
            Normal = normal;
            Q0 = q0;
            O = o;
            Scale = scale;
            Prepass = prepass;
            OrientationSingular = orientationSingular;
        }

        public Vector3[] V { get; }

        public int[][] F { get; }

        /// <summary>Directed-edge twin table (dedge face*3+slot → opposite, −1 on a boundary), from the dedge mesh.</summary>
        public int[] E2E { get; }

        public Vector3[] Normal { get; }

        public Vector3[] Q0 { get; }

        public Vector3[] O { get; }

        public double Scale { get; }

        internal FaithfulPrepass Prepass { get; }

        public ISet<int> OrientationSingular { get; }
    }

    /// <summary>Runs the faithful field stack (prepass → dedge → hierarchy → orientation → position).</summary>
    public static FaithfulField ComputeField(TriangleMesh input, int targetFaces)
    {
        ArgumentNullException.ThrowIfNull(input);

        var rawV = new Vector3[input.VertexCount];
        Array.Copy(input.Positions, rawV, rawV.Length);
        var rawF = new int[input.TriangleCount][];
        for (int t = 0; t < input.TriangleCount; t++)
        {
            (int a, int b, int c) = input.Triangle(t);
            rawF[t] = new[] { a, b, c };
        }

        // Re-index vertices by first-appearance-in-faces and drop unreferenced ones, exactly as QuadriFlow's
        // loader.cpp does (NOT our ObjReader's file order). On symmetric / high-genus meshes the vertex
        // labeling drives the coarsening + field tie-breaks, so matching it is required for a faithful field.
        (Vector3[] inV, int[][] inF) = ReindexLikeQuadriflow(rawV, rawF);

        FaithfulPrepass pre = FaithfulPrepass.Run(inV, inF, targetFaces);
        DirectedEdgeMesh dm = DirectedEdgeMesh.Build(pre.V, pre.F);
        Vector3[] normal = dm.ComputeSmoothNormal();
        double[] area = dm.ComputeVertexArea();
        int[][] adj0 = dm.GenerateAdjacency();
        FaithfulHierarchy h = FaithfulHierarchy.Build(pre.V, normal, area, adj0);

        int levels = h.LevelCount;
        var phases = new int[levels][][];
        for (int l = 0; l < levels; l++)
        {
            phases[l] = FaithfulHierarchy.GenerateColoring(h.Adj[l]);
        }

        Vector3[][] nLv = h.N.ToArray();
        Vector3[][] vLv = h.V.ToArray();
        int[][][] adjId = h.Adj.ToArray();
        double[][][] adjW = h.AdjWeight.ToArray();
        int[][,] toUpper = h.ToUpper.ToArray();

        FaithfulOrientation.Result orient = FaithfulOrientation.Solve(
            nLv, adjId, adjW, phases, toUpper, pre.Scale, pre.F, seed: 0);
        Vector3[] anchor = FaithfulPosition.Solve(
            vLv, nLv, orient.Q, adjId, adjW, phases, toUpper, pre.Scale, seed: 0);

        return new FaithfulField(
            pre.V, pre.F, dm.E2E, normal, orient.Q[0], anchor, pre.Scale, pre,
            new HashSet<int>(orient.Singularities.Keys));
    }

    /// <summary>
    /// Remeshes a triangle mesh to a faithful quad mesh. <paramref name="targetFaces"/> is the field face
    /// budget — QuadriFlow's <c>-f N</c> — NOT the output count: the integer extraction undershoots it
    /// (mesh-dependently; a simple sphere undershoots far more than a dense organic mesh), exactly as
    /// QuadriFlow's <c>-f</c> does. Resolution matters for genus/holes — happy is genus-6 watertight at
    /// <c>-f5000</c> but develops triangular holes at coarser budgets (QuadriFlow does too); pick the budget,
    /// like <c>-f</c>.
    /// </summary>
    public static QuadMesh Remesh(TriangleMesh input, int targetFaces) => Remesh(input, targetFaces, faithfulFlow: true);

    /// <summary>
    /// Remeshes to a faithful quad mesh. When <paramref name="faithfulFlow"/> is true (the default) the integer
    /// flow uses the bit-exact <see cref="QuadriFlowSolver.SolveFaithful"/> (LEMON NetworkSimplex +
    /// mt19937/MSVC-shuffle Balance), which reproduces QuadriFlow's <c>-mcf</c> solution and makes the dense
    /// corpus watertight (6/7 at golden-count). Passing false selects the older SSP
    /// <see cref="QuadriFlowSolver.Solve"/>, kept for comparison.
    /// </summary>
    public static QuadMesh Remesh(TriangleMesh input, int targetFaces, bool faithfulFlow)
    {
        ArgumentNullException.ThrowIfNull(input);

        Stage("computeField:start");
        FaithfulField field = ComputeField(input, targetFaces);
        Stage("computeField:done");
        FaithfulPrepass pre = field.Prepass;
        Vector3[] normal = field.Normal;
        Vector3[] anchor = field.O;

        // --- Existing QuadriFlow integer solve + extraction, fed the faithful field --------------------
        var flatF = new int[pre.F.Length * 3];
        for (int f = 0; f < pre.F.Length; f++)
        {
            flatF[(f * 3) + 0] = pre.F[f][0];
            flatF[(f * 3) + 1] = pre.F[f][1];
            flatF[(f * 3) + 2] = pre.F[f][2];
        }

        var fieldMesh = new TriangleMesh(pre.V, flatF);
        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(fieldMesh);
        var scaleArr = new double[pre.V.Length];
        Array.Fill(scaleArr, pre.Scale);

        // Bit-exact faithful integer setup (validated vs the reference dump): the 4×4-midpoint-search
        // edge_diff + the DisjointOrientTree comb, instead of QuadriFlowSolver's naive round() + rank comb.
        // The solver then only runs Balance + the (single-level) integer flow on this faithful state.
        Stage("buildEdgeDiff:start");
        FaithfulIntegerSolve.EdgeInfo edgeInfo = FaithfulIntegerSolve.BuildEdgeDiff(
            field.V, normal, field.Q0, anchor, field.F, field.E2E, pre.Scale, field.OrientationSingular);
        int[][] faceEdgeOrients = FaithfulIntegerSolve.ComputeFaceEdgeOrients(
            normal, field.Q0, field.F, edgeInfo.FaceEdgeIds, edgeInfo.EdgeDiff.Length, field.OrientationSingular);
        Stage("buildEdgeDiff:done");

        var solver = new QuadriFlowSolver(
            he, field.Q0, normal, anchor, scaleArr, field.F, edgeInfo.FaceEdgeIds, edgeInfo.EdgeDiff,
            edgeInfo.EdgeValues, faceEdgeOrients, field.OrientationSingular);
        // SolveFaithful (plan 08) is bit-exact to QuadriFlow's -mcf post-flow edge_diff; opt-in while it's
        // productionized (armadillo robustness + resolution reconciliation). Default stays the SSP Solve().
        Stage("solve:start");
        if (faithfulFlow)
        {
            solver.SolveFaithful();
        }
        else
        {
            solver.Solve();
        }

        Stage("solve:done");
        IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
        Stage("align:start");
        AlignLatticeFaithful(grid);
        Stage("align:done");

        // Faithful coarsening (plan 07 D10): the connectivity for extraction comes from the reference's
        // DownsampleEdgeGraph(...).back() (QuadriFlow's AdvancedExtractQuad), NOT the ad-hoc vertex union.
        // The ad-hoc dual keys coarse edges by vertex pair, which mis-pairs the diagonals around high-genus
        // handles and leaves unfillable boundary loops — the structural holes on happy (cuts its unpaired
        // diagonals 20→2, boundary 34→10). A wash on the lower-genus meshes (they already extract clean).
        Stage("extract:start");
        QuadMesh raw = new QuadriFlowExtractor(grid, faithfulCoarsening: true).Extract();
        Stage("extract:done");

        // Fill small artifact holes UNCONDITIONALLY, exactly as QuadriFlow's AdvancedExtractQuad calls
        // FixHoles (parametrizer-flip.cpp:516) — it walks every boundary loop and fills only those shorter
        // than QuadHoleFiller.MaxLoopVertices (25, matching QuadriFlow), so genuine open boundaries are left
        // intact. The old `IsClosed` gate skipped filling whenever the field mesh had even one stray boundary
        // half-edge (e.g. happy has 7 on an 81k-vertex closed surface), which left happy's quad holes open.
        Stage("holeFill:start");
        raw = QuadHoleFiller.Fill(raw);
        Stage("holeFill:done");

        Stage("fixValence:start");
        QuadMesh quads = QuadMeshCleanup.FixValence(raw);
        Stage("fixValence:done");

        // Robustness: close any short ODD (e.g. triangular) boundary loop the quad hole-filler can't tile.
        // These only occur at coarse resolution (QuadriFlow leaves them too); the resolution headroom avoids
        // them in practice, so this is usually a no-op. Quad-dominant by a few triangles when it fires.
        quads = QuadHoleFiller.CloseOddBoundaryLoops(quads);

        // The whole field/extraction runs in NormalizeMesh's unit-box space; map vertices back to the
        // input's coordinates before snapping to the original surface (else off-origin meshes collapse).
        var unnorm = new Vector3[quads.VertexCount];
        for (int i = 0; i < unnorm.Length; i++)
        {
            unnorm[i] = (quads.Positions[i] * pre.NormalizeScale) + pre.NormalizeOffset;
        }

        var unnormFaces = new int[quads.FaceCount][];
        for (int f = 0; f < quads.FaceCount; f++)
        {
            unnormFaces[f] = quads.Face(f).ToArray();
        }

        Stage("project:start");
        QuadMesh result = ProjectToSurface(new QuadMesh(unnorm, unnormFaces), Bvh.Build(input));
        Stage("project:done");
        return result;
    }

    /// <summary>When <c>QUADREMESH_STAGELOG</c> names a file path, stage markers are appended to it so a hung
    /// or crashed run still records the last stage it entered (this is how the SSP subdivide OOM was localized).</summary>
    private static readonly string? StageLogPath =
        Environment.GetEnvironmentVariable("QUADREMESH_STAGELOG") is { Length: > 0 } p ? p : null;

    private static bool StageOn => StageLogPath is not null;

    /// <summary>Appends a stage marker with a timestamp and flushes immediately. No-op (and never evaluated —
    /// callers guard interpolated messages with <see cref="StageOn"/>) unless stage logging is enabled.</summary>
    private static void Stage(string name)
    {
        if (StageLogPath is null)
        {
            return;
        }

        System.IO.File.AppendAllText(StageLogPath, $"{DateTime.Now:HH:mm:ss.fff} {name}{Environment.NewLine}");
    }

    /// <summary>
    /// Aligns the lattice to the integer grid in QuadriFlow's order (parametrizer.cpp:110–133):
    /// subdivide → FixFlip → subdivide. Production's <see cref="QuadRemesher.AlignLatticeForExtraction"/>
    /// adds a <i>leading</i> FixFlip to shrink the OLD dirty field's foldover rings before the first
    /// subdivide; the faithful field doesn't need it (and the leading FixFlip badly hurt the ref-order
    /// CAD meshes — rocker-arm's foldovers jumped 60→532). Subdividing first normalizes |edge_diff|≤1 so
    /// FixFlip then operates on a clean ±1 graph. We additionally run <see cref="IntegerGridMesh.FixFlipSat"/>
    /// — the reference gates it behind <c>-sat</c> (off by default) but it only ever removes foldovers, a
    /// strict quality gain (armadillo extraction negArea 259→237) at ~15% runtime on dense meshes.
    /// </summary>
    private static void AlignLatticeFaithful(IntegerGridMesh grid)
    {
        // A well-resolved field needs only a tiny fraction of FaceCount splits to drive offsets to a unit
        // step (≈1% in practice). A non-convergent field — e.g. the SSP flow can't fully resolve the
        // re-indexed faithful field, leaving foldover patches — would otherwise split without bound. Cap
        // both subdivide calls at a fraction of the ORIGINAL field size (computed once, so the bound can't
        // compound across calls as the mesh grows): generous headroom over any legitimate need, but it keeps
        // a pathological field to an input-comparable mesh instead of one so large the downstream hole-fill /
        // cleanup stages exhaust memory (the xyzrgb_dragon OOM was in QuadHoleFiller on a 4.2M-face blowup).
        int subdivideCap = (grid.FaceCount / 2) + 4096;
        if (StageOn)
        {
            Stage($"  subdiv1:start V={grid.VertexCount} F={grid.FaceCount} maxDiff={grid.MaxEdgeDiff()} cap={subdivideCap}");
        }

        grid.SubdivideEdgeDiff(1, subdivideCap);
        if (StageOn)
        {
            Stage($"  subdiv1:done V={grid.VertexCount} F={grid.FaceCount} splits={grid.LastSplitCount} hitGuard={grid.LastHitGuard} peak={grid.LastPeakDiff}");
        }

        grid.FixFlip();
        if (StageOn)
        {
            Stage($"  fixFlip:done V={grid.VertexCount} F={grid.FaceCount}");
        }

        grid.SubdivideEdgeDiff(1, subdivideCap);
        if (StageOn)
        {
            Stage($"  subdiv2:done V={grid.VertexCount} F={grid.FaceCount} splits={grid.LastSplitCount} hitGuard={grid.LastHitGuard} peak={grid.LastPeakDiff}");
        }

        grid.FixFlipSat();
        if (StageOn)
        {
            Stage($"  fixFlipSat:done V={grid.VertexCount} F={grid.FaceCount}");
        }
    }

    /// <summary>
    /// Re-indexes vertices by first-appearance-in-faces and drops unreferenced ones, replicating QuadriFlow's
    /// <c>loader.cpp</c> so the field's labeling-dependent tie-breaks match the reference (memory gotcha:
    /// our <c>ObjReader</c> keeps file v-order, which diverges on symmetric / high-genus meshes).
    /// </summary>
    private static (Vector3[] V, int[][] F) ReindexLikeQuadriflow(Vector3[] v, int[][] f)
    {
        var map = new Dictionary<int, int>(v.Length);
        var order = new List<int>(v.Length);
        var newF = new int[f.Length][];
        for (int i = 0; i < f.Length; i++)
        {
            var tri = new int[f[i].Length];
            for (int j = 0; j < f[i].Length; j++)
            {
                int old = f[i][j];
                if (!map.TryGetValue(old, out int idx))
                {
                    idx = order.Count;
                    map[old] = idx;
                    order.Add(old);
                }

                tri[j] = idx;
            }

            newF[i] = tri;
        }

        var newV = new Vector3[order.Count];
        for (int i = 0; i < order.Count; i++)
        {
            newV[i] = v[order[i]];
        }

        return (newV, newF);
    }

    private static QuadMesh ProjectToSurface(QuadMesh mesh, Bvh surface)
    {
        var positions = new Vector3[mesh.VertexCount];
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            positions[v] = surface.ClosestPoint(mesh.Positions[v]);
        }

        var faces = new int[mesh.FaceCount][];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            faces[f] = mesh.Face(f).ToArray();
        }

        return new QuadMesh(positions, faces);
    }
}

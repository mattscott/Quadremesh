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
    /// <summary>Remeshes a triangle mesh to a faithful quad mesh targeting <paramref name="targetFaces"/> quads.</summary>
    public static QuadMesh Remesh(TriangleMesh input, int targetFaces)
    {
        ArgumentNullException.ThrowIfNull(input);

        // --- Faithful field stack (validated bit-exact against the reference per stage) ---------------
        var inV = new Vector3[input.VertexCount];
        Array.Copy(input.Positions, inV, inV.Length);
        var inF = new int[input.TriangleCount][];
        for (int t = 0; t < input.TriangleCount; t++)
        {
            (int a, int b, int c) = input.Triangle(t);
            inF[t] = new[] { a, b, c };
        }

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

        var solver = new QuadriFlowSolver(he, orient.Q[0], normal, anchor, scaleArr);
        solver.Solve();

        IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
        QuadRemesher.AlignLatticeForExtraction(grid);
        QuadMesh raw = new QuadriFlowExtractor(grid).Extract();
        if (IsClosed(he))
        {
            raw = QuadHoleFiller.Fill(raw);
        }

        QuadMesh quads = QuadMeshCleanup.FixValence(raw);
        return ProjectToSurface(quads, Bvh.Build(input));
    }

    private static bool IsClosed(HalfEdgeMesh mesh)
    {
        for (int hh = 0; hh < mesh.HalfEdgeCount; hh++)
        {
            if (mesh.Twin(hh) < 0)
            {
                return false;
            }
        }

        return true;
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

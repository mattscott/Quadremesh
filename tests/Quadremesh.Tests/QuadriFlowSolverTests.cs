using Quadremesh.Core.Extraction;
using Quadremesh.Core.Fields;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.CorpusGen;
using Xunit;

namespace Quadremesh.Tests;

public class QuadriFlowSolverTests
{
    /// <summary>Builds a solver with placeholder fields (enough for the topology/A1 checks).</summary>
    private static QuadriFlowSolver SolverFor(TriangleMesh tri)
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(tri);
        Vector3[] normal = MeshAttributes.VertexNormals(mesh);
        var orientation = new Vector3[mesh.VertexCount];
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            orientation[v] = TangentFrame.AnyTangent(normal[v]);
        }

        var scale = new double[mesh.VertexCount];
        Array.Fill(scale, 1.0);
        return new QuadriFlowSolver(mesh, orientation, normal, mesh.Positions, scale);
    }

    /// <summary>Runs the real field pipeline (remesh → orientation → position) and builds the solver.</summary>
    private static QuadriFlowSolver RealSolver(TriangleMesh input, double ell, double? fieldEll = null)
    {
        double meshEll = fieldEll ?? ell;
        var iso = Quadremesh.Core.Remeshing.IsotropicRemesher.Run(
            input, new Quadremesh.Core.Remeshing.IsotropicRemeshOptions { TargetEdgeLength = meshEll, Iterations = 8 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(iso);
        var hierarchy = Quadremesh.Core.Hierarchy.MultiresHierarchy.Build(mesh);
        OrientationSolveResult ori = OrientationFieldSolver.Solve(hierarchy, new OrientationSolveOptions { SweepsPerLevel = 20 });
        PositionSolveResult pos = PositionFieldSolver.Solve(
            hierarchy, ori.OrientationByLevel, new UniformSizingField(ell), new PositionSolveOptions { SweepsPerLevel = 20 });
        var scale = new double[mesh.VertexCount];
        Array.Fill(scale, ell);
        return new QuadriFlowSolver(mesh, ori.Orientation, hierarchy.Finest.Normal, pos.Position, scale);
    }

    private static int FindEdge(QuadriFlowSolver solver, int a, int b)
    {
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            (int v0, int v1) = solver.EdgeValue(e);
            if ((v0 == a && v1 == b) || (v0 == b && v1 == a))
            {
                return e;
            }
        }

        return -1;
    }

    [Fact]
    public void A1_each_face_has_three_distinct_edges_consistent_with_its_corners()
    {
        QuadriFlowSolver solver = SolverFor(Primitives.Icosphere(subdivisions: 2));

        for (int f = 0; f < solver.FaceCount; f++)
        {
            int e0 = solver.FaceEdgeId(f, 0);
            int e1 = solver.FaceEdgeId(f, 1);
            int e2 = solver.FaceEdgeId(f, 2);
            Assert.True(e0 != e1 && e1 != e2 && e0 != e2, $"face {f} has a repeated edge id");

            for (int j = 0; j < 3; j++)
            {
                int a = solver.FaceCorner(f, j);
                int b = solver.FaceCorner(f, (j + 1) % 3);
                (int v0, int v1) = solver.EdgeValue(solver.FaceEdgeId(f, j));
                Assert.True(
                    (v0 == a && v1 == b) || (v0 == b && v1 == a),
                    $"face {f} edge {j} endpoints {(v0, v1)} != corners {(a, b)}");
            }
        }
    }

    [Fact]
    public void A1_every_edge_of_a_closed_mesh_is_referenced_by_exactly_two_faces()
    {
        QuadriFlowSolver solver = SolverFor(Primitives.Icosphere(subdivisions: 2));

        var refs = new int[solver.EdgeCount];
        for (int f = 0; f < solver.FaceCount; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                refs[solver.FaceEdgeId(f, j)]++;
            }
        }

        Assert.All(refs, r => Assert.Equal(2, r));
    }

    [Fact]
    public void A1_boundary_edges_are_referenced_by_one_face()
    {
        QuadriFlowSolver solver = SolverFor(Primitives.Cylinder());

        var refs = new int[solver.EdgeCount];
        for (int f = 0; f < solver.FaceCount; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                refs[solver.FaceEdgeId(f, j)]++;
            }
        }

        int boundary = refs.Count(r => r == 1);
        int interior = refs.Count(r => r == 2);
        Assert.All(refs, r => Assert.InRange(r, 1, 2));
        Assert.True(boundary > 0 && interior > 0);
    }

    [Fact]
    public void A2_edge_diff_is_the_exact_lattice_step_on_an_axis_aligned_unit_patch()
    {
        // Unit square in z=0, field aligned to +X, scale 1, anchors at the lattice points.
        var positions = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 1, 0),
        };
        var tri = new TriangleMesh(positions, new[] { 0, 1, 3, 0, 3, 2 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(tri);

        var orientation = new Vector3[4];
        var normal = new Vector3[4];
        Array.Fill(orientation, Vector3.UnitX);
        Array.Fill(normal, Vector3.UnitZ);
        var scale = new double[4];
        Array.Fill(scale, 1.0);
        var solver = new QuadriFlowSolver(mesh, orientation, normal, mesh.Positions, scale);

        // axis0 = +X, axis1 = n × axis0 = +Y. Offsets become exact integer grid steps.
        Assert.Equal(new Vec2i(1, 0), solver.EdgeDiff(FindEdge(solver, 0, 1))); // along X
        Assert.Equal(new Vec2i(0, 1), solver.EdgeDiff(FindEdge(solver, 0, 2))); // along Y
        Assert.Equal(new Vec2i(1, 1), solver.EdgeDiff(FindEdge(solver, 0, 3))); // diagonal
    }

    [Fact]
    public void A3_coupled_divergence_telescopes_to_zero_on_a_flat_patch()
    {
        // On a flat, perfectly-aligned patch the rotated edge diffs must telescope exactly to zero
        // around every face — the core invariant the face_edgeOrients formula must satisfy.
        var positions = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 1, 0),
        };
        var tri = new TriangleMesh(positions, new[] { 0, 1, 3, 0, 3, 2 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(tri);

        var orientation = new Vector3[4];
        var normal = new Vector3[4];
        Array.Fill(orientation, Vector3.UnitX);
        Array.Fill(normal, Vector3.UnitZ);
        var scale = new double[4];
        Array.Fill(scale, 1.0);
        var solver = new QuadriFlowSolver(mesh, orientation, normal, mesh.Positions, scale);

        for (int f = 0; f < solver.FaceCount; f++)
        {
            Assert.Equal(new Vec2i(0, 0), solver.FaceDivergence(f));
        }
    }

    [Fact]
    public void A4_sphere_has_singular_faces_torus_far_fewer()
    {
        Assert.True(RealSolver(Primitives.Icosphere(subdivisions: 3), 0.15).SingularCount > 0);
        // A torus admits a (near) singularity-free field.
        int torus = RealSolver(Primitives.Torus(), 0.15).SingularCount;
        int sphere = RealSolver(Primitives.Icosphere(subdivisions: 3), 0.15).SingularCount;
        Assert.True(torus < sphere, $"torus {torus} should have fewer singular faces than sphere {sphere}");
    }

    [Fact]
    public void A5_comb_makes_non_singular_shared_edges_consistent()
    {
        QuadriFlowSolver solver = RealSolver(Primitives.Icosphere(subdivisions: 3), 0.15);

        int total = 0;
        int consistent = 0;
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            var inc = solver.EdgeIncidence(e);
            if (inc.Count != 2)
            {
                continue;
            }

            (int f0, int j0) = inc[0];
            (int f1, int j1) = inc[1];
            if (solver.IsSingular(f0) || solver.IsSingular(f1))
            {
                continue;
            }

            total++;
            // After combing, the two faces' orientations on a shared non-singular edge are opposite
            // everywhere except the few "seam" edges that cross the cut around a singularity.
            if ((solver.FaceEdgeOrient(f0, j0) - solver.FaceEdgeOrient(f1, j1) + 4) % 4 == 2)
            {
                consistent++;
            }
        }

        Assert.True(total > 0);
        Assert.True(consistent > 0.9 * total, $"only {consistent}/{total} non-singular edges are combed-consistent");
    }

    [Fact]
    public void A5_most_faces_have_zero_divergence_after_comb()
    {
        QuadriFlowSolver solver = RealSolver(Primitives.Icosphere(subdivisions: 3), 0.15);

        int zero = 0;
        for (int f = 0; f < solver.FaceCount; f++)
        {
            if (solver.FaceDivergence(f).Equals(new Vec2i(0, 0)))
            {
                zero++;
            }
        }

        // The vast majority of faces close up; only singular neighbourhoods carry divergence (the flow
        // in Stage B drives the rest to zero).
        Assert.True(zero > 0.7 * solver.FaceCount, $"only {zero}/{solver.FaceCount} faces have zero divergence");
    }

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void C_pure_quad_extraction_diagnostic(string name)
    {
        QuadMesh q = Quadremesh.Core.QuadRemesher.Remesh(
            Primitives.All()[name],
            new Quadremesh.Core.QuadRemesherOptions { TargetQuadCount = 1200, Extraction = Quadremesh.Core.ExtractionMode.PureQuad });

        var hist = new SortedDictionary<int, int>();
        for (int f = 0; f < q.FaceCount; f++)
        {
            hist[q.FaceSize(f)] = hist.GetValueOrDefault(q.FaceSize(f)) + 1;
        }

        var valence = new int[q.VertexCount];
        for (int f = 0; f < q.FaceCount; f++)
        {
            foreach (int v in q.Face(f))
            {
                valence[v]++;
            }
        }

        int irregular = valence.Count(v => v != 4);

        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(q);
        int euler = he.VertexCount - he.EdgeCount + he.FaceCount;
        System.Console.WriteLine(
            $"{name}: {q.VertexCount} verts, {q.FaceCount} faces, {q.QuadFraction:P1} quads, " +
            $"euler {euler}, irregular {irregular}, {he.Defects}");
        System.Console.WriteLine($"  sizes: {string.Join(", ", hist.Select(kv => $"{kv.Key}:{kv.Value}"))}");

        // PureQuad is now topology-safe on closed inputs: a watertight, edge-manifold, all-quad mesh
        // with the input's Euler characteristic (sphere 2, torus 0).
        Assert.Equal(1.0, q.QuadFraction);
        Assert.Equal(0, he.Defects.NonManifoldEdges);
        Assert.Equal(0, he.Defects.BoundaryEdges);
        Assert.Equal(name == "sphere" ? 2 : 0, euler);

        // Irregular-vertex budget vs the QuadriFlow goldens (sphere ≈ 8, torus ≈ 22), within the
        // documented 2–4× acceptance. Guards against the foldover blow-up that FixFlip-first removed.
        int budget = name == "sphere" ? 12 : 60;
        Assert.True(irregular <= budget, $"{name}: {irregular} irregular vertices exceeds budget {budget}");
    }

    [Fact]
    public void B1_constraint_divergence_matches_per_face_divergence()
    {
        QuadriFlowSolver solver = RealSolver(Primitives.Icosphere(subdivisions: 3), 0.15);
        int[] initial = solver.ConstraintDivergence();

        for (int f = 0; f < solver.FaceCount; f++)
        {
            Vec2i d = solver.FaceDivergence(f);
            Assert.Equal(d.X, initial[(f * 2) + 0]);
            Assert.Equal(d.Y, initial[(f * 2) + 1]);
        }
    }

    [Fact]
    public void B3_flow_drives_every_non_singular_face_to_zero_divergence()
    {
        foreach (var (name, input) in new[]
                 {
                     ("sphere", Primitives.Icosphere(subdivisions: 3)),
                     ("torus", Primitives.Torus()),
                 })
        {
            QuadriFlowSolver solver = RealSolver(input, 0.15);
            solver.Solve();

            int residual = 0;
            for (int f = 0; f < solver.FaceCount; f++)
            {
                if (!solver.IsSingular(f) && !solver.FaceDivergence(f).Equals(new Vec2i(0, 0)))
                {
                    residual++;
                }
            }

            System.Console.WriteLine($"{name}: {residual} residual non-singular faces of {solver.FaceCount}");
            Assert.Equal(0, residual);
        }
    }

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void C1_subdivide_drives_every_edge_diff_to_a_unit_step(string name)
    {
        // Mirror the PureQuad pipeline resolution: fields at ℓ/2, lattice scale ℓ.
        QuadriFlowSolver solver = RealSolver(Primitives.All()[name], 0.15, fieldEll: 0.075);
        solver.Solve();

        IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
        int residualBefore = CountNonSingularResidual(grid);
        grid.SubdivideEdgeDiff(1);

        // The defining post-condition of subdivide_edgeDiff: no offset spans more than one lattice step.
        Assert.True(grid.MaxEdgeDiff() <= 1, $"{name}: max |edge_diff| = {grid.MaxEdgeDiff()} after subdivide");

        // Subdivision must not introduce new divergence on the previously-consistent faces.
        int residualAfter = CountNonSingularResidual(grid);
        Assert.True(
            residualAfter <= residualBefore,
            $"{name}: residual divergence grew {residualBefore} -> {residualAfter} after subdivide");
    }

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void C2_subdivide_fixflip_subdivide_keeps_unit_offsets(string name)
    {
        QuadriFlowSolver solver = RealSolver(Primitives.All()[name], 0.15, fieldEll: 0.075);
        solver.Solve();
        IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
        grid.SubdivideEdgeDiff(1);
        grid.FixFlip();
        grid.SubdivideEdgeDiff(1);

        // The full alignment sequence must terminate and leave only unit/zero offsets.
        Assert.True(grid.MaxEdgeDiff() <= 1, $"{name}: max |edge_diff| = {grid.MaxEdgeDiff()} after fix sequence");
    }

    /// <summary>Counts faces whose coupled divergence is non-zero, treating them as residual defects.</summary>
    private static int CountNonSingularResidual(IntegerGridMesh grid)
    {
        int residual = 0;
        for (int f = 0; f < grid.FaceCount; f++)
        {
            if (!grid.FaceDivergence(f).Equals(new Vec2i(0, 0)))
            {
                residual++;
            }
        }

        return residual;
    }

    [Fact]
    public void A2_most_edges_of_a_matched_resolution_mesh_are_unit_steps()
    {
        // Field mesh at lattice spacing: most edges should round to a single grid step.
        var input = Primitives.Icosphere(subdivisions: 3);
        var iso = Quadremesh.Core.Remeshing.IsotropicRemesher.Run(
            input, new Quadremesh.Core.Remeshing.IsotropicRemeshOptions { TargetEdgeLength = 0.15, Iterations = 8 });
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(iso);
        var hierarchy = Quadremesh.Core.Hierarchy.MultiresHierarchy.Build(mesh);
        OrientationSolveResult ori = OrientationFieldSolver.Solve(hierarchy, new OrientationSolveOptions { SweepsPerLevel = 20 });
        PositionSolveResult pos = PositionFieldSolver.Solve(
            hierarchy, ori.OrientationByLevel, new UniformSizingField(0.15), new PositionSolveOptions { SweepsPerLevel = 20 });

        var scale = new double[mesh.VertexCount];
        Array.Fill(scale, 0.15);
        var solver = new QuadriFlowSolver(mesh, ori.Orientation, hierarchy.Finest.Normal, pos.Position, scale);

        int unit = 0;   // axis-aligned quad edges
        int small = 0;  // unit or diagonal (the triangulation's diagonals are (±1, ±1))
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            Vec2i d = solver.EdgeDiff(e);
            int mag = System.Math.Abs(d.X) + System.Math.Abs(d.Y);
            if (mag == 1)
            {
                unit++;
            }

            if (mag is >= 1 and <= 2)
            {
                small++;
            }
        }

        // A healthy fraction are axis-aligned unit steps; almost none span more than a diagonal
        // (large offsets would mean ℓ doesn't match the mesh resolution).
        Assert.True(unit > 0.4 * solver.EdgeCount, $"only {unit}/{solver.EdgeCount} edges are unit steps");
        Assert.True(small > 0.9 * solver.EdgeCount, $"only {small}/{solver.EdgeCount} edges are within a diagonal");
    }
}

using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Extraction;
using Quadremesh.Core.Fields;
using Quadremesh.Core.Hierarchy;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.Core.Remeshing;

namespace Quadremesh.Core;

/// <summary>Which extraction back-end to use.</summary>
public enum ExtractionMode
{
    /// <summary>Greedy lattice extraction (P5): quad-dominant, faster, some non-quads.</summary>
    QuadDominant,

    /// <summary>QuadriFlow integer-consistency solve (P6): far fewer non-quads/singularities.</summary>
    PureQuad,
}

/// <summary>Options controlling a quad-remesh run. Grows over later phases (features, guides, symmetry).</summary>
public sealed class QuadRemesherOptions
{
    /// <summary>
    /// Extraction back-end. Defaults to <see cref="ExtractionMode.QuadDominant"/>;
    /// <see cref="ExtractionMode.PureQuad"/> runs the QuadriFlow integer-grid-map (solve → subdivide →
    /// FixFlip → subdivide → manifold extraction) and now produces watertight, all-quad output with
    /// correct genus on closed inputs (sphere/torus). It stays opt-in until feature/boundary handling
    /// (P7) extends it to creased and open surfaces.
    /// </summary>
    public ExtractionMode Extraction { get; set; } = ExtractionMode.QuadDominant;

    /// <summary>Target quad edge length ℓ. Takes precedence over <see cref="TargetQuadCount"/>.</summary>
    public double? TargetEdgeLength { get; set; }

    /// <summary>Target quad count; ℓ is derived as sqrt(surfaceArea / count). Ignored if an edge length is set.</summary>
    public int? TargetQuadCount { get; set; }

    /// <summary>Isotropic pre-pass iterations (P2).</summary>
    public int RemeshIterations { get; set; } = 8;

    /// <summary>Gauss–Seidel sweeps per hierarchy level for each field solve (P3/P4).</summary>
    public int FieldSweeps { get; set; } = 20;

    /// <summary>
    /// When true (default), detects sharp creases (dihedral) and open boundaries and pins the
    /// orientation field to them, so the output's quad edge loops follow the model's hard edges (P7).
    /// </summary>
    public bool PreserveSharpFeatures { get; set; } = true;
}

/// <summary>
/// Top-level orchestrator: triangle mesh in, quad-dominant mesh out. Wires the pipeline P2→P5
/// (isotropic pre-pass → orientation field → position field → lattice extraction). Pure-quad output
/// (P6), feature/guide/symmetry controls (P7), and the streaming/progress API arrive in later phases.
/// </summary>
public static class QuadRemesher
{
    /// <summary>Remeshes <paramref name="input"/> into a quad-dominant mesh.</summary>
    public static QuadMesh Remesh(TriangleMesh input, QuadRemesherOptions options)
    {
        ArgumentNullException.ThrowIfNull(input);
        ArgumentNullException.ThrowIfNull(options);

        double ell = ResolveEdgeLength(input, options);

        // PureQuad runs the fields on a mesh finer than the quad lattice ℓ, so each lattice cell holds
        // several vertices that collapse into one output vertex (QuadriFlow works this way).
        double fieldEll = options.Extraction == ExtractionMode.PureQuad ? ell * 0.5 : ell;

        // Feature-preserving remesh stays opt-in: razor-sharp creases combined with the sharp-edge
        // integer locks below currently leave a small hole in the box (a lock/closure interaction the
        // reference handles but our extractor does not yet). With the standard remesh the locks still
        // pull quad edges toward the creases while keeping the output watertight.
        TriangleMesh iso = IsotropicRemesher.Run(
            input, new IsotropicRemeshOptions { TargetEdgeLength = fieldEll, Iterations = options.RemeshIterations });

        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(iso);
        MultiresHierarchy hierarchy = MultiresHierarchy.Build(mesh);

        // Detect sharp creases / open boundaries once: they pin the orientation field (so quad loops
        // follow them) and, in PureQuad, lock the integer solve onto them (so quad edges land on them).
        FeatureResult? features = options.PreserveSharpFeatures && HasSharpFeatures(mesh, out FeatureResult fr) ? fr : null;
        OrientationConstraints? creaseConstraints = features is not null ? BuildCreaseConstraints(mesh, features) : null;

        OrientationSolveResult orientation = OrientationFieldSolver.Solve(
            hierarchy, new OrientationSolveOptions { SweepsPerLevel = options.FieldSweeps }, creaseConstraints);
        PositionSolveResult position = PositionFieldSolver.Solve(
            hierarchy, orientation.OrientationByLevel, new UniformSizingField(ell),
            new PositionSolveOptions { SweepsPerLevel = options.FieldSweeps });

        var scale = new double[mesh.VertexCount];
        Array.Fill(scale, ell);

        Bvh surface = Bvh.Build(input);

        if (options.Extraction == ExtractionMode.PureQuad)
        {
            // QuadriFlow integer-grid-map solve + extraction.
            var solver = new QuadriFlowSolver(mesh, orientation.Orientation, hierarchy.Finest.Normal, position.Position, scale);

            // Lock the integer solve onto the creases so a quad edge runs along each one.
            if (features is not null)
            {
                solver.ApplySharpConstraints(features.SharpEdge);
            }

            solver.Solve();

            // Align the triangulation with the integer lattice before extraction. QuadriFlow's order is
            // subdivide → FixFlip → subdivide; we run a FixFlip FIRST as well. Our position field can
            // carry parametrization foldovers (negative-area lattice cells, offsets of several cells
            // across one edge) clustered at orientation singularities — far worse than QuadriFlow's
            // cleaner field. Removing those foldovers up front (FixFlip shrinks the offending edge
            // rings) stops the subsequent subdivide from choking on a stretched cell near a cone point,
            // which is what was leaving holes and inflating the irregular-vertex count on the torus.
            IntegerGridMesh grid = IntegerGridMesh.FromSolver(solver);
            grid.FixFlip();
            grid.SubdivideEdgeDiff(1);
            grid.FixFlip();
            grid.SubdivideEdgeDiff(1);

            QuadMesh quads = QuadMeshCleanup.FixValence(new QuadriFlowExtractor(grid).Extract());
            return ProjectToSurface(quads, surface);
        }

        return QuadExtractor.Extract(new ExtractionInput
        {
            Mesh = mesh,
            Orientation = orientation.Orientation,
            Normal = hierarchy.Finest.Normal,
            Anchor = position.Position,
            Scale = scale,
            Surface = surface,
        });
    }

    /// <summary>Detects features; returns false (and a throwaway result) when the mesh has no sharp edges.</summary>
    private static bool HasSharpFeatures(HalfEdgeMesh mesh, out FeatureResult features)
    {
        features = FeatureDetector.Detect(mesh);
        return features.SharpEdgeCount > 0;
    }

    /// <summary>Builds orientation constraints from the mesh's sharp creases and open boundaries.</summary>
    private static OrientationConstraints BuildCreaseConstraints(HalfEdgeMesh mesh, FeatureResult features)
    {
        var constrained = new bool[mesh.VertexCount];
        var tangent = new Vector3[mesh.VertexCount];
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            // Pin clean creases (a single through-tangent); corners/junctions stay free and are fixed
            // implicitly by the creases meeting there.
            if (features.OnFeature[v] && !features.Corner[v])
            {
                constrained[v] = true;
                tangent[v] = features.CreaseTangent[v];
            }
        }

        return new OrientationConstraints { Constrained = constrained, Tangent = tangent };
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

    private static double ResolveEdgeLength(TriangleMesh input, QuadRemesherOptions options)
    {
        if (options.TargetEdgeLength is { } len and > 0)
        {
            return len;
        }

        if (options.TargetQuadCount is { } quads and > 0)
        {
            double area = 0.0;
            for (int t = 0; t < input.TriangleCount; t++)
            {
                (int a, int b, int c) = input.Triangle(t);
                area += MathUtils.TriangleArea(input.Positions[a], input.Positions[b], input.Positions[c]);
            }

            return System.Math.Sqrt(area / quads);
        }

        throw new ArgumentException("Either TargetEdgeLength or TargetQuadCount must be set.", nameof(options));
    }
}

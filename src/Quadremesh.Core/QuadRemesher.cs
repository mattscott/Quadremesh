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
    /// Extraction back-end. Defaults to <see cref="ExtractionMode.QuadDominant"/> because it
    /// guarantees correct topology; <see cref="ExtractionMode.PureQuad"/> raises the quad fraction
    /// markedly but is not yet topology-safe at singularities (the rotation-coupled solve is pending).
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

        TriangleMesh iso = IsotropicRemesher.Run(
            input, new IsotropicRemeshOptions { TargetEdgeLength = ell, Iterations = options.RemeshIterations });

        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(iso);
        MultiresHierarchy hierarchy = MultiresHierarchy.Build(mesh);

        OrientationSolveResult orientation = OrientationFieldSolver.Solve(
            hierarchy, new OrientationSolveOptions { SweepsPerLevel = options.FieldSweeps });
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
            solver.Solve();
            QuadMesh quads = solver.ExtractQuadMesh();
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

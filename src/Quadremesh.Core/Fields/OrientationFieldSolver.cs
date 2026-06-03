using System.Threading.Tasks;
using Quadremesh.Core.Hierarchy;
using Quadremesh.Core.Math;

namespace Quadremesh.Core.Fields;

/// <summary>Options for the <see cref="OrientationFieldSolver"/>.</summary>
public sealed class OrientationSolveOptions
{
    /// <summary>Gauss–Seidel sweeps performed at each hierarchy level.</summary>
    public int SweepsPerLevel { get; set; } = 20;

    /// <summary>Minimum colour-class size before a sweep parallelises across cores.</summary>
    public int ParallelThreshold { get; set; } = 2048;

    /// <summary>When true, records the finest-level field energy after every sweep (for diagnostics/tests).</summary>
    public bool RecordEnergy { get; set; }
}

/// <summary>
/// Per-finest-vertex orientation constraints from sharp features: where <see cref="Constrained"/> is
/// set, one cross axis is pinned to <see cref="Tangent"/> (the crease/boundary direction) instead of
/// being free to smooth. Build from <see cref="FeatureDetector"/>.
/// </summary>
public sealed class OrientationConstraints
{
    /// <summary>Whether each finest-level vertex's field is pinned to a feature tangent.</summary>
    public required bool[] Constrained { get; init; }

    /// <summary>The tangent (crease/boundary direction) each constrained vertex is pinned to.</summary>
    public required Vector3[] Tangent { get; init; }
}

/// <summary>The outcome of an orientation solve.</summary>
public sealed class OrientationSolveResult
{
    /// <summary>The converged 4-RoSy direction per finest-level vertex (unit, in the tangent plane).</summary>
    public required Vector3[] Orientation { get; init; }

    /// <summary>
    /// The converged direction at every hierarchy level (index 0 = finest). The position solver (P4)
    /// reuses these per-level orientations to define the lattice frame at each level.
    /// </summary>
    public required IReadOnlyList<Vector3[]> OrientationByLevel { get; init; }

    /// <summary>Finest-level energy after each sweep, if <see cref="OrientationSolveOptions.RecordEnergy"/> was set.</summary>
    public required IReadOnlyList<double> EnergyTrace { get; init; }
}

/// <summary>
/// Solves the 4-RoSy orientation field by local Gauss–Seidel smoothing over the multiresolution
/// hierarchy (Instant Meshes Stage 1). No global solve: each node is repeatedly replaced by the
/// tangent-projected average of its neighbours' closest cross representatives. Solving coarse-to-fine
/// avoids the field vortices a flat solve falls into.
/// </summary>
public static class OrientationFieldSolver
{
    private const double SelfWeight = 1.0;

    /// <summary>Solves the orientation field and returns the finest-level result.</summary>
    public static OrientationSolveResult Solve(
        MultiresHierarchy hierarchy, OrientationSolveOptions options, OrientationConstraints? constraints = null)
    {
        ArgumentNullException.ThrowIfNull(hierarchy);
        ArgumentNullException.ThrowIfNull(options);

        int levelCount = hierarchy.Levels.Count;
        var orient = new Vector3[levelCount][];
        var energyTrace = new List<double>();

        (bool[][] constrained, Vector3[][] tangent) = BuildConstraintsPerLevel(hierarchy, constraints);

        // Seed the coarsest level: pin constrained nodes to their feature tangent, others arbitrary.
        GraphLevel coarsest = hierarchy.Levels[levelCount - 1];
        orient[levelCount - 1] = new Vector3[coarsest.NodeCount];
        for (int i = 0; i < coarsest.NodeCount; i++)
        {
            orient[levelCount - 1][i] = constrained[levelCount - 1][i]
                ? PinnedAxis(tangent[levelCount - 1][i], coarsest.Normal[i])
                : TangentFrame.AnyTangent(coarsest.Normal[i]);
        }

        for (int level = levelCount - 1; level >= 0; level--)
        {
            GraphLevel g = hierarchy.Levels[level];
            Vector3[] o = orient[level];
            bool[] c = constrained[level];
            Vector3[] t = tangent[level];
            int[][] colors = GraphColoring.Color(g);

            for (int sweep = 0; sweep < options.SweepsPerLevel; sweep++)
            {
                foreach (int[] group in colors)
                {
                    Sweep(g, o, c, t, group, options.ParallelThreshold);
                }

                if (options.RecordEnergy && level == 0)
                {
                    energyTrace.Add(Energy(g, o));
                }
            }

            if (level > 0)
            {
                orient[level - 1] = Prolong(hierarchy, level, o);
            }
        }

        return new OrientationSolveResult
        {
            Orientation = orient[0],
            OrientationByLevel = orient,
            EnergyTrace = energyTrace,
        };
    }

    /// <summary>Field energy of a level: Σ w_ij ‖o_i − closestRep(o_i, o_j)‖² over undirected edges.</summary>
    public static double Energy(GraphLevel g, Vector3[] orient)
    {
        double energy = 0.0;
        for (int i = 0; i < g.NodeCount; i++)
        {
            ReadOnlySpan<int> neighbors = g.Neighbors(i);
            ReadOnlySpan<double> weights = g.Weights(i);
            for (int k = 0; k < neighbors.Length; k++)
            {
                int j = neighbors[k];
                if (j <= i)
                {
                    continue; // count each undirected edge once
                }

                Vector3 rep = Rosy.ClosestRepresentative(orient[i], orient[j], g.Normal[j]);
                energy += weights[k] * (orient[i] - rep).LengthSquared;
            }
        }

        return energy;
    }

    private static void Sweep(
        GraphLevel g, Vector3[] o, bool[] constrained, Vector3[] tangent, int[] group, int parallelThreshold)
    {
        if (group.Length >= parallelThreshold)
        {
            Parallel.For(0, group.Length, idx => UpdateNode(g, o, constrained, tangent, group[idx]));
        }
        else
        {
            foreach (int node in group)
            {
                UpdateNode(g, o, constrained, tangent, node);
            }
        }
    }

    private static void UpdateNode(GraphLevel g, Vector3[] o, bool[] constrained, Vector3[] tangent, int i)
    {
        Vector3 ni = g.Normal[i];

        // A feature vertex is hard-pinned: one cross axis stays aligned to the crease/boundary tangent.
        if (constrained[i])
        {
            o[i] = PinnedAxis(tangent[i], ni);
            return;
        }

        // Accumulate into a running sum, aligning each neighbour to the sum so far (not to the fixed
        // old direction). This lets a cross escape a 90° domain wall instead of locking to its current
        // representative — the key to convergence (Instant Meshes orientation smoothing).
        Vector3 sum = o[i] * SelfWeight;

        ReadOnlySpan<int> neighbors = g.Neighbors(i);
        ReadOnlySpan<double> weights = g.Weights(i);
        for (int k = 0; k < neighbors.Length; k++)
        {
            int j = neighbors[k];
            Vector3 rep = Rosy.ClosestRepresentative(sum, o[j], g.Normal[j]);
            sum += weights[k] * rep;
        }

        sum = TangentFrame.ProjectToTangent(sum, ni);
        if (sum.TryNormalize(out Vector3 unit))
        {
            o[i] = unit;
        }
    }

    /// <summary>The unit cross axis at a feature vertex: the tangent projected onto the tangent plane.</summary>
    private static Vector3 PinnedAxis(Vector3 tangent, Vector3 normal)
    {
        Vector3 projected = TangentFrame.ProjectToTangent(tangent, normal);
        return projected.TryNormalize(out Vector3 unit) ? unit : TangentFrame.AnyTangent(normal);
    }

    /// <summary>
    /// Propagates the finest-level feature constraints up the hierarchy: a coarse node is constrained
    /// if any child is, with tangent the (aligned) sum of its constrained children's tangents.
    /// </summary>
    private static (bool[][] Constrained, Vector3[][] Tangent) BuildConstraintsPerLevel(
        MultiresHierarchy hierarchy, OrientationConstraints? constraints)
    {
        int levelCount = hierarchy.Levels.Count;
        var constrained = new bool[levelCount][];
        var tangent = new Vector3[levelCount][];
        for (int level = 0; level < levelCount; level++)
        {
            constrained[level] = new bool[hierarchy.Levels[level].NodeCount];
            tangent[level] = new Vector3[hierarchy.Levels[level].NodeCount];
        }

        if (constraints is not null)
        {
            Array.Copy(constraints.Constrained, constrained[0], constraints.Constrained.Length);
            Array.Copy(constraints.Tangent, tangent[0], constraints.Tangent.Length);
        }

        for (int level = 0; level < levelCount - 1; level++)
        {
            GraphLevel fine = hierarchy.Levels[level];
            for (int child = 0; child < fine.NodeCount; child++)
            {
                if (!constrained[level][child])
                {
                    continue;
                }

                int parent = hierarchy.Parent(level, child);
                Vector3 childTangent = tangent[level][child];
                if (constrained[level + 1][parent])
                {
                    // Align before summing so opposite (mod-180°) crease tangents reinforce, not cancel.
                    if (Vector3.Dot(childTangent, tangent[level + 1][parent]) < 0)
                    {
                        childTangent = -childTangent;
                    }
                }

                constrained[level + 1][parent] = true;
                tangent[level + 1][parent] += childTangent;
            }
        }

        return (constrained, tangent);
    }

    private static Vector3[] Prolong(MultiresHierarchy hierarchy, int level, Vector3[] coarse)
    {
        GraphLevel fine = hierarchy.Levels[level - 1];
        var result = new Vector3[fine.NodeCount];
        for (int c = 0; c < fine.NodeCount; c++)
        {
            int parent = hierarchy.Parent(level - 1, c);
            Vector3 inherited = TangentFrame.ProjectToTangent(coarse[parent], fine.Normal[c]);
            result[c] = inherited.TryNormalize(out Vector3 unit) ? unit : TangentFrame.AnyTangent(fine.Normal[c]);
        }

        return result;
    }
}

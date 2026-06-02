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

/// <summary>The outcome of an orientation solve.</summary>
public sealed class OrientationSolveResult
{
    /// <summary>The converged 4-RoSy direction per finest-level vertex (unit, in the tangent plane).</summary>
    public required Vector3[] Orientation { get; init; }

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
    public static OrientationSolveResult Solve(MultiresHierarchy hierarchy, OrientationSolveOptions options)
    {
        ArgumentNullException.ThrowIfNull(hierarchy);
        ArgumentNullException.ThrowIfNull(options);

        int levelCount = hierarchy.Levels.Count;
        var orient = new Vector3[levelCount][];
        var energyTrace = new List<double>();

        // Seed the coarsest level with an arbitrary tangent direction per node.
        GraphLevel coarsest = hierarchy.Levels[levelCount - 1];
        orient[levelCount - 1] = new Vector3[coarsest.NodeCount];
        for (int i = 0; i < coarsest.NodeCount; i++)
        {
            orient[levelCount - 1][i] = TangentFrame.AnyTangent(coarsest.Normal[i]);
        }

        for (int level = levelCount - 1; level >= 0; level--)
        {
            GraphLevel g = hierarchy.Levels[level];
            Vector3[] o = orient[level];
            int[][] colors = GraphColoring.Color(g);

            for (int sweep = 0; sweep < options.SweepsPerLevel; sweep++)
            {
                foreach (int[] group in colors)
                {
                    Sweep(g, o, group, options.ParallelThreshold);
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

        return new OrientationSolveResult { Orientation = orient[0], EnergyTrace = energyTrace };
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

    private static void Sweep(GraphLevel g, Vector3[] o, int[] group, int parallelThreshold)
    {
        if (group.Length >= parallelThreshold)
        {
            Parallel.For(0, group.Length, idx => UpdateNode(g, o, group[idx]));
        }
        else
        {
            foreach (int node in group)
            {
                UpdateNode(g, o, node);
            }
        }
    }

    private static void UpdateNode(GraphLevel g, Vector3[] o, int i)
    {
        Vector3 ni = g.Normal[i];

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

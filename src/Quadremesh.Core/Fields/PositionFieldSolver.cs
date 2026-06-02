using System.Threading.Tasks;
using Quadremesh.Core.Hierarchy;
using Quadremesh.Core.Math;

namespace Quadremesh.Core.Fields;

/// <summary>Options for the <see cref="PositionFieldSolver"/>.</summary>
public sealed class PositionSolveOptions
{
    /// <summary>Gauss–Seidel sweeps performed at each hierarchy level.</summary>
    public int SweepsPerLevel { get; set; } = 20;

    /// <summary>Minimum colour-class size before a sweep parallelises across cores.</summary>
    public int ParallelThreshold { get; set; } = 2048;

    /// <summary>When true, records the finest-level position energy after every sweep.</summary>
    public bool RecordEnergy { get; set; }
}

/// <summary>The outcome of a position solve.</summary>
public sealed class PositionSolveResult
{
    /// <summary>The converged lattice anchor per finest-level vertex (a point near the vertex, on its tangent plane).</summary>
    public required Vector3[] Position { get; init; }

    /// <summary>Finest-level energy after each sweep, if requested.</summary>
    public required IReadOnlyList<double> EnergyTrace { get; init; }
}

/// <summary>
/// Solves the 4-PoSy position field (Instant Meshes Stage 2): each vertex gets a lattice anchor that,
/// across every edge, differs from its neighbour's by a near-integer number of quad cells. Like the
/// orientation field it is pure local Gauss–Seidel over the multiresolution hierarchy, reusing the
/// per-level orientation to define the lattice frame (o_i, n_i × o_i) at spacing ℓ_i.
/// </summary>
public static class PositionFieldSolver
{
    private const double SelfWeight = 1.0;

    /// <summary>Solves the position field and returns the finest-level anchors.</summary>
    public static PositionSolveResult Solve(
        MultiresHierarchy hierarchy,
        IReadOnlyList<Vector3[]> orientationByLevel,
        ISizingField sizing,
        PositionSolveOptions options)
    {
        ArgumentNullException.ThrowIfNull(hierarchy);
        ArgumentNullException.ThrowIfNull(orientationByLevel);
        ArgumentNullException.ThrowIfNull(sizing);
        ArgumentNullException.ThrowIfNull(options);

        int levelCount = hierarchy.Levels.Count;
        double[][] scale = BuildScalePerLevel(hierarchy, sizing);
        var position = new Vector3[levelCount][];
        var energyTrace = new List<double>();

        // The position field only solves levels where the lattice spacing ℓ is meaningful relative to
        // the node spacing. At coarse levels nodes are many cells apart and the phase is ill-posed, so
        // we start the coarse-to-fine pass at the finest level whose mean edge length is within a few ℓ.
        int startLevel = SelectStartLevel(hierarchy, scale);

        GraphLevel start = hierarchy.Levels[startLevel];
        position[startLevel] = (Vector3[])start.Position.Clone();

        for (int level = startLevel; level >= 0; level--)
        {
            GraphLevel g = hierarchy.Levels[level];
            Vector3[] p = position[level];
            Vector3[] o = orientationByLevel[level];
            double[] s = scale[level];
            int[][] colors = GraphColoring.Color(g);

            for (int sweep = 0; sweep < options.SweepsPerLevel; sweep++)
            {
                foreach (int[] group in colors)
                {
                    Sweep(g, p, o, s, group, options.ParallelThreshold);
                }

                if (options.RecordEnergy && level == 0)
                {
                    energyTrace.Add(Energy(g, p, o, s));
                }
            }

            if (level > 0)
            {
                position[level - 1] = Prolong(hierarchy, level, p);
            }
        }

        return new PositionSolveResult { Position = position[0], EnergyTrace = energyTrace };
    }

    /// <summary>
    /// Picks the coarsest level at which the position field is still well-posed: the mean edge length
    /// must not exceed a few lattice cells. Coarser levels (nodes many cells apart) are skipped.
    /// </summary>
    private static int SelectStartLevel(MultiresHierarchy hierarchy, double[][] scale)
    {
        const double maxCellsPerEdge = 4.0;
        for (int level = hierarchy.Levels.Count - 1; level >= 0; level--)
        {
            GraphLevel g = hierarchy.Levels[level];
            double meanEdge = MeanEdgeLength(g);
            double meanScale = Mean(scale[level]);
            if (meanScale > 0 && meanEdge <= maxCellsPerEdge * meanScale)
            {
                return level;
            }
        }

        return 0;
    }

    private static double MeanEdgeLength(GraphLevel g)
    {
        double sum = 0;
        int count = 0;
        for (int i = 0; i < g.NodeCount; i++)
        {
            ReadOnlySpan<int> neighbors = g.Neighbors(i);
            foreach (int j in neighbors)
            {
                if (j > i)
                {
                    sum += Vector3.Distance(g.Position[i], g.Position[j]);
                    count++;
                }
            }
        }

        return count == 0 ? 0 : sum / count;
    }

    private static double Mean(double[] values)
    {
        if (values.Length == 0)
        {
            return 0;
        }

        double sum = 0;
        foreach (double v in values)
        {
            sum += v;
        }

        return sum / values.Length;
    }

    /// <summary>
    /// Position-field energy: Σ_i Σ_{j∈N(i)} w_ij ‖p_i − latticeRep_i(p_i, p_j)‖², summed over directed
    /// edges with each node's own lattice frame — exactly the objective each Gauss–Seidel node update
    /// minimises locally.
    /// </summary>
    public static double Energy(GraphLevel g, Vector3[] position, Vector3[] orientation, double[] scale)
    {
        double energy = 0.0;
        for (int i = 0; i < g.NodeCount; i++)
        {
            Vector3 e1 = orientation[i];
            Vector3 e2 = Vector3.Cross(g.Normal[i], e1);
            ReadOnlySpan<int> neighbors = g.Neighbors(i);
            ReadOnlySpan<double> weights = g.Weights(i);
            for (int k = 0; k < neighbors.Length; k++)
            {
                Vector3 rep = LatticeRepresentative(position[i], position[neighbors[k]], e1, e2, scale[i]);
                energy += weights[k] * (position[i] - rep).LengthSquared;
            }
        }

        return energy;
    }

    /// <summary>Snaps <paramref name="pj"/> into the lattice cell of <paramref name="pi"/> spanned by (e1, e2) at spacing ℓ.</summary>
    public static Vector3 LatticeRepresentative(Vector3 pi, Vector3 pj, Vector3 e1, Vector3 e2, double scale)
    {
        Vector3 d = pj - pi;
        double k1 = System.Math.Round(Vector3.Dot(d, e1) / scale);
        double k2 = System.Math.Round(Vector3.Dot(d, e2) / scale);
        return pj - (scale * ((k1 * e1) + (k2 * e2)));
    }

    private static void Sweep(GraphLevel g, Vector3[] p, Vector3[] o, double[] s, int[] group, int parallelThreshold)
    {
        if (group.Length >= parallelThreshold)
        {
            Parallel.For(0, group.Length, idx => UpdateNode(g, p, o, s, group[idx]));
        }
        else
        {
            foreach (int node in group)
            {
                UpdateNode(g, p, o, s, node);
            }
        }
    }

    private static void UpdateNode(GraphLevel g, Vector3[] p, Vector3[] o, double[] s, int i)
    {
        Vector3 ni = g.Normal[i];
        Vector3 vi = g.Position[i];
        Vector3 e1 = o[i];
        Vector3 e2 = Vector3.Cross(ni, e1);
        double li = s[i];

        // Snap every neighbour's anchor into the cell of the FIXED current anchor, then average.
        // (A running reference is unstable here: unlike unit orientation vectors, lattice reps can
        // translate by whole cells, so a drifting reference chases itself.)
        Vector3 acc = p[i] * SelfWeight;
        double wsum = SelfWeight;

        ReadOnlySpan<int> neighbors = g.Neighbors(i);
        ReadOnlySpan<double> weights = g.Weights(i);
        for (int k = 0; k < neighbors.Length; k++)
        {
            int j = neighbors[k];
            Vector3 rep = LatticeRepresentative(p[i], p[j], e1, e2, li);
            acc += weights[k] * rep;
            wsum += weights[k];
        }

        // Project the smoothed anchor onto i's tangent plane, then wrap its offset back into the
        // fundamental lattice cell around v_i. Integer lattice shifts are meaningless (p and p + ℓk
        // denote the same grid), so wrapping keeps the anchor within half a cell of its vertex —
        // preventing the tangential drift that would otherwise carry anchors off the curved surface.
        Vector3 offset = TangentFrame.ProjectToTangent((acc / wsum) - vi, ni);
        double k1 = System.Math.Round(Vector3.Dot(offset, e1) / li);
        double k2 = System.Math.Round(Vector3.Dot(offset, e2) / li);
        offset -= li * ((k1 * e1) + (k2 * e2));
        p[i] = vi + offset;
    }

    private static Vector3[] Prolong(MultiresHierarchy hierarchy, int level, Vector3[] coarse)
    {
        GraphLevel fine = hierarchy.Levels[level - 1];
        GraphLevel coarseLevel = hierarchy.Levels[level];
        var result = new Vector3[fine.NodeCount];
        for (int c = 0; c < fine.NodeCount; c++)
        {
            int parent = hierarchy.Parent(level - 1, c);
            // Transfer the parent's tangential anchor offset down to the child.
            Vector3 offset = coarse[parent] - coarseLevel.Position[parent];
            result[c] = fine.Position[c] + TangentFrame.ProjectToTangent(offset, fine.Normal[c]);
        }

        return result;
    }

    private static double[][] BuildScalePerLevel(MultiresHierarchy hierarchy, ISizingField sizing)
    {
        int levelCount = hierarchy.Levels.Count;
        var scale = new double[levelCount][];

        GraphLevel finest = hierarchy.Levels[0];
        scale[0] = new double[finest.NodeCount];
        for (int v = 0; v < finest.NodeCount; v++)
        {
            scale[0][v] = sizing.LengthAt(v);
        }

        for (int level = 0; level < levelCount - 1; level++)
        {
            GraphLevel fine = hierarchy.Levels[level];
            GraphLevel coarse = hierarchy.Levels[level + 1];
            var accum = new double[coarse.NodeCount];
            var areaAccum = new double[coarse.NodeCount];
            for (int node = 0; node < fine.NodeCount; node++)
            {
                int parent = hierarchy.Parent(level, node);
                double a = fine.Area[node];
                accum[parent] += scale[level][node] * a;
                areaAccum[parent] += a;
            }

            var coarseScale = new double[coarse.NodeCount];
            for (int p = 0; p < coarse.NodeCount; p++)
            {
                coarseScale[p] = areaAccum[p] > 0 ? accum[p] / areaAccum[p] : scale[level][0];
            }

            scale[level + 1] = coarseScale;
        }

        return scale;
    }
}

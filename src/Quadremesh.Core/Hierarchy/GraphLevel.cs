using Quadremesh.Core.Math;

namespace Quadremesh.Core.Hierarchy;

/// <summary>
/// One level of the multiresolution graph: nodes carrying geometry (position, normal, area) and a
/// CSR weighted adjacency. The field solvers run on these levels coarse-to-fine. Level 0 is the
/// finest (one node per mesh vertex); each coarser level is a quotient graph from edge matching.
/// </summary>
public sealed class GraphLevel
{
    /// <summary>Representative position of each node.</summary>
    public required Vector3[] Position { get; init; }

    /// <summary>Unit normal of each node.</summary>
    public required Vector3[] Normal { get; init; }

    /// <summary>Accumulated area weight of each node.</summary>
    public required double[] Area { get; init; }

    /// <summary>CSR neighbour ranges, length <see cref="NodeCount"/> + 1.</summary>
    public required int[] AdjStart { get; init; }

    /// <summary>CSR neighbour node ids.</summary>
    public required int[] AdjNode { get; init; }

    /// <summary>CSR edge weights (parallel to <see cref="AdjNode"/>).</summary>
    public required double[] AdjWeight { get; init; }

    /// <summary>Number of nodes at this level.</summary>
    public int NodeCount => Position.Length;

    /// <summary>The neighbour node ids of <paramref name="node"/>.</summary>
    public ReadOnlySpan<int> Neighbors(int node) =>
        AdjNode.AsSpan(AdjStart[node], AdjStart[node + 1] - AdjStart[node]);

    /// <summary>The edge weights of <paramref name="node"/> (parallel to <see cref="Neighbors"/>).</summary>
    public ReadOnlySpan<double> Weights(int node) =>
        AdjWeight.AsSpan(AdjStart[node], AdjStart[node + 1] - AdjStart[node]);
}

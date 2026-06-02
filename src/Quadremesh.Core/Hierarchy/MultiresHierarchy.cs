using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Hierarchy;

/// <summary>
/// A coarse-to-fine stack of <see cref="GraphLevel"/>s built by greedy edge-matching coarsening,
/// plus the child→parent maps needed to prolong a solution from a coarse level to the next finer one.
/// Shared by the orientation (P3) and position (P4) solvers: solving coarse first then refining
/// escapes the local minima a flat Gauss–Seidel sweep would get stuck in.
/// </summary>
public sealed class MultiresHierarchy
{
    private readonly GraphLevel[] _levels;
    private readonly int[][] _childToParent; // _childToParent[L][child] = parent node at level L+1

    private MultiresHierarchy(GraphLevel[] levels, int[][] childToParent)
    {
        _levels = levels;
        _childToParent = childToParent;
    }

    /// <summary>The levels, finest first (index 0) to coarsest last.</summary>
    public IReadOnlyList<GraphLevel> Levels => _levels;

    /// <summary>The finest level (one node per mesh vertex).</summary>
    public GraphLevel Finest => _levels[0];

    /// <summary>The coarsest level.</summary>
    public GraphLevel Coarsest => _levels[^1];

    /// <summary>Parent id at level <paramref name="level"/>+1 for node <paramref name="node"/> at level <paramref name="level"/>.</summary>
    public int Parent(int level, int node) => _childToParent[level][node];

    /// <summary>
    /// Builds the hierarchy from mesh connectivity. Edge weights are uniform (1.0) — adequate for the
    /// orientation/position smoothing — while coarsening matches each unmatched node to a neighbour.
    /// </summary>
    public static MultiresHierarchy Build(HalfEdgeMesh mesh, int coarsestNodeLimit = 1)
    {
        ArgumentNullException.ThrowIfNull(mesh);

        GraphLevel finest = BuildFinest(mesh);
        var levels = new List<GraphLevel> { finest };
        var maps = new List<int[]>();

        GraphLevel current = finest;
        while (current.NodeCount > System.Math.Max(1, coarsestNodeLimit))
        {
            (GraphLevel coarser, int[] childToParent) = Coarsen(current);
            if (coarser.NodeCount >= current.NodeCount)
            {
                break; // no further reduction possible (e.g. fully matched already)
            }

            maps.Add(childToParent);
            levels.Add(coarser);
            current = coarser;
        }

        return new MultiresHierarchy(levels.ToArray(), maps.ToArray());
    }

    private static GraphLevel BuildFinest(HalfEdgeMesh mesh)
    {
        int n = mesh.VertexCount;
        var position = (Vector3[])mesh.Positions.Clone();
        Vector3[] normal = MeshAttributes.VertexNormals(mesh);
        double[] area = MeshAttributes.VertexAreas(mesh);

        // CSR adjacency straight from the mesh 1-ring, uniform weights.
        var adjStart = new int[n + 1];
        for (int v = 0; v < n; v++)
        {
            adjStart[v + 1] = adjStart[v] + mesh.Valence(v);
        }

        var adjNode = new int[adjStart[n]];
        var adjWeight = new double[adjStart[n]];
        for (int v = 0; v < n; v++)
        {
            ReadOnlySpan<int> neighbors = mesh.Neighbors(v);
            int start = adjStart[v];
            for (int k = 0; k < neighbors.Length; k++)
            {
                adjNode[start + k] = neighbors[k];
                adjWeight[start + k] = 1.0;
            }
        }

        return new GraphLevel
        {
            Position = position,
            Normal = normal,
            Area = area,
            AdjStart = adjStart,
            AdjNode = adjNode,
            AdjWeight = adjWeight,
        };
    }

    private static (GraphLevel Coarser, int[] ChildToParent) Coarsen(GraphLevel fine)
    {
        int n = fine.NodeCount;
        var parentOf = new int[n];
        Array.Fill(parentOf, -1);
        int parentCount = 0;

        // Greedy maximal matching: match each unmatched node to its first unmatched neighbour.
        for (int v = 0; v < n; v++)
        {
            if (parentOf[v] >= 0)
            {
                continue;
            }

            int mate = -1;
            foreach (int w in fine.Neighbors(v))
            {
                if (parentOf[w] < 0 && w != v)
                {
                    mate = w;
                    break;
                }
            }

            parentOf[v] = parentCount;
            if (mate >= 0)
            {
                parentOf[mate] = parentCount;
            }

            parentCount++;
        }

        // Accumulate parent geometry.
        var position = new Vector3[parentCount];
        var normalAccum = new Vector3[parentCount];
        var area = new double[parentCount];
        for (int v = 0; v < n; v++)
        {
            int p = parentOf[v];
            double a = fine.Area[v];
            position[p] += fine.Position[v] * a;
            normalAccum[p] += fine.Normal[v];
            area[p] += a;
        }

        var normal = new Vector3[parentCount];
        for (int p = 0; p < parentCount; p++)
        {
            position[p] = area[p] > 0 ? position[p] / area[p] : position[p];
            normal[p] = normalAccum[p].TryNormalize(out Vector3 unit) ? unit : Vector3.UnitZ;
        }

        // Accumulate parent adjacency (sum weights of merged edges, drop self-loops).
        var parentEdges = new Dictionary<long, double>();
        for (int v = 0; v < n; v++)
        {
            int pv = parentOf[v];
            ReadOnlySpan<int> neighbors = fine.Neighbors(v);
            ReadOnlySpan<double> weights = fine.Weights(v);
            for (int k = 0; k < neighbors.Length; k++)
            {
                int pw = parentOf[neighbors[k]];
                if (pv == pw)
                {
                    continue;
                }

                long key = ((long)System.Math.Min(pv, pw) << 32) | (uint)System.Math.Max(pv, pw);
                parentEdges.TryGetValue(key, out double existing);
                parentEdges[key] = existing + weights[k];
            }
        }

        GraphLevel coarser = BuildLevelFromEdges(position, normal, area, parentEdges);
        return (coarser, parentOf);
    }

    private static GraphLevel BuildLevelFromEdges(
        Vector3[] position, Vector3[] normal, double[] area, Dictionary<long, double> edges)
    {
        int n = position.Length;
        var degree = new int[n];
        foreach (long key in edges.Keys)
        {
            int a = (int)(key >> 32);
            int b = (int)(key & 0xFFFFFFFF);
            degree[a]++;
            degree[b]++;
        }

        var adjStart = new int[n + 1];
        for (int i = 0; i < n; i++)
        {
            adjStart[i + 1] = adjStart[i] + degree[i];
        }

        var adjNode = new int[adjStart[n]];
        var adjWeight = new double[adjStart[n]];
        var cursor = new int[n];
        Array.Copy(adjStart, cursor, n);
        foreach ((long key, double w) in edges)
        {
            int a = (int)(key >> 32);
            int b = (int)(key & 0xFFFFFFFF);
            adjNode[cursor[a]] = b;
            adjWeight[cursor[a]] = w;
            cursor[a]++;
            adjNode[cursor[b]] = a;
            adjWeight[cursor[b]] = w;
            cursor[b]++;
        }

        return new GraphLevel
        {
            Position = position,
            Normal = normal,
            Area = area,
            AdjStart = adjStart,
            AdjNode = adjNode,
            AdjWeight = adjWeight,
        };
    }
}

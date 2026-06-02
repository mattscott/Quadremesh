using Quadremesh.Core.Hierarchy;

namespace Quadremesh.Core.Fields;

/// <summary>
/// Greedy graph coloring used to parallelise Gauss–Seidel sweeps. Each colour class is an independent
/// set, so all nodes of one colour can be updated concurrently without two neighbours writing at once.
/// (A triangle-mesh graph is not bipartite, so this generalises the paper's "red-black" scheme to as
/// many colours as the graph needs.) The colouring is deterministic given the node order.
/// </summary>
public static class GraphColoring
{
    /// <summary>Returns the node ids grouped by colour.</summary>
    public static int[][] Color(GraphLevel level)
    {
        int n = level.NodeCount;
        var color = new int[n];
        Array.Fill(color, -1);
        int maxColor = -1;
        var used = new List<bool>();

        for (int v = 0; v < n; v++)
        {
            for (int c = 0; c <= maxColor; c++)
            {
                used[c] = false;
            }

            foreach (int w in level.Neighbors(v))
            {
                int cw = color[w];
                if (cw >= 0)
                {
                    used[cw] = true;
                }
            }

            int chosen = 0;
            while (chosen <= maxColor && used[chosen])
            {
                chosen++;
            }

            if (chosen > maxColor)
            {
                maxColor = chosen;
                used.Add(false);
            }

            color[v] = chosen;
        }

        var counts = new int[maxColor + 1];
        for (int v = 0; v < n; v++)
        {
            counts[color[v]]++;
        }

        var groups = new int[maxColor + 1][];
        for (int c = 0; c <= maxColor; c++)
        {
            groups[c] = new int[counts[c]];
        }

        var cursor = new int[maxColor + 1];
        for (int v = 0; v < n; v++)
        {
            int c = color[v];
            groups[c][cursor[c]++] = v;
        }

        return groups;
    }
}

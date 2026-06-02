namespace Quadremesh.Core.Acceleration;

/// <summary>
/// Minimum-cost flow via successive shortest paths with Johnson potentials (one Bellman–Ford to seed
/// potentials, then Dijkstra on reduced costs). Supports node supplies/demands (a b-flow), which is
/// the form QuadriFlow's integer consistency solve needs. No native dependency — this is the only
/// "heavy" numeric component in the whole pipeline (see Agents/Docs/05).
/// </summary>
public sealed class MinCostFlow
{
    private readonly List<int> _head = new();   // edge -> destination node
    private readonly List<long> _cap = new();    // residual capacity
    private readonly List<long> _cost = new();   // cost per unit
    private readonly List<List<int>> _adj = new(); // node -> incident edge ids
    private readonly List<long> _supply = new();   // node -> supply (+) / demand (-)

    /// <summary>Adds a node and returns its id.</summary>
    public int AddNode()
    {
        _adj.Add(new List<int>());
        _supply.Add(0);
        return _adj.Count - 1;
    }

    /// <summary>Sets the supply (positive) or demand (negative) at a node.</summary>
    public void SetSupply(int node, long supply) => _supply[node] = supply;

    /// <summary>Adds a directed arc with the given capacity and per-unit cost; returns the forward edge id.</summary>
    public int AddEdge(int from, int to, long capacity, long cost)
    {
        int forward = _head.Count;
        _head.Add(to);
        _cap.Add(capacity);
        _cost.Add(cost);
        _adj[from].Add(forward);

        _head.Add(from);
        _cap.Add(0);
        _cost.Add(-cost);
        _adj[to].Add(forward + 1);
        return forward;
    }

    /// <summary>Flow currently pushed along the forward arc <paramref name="edgeId"/>.</summary>
    public long Flow(int edgeId) => _cap[edgeId ^ 1];

    /// <summary>Node count.</summary>
    public int NodeCount => _adj.Count;

    /// <summary>
    /// Solves for a minimum-cost flow meeting all supplies/demands. Returns (feasible, totalCost).
    /// Infeasible if the supplies cannot be routed to the demands within the capacities.
    /// </summary>
    public (bool Feasible, long Cost) Solve()
    {
        // Route all supply to all demand through a super-source/super-sink, then min-cost-max-flow.
        int source = AddNode();
        int sink = AddNode();
        long required = 0;
        for (int v = 0; v < source; v++)
        {
            if (_supply[v] > 0)
            {
                AddEdge(source, v, _supply[v], 0);
                required += _supply[v];
            }
            else if (_supply[v] < 0)
            {
                AddEdge(v, sink, -_supply[v], 0);
            }
        }

        (long flow, long cost) = MinCostMaxFlow(source, sink);
        return (flow == required, cost);
    }

    /// <summary>Minimum-cost maximum flow from <paramref name="source"/> to <paramref name="sink"/>.</summary>
    public (long Flow, long Cost) MinCostMaxFlow(int source, int sink)
    {
        int n = _adj.Count;
        var potential = new long[n];
        SeedPotentials(source, potential);

        long totalFlow = 0;
        long totalCost = 0;
        var dist = new long[n];
        var prevEdge = new int[n];

        while (Dijkstra(source, sink, potential, dist, prevEdge))
        {
            for (int v = 0; v < n; v++)
            {
                if (dist[v] < long.MaxValue)
                {
                    potential[v] += dist[v];
                }
            }

            // Find the bottleneck along the found path.
            long push = long.MaxValue;
            for (int v = sink; v != source; v = _head[prevEdge[v] ^ 1])
            {
                push = System.Math.Min(push, _cap[prevEdge[v]]);
            }

            for (int v = sink; v != source; v = _head[prevEdge[v] ^ 1])
            {
                _cap[prevEdge[v]] -= push;
                _cap[prevEdge[v] ^ 1] += push;
            }

            totalFlow += push;
            totalCost += push * (potential[sink] - potential[source]);
        }

        return (totalFlow, totalCost);
    }

    /// <summary>Bellman–Ford (SPFA) to seed potentials so Dijkstra can run on non-negative reduced costs.</summary>
    private void SeedPotentials(int source, long[] potential)
    {
        int n = _adj.Count;
        Array.Fill(potential, long.MaxValue);
        potential[source] = 0;
        var inQueue = new bool[n];
        var queue = new Queue<int>();
        queue.Enqueue(source);
        inQueue[source] = true;

        while (queue.Count > 0)
        {
            int u = queue.Dequeue();
            inQueue[u] = false;
            foreach (int e in _adj[u])
            {
                if (_cap[e] <= 0)
                {
                    continue;
                }

                int v = _head[e];
                long nd = potential[u] + _cost[e];
                if (potential[u] != long.MaxValue && nd < potential[v])
                {
                    potential[v] = nd;
                    if (!inQueue[v])
                    {
                        inQueue[v] = true;
                        queue.Enqueue(v);
                    }
                }
            }
        }

        for (int v = 0; v < n; v++)
        {
            if (potential[v] == long.MaxValue)
            {
                potential[v] = 0; // unreachable nodes: any finite potential is fine
            }
        }
    }

    private bool Dijkstra(int source, int sink, long[] potential, long[] dist, int[] prevEdge)
    {
        int n = _adj.Count;
        Array.Fill(dist, long.MaxValue);
        dist[source] = 0;
        var visited = new bool[n];
        var pq = new PriorityQueue<int, long>();
        pq.Enqueue(source, 0);

        while (pq.TryDequeue(out int u, out _))
        {
            if (visited[u])
            {
                continue;
            }

            visited[u] = true;
            foreach (int e in _adj[u])
            {
                if (_cap[e] <= 0)
                {
                    continue;
                }

                int v = _head[e];
                long reduced = _cost[e] + potential[u] - potential[v];
                long nd = dist[u] + reduced;
                if (!visited[v] && nd < dist[v])
                {
                    dist[v] = nd;
                    prevEdge[v] = e;
                    pq.Enqueue(v, nd);
                }
            }
        }

        return dist[sink] != long.MaxValue;
    }
}

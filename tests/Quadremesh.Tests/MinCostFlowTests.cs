using Quadremesh.Core.Acceleration;
using Xunit;

namespace Quadremesh.Tests;

public class MinCostFlowTests
{
    [Fact]
    public void Single_path_flow_and_cost()
    {
        var f = new MinCostFlow();
        int s = f.AddNode();
        int t = f.AddNode();
        f.AddEdge(s, t, capacity: 5, cost: 4);

        (long flow, long cost) = f.MinCostMaxFlow(s, t);
        Assert.Equal(5, flow);
        Assert.Equal(20, cost); // 5 units * cost 4
    }

    [Fact]
    public void Parallel_arcs_saturate_both_at_their_costs()
    {
        var f = new MinCostFlow();
        int s = f.AddNode();
        int t = f.AddNode();
        f.AddEdge(s, t, capacity: 3, cost: 1);
        f.AddEdge(s, t, capacity: 2, cost: 5);

        (long flow, long cost) = f.MinCostMaxFlow(s, t);
        Assert.Equal(5, flow);
        Assert.Equal((3 * 1) + (2 * 5), cost);
    }

    [Fact]
    public void Routes_through_the_cheaper_intermediate_path()
    {
        // s->a->t (cost 1+1) vs s->b->t (cost 5+5); only 1 unit needed, takes the cheap route.
        var f = new MinCostFlow();
        int s = f.AddNode();
        int a = f.AddNode();
        int b = f.AddNode();
        int t = f.AddNode();
        f.AddEdge(s, a, 1, 1);
        f.AddEdge(a, t, 1, 1);
        f.AddEdge(s, b, 1, 5);
        f.AddEdge(b, t, 1, 5);

        (long flow, long cost) = f.MinCostMaxFlow(s, t);
        Assert.Equal(2, flow);          // both paths saturate at max flow
        Assert.Equal(2 + 10, cost);     // cheap path (2) + expensive path (10)
    }

    [Fact]
    public void Transportation_problem_reaches_known_optimum()
    {
        // Supplies S1=4, S2=2; demands D1=3, D2=3. Optimal assignment cost is 7.
        var f = new MinCostFlow();
        int s1 = f.AddNode();
        int s2 = f.AddNode();
        int d1 = f.AddNode();
        int d2 = f.AddNode();
        f.SetSupply(s1, 4);
        f.SetSupply(s2, 2);
        f.SetSupply(d1, -3);
        f.SetSupply(d2, -3);
        f.AddEdge(s1, d1, 100, 1);
        f.AddEdge(s1, d2, 100, 2);
        f.AddEdge(s2, d1, 100, 3);
        f.AddEdge(s2, d2, 100, 1);

        (bool feasible, long cost) = f.Solve();
        Assert.True(feasible);
        Assert.Equal(7, cost);
    }

    [Fact]
    public void Infeasible_when_demand_cannot_be_met()
    {
        var f = new MinCostFlow();
        int a = f.AddNode();
        int b = f.AddNode();
        f.SetSupply(a, 5);
        f.SetSupply(b, -5);
        f.AddEdge(a, b, capacity: 2, cost: 1); // only 2 of the 5 can flow

        (bool feasible, _) = f.Solve();
        Assert.False(feasible);
    }

    [Fact]
    public void Negative_cost_arcs_are_handled()
    {
        // A cheaper detour with a negative-cost arc must be preferred.
        var f = new MinCostFlow();
        int s = f.AddNode();
        int a = f.AddNode();
        int t = f.AddNode();
        f.AddEdge(s, t, 1, 0);
        f.AddEdge(s, a, 1, 1);
        f.AddEdge(a, t, 1, -5);

        (long flow, long cost) = f.MinCostMaxFlow(s, t);
        Assert.Equal(2, flow);
        Assert.Equal(0 + (1 - 5), cost); // direct (0) + detour (1 + -5 = -4)
    }
}

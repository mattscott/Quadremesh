using Quadremesh.Core.Extraction;
using Xunit;

namespace Quadremesh.Tests;

/// <summary>Unit tests for the dependency-free DPLL SAT solver behind the flip-fixer (LocalSat).</summary>
public class LocalSatTests
{
    private static readonly (int, int, int)[] NoEq = System.Array.Empty<(int, int, int)>();
    private static readonly (int, int, int, int)[] NoGeV = System.Array.Empty<(int, int, int, int)>();
    private static readonly (int, int)[] NoGeC = System.Array.Empty<(int, int)>();

    [Fact]
    public void Unconstrained_keeps_current_values()
    {
        var value = new[] { 1, -1, 0 };
        var flex = new[] { true, true, true };
        LocalSat.Status s = LocalSat.Solve(3, value, flex, NoEq, NoEq, NoGeV, NoGeC);
        Assert.Equal(LocalSat.Status.Sat, s);
        Assert.Equal(new[] { 1, -1, 0 }, value); // minimal-change bias keeps the originals
    }

    [Fact]
    public void Equality_constraint_is_satisfied()
    {
        // v0 + v1 + v2 == 0, currently (1,1,1) -> sum 3, must change.
        var value = new[] { 1, 1, 1 };
        var flex = new[] { true, true, true };
        var varEq = new[] { (0, 1, 2) };
        var cstEq = new[] { (1, 1, 1) };
        LocalSat.Status s = LocalSat.Solve(3, value, flex, varEq, cstEq, NoGeV, NoGeC);
        Assert.Equal(LocalSat.Status.Sat, s);
        Assert.Equal(0, value[0] + value[1] + value[2]);
    }

    [Fact]
    public void Inequality_constraint_is_satisfied()
    {
        // area = v0*v1 - v2*v3 >= 0, currently (1,-1, 1, 1) -> -1 -1 = -2 < 0 -> must flip.
        var value = new[] { 1, -1, 1, 1 };
        var flex = new[] { true, true, true, true };
        var varGe = new[] { (0, 1, 2, 3) };
        var cstGe = new[] { (1, 1) };
        LocalSat.Status s = LocalSat.Solve(4, value, flex, NoEq, NoEq, varGe, cstGe);
        Assert.Equal(LocalSat.Status.Sat, s);
        Assert.True(((value[0] * value[1]) - (value[2] * value[3])) >= 0);
    }

    [Fact]
    public void Infeasible_returns_unsat()
    {
        // v0 fixed to 1, but 3*v0 == 0 forces v0 == 0 -> contradiction.
        var value = new[] { 1 };
        var flex = new[] { false };
        var varEq = new[] { (0, 0, 0) };
        var cstEq = new[] { (1, 1, 1) };
        LocalSat.Status s = LocalSat.Solve(1, value, flex, varEq, cstEq, NoGeV, NoGeC);
        Assert.Equal(LocalSat.Status.Unsat, s);
    }

    [Fact]
    public void Fixed_variables_are_respected()
    {
        // v0 fixed to 1, v1/v2 flexible, sum must be 0 -> solver adjusts v1,v2 only.
        var value = new[] { 1, 1, 1 };
        var flex = new[] { false, true, true };
        var varEq = new[] { (0, 1, 2) };
        var cstEq = new[] { (1, 1, 1) };
        LocalSat.Status s = LocalSat.Solve(3, value, flex, varEq, cstEq, NoGeV, NoGeC);
        Assert.Equal(LocalSat.Status.Sat, s);
        Assert.Equal(1, value[0]);
        Assert.Equal(0, value[0] + value[1] + value[2]);
    }
}

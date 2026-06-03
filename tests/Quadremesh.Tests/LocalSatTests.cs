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

    // --- CDCL backend: differential vs DPLL, UNSAT (clause learning), and a stress instance ---------

    [Fact]
    public void Cdcl_matches_dpll_on_random_3sat()
    {
        var rng = new System.Random(20260603);
        int sat = 0, unsat = 0;
        for (int iter = 0; iter < 300; iter++)
        {
            int n = rng.Next(3, 28);
            int m = (int)(n * 4.26) + rng.Next(-3, 4);
            List<int[]> clauses = RandomThreeSat(n, m, rng);
            var prefer = new bool[n + 1];

            LocalSat.Status cdcl = LocalSat.SolveRaw(n, Clone(clauses), prefer, 5_000_000, useCdcl: true, out bool[] modelC);
            LocalSat.Status dpll = LocalSat.SolveRaw(n, Clone(clauses), prefer, 5_000_000, useCdcl: false, out _);

            Assert.NotEqual(LocalSat.Status.Timeout, cdcl);
            Assert.NotEqual(LocalSat.Status.Timeout, dpll);
            Assert.Equal(dpll, cdcl); // SAT/UNSAT verdict must agree

            if (cdcl == LocalSat.Status.Sat)
            {
                Assert.True(Satisfies(clauses, modelC), $"CDCL model does not satisfy instance (iter {iter})");
                sat++;
            }
            else
            {
                unsat++;
            }
        }

        // Sanity that the corpus actually exercised both verdicts (phase-transition ratio).
        Assert.True(sat > 20 && unsat > 20, $"degenerate corpus: sat={sat} unsat={unsat}");
    }

    [Theory]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    public void Cdcl_proves_pigeonhole_unsat(int holes)
    {
        // PHP(holes+1 pigeons, holes holes) is UNSAT — only provable with clause learning at speed.
        int pigeons = holes + 1;
        int Var(int p, int h) => (p * holes) + h + 1;
        int n = pigeons * holes;
        var clauses = new List<int[]>();
        for (int p = 0; p < pigeons; p++)
        {
            var atLeastOne = new int[holes];
            for (int h = 0; h < holes; h++)
            {
                atLeastOne[h] = Var(p, h);
            }

            clauses.Add(atLeastOne);
        }

        for (int h = 0; h < holes; h++)
        {
            for (int p = 0; p < pigeons; p++)
            {
                for (int q = p + 1; q < pigeons; q++)
                {
                    clauses.Add(new[] { -Var(p, h), -Var(q, h) });
                }
            }
        }

        LocalSat.Status st = LocalSat.SolveRaw(n, clauses, new bool[n + 1], 50_000_000, useCdcl: true, out _);
        Assert.Equal(LocalSat.Status.Unsat, st);
    }

    [Fact]
    public void Cdcl_solves_large_satisfiable_instance_fast()
    {
        // Scale up random 3-SAT below the threshold (mostly SAT), file-flushing progress so a crash
        // leaves the breaking size on disk.
        string dir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "quadremesh-cdcl");
        System.IO.Directory.CreateDirectory(dir);
        string logPath = System.IO.Path.Combine(dir, "stress.log");
        using var log = new System.IO.StreamWriter(logPath, append: false) { AutoFlush = true };

        // Random 3-SAT at the phase transition is adversarially hard; our real FixFlipSat instances are
        // far easier (structured, mostly-SAT-near-current). This bounds the size to keep CI fast while
        // still exercising thousands of variables + clause learning + deletion + restarts.
        var rng = new System.Random(7);
        foreach (int n in new[] { 50, 100, 200, 400, 800, 1600 })
        {
            List<int[]> clauses = RandomThreeSat(n, (int)(n * 3.5), rng);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var solver = new Cdcl(n, clauses, new bool[n + 1], 20_000_000);
            LocalSat.Status st = solver.Run();
            sw.Stop();
            log.WriteLine($"n={n} status={st} conflicts={solver.ConflictCount} clauses={solver.ClauseCount} ms={sw.ElapsedMilliseconds}");

            Assert.NotEqual(LocalSat.Status.Timeout, st);
            if (st == LocalSat.Status.Sat)
            {
                var model = new bool[n + 1];
                for (int v = 1; v <= n; v++)
                {
                    model[v] = solver.IsTrue(v);
                }

                Assert.True(Satisfies(clauses, model), $"n={n} model invalid");
            }
        }
    }

    private static List<int[]> RandomThreeSat(int n, int m, System.Random rng)
    {
        var clauses = new List<int[]>(m);
        for (int c = 0; c < m; c++)
        {
            int a = rng.Next(1, n + 1);
            int b;
            do
            {
                b = rng.Next(1, n + 1);
            }
            while (b == a);

            int d;
            do
            {
                d = rng.Next(1, n + 1);
            }
            while (d == a || d == b);

            clauses.Add(new[]
            {
                rng.Next(2) == 0 ? a : -a,
                rng.Next(2) == 0 ? b : -b,
                rng.Next(2) == 0 ? d : -d,
            });
        }

        return clauses;
    }

    private static List<int[]> Clone(List<int[]> clauses)
    {
        var copy = new List<int[]>(clauses.Count);
        foreach (int[] cl in clauses)
        {
            copy.Add((int[])cl.Clone());
        }

        return copy;
    }

    private static bool Satisfies(List<int[]> clauses, bool[] model)
    {
        foreach (int[] cl in clauses)
        {
            bool ok = false;
            foreach (int lit in cl)
            {
                if ((lit > 0) == model[System.Math.Abs(lit)])
                {
                    ok = true;
                    break;
                }
            }

            if (!ok)
            {
                return false;
            }
        }

        return true;
    }
}

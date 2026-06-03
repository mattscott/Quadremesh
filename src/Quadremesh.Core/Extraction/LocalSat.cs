namespace Quadremesh.Core.Extraction;

/// <summary>
/// Self-contained replacement for QuadriFlow's localsat.cpp (which shells out to <c>minisat</c>): a
/// small DPLL SAT solver plus <see cref="Solve"/>, the per-group integer feasibility problem used by
/// <see cref="EdgeGraphHierarchy"/>'s SAT flip-fixer. Every variable is an edge-diff component
/// constrained to {−1, 0, +1}; the problem asks for an assignment that keeps every face's divergence
/// zero (equality constraints) and every face's lattice area non-negative (inequality constraints),
/// biased toward keeping the current values (minimal change). Reimplemented clean-room from QuadriFlow
/// (MIT); the CNF encoding mirrors <c>SolveSatProblem</c> exactly, only the back-end solver differs.
/// </summary>
internal static class LocalSat
{
    internal enum Status
    {
        Sat,
        Unsat,
        Timeout,
    }

    /// <summary>
    /// Solves one flip-fix group. <paramref name="value"/> holds the current diff component of each
    /// variable (each in {−1,0,+1}); on <see cref="Status.Sat"/> it is overwritten with the solution.
    /// <paramref name="flexible"/> marks the variables the solver may change. Each equality constraint
    /// <c>(var, cst)</c> requires <c>cst.a·v[var.a] + cst.b·v[var.b] + cst.c·v[var.c] == 0</c>; each
    /// inequality requires <c>cst.a·v[var.a]·v[var.b] − cst.b·v[var.c]·v[var.d] ≥ 0</c>.
    /// </summary>
    public static Status Solve(
        int nVariable,
        int[] value,
        bool[] flexible,
        IReadOnlyList<(int A, int B, int C)> variableEq,
        IReadOnlyList<(int A, int B, int C)> constantEq,
        IReadOnlyList<(int A, int B, int C, int D)> variableGe,
        IReadOnlyList<(int A, int B)> constantGe,
        long conflictBudget = 2_000_000)
    {
        // SAT variable layout: boolean (3*i + v + 1) is true iff integer variable i takes value (v-1),
        // for v in {0,1,2} -> {-1,0,+1}. (1-based literals, DIMACS-style.)
        static int Lit(int i, int valueMinus1Plus1) => (3 * i) + (valueMinus1Plus1 + 1) + 1;

        var clauses = new List<int[]>();

        for (int i = 0; i < nVariable; i++)
        {
            clauses.Add(new[] { -Lit(i, -1), -Lit(i, 0) });
            clauses.Add(new[] { -Lit(i, 1), -Lit(i, 0) });
            clauses.Add(new[] { -Lit(i, -1), -Lit(i, 1) });
            clauses.Add(new[] { Lit(i, -1), Lit(i, 0), Lit(i, 1) });
            if (!flexible[i])
            {
                clauses.Add(new[] { Lit(i, value[i]) });
            }
        }

        for (int i = 0; i < variableEq.Count; i++)
        {
            (int a, int b, int c) = variableEq[i];
            (int ca, int cb, int cc) = constantEq[i];
            for (int v0 = -1; v0 <= 1; v0++)
            {
                for (int v1 = -1; v1 <= 1; v1++)
                {
                    for (int v2 = -1; v2 <= 1; v2++)
                    {
                        if ((ca * v0) + (cb * v1) + (cc * v2) != 0)
                        {
                            clauses.Add(new[] { -Lit(a, v0), -Lit(b, v1), -Lit(c, v2) });
                        }
                    }
                }
            }
        }

        for (int i = 0; i < variableGe.Count; i++)
        {
            (int a, int b, int c, int d) = variableGe[i];
            (int ca, int cb) = constantGe[i];
            for (int v0 = -1; v0 <= 1; v0++)
            {
                for (int v1 = -1; v1 <= 1; v1++)
                {
                    for (int v2 = -1; v2 <= 1; v2++)
                    {
                        for (int v3 = -1; v3 <= 1; v3++)
                        {
                            if (((ca * v0 * v1) - (cb * v2 * v3)) < 0)
                            {
                                clauses.Add(new[] { -Lit(a, v0), -Lit(b, v1), -Lit(c, v2), -Lit(d, v3) });
                            }
                        }
                    }
                }
            }
        }

        int nSat = 3 * nVariable;

        // Bias each integer variable toward its current value (minimal change), matching the reference's
        // "initialize to current, flip only if needed" objective.
        var prefer = new bool[nSat + 1];
        for (int i = 0; i < nVariable; i++)
        {
            prefer[Lit(i, value[i])] = true;
        }

        var solver = new Cdcl(nSat, clauses, prefer, conflictBudget);
        Status status = solver.Run();
        if (status != Status.Sat)
        {
            return status;
        }

        for (int i = 0; i < nVariable; i++)
        {
            int found = value[i];
            for (int v = -1; v <= 1; v++)
            {
                if (solver.IsTrue(Lit(i, v)))
                {
                    found = v;
                }
            }

            value[i] = found;
        }

        return Status.Sat;
    }

    /// <summary>
    /// Test/diagnostic hook: solve a raw boolean CNF with either backend (CDCL or the legacy DPLL),
    /// used to differential-test the two solvers. Returns the status and, on SAT, the model (var → bool).
    /// </summary>
    internal static Status SolveRaw(int nVar, List<int[]> clauses, bool[] prefer, long budget, bool useCdcl, out bool[] model)
    {
        model = new bool[nVar + 1];
        if (useCdcl)
        {
            var s = new Cdcl(nVar, clauses, prefer, budget);
            Status st = s.Run();
            if (st == Status.Sat)
            {
                for (int v = 1; v <= nVar; v++)
                {
                    model[v] = s.IsTrue(v);
                }
            }

            return st;
        }

        var d = new Dpll(nVar, clauses, prefer, budget);
        Status dst = d.Run();
        if (dst == Status.Sat)
        {
            for (int v = 1; v <= nVar; v++)
            {
                model[v] = d.IsTrue(v);
            }
        }

        return dst;
    }

    /// <summary>Minimal iterative DPLL: unit propagation, chronological backtracking, polarity bias, budget.</summary>
    private sealed class Dpll
    {
        private readonly int _nVar;
        private readonly int[][] _clauses;
        private readonly bool[] _prefer;
        private readonly long _conflictBudget;
        private readonly sbyte[] _assign;       // var 1..n: -1 unassigned, 0 false, 1 true
        private readonly List<int> _trail = new();
        private readonly List<Decision> _decisions = new();
        private long _conflicts;

        private struct Decision
        {
            public int TrailMark;   // _trail.Count before the decision
            public int Var;
            public bool TriedSecond;
        }

        public Dpll(int nVar, List<int[]> clauses, bool[] prefer, long conflictBudget)
        {
            _nVar = nVar;
            _clauses = clauses.ToArray();
            _prefer = prefer;
            _conflictBudget = conflictBudget;
            _assign = new sbyte[nVar + 1];
            Array.Fill(_assign, (sbyte)-1);
        }

        public bool IsTrue(int var) => _assign[var] == 1;

        public Status Run()
        {
            if (!Propagate())
            {
                return Status.Unsat;
            }

            while (true)
            {
                int v = PickUnassigned();
                if (v == 0)
                {
                    return Status.Sat;
                }

                _decisions.Add(new Decision { TrailMark = _trail.Count, Var = v, TriedSecond = false });
                Set(v, _prefer[v]);

                while (!Propagate())
                {
                    if (++_conflicts > _conflictBudget)
                    {
                        return Status.Timeout;
                    }

                    if (!Backtrack())
                    {
                        return Status.Unsat;
                    }
                }
            }
        }

        private bool Backtrack()
        {
            while (_decisions.Count > 0)
            {
                Decision d = _decisions[^1];
                UndoTo(d.TrailMark);
                if (!d.TriedSecond)
                {
                    d.TriedSecond = true;
                    _decisions[^1] = d;
                    Set(d.Var, !_prefer[d.Var]);
                    return true;
                }

                _decisions.RemoveAt(_decisions.Count - 1);
            }

            return false;
        }

        private void UndoTo(int mark)
        {
            for (int i = _trail.Count - 1; i >= mark; i--)
            {
                _assign[_trail[i]] = -1;
            }

            _trail.RemoveRange(mark, _trail.Count - mark);
        }

        private void Set(int var, bool val)
        {
            _assign[var] = (sbyte)(val ? 1 : 0);
            _trail.Add(var);
        }

        private bool Propagate()
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                foreach (int[] clause in _clauses)
                {
                    int unassigned = 0;
                    int lastLit = 0;
                    bool satisfied = false;
                    foreach (int lit in clause)
                    {
                        sbyte a = _assign[lit < 0 ? -lit : lit];
                        if (a == -1)
                        {
                            unassigned++;
                            lastLit = lit;
                        }
                        else if ((lit > 0) == (a == 1))
                        {
                            satisfied = true;
                            break;
                        }
                    }

                    if (satisfied)
                    {
                        continue;
                    }

                    if (unassigned == 0)
                    {
                        return false; // conflict
                    }

                    if (unassigned == 1)
                    {
                        Set(lastLit < 0 ? -lastLit : lastLit, lastLit > 0);
                        changed = true;
                    }
                }
            }

            return true;
        }

        private int PickUnassigned()
        {
            for (int v = 1; v <= _nVar; v++)
            {
                if (_assign[v] == -1)
                {
                    return v;
                }
            }

            return 0;
        }
    }
}

namespace Quadremesh.Core.Extraction;

/// <summary>
/// A compact, dependency-free CDCL SAT solver (Conflict-Driven Clause Learning) used by
/// <see cref="LocalSat"/> behind the flip-fixer. It replaces the naive DPLL backend, which could not
/// solve the larger flip-fix groups in reasonable time. Implements the standard MiniSat-style machinery:
/// two-watched-literal unit propagation, 1-UIP conflict analysis with clause learning, non-chronological
/// backjumping, a VSIDS branching heuristic (binary heap), geometric restarts, and phase saving (biased
/// to the caller's preferred polarity, giving the minimal-change behaviour the flip-fixer wants).
/// Reimplemented clean-room from the MiniSat description (Eén &amp; Sörensson, MIT) — read for
/// understanding only.
/// </summary>
/// <remarks>
/// Literals are signed ints (±var, var in 1..n), matching <see cref="LocalSat"/>'s clause format. The
/// reason clause of a propagated literal always keeps that literal at index 0 (the watched-literal scheme
/// guarantees it), which conflict analysis relies on.
/// </remarks>
internal sealed class Cdcl
{
    private readonly int _nVar;
    private readonly List<int[]?> _clauses;         // original (shared, reordered in place) + learnt; null = tombstoned
    private readonly bool[] _prefer;                // initial decision polarity per var (1..n)
    private readonly long _conflictBudget;

    private readonly sbyte[] _value;                // var -> -1 unassigned / 0 false / 1 true
    private readonly int[] _level;                  // var -> decision level
    private readonly int[] _reason;                 // var -> clause id that implied it (-1 decision/none)
    private readonly bool[] _polarity;              // saved phase per var
    private readonly List<int> _trail = new();
    private readonly List<int> _trailLim = new();   // trail index where each decision level begins
    private readonly List<int>[] _watches;          // litIndex -> clause ids watching that literal
    private readonly List<int> _units = new();      // size-1 input clauses

    private readonly double[] _activity;            // var -> VSIDS activity
    private double _varInc = 1.0;
    private const double VarDecay = 0.95;
    private readonly VarHeap _order;

    private readonly List<double> _clauseAct = new(); // per-clause activity (parallel to _clauses)
    private double _clauseInc = 1.0;
    private int _nOriginal;                          // clauses[0.._nOriginal) are original (never deleted)
    private int _liveLearnt;                         // live (non-tombstoned) learnt clauses
    private long _maxLearnt;

    private readonly bool[] _seen;                  // analyze scratch
    private int _qhead;
    private bool _rootUnsat;

    public Cdcl(int nVar, List<int[]> clauses, bool[] prefer, long conflictBudget = 20_000_000)
    {
        _nVar = nVar;
        _prefer = prefer;
        _conflictBudget = conflictBudget;
        _value = new sbyte[nVar + 1];
        Array.Fill(_value, (sbyte)-1);
        _level = new int[nVar + 1];
        _reason = new int[nVar + 1];
        Array.Fill(_reason, -1);
        _polarity = new bool[nVar + 1];
        for (int v = 1; v <= nVar; v++)
        {
            _polarity[v] = prefer[v];
        }

        _watches = new List<int>[(2 * nVar) + 2];
        for (int i = 0; i < _watches.Length; i++)
        {
            _watches[i] = new List<int>();
        }

        _activity = new double[nVar + 1];
        _seen = new bool[nVar + 1];
        _order = new VarHeap(_activity, nVar);

        _clauses = new List<int[]?>(clauses.Count);
        foreach (int[] cl in clauses)
        {
            if (cl.Length == 0)
            {
                _rootUnsat = true;
                continue;
            }

            if (cl.Length == 1)
            {
                _units.Add(cl[0]);
                continue;
            }

            int id = _clauses.Count;
            _clauses.Add(cl);
            _clauseAct.Add(0);
            _watches[LitIndex(cl[0])].Add(id);
            _watches[LitIndex(cl[1])].Add(id);
        }

        _nOriginal = _clauses.Count;
        _maxLearnt = System.Math.Max(_nOriginal / 3, 1000);

        for (int v = 1; v <= nVar; v++)
        {
            _order.Insert(v);
        }
    }

    /// <summary>Truth of variable <paramref name="var"/> in the found model (call only after <see cref="Status.Sat"/>).</summary>
    public bool IsTrue(int var) => _value[var] == 1;

    /// <summary>Conflicts encountered in the last <see cref="Run"/> (diagnostic).</summary>
    internal long ConflictCount { get; private set; }

    /// <summary>Learnt clauses retained (diagnostic; original clause count is fixed at construction).</summary>
    internal int ClauseCount => _clauses.Count;

    public LocalSat.Status Run()
    {
        if (_rootUnsat)
        {
            return LocalSat.Status.Unsat;
        }

        // Root-level units.
        foreach (int lit in _units)
        {
            int v = System.Math.Abs(lit);
            sbyte want = (sbyte)(lit > 0 ? 1 : 0);
            if (_value[v] == -1)
            {
                Enqueue(lit, -1);
            }
            else if (_value[v] != want)
            {
                return LocalSat.Status.Unsat;
            }
        }

        if (Propagate() != -1)
        {
            return LocalSat.Status.Unsat;
        }

        long conflicts = 0;
        int restartNum = 1;
        long restartLimit = 100;
        long sinceRestart = 0;
        var learnt = new List<int>();

        while (true)
        {
            int confl = Propagate();
            if (confl != -1)
            {
                conflicts++;
                ConflictCount = conflicts;
                sinceRestart++;
                if (DecisionLevel() == 0)
                {
                    return LocalSat.Status.Unsat;
                }

                int btLevel = Analyze(confl, learnt);
                CancelUntil(btLevel);
                Record(learnt);
                DecayVarActivity();
                _clauseInc *= 1.0 / 0.999;

                if (conflicts > _conflictBudget)
                {
                    return LocalSat.Status.Timeout;
                }

                // Bound memory independently of restarts: ReduceDb keeps locked reasons so it is valid at
                // any decision level.
                if (_liveLearnt > _maxLearnt)
                {
                    ReduceDb();
                    _maxLearnt += _maxLearnt / 10;
                }

                if (sinceRestart >= restartLimit)
                {
                    sinceRestart = 0;
                    restartNum++;
                    restartLimit = 100 * Luby(restartNum);
                    CancelUntil(0);
                }
            }
            else
            {
                if (_trail.Count == _nVar)
                {
                    return LocalSat.Status.Sat;
                }

                int v = PickBranch();
                if (v == 0)
                {
                    return LocalSat.Status.Sat;
                }

                _trailLim.Add(_trail.Count);
                Enqueue(_polarity[v] ? v : -v, -1);
            }
        }
    }

    // --- core ------------------------------------------------------------------------------------

    private int DecisionLevel() => _trailLim.Count;

    private static int LitIndex(int lit) => lit > 0 ? 2 * lit : (2 * -lit) + 1;

    private bool IsTrueLit(int lit)
    {
        sbyte a = _value[System.Math.Abs(lit)];
        return a != -1 && ((lit > 0) == (a == 1));
    }

    private bool IsFalseLit(int lit)
    {
        sbyte a = _value[System.Math.Abs(lit)];
        return a != -1 && ((lit > 0) == (a == 0));
    }

    private void Enqueue(int lit, int reason)
    {
        int v = System.Math.Abs(lit);
        _value[v] = (sbyte)(lit > 0 ? 1 : 0);
        _level[v] = DecisionLevel();
        _reason[v] = reason;
        _trail.Add(v);
    }

    private int Propagate()
    {
        while (_qhead < _trail.Count)
        {
            int v = _trail[_qhead++];
            int trueLit = _value[v] == 1 ? v : -v;
            int falseLit = -trueLit;
            List<int> ws = _watches[LitIndex(falseLit)];
            int i = 0, j = 0;
            while (i < ws.Count)
            {
                int c = ws[i++];
                int[]? cl = _clauses[c];
                if (cl is null)
                {
                    continue; // tombstoned (deleted) clause — lazily drop from this watch list
                }

                if (cl[0] == falseLit)
                {
                    cl[0] = cl[1];
                    cl[1] = falseLit;
                }

                int first = cl[0];
                if (IsTrueLit(first))
                {
                    ws[j++] = c; // clause already satisfied; keep the watch
                    continue;
                }

                int newWatch = -1;
                for (int k = 2; k < cl.Length; k++)
                {
                    if (!IsFalseLit(cl[k]))
                    {
                        newWatch = k;
                        break;
                    }
                }

                if (newWatch != -1)
                {
                    cl[1] = cl[newWatch];
                    cl[newWatch] = falseLit;
                    _watches[LitIndex(cl[1])].Add(c); // moved to a new watch list; drop from this one
                    continue;
                }

                ws[j++] = c; // still watching falseLit
                if (IsFalseLit(first))
                {
                    while (i < ws.Count)
                    {
                        ws[j++] = ws[i++];
                    }

                    ws.RemoveRange(j, ws.Count - j);
                    return c; // conflict
                }

                Enqueue(first, c); // unit
            }

            ws.RemoveRange(j, ws.Count - j);
        }

        return -1;
    }

    /// <summary>1-UIP conflict analysis; fills <paramref name="learnt"/> (UIP at index 0) and returns the backjump level.</summary>
    private int Analyze(int confl, List<int> learnt)
    {
        learnt.Clear();
        learnt.Add(0); // placeholder for the UIP literal
        int pathCount = 0;
        int p = 0;
        int idx = _trail.Count - 1;

        do
        {
            int[] cl = _clauses[confl]!;
            if (confl >= _nOriginal)
            {
                BumpClause(confl);
            }

            for (int jj = p == 0 ? 0 : 1; jj < cl.Length; jj++)
            {
                int q = cl[jj];
                int v = System.Math.Abs(q);
                if (!_seen[v] && _level[v] > 0)
                {
                    BumpVar(v);
                    _seen[v] = true;
                    if (_level[v] >= DecisionLevel())
                    {
                        pathCount++;
                    }
                    else
                    {
                        learnt.Add(q);
                    }
                }
            }

            while (!_seen[_trail[idx]])
            {
                idx--;
            }

            int uv = _trail[idx];
            p = _value[uv] == 1 ? uv : -uv;
            confl = _reason[uv];
            _seen[uv] = false;
            pathCount--;
            idx--;
        }
        while (pathCount > 0);

        learnt[0] = -p;

        int btLevel;
        if (learnt.Count == 1)
        {
            btLevel = 0;
        }
        else
        {
            int maxI = 1;
            for (int k = 2; k < learnt.Count; k++)
            {
                if (_level[System.Math.Abs(learnt[k])] > _level[System.Math.Abs(learnt[maxI])])
                {
                    maxI = k;
                }
            }

            (learnt[1], learnt[maxI]) = (learnt[maxI], learnt[1]);
            btLevel = _level[System.Math.Abs(learnt[1])];
        }

        for (int k = 1; k < learnt.Count; k++)
        {
            _seen[System.Math.Abs(learnt[k])] = false;
        }

        return btLevel;
    }

    private void Record(List<int> learnt)
    {
        if (learnt.Count == 1)
        {
            Enqueue(learnt[0], -1);
            return;
        }

        int c = _clauses.Count;
        _clauses.Add(learnt.ToArray());
        _clauseAct.Add(_clauseInc);
        _liveLearnt++;
        _watches[LitIndex(learnt[0])].Add(c);
        _watches[LitIndex(learnt[1])].Add(c);
        Enqueue(learnt[0], c);
    }

    private void BumpClause(int c)
    {
        _clauseAct[c] += _clauseInc;
        if (_clauseAct[c] > 1e20)
        {
            for (int i = _nOriginal; i < _clauseAct.Count; i++)
            {
                _clauseAct[i] *= 1e-20;
            }

            _clauseInc *= 1e-20;
        }
    }

    /// <summary>
    /// Tombstone the lower-activity half of learnt clauses (keeping originals, currently-locked reasons,
    /// and size-2 clauses). Must be called at decision level 0. Dead clause ids are scrubbed from watch
    /// lists lazily during propagation. This bounds memory so hard instances don't OOM.
    /// </summary>
    private void ReduceDb()
    {
        var locked = new bool[_clauses.Count];
        foreach (int v in _trail)
        {
            int r = _reason[v];
            if (r >= 0)
            {
                locked[r] = true;
            }
        }

        var learnt = new List<int>();
        for (int c = _nOriginal; c < _clauses.Count; c++)
        {
            if (_clauses[c] is not null)
            {
                learnt.Add(c);
            }
        }

        learnt.Sort((a, b) => _clauseAct[a].CompareTo(_clauseAct[b]));
        int half = learnt.Count / 2;
        for (int k = 0; k < half; k++)
        {
            int c = learnt[k];
            if (!locked[c] && _clauses[c]!.Length > 2)
            {
                _clauses[c] = null;
                _liveLearnt--;
            }
        }
    }

    private void CancelUntil(int level)
    {
        if (DecisionLevel() <= level)
        {
            return;
        }

        int from = _trailLim[level];
        for (int c = _trail.Count - 1; c >= from; c--)
        {
            int v = _trail[c];
            _polarity[v] = _value[v] == 1;
            _value[v] = -1;
            _reason[v] = -1;
            _order.Insert(v);
        }

        _trail.RemoveRange(from, _trail.Count - from);
        _trailLim.RemoveRange(level, _trailLim.Count - level);
        _qhead = _trail.Count;
    }

    private int PickBranch()
    {
        while (true)
        {
            int v = _order.RemoveMax();
            if (v == 0)
            {
                return 0;
            }

            if (_value[v] == -1)
            {
                return v;
            }
        }
    }

    private void BumpVar(int v)
    {
        _activity[v] += _varInc;
        if (_activity[v] > 1e100)
        {
            for (int i = 1; i <= _nVar; i++)
            {
                _activity[i] *= 1e-100;
            }

            _varInc *= 1e-100;
        }

        _order.Increase(v);
    }

    private void DecayVarActivity() => _varInc *= 1.0 / VarDecay;

    /// <summary>The i-th (1-indexed) value of the Luby restart sequence: 1,1,2,1,1,2,4,1,1,2,1,1,2,4,8,…</summary>
    private static long Luby(int i)
    {
        int k = 1;
        while (true)
        {
            if (i == (1 << k) - 1)
            {
                return 1L << (k - 1);
            }

            if (i < (1 << k) - 1)
            {
                return Luby(i - (1 << (k - 1)) + 1);
            }

            k++;
        }
    }

    /// <summary>Indexed binary max-heap over variables keyed by VSIDS activity (deterministic ties by var id).</summary>
    private sealed class VarHeap
    {
        private readonly double[] _act;
        private readonly List<int> _heap = new();
        private readonly int[] _pos; // var -> index in _heap, or -1 if absent

        public VarHeap(double[] activity, int nVar)
        {
            _act = activity;
            _pos = new int[nVar + 1];
            Array.Fill(_pos, -1);
        }

        public void Insert(int v)
        {
            if (_pos[v] != -1)
            {
                return;
            }

            _heap.Add(v);
            int i = _heap.Count - 1;
            _pos[v] = i;
            Up(i);
        }

        public void Increase(int v)
        {
            int i = _pos[v];
            if (i != -1)
            {
                Up(i);
            }
        }

        public int RemoveMax()
        {
            if (_heap.Count == 0)
            {
                return 0;
            }

            int top = _heap[0];
            _pos[top] = -1;
            int last = _heap[^1];
            _heap.RemoveAt(_heap.Count - 1);
            if (_heap.Count > 0)
            {
                _heap[0] = last;
                _pos[last] = 0;
                Down(0);
            }

            return top;
        }

        private bool Higher(int a, int b) => _act[a] > _act[b] || (_act[a] == _act[b] && a < b);

        private void Up(int i)
        {
            int v = _heap[i];
            while (i > 0)
            {
                int parent = (i - 1) >> 1;
                if (!Higher(v, _heap[parent]))
                {
                    break;
                }

                _heap[i] = _heap[parent];
                _pos[_heap[i]] = i;
                i = parent;
            }

            _heap[i] = v;
            _pos[v] = i;
        }

        private void Down(int i)
        {
            int v = _heap[i];
            int n = _heap.Count;
            while (true)
            {
                int l = (2 * i) + 1;
                if (l >= n)
                {
                    break;
                }

                int r = l + 1;
                int child = (r < n && Higher(_heap[r], _heap[l])) ? r : l;
                if (!Higher(_heap[child], v))
                {
                    break;
                }

                _heap[i] = _heap[child];
                _pos[_heap[i]] = i;
                i = child;
            }

            _heap[i] = v;
            _pos[v] = i;
        }
    }
}

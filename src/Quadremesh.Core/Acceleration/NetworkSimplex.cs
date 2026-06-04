namespace Quadremesh.Core.Acceleration;

/// <summary>
/// Faithful clean-room port of <c>lemon::NetworkSimplex</c> (BLOCK_SEARCH pivot rule), restricted to the
/// EQ-supply case (Σ supply = 0) that QuadriFlow's <c>-mcf</c> flow uses via
/// <c>stSupply(source, sink, maxflow)</c>. The goal is a <b>bit-identical</b> min-cost-max-flow solution to
/// LEMON's, so our post-flow <c>edge_diff</c> matches the reference dump (plan 08). Reproduces the
/// determinism-critical details: the default <c>arc_mixing</c> permutation, SmartDigraph's reverse-insertion
/// <c>ArcIt</c>/<c>InArcIt</c> order, <c>initialPivots</c>' reverse sink→source search, and the exact
/// tie-breaks in <c>findLeavingArc</c>. Reimplemented from LEMON 1.3.1 (Boost Software License, read for
/// understanding only).
/// </summary>
internal sealed class NetworkSimplex
{
    private const long Max = long.MaxValue;
    private const long Inf = long.MaxValue;
    private const long ArtCost = 1L << 60; // dominates any real cost; exact value doesn't affect the optimum

    private const sbyte StateUpper = -1;
    private const sbyte StateTree = 0;
    private const sbyte StateLower = 1;
    private const sbyte DirDown = -1;
    private const sbyte DirUp = 1;

    private int _nodeNum;

    // Arcs in INSERTION order (the SmartDigraph build order). Each AddEdge appends 1–2 of these.
    private readonly List<int> _insSource = new();
    private readonly List<int> _insTarget = new();
    private readonly List<long> _insCap = new();
    private readonly List<long> _insCost = new();
    private readonly List<int> _insVar = new();   // edge-component id (var) or -1
    private readonly List<int> _insSign = new();  // +1 forward, -1 reverse

    // Internal (post-mixing) arc arrays, sized _allArcNum = _arcNum + _nodeNum.
    private int _arcNum;
    private int _allArcNum;
    private int _searchArcNum;
    private int[] _source = System.Array.Empty<int>();
    private int[] _target = System.Array.Empty<int>();
    private long[] _cap = System.Array.Empty<long>();
    private long[] _cost = System.Array.Empty<long>();
    private long[] _flow = System.Array.Empty<long>();
    private sbyte[] _state = System.Array.Empty<sbyte>();

    // insertion arc k -> internal mixed id; and the reverse for applyTo.
    private int[] _internalId = System.Array.Empty<int>();

    // Spanning-tree structure (size _nodeNum + 1; index _root = _nodeNum).
    private long[] _supply = System.Array.Empty<long>();
    private long[] _pi = System.Array.Empty<long>();
    private int[] _parent = System.Array.Empty<int>();
    private int[] _pred = System.Array.Empty<int>();
    private int[] _thread = System.Array.Empty<int>();
    private int[] _revThread = System.Array.Empty<int>();
    private int[] _succNum = System.Array.Empty<int>();
    private int[] _lastSucc = System.Array.Empty<int>();
    private sbyte[] _predDir = System.Array.Empty<sbyte>();
    private readonly List<int> _dirtyRevs = new();
    private int _root;

    // Pivot iteration temporaries.
    private int _inArc, _join, _uIn, _vIn, _uOut, _vOut;
    private long _delta;
    private int _blockSize;
    private int _nextArc;

    /// <summary>Sets the node count (nodes are 0..n-1; the reference's super-source = 0, super-sink = n-1).</summary>
    public void Resize(int nodeCount) => _nodeNum = nodeCount;

    /// <summary>
    /// Adds an undirected pair like LEMON's helper: a forward arc x→y with capacity <paramref name="c"/> and
    /// a reverse arc y→x with capacity <paramref name="rc"/> (each only if its capacity is non-zero), both
    /// cost <paramref name="cost"/>, carrying edge-component <paramref name="v"/> (or -1) for applyTo.
    /// </summary>
    public void AddEdge(int x, int y, long c, long rc, int v, long cost = 1)
    {
        if (c != 0)
        {
            _insSource.Add(x);
            _insTarget.Add(y);
            _insCap.Add(c);
            _insCost.Add(cost);
            _insVar.Add(v);
            _insSign.Add(1);
        }

        if (rc != 0)
        {
            _insSource.Add(y);
            _insTarget.Add(x);
            _insCap.Add(rc);
            _insCost.Add(cost);
            _insVar.Add(v);
            _insSign.Add(-1);
        }
    }

    /// <summary>Runs Preflow (max-flow value) then NetworkSimplex (min-cost flow of that value). Returns maxflow.</summary>
    public long Compute()
    {
        long maxflow = MaxFlowValue();
        BuildInternalArcs();
        Init(maxflow);
        if (!InitialPivots())
        {
            throw new System.InvalidOperationException("NetworkSimplex: unbounded (initialPivots).");
        }

        Start();
        return maxflow;
    }

    /// <summary>
    /// Accumulates the solved flow into <paramref name="componentDelta"/> (indexed by edge component
    /// <c>e*2+dim</c>): <c>componentDelta[var] -= sign · flow</c> per arc, mirroring the reference's
    /// <c>edge_diff[var/2][var%2] -= sgn·flow</c>. The caller adds this onto its edge_diff (kept free of the
    /// Vec2i type to avoid an Acceleration→Extraction dependency).
    /// </summary>
    public void ApplyTo(int[] componentDelta)
    {
        for (int k = 0; k < _insVar.Count; k++)
        {
            int var = _insVar[k];
            if (var < 0)
            {
                continue;
            }

            componentDelta[var] -= (int)(_insSign[k] * _flow[_internalId[k]]);
        }
    }

    // --- Preflow: only the max-flow VALUE is needed (unique); a simple Dinic over the insertion arcs. ----
    private long MaxFlowValue()
    {
        int n = _nodeNum;
        int m = _insSource.Count;
        var head = new int[2 * m];
        var capr = new long[2 * m];
        var next = new int[2 * m];
        var first = new int[n];
        System.Array.Fill(first, -1);
        int ec = 0;
        void Add(int u, int v, long cap)
        {
            head[ec] = v; capr[ec] = cap; next[ec] = first[u]; first[u] = ec++;
            head[ec] = u; capr[ec] = 0; next[ec] = first[v]; first[v] = ec++;
        }

        for (int k = 0; k < m; k++)
        {
            Add(_insSource[k], _insTarget[k], _insCap[k]);
        }

        int source = 0, sink = n - 1;
        var level = new int[n];
        var it = new int[n];
        long total = 0;
        var q = new int[n];
        while (true)
        {
            System.Array.Fill(level, -1);
            int qh = 0, qt = 0;
            level[source] = 0;
            q[qt++] = source;
            while (qh < qt)
            {
                int u = q[qh++];
                for (int e = first[u]; e != -1; e = next[e])
                {
                    if (capr[e] > 0 && level[head[e]] < 0)
                    {
                        level[head[e]] = level[u] + 1;
                        q[qt++] = head[e];
                    }
                }
            }

            if (level[sink] < 0)
            {
                break;
            }

            System.Array.Copy(first, it, n);
            long f;
            while ((f = Dfs(source, sink, Inf, head, capr, next, it, level)) > 0)
            {
                total += f;
            }
        }

        return total;
    }

    private static long Dfs(int u, int sink, long pushed, int[] head, long[] capr, int[] next, int[] it, int[] level)
    {
        if (u == sink)
        {
            return pushed;
        }

        for (; it[u] != -1; it[u] = next[it[u]])
        {
            int e = it[u];
            int v = head[e];
            if (capr[e] > 0 && level[v] == level[u] + 1)
            {
                long d = Dfs(v, sink, System.Math.Min(pushed, capr[e]), head, capr, next, it, level);
                if (d > 0)
                {
                    capr[e] -= d;
                    capr[e ^ 1] += d;
                    return d;
                }
            }
        }

        return 0;
    }

    // --- Arc mixing (SmartDigraph default): assign internal ids scattering by `skip`, iterating arcs in
    //     ArcIt order = REVERSE insertion. ----------------------------------------------------------------
    private void BuildInternalArcs()
    {
        _arcNum = _insSource.Count;
        _allArcNum = _arcNum + _nodeNum;
        _source = new int[_allArcNum];
        _target = new int[_allArcNum];
        _cap = new long[_allArcNum];
        _cost = new long[_allArcNum];
        _flow = new long[_allArcNum];
        _state = new sbyte[_allArcNum];
        _internalId = new int[_arcNum];
        var insVarSign = new (int Var, int Sign)[_allArcNum];

        if (_arcNum == 0)
        {
            return;
        }

        // LEMON's NodeIt is reverse-insertion too, so its _node_id reverses the logical node numbering:
        // logical node v -> internal node (_nodeNum-1-v). Apply that relabel to every arc endpoint.
        int last = _nodeNum - 1;
        bool mixing = _nodeNum > 1;
        if (mixing)
        {
            int skip = System.Math.Max(_arcNum / _nodeNum, 3);
            int i = 0, j = 0;
            // ArcIt iterates newest→oldest: insertion index k = _arcNum-1 .. 0.
            for (int k = _arcNum - 1; k >= 0; k--)
            {
                _internalId[k] = i;
                _source[i] = last - _insSource[k];
                _target[i] = last - _insTarget[k];
                _cap[i] = _insCap[k];
                _cost[i] = _insCost[k];
                insVarSign[i] = (_insVar[k], _insSign[k]);
                i += skip;
                if (i >= _arcNum)
                {
                    i = ++j;
                }
            }
        }
        else
        {
            for (int k = _arcNum - 1, i = 0; k >= 0; k--, i++)
            {
                _internalId[k] = i;
                _source[i] = last - _insSource[k];
                _target[i] = last - _insTarget[k];
                _cap[i] = _insCap[k];
                _cost[i] = _insCost[k];
                insVarSign[i] = (_insVar[k], _insSign[k]);
            }
        }

        // Per-node in-arc lists (indexed by INTERNAL node id) in reverse-insertion order (InArcIt): newest
        // in-arc first. Built from insertion order so the LAST inserted in-arc heads the list.
        _inArcsByNode = new List<int>[_nodeNum];
        for (int v = 0; v < _nodeNum; v++)
        {
            _inArcsByNode[v] = new List<int>();
        }

        for (int k = _arcNum - 1; k >= 0; k--)
        {
            _inArcsByNode[last - _insTarget[k]].Add(_internalId[k]); // internal target id of insertion arc k
        }
    }

    private List<int>[] _inArcsByNode = System.Array.Empty<List<int>>();

    private void Init(long maxflow)
    {
        _supply = new long[_nodeNum + 1];
        _pi = new long[_nodeNum + 1];
        _parent = new int[_nodeNum + 1];
        _pred = new int[_nodeNum + 1];
        _thread = new int[_nodeNum + 1];
        _revThread = new int[_nodeNum + 1];
        _succNum = new int[_nodeNum + 1];
        _lastSucc = new int[_nodeNum + 1];
        _predDir = new sbyte[_nodeNum + 1];

        // Source (logical node 0) -> internal node _nodeNum-1; sink (logical _nodeNum-1) -> internal 0.
        _supply[_nodeNum - 1] = maxflow;
        _supply[0] = -maxflow;

        // EQ branch (Σ supply = 0).
        for (int i = 0; i < _arcNum; i++)
        {
            _flow[i] = 0;
            _state[i] = StateLower;
        }

        _root = _nodeNum;
        _parent[_root] = -1;
        _pred[_root] = -1;
        _thread[_root] = 0;
        _revThread[0] = _root;
        _succNum[_root] = _nodeNum + 1;
        _lastSucc[_root] = _root - 1;
        _supply[_root] = 0; // -Σ supply
        _pi[_root] = 0;

        _searchArcNum = _arcNum;
        for (int u = 0, e = _arcNum; u != _nodeNum; ++u, ++e)
        {
            _parent[u] = _root;
            _pred[u] = e;
            _thread[u] = u + 1;
            _revThread[u + 1] = u;
            _succNum[u] = 1;
            _lastSucc[u] = u;
            _cap[e] = Inf;
            _state[e] = StateTree;
            if (_supply[u] >= 0)
            {
                _predDir[u] = DirUp;
                _pi[u] = 0;
                _source[e] = u;
                _target[e] = _root;
                _flow[e] = _supply[u];
                _cost[e] = 0;
            }
            else
            {
                _predDir[u] = DirDown;
                _pi[u] = ArtCost;
                _source[e] = _root;
                _target[e] = u;
                _flow[e] = -_supply[u];
                _cost[e] = ArtCost;
            }
        }
    }

    private bool InitialPivots()
    {
        long total = 0;
        var supplyNodes = new List<int>();
        var demandNodes = new List<int>();
        for (int u = 0; u < _nodeNum; u++)
        {
            long curr = _supply[u];
            if (curr > 0)
            {
                total += curr;
                supplyNodes.Add(u);
            }
            else if (curr < 0)
            {
                demandNodes.Add(u);
            }
        }

        if (total <= 0)
        {
            return true;
        }

        var arcVector = new List<int>();

        // Σ supply == 0, single source + single sink → reverse search from sink to source via InArcIt.
        if (supplyNodes.Count == 1 && demandNodes.Count == 1)
        {
            var reached = new bool[_nodeNum];
            int s = supplyNodes[0], t = demandNodes[0];
            var stack = new List<int> { t };
            reached[t] = true;
            while (stack.Count > 0)
            {
                int v = stack[^1];
                stack.RemoveAt(stack.Count - 1);
                if (v == s)
                {
                    break;
                }

                foreach (int jInternal in _inArcsByNode[v]) // InArcIt: reverse-insertion in-arcs of v
                {
                    int u = _source[jInternal];
                    if (reached[u])
                    {
                        continue;
                    }

                    if (_cap[jInternal] >= total)
                    {
                        arcVector.Add(jInternal);
                        reached[u] = true;
                        stack.Add(u);
                    }
                }
            }
        }
        else
        {
            // Not reachable for our stSupply formulation; would need the min-cost in/out-arc heuristic.
            throw new System.InvalidOperationException("NetworkSimplex.InitialPivots: unexpected supply shape.");
        }

        foreach (int arc in arcVector)
        {
            _inArc = arc;
            if (_state[_inArc] * (_cost[_inArc] + _pi[_source[_inArc]] - _pi[_target[_inArc]]) >= 0)
            {
                continue;
            }

            FindJoinNode();
            bool change = FindLeavingArc();
            if (_delta >= Max)
            {
                return false;
            }

            ChangeFlow(change);
            if (change)
            {
                UpdateTreeStructure();
                UpdatePotential();
            }
        }

        return true;
    }

    private void Start()
    {
        _blockSize = System.Math.Max((int)System.Math.Sqrt(_searchArcNum), 10);
        _nextArc = 0;
        while (FindEnteringArc())
        {
            FindJoinNode();
            bool change = FindLeavingArc();
            if (_delta >= Max)
            {
                throw new System.InvalidOperationException("NetworkSimplex: unbounded.");
            }

            ChangeFlow(change);
            if (change)
            {
                UpdateTreeStructure();
                UpdatePotential();
            }
        }

        for (int e = _searchArcNum; e != _allArcNum; ++e)
        {
            if (_flow[e] != 0)
            {
                throw new System.InvalidOperationException("NetworkSimplex: INFEASIBLE (artificial flow remains).");
            }
        }
    }

    // BlockSearchPivotRule.findEnteringArc.
    private bool FindEnteringArc()
    {
        long c, min = 0;
        int cnt = _blockSize;
        int e;
        for (e = _nextArc; e != _searchArcNum; ++e)
        {
            c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
            if (c < min)
            {
                min = c;
                _inArc = e;
            }

            if (--cnt == 0)
            {
                if (min < 0)
                {
                    goto searchEnd;
                }

                cnt = _blockSize;
            }
        }

        for (e = 0; e != _nextArc; ++e)
        {
            c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
            if (c < min)
            {
                min = c;
                _inArc = e;
            }

            if (--cnt == 0)
            {
                if (min < 0)
                {
                    goto searchEnd;
                }

                cnt = _blockSize;
            }
        }

        if (min >= 0)
        {
            return false;
        }

    searchEnd:
        _nextArc = e;
        return true;
    }

    private void FindJoinNode()
    {
        int u = _source[_inArc];
        int v = _target[_inArc];
        while (u != v)
        {
            if (_succNum[u] < _succNum[v])
            {
                u = _parent[u];
            }
            else
            {
                v = _parent[v];
            }
        }

        _join = u;
    }

    private bool FindLeavingArc()
    {
        int first, second;
        if (_state[_inArc] == StateLower)
        {
            first = _source[_inArc];
            second = _target[_inArc];
        }
        else
        {
            first = _target[_inArc];
            second = _source[_inArc];
        }

        _delta = _cap[_inArc];
        int result = 0;
        long c, d;
        int e;

        for (int u = first; u != _join; u = _parent[u])
        {
            e = _pred[u];
            d = _flow[e];
            if (_predDir[u] == DirDown)
            {
                c = _cap[e];
                d = c >= Max ? Inf : c - d;
            }

            if (d < _delta)
            {
                _delta = d;
                _uOut = u;
                result = 1;
            }
        }

        for (int u = second; u != _join; u = _parent[u])
        {
            e = _pred[u];
            d = _flow[e];
            if (_predDir[u] == DirUp)
            {
                c = _cap[e];
                d = c >= Max ? Inf : c - d;
            }

            if (d <= _delta)
            {
                _delta = d;
                _uOut = u;
                result = 2;
            }
        }

        if (result == 1)
        {
            _uIn = first;
            _vIn = second;
        }
        else
        {
            _uIn = second;
            _vIn = first;
        }

        return result != 0;
    }

    private void ChangeFlow(bool change)
    {
        if (_delta > 0)
        {
            long val = _state[_inArc] * _delta;
            _flow[_inArc] += val;
            for (int u = _source[_inArc]; u != _join; u = _parent[u])
            {
                _flow[_pred[u]] -= _predDir[u] * val;
            }

            for (int u = _target[_inArc]; u != _join; u = _parent[u])
            {
                _flow[_pred[u]] += _predDir[u] * val;
            }
        }

        if (change)
        {
            _state[_inArc] = StateTree;
            _state[_pred[_uOut]] = _flow[_pred[_uOut]] == 0 ? StateLower : StateUpper;
        }
        else
        {
            _state[_inArc] = (sbyte)(-_state[_inArc]);
        }
    }

    private void UpdateTreeStructure()
    {
        int oldRevThread = _revThread[_uOut];
        int oldSuccNum = _succNum[_uOut];
        int oldLastSucc = _lastSucc[_uOut];
        _vOut = _parent[_uOut];

        if (_uIn == _uOut)
        {
            _parent[_uIn] = _vIn;
            _pred[_uIn] = _inArc;
            _predDir[_uIn] = _uIn == _source[_inArc] ? DirUp : DirDown;

            if (_thread[_vIn] != _uOut)
            {
                int after = _thread[oldLastSucc];
                _thread[oldRevThread] = after;
                _revThread[after] = oldRevThread;
                after = _thread[_vIn];
                _thread[_vIn] = _uOut;
                _revThread[_uOut] = _vIn;
                _thread[oldLastSucc] = after;
                _revThread[after] = oldLastSucc;
            }
        }
        else
        {
            int threadContinue = oldRevThread == _vIn ? _thread[oldLastSucc] : _thread[_vIn];

            int stem = _uIn;
            int parStem = _vIn;
            int nextStem;
            int last = _lastSucc[_uIn];
            int before, after = _thread[last];
            _thread[_vIn] = _uIn;
            _dirtyRevs.Clear();
            _dirtyRevs.Add(_vIn);
            while (stem != _uOut)
            {
                nextStem = _parent[stem];
                _thread[last] = nextStem;
                _dirtyRevs.Add(last);

                before = _revThread[stem];
                _thread[before] = after;
                _revThread[after] = before;

                _parent[stem] = parStem;
                parStem = stem;
                stem = nextStem;

                last = _lastSucc[stem] == _lastSucc[parStem] ? _revThread[parStem] : _lastSucc[stem];
                after = _thread[last];
            }

            _parent[_uOut] = parStem;
            _thread[last] = threadContinue;
            _revThread[threadContinue] = last;
            _lastSucc[_uOut] = last;

            if (oldRevThread != _vIn)
            {
                _thread[oldRevThread] = after;
                _revThread[after] = oldRevThread;
            }

            for (int i = 0; i < _dirtyRevs.Count; i++)
            {
                int u = _dirtyRevs[i];
                _revThread[_thread[u]] = u;
            }

            int tmpSc = 0, tmpLs = _lastSucc[_uOut];
            for (int u = _uOut, p = _parent[u]; u != _uIn; u = p, p = _parent[u])
            {
                _pred[u] = _pred[p];
                _predDir[u] = (sbyte)(-_predDir[p]);
                tmpSc += _succNum[u] - _succNum[p];
                _succNum[u] = tmpSc;
                _lastSucc[p] = tmpLs;
            }

            _pred[_uIn] = _inArc;
            _predDir[_uIn] = _uIn == _source[_inArc] ? DirUp : DirDown;
            _succNum[_uIn] = oldSuccNum;
        }

        int upLimitOut = _lastSucc[_join] == _vIn ? _join : -1;
        int lastSuccOut = _lastSucc[_uOut];
        for (int u = _vIn; u != -1 && _lastSucc[u] == _vIn; u = _parent[u])
        {
            _lastSucc[u] = lastSuccOut;
        }

        if (_join != oldRevThread && _vIn != oldRevThread)
        {
            for (int u = _vOut; u != upLimitOut && _lastSucc[u] == oldLastSucc; u = _parent[u])
            {
                _lastSucc[u] = oldRevThread;
            }
        }
        else if (lastSuccOut != oldLastSucc)
        {
            for (int u = _vOut; u != upLimitOut && _lastSucc[u] == oldLastSucc; u = _parent[u])
            {
                _lastSucc[u] = lastSuccOut;
            }
        }

        for (int u = _vIn; u != _join; u = _parent[u])
        {
            _succNum[u] += oldSuccNum;
        }

        for (int u = _vOut; u != _join; u = _parent[u])
        {
            _succNum[u] -= oldSuccNum;
        }
    }

    private void UpdatePotential()
    {
        long sigma = _pi[_vIn] - _pi[_uIn] - (_predDir[_uIn] * _cost[_inArc]);
        int end = _thread[_lastSucc[_uIn]];
        for (int u = _uIn; u != end; u = _thread[u])
        {
            _pi[u] += sigma;
        }
    }
}

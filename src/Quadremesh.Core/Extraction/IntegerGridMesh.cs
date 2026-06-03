using Quadremesh.Core.Math;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// A mutable, flat-array triangle mesh carrying the integer-grid-map data (per-edge offsets, per-face
/// edge ids and orients), built from a solved <see cref="QuadriFlowSolver"/>. It supports the two
/// triangulation-alignment passes QuadriFlow runs before extraction:
/// <see cref="SubdivideEdgeDiff"/> (split any edge whose offset spans more than one lattice step) and
/// <see cref="FixFlip"/> (remove integer-parametrization foldovers). After these the offsets are unit
/// steps that align with the lattice, so collapse + manifold-ization yields a watertight quad mesh.
/// Reimplemented clean-room from QuadriFlow (MIT, read for understanding only).
/// </summary>
internal sealed class IntegerGridMesh
{
    private readonly List<Vector3> _v = new();   // positions
    private readonly List<Vector3> _n = new();   // normals
    private readonly List<Vector3> _q = new();   // orientation field
    private readonly List<Vector3> _o = new();   // position-field anchor
    private readonly List<double> _scale = new();

    private readonly List<int[]> _f = new();     // face -> 3 vertex ids
    private readonly List<int[]> _fei = new();   // face -> 3 edge ids (face_edgeIds)
    private readonly List<int[]> _feo = new();   // face -> 3 edge orients (face_edgeOrients)

    private readonly List<Vec2i> _edgeDiff = new();        // per edge
    private readonly List<(int X, int Y)> _edgeValues = new(); // per edge, sorted endpoints
    private readonly List<int> _allow = new();             // per edge component (e*2+dim): 0 = sharp lock

    private readonly List<int> _e2e = new();     // dedge twin (face*3+slot)
    private readonly List<int> _v2e = new();
    private readonly List<bool> _boundary = new();
    private readonly List<bool> _nonManifold = new();

    private IntegerGridMesh()
    {
    }

    /// <summary>Builds the working mesh from a solved field.</summary>
    public static IntegerGridMesh FromSolver(QuadriFlowSolver solver)
    {
        ArgumentNullException.ThrowIfNull(solver);
        var m = new IntegerGridMesh();

        Vector3[] pos = solver.Positions;
        Vector3[] nrm = solver.Normal;
        Vector3[] ori = solver.Orientation;
        Vector3[] anc = solver.Anchor;
        double[] scale = solver.Scale;
        for (int v = 0; v < solver.VertexCount; v++)
        {
            m._v.Add(pos[v]);
            m._n.Add(nrm[v]);
            m._q.Add(ori[v]);
            m._o.Add(anc[v]);
            m._scale.Add(scale[v]);
            m._v2e.Add(-1);
            m._boundary.Add(false);
            m._nonManifold.Add(false);
        }

        for (int f = 0; f < solver.FaceCount; f++)
        {
            m._f.Add(new[] { solver.FaceCorner(f, 0), solver.FaceCorner(f, 1), solver.FaceCorner(f, 2) });
            m._fei.Add(new[] { solver.FaceEdgeId(f, 0), solver.FaceEdgeId(f, 1), solver.FaceEdgeId(f, 2) });
            m._feo.Add(new[] { solver.FaceEdgeOrient(f, 0), solver.FaceEdgeOrient(f, 1), solver.FaceEdgeOrient(f, 2) });
        }

        int[] allow = solver.AllowChanges;
        for (int e = 0; e < solver.EdgeCount; e++)
        {
            m._edgeDiff.Add(solver.EdgeDiff(e));
            (int a, int b) = solver.EdgeValue(e);
            m._edgeValues.Add(a <= b ? (a, b) : (b, a));
            m._allow.Add(allow[(e * 2) + 0]);
            m._allow.Add(allow[(e * 2) + 1]);
        }

        m._e2e.AddRange(solver.BuildDedgeTwins());

        for (int f = 0; f < m._f.Count; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                m._v2e[m._f[f][j]] = (f * 3) + j;
                if (m._e2e[(f * 3) + j] == -1)
                {
                    m._boundary[m._f[f][j]] = true;
                    m._boundary[m._f[f][(j + 1) % 3]] = true;
                }
            }
        }

        return m;
    }

    // --- Diagnostic serialization (cache a solved grid to iterate alignment without re-solving) ----

    internal void Save(System.IO.BinaryWriter w)
    {
        void WV(List<Vector3> l) { w.Write(l.Count); foreach (Vector3 v in l) { w.Write(v.X); w.Write(v.Y); w.Write(v.Z); } }
        void WI3(List<int[]> l) { w.Write(l.Count); foreach (int[] a in l) { w.Write(a[0]); w.Write(a[1]); w.Write(a[2]); } }
        WV(_v); WV(_n); WV(_q); WV(_o);
        w.Write(_scale.Count); foreach (double d in _scale) { w.Write(d); }
        WI3(_f); WI3(_fei); WI3(_feo);
        w.Write(_edgeDiff.Count); foreach (Vec2i d in _edgeDiff) { w.Write(d.X); w.Write(d.Y); }
        w.Write(_edgeValues.Count); foreach ((int x, int y) in _edgeValues) { w.Write(x); w.Write(y); }
        w.Write(_allow.Count); foreach (int a in _allow) { w.Write(a); }
        w.Write(_e2e.Count); foreach (int e in _e2e) { w.Write(e); }
        w.Write(_v2e.Count); foreach (int e in _v2e) { w.Write(e); }
        w.Write(_boundary.Count); foreach (bool b in _boundary) { w.Write(b); }
        w.Write(_nonManifold.Count); foreach (bool b in _nonManifold) { w.Write(b); }
    }

    internal static IntegerGridMesh Load(System.IO.BinaryReader r)
    {
        var m = new IntegerGridMesh();
        void RV(List<Vector3> l) { int n = r.ReadInt32(); for (int i = 0; i < n; i++) { l.Add(new Vector3(r.ReadDouble(), r.ReadDouble(), r.ReadDouble())); } }
        void RI3(List<int[]> l) { int n = r.ReadInt32(); for (int i = 0; i < n; i++) { l.Add(new[] { r.ReadInt32(), r.ReadInt32(), r.ReadInt32() }); } }
        RV(m._v); RV(m._n); RV(m._q); RV(m._o);
        int sc = r.ReadInt32(); for (int i = 0; i < sc; i++) { m._scale.Add(r.ReadDouble()); }
        RI3(m._f); RI3(m._fei); RI3(m._feo);
        int ed = r.ReadInt32(); for (int i = 0; i < ed; i++) { m._edgeDiff.Add(new Vec2i(r.ReadInt32(), r.ReadInt32())); }
        int ev = r.ReadInt32(); for (int i = 0; i < ev; i++) { m._edgeValues.Add((r.ReadInt32(), r.ReadInt32())); }
        int al = r.ReadInt32(); for (int i = 0; i < al; i++) { m._allow.Add(r.ReadInt32()); }
        int e2 = r.ReadInt32(); for (int i = 0; i < e2; i++) { m._e2e.Add(r.ReadInt32()); }
        int v2 = r.ReadInt32(); for (int i = 0; i < v2; i++) { m._v2e.Add(r.ReadInt32()); }
        int bd = r.ReadInt32(); for (int i = 0; i < bd; i++) { m._boundary.Add(r.ReadBoolean()); }
        int nm = r.ReadInt32(); for (int i = 0; i < nm; i++) { m._nonManifold.Add(r.ReadBoolean()); }
        return m;
    }

    // --- Read accessors used by the extractor -----------------------------------------------------

    public int VertexCount => _v.Count;

    public int FaceCount => _f.Count;

    public int EdgeCount => _edgeDiff.Count;

    public Vector3 Position(int v) => _v[v];

    public int FaceCorner(int f, int j) => _f[f][j];

    public int FaceEdgeId(int f, int j) => _fei[f][j];

    public int FaceEdgeOrient(int f, int j) => _feo[f][j];

    public Vec2i EdgeDiff(int e) => _edgeDiff[e];

    public (int X, int Y) EdgeValue(int e) => _edgeValues[e];

    /// <summary>Maximum |component| of any edge offset — 1 once <see cref="SubdivideEdgeDiff"/> is done.</summary>
    public int MaxEdgeDiff()
    {
        int max = 0;
        foreach (Vec2i d in _edgeDiff)
        {
            max = System.Math.Max(max, System.Math.Max(System.Math.Abs(d.X), System.Math.Abs(d.Y)));
        }

        return max;
    }

    // --- Diagnostics (instrumentation for the foldover/hole investigation) -------------------------

    /// <summary>Splits performed by the last <see cref="SubdivideEdgeDiff"/> call.</summary>
    internal int LastSplitCount { get; private set; }

    /// <summary>True if the last <see cref="SubdivideEdgeDiff"/> tripped its termination guard.</summary>
    internal bool LastHitGuard { get; private set; }

    /// <summary>Peak |edge_diff| component observed during the last <see cref="SubdivideEdgeDiff"/>.</summary>
    internal int LastPeakDiff { get; private set; }

    /// <summary>Count of faces with negative integer (lattice) area — i.e. parametrization foldovers.</summary>
    internal int NegativeAreaFaceCount()
    {
        int n = 0;
        for (int f = 0; f < _f.Count; f++)
        {
            Vec2i d1 = Lattice2D.Rshift90(_edgeDiff[_fei[f][0]], _feo[f][0]);
            Vec2i d2 = Lattice2D.Rshift90(_edgeDiff[_fei[f][1]], _feo[f][1]);
            if ((d1.X * d2.Y) - (d1.Y * d2.X) < 0)
            {
                n++;
            }
        }

        return n;
    }

    /// <summary>Coupled divergence of face <paramref name="f"/> (zero on consistent faces).</summary>
    public Vec2i FaceDivergence(int f)
    {
        var sum = new Vec2i(0, 0);
        for (int j = 0; j < 3; j++)
        {
            sum += Lattice2D.Rshift90(_edgeDiff[_fei[f][j]], _feo[f][j]);
        }

        return sum;
    }

    /// <summary>Runs the flip-fix on the current edge graph (delegates to <see cref="EdgeGraphHierarchy"/>).</summary>
    public void FixFlip(int maxShrinkLen = 2)
    {
        var fq = new int[_f.Count][];
        var f2e = new int[_f.Count][];
        for (int f = 0; f < _f.Count; f++)
        {
            fq[f] = new[] { _feo[f][0], _feo[f][1], _feo[f][2] };
            f2e[f] = new[] { _fei[f][0], _fei[f][1], _fei[f][2] };
        }

        var diff = _edgeDiff.ToArray();
        var allow = _allow.ToArray();

        EdgeGraphHierarchy.Run(fq, f2e, diff, allow, maxShrinkLen);

        for (int f = 0; f < _f.Count; f++)
        {
            _feo[f][0] = fq[f][0];
            _feo[f][1] = fq[f][1];
            _feo[f][2] = fq[f][2];
        }

        for (int e = 0; e < _edgeDiff.Count; e++)
        {
            _edgeDiff[e] = diff[e];
        }
    }

    /// <summary>
    /// Runs the SAT flip-fixer (<see cref="FlipSat"/>) on the current edge graph: clears the residual
    /// foldovers that <see cref="FixFlip"/>'s edge-ring shrink cannot, by solving a small per-cluster
    /// integer feasibility problem. Only edge offsets change (orients are untouched). Returns the
    /// foldover count remaining.
    /// </summary>
    public int FixFlipSat()
    {
        var fq = new int[_f.Count][];
        var f2e = new int[_f.Count][];
        for (int f = 0; f < _f.Count; f++)
        {
            fq[f] = new[] { _feo[f][0], _feo[f][1], _feo[f][2] };
            f2e[f] = new[] { _fei[f][0], _fei[f][1], _fei[f][2] };
        }

        var diff = _edgeDiff.ToArray();
        var allow = _allow.ToArray();

        int remaining = EdgeGraphHierarchy.RunSat(fq, f2e, diff, allow);

        for (int f = 0; f < _f.Count; f++)
        {
            _feo[f][0] = fq[f][0];
            _feo[f][1] = fq[f][1];
            _feo[f][2] = fq[f][2];
        }

        for (int e = 0; e < _edgeDiff.Count; e++)
        {
            _edgeDiff[e] = diff[e];
        }

        return remaining;
    }

    // --- subdivide_edgeDiff (subdivide.cpp:148) ---------------------------------------------------

    private sealed class FaceOrient
    {
        public int Orient;
        public int[] D = new int[3];
        public Vector3 Q;
        public Vector3 N;

        public FaceOrient Clone() => new() { Orient = Orient, D = new[] { D[0], D[1], D[2] }, Q = Q, N = N };
    }

    /// <summary>
    /// Splits every edge whose integer offset has a component larger than <paramref name="maxLen"/>
    /// (1 quad step) so all offsets become unit/zero. Mirrors <c>subdivide_edgeDiff</c>: each split
    /// inserts the midpoint and rebuilds the four sub-faces' offsets/orients from the parent's.
    /// </summary>
    public void SubdivideEdgeDiff(int maxLen)
    {
        // diffs[dedge] = the edge's offset expressed in the face's frame.
        var diffs = new List<Vec2i>(_f.Count * 3);
        for (int i = 0; i < _f.Count; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                diffs.Add(Lattice2D.Rshift90(_edgeDiff[_fei[i][j]], _feo[i][j]));
            }
        }

        var faceSpaces = new List<FaceOrient>(_f.Count);
        for (int i = 0; i < _f.Count; i++)
        {
            var orient = new FaceOrient { Q = _q[_f[i][0]], N = _n[_f[i][0]] };
            var orientDiff = new int[3];
            for (int j = 0; j < 3; j++)
            {
                int finalOrient = _feo[i][j];
                int eid = _fei[i][j];
                (int a, int b) = Lattice2D.Match(_q[_edgeValues[eid].X], _n[_edgeValues[eid].X], orient.Q, orient.N);
                int targetOrient = (b - a + 4) % 4;
                if (_f[i][j] == _edgeValues[eid].Y)
                {
                    targetOrient = (targetOrient + 2) % 4;
                }

                orientDiff[j] = (finalOrient - targetOrient + 4) % 4;
            }

            if (orientDiff[0] == orientDiff[1])
            {
                orient.Orient = orientDiff[0];
            }
            else if (orientDiff[0] == orientDiff[2])
            {
                orient.Orient = orientDiff[2];
            }
            else if (orientDiff[1] == orientDiff[2])
            {
                orient.Orient = orientDiff[1];
            }

            orient.D = new[]
            {
                (orientDiff[0] - orient.Orient + 4) % 4,
                (orientDiff[1] - orient.Orient + 4) % 4,
                (orientDiff[2] - orient.Orient + 4) % 4,
            };
            faceSpaces.Add(orient);
        }

        // Max-heap on max(|diff.X|,|diff.Y|): split the most stretched edges first.
        var queue = new PriorityQueue<(int Id, double Length), int>();
        for (int i = 0; i < _e2e.Count; i++)
        {
            int v0 = _f[i / 3][i % 3];
            int v1 = _f[i / 3][(i + 1) % 3];
            if (_nonManifold[v0] || _nonManifold[v1])
            {
                continue;
            }

            Vec2i diff = diffs[i];
            if (System.Math.Abs(diff.X) > maxLen || System.Math.Abs(diff.Y) > maxLen)
            {
                int other = _e2e[i];
                if (other == -1 || other > i)
                {
                    double length = (_v[v0] - _v[v1]).LengthSquared;
                    queue.Enqueue((i, length), -System.Math.Max(System.Math.Abs(diff.X), System.Math.Abs(diff.Y)));
                }
            }
        }

        void Schedule(int f)
        {
            for (int i = 0; i < 3; i++)
            {
                Vec2i d = diffs[(f * 3) + i];
                if (System.Math.Abs(d.X) > maxLen || System.Math.Abs(d.Y) > maxLen)
                {
                    double length = (_v[_f[f][(i + 1) % 3]] - _v[_f[f][i]]).LengthSquared;
                    queue.Enqueue(((f * 3) + i, length), -System.Math.Max(System.Math.Abs(d.X), System.Math.Abs(d.Y)));
                }
            }
        }

        // Recomputes a face's per-edge orients and offsets from its frame and per-corner rotation d.
        void AnalyzeOrient(int f0, int[] d)
        {
            for (int j = 0; j < 3; j++)
            {
                int orient = faceSpaces[f0].Orient + d[j];
                int vmin = System.Math.Min(_f[f0][j], _f[f0][(j + 1) % 3]);
                (int a, int b) = Lattice2D.Match(_q[vmin], _n[vmin], faceSpaces[f0].Q, faceSpaces[f0].N);
                if (_f[f0][j] != vmin)
                {
                    orient += 2;
                }

                _feo[f0][j] = (orient + b - a + 4) % 4;
            }

            faceSpaces[f0].D = new[] { d[0], d[1], d[2] };
            for (int j = 0; j < 3; j++)
            {
                int eid = _fei[f0][j];
                int orient = _feo[f0][j];
                _edgeDiff[eid] = Lattice2D.Rshift90(diffs[(f0 * 3) + j], (4 - orient) % 4);
            }
        }

        void FixOrient(int f0)
        {
            for (int j = 0; j < 3; j++)
            {
                Vec2i diff = _edgeDiff[_fei[f0][j]];
                if (!Lattice2D.Rshift90(diff, _feo[f0][j]).Equals(diffs[(f0 * 3) + j]))
                {
                    int orient = 0;
                    while (orient < 4 && !Lattice2D.Rshift90(diff, orient).Equals(diffs[(f0 * 3) + j]))
                    {
                        orient += 1;
                    }

                    faceSpaces[f0].D[j] = (faceSpaces[f0].D[j] + orient - _feo[f0][j]) % 4;
                    _feo[f0][j] = orient;
                }
            }
        }

        // Termination guard. At a well-resolved field, offsets are small and the split count is bounded
        // by roughly the number of over-long edges. A position-field foldover (a near-singular patch
        // where anchors jump several lattice steps across one edge) can make a sub-face's "spoke" edge
        // regenerate an over-long offset indefinitely. Rather than hang, we stop subdividing once the
        // count far exceeds any legitimate need; the still-over-long offsets simply yield no quad there
        // (a localized hole around the foldover) instead of crashing. Each split leaves the mesh
        // offset-consistent, so stopping early is safe.
        int guard = 0;
        int guardLimit = (8 * _e2e.Count) + 4096;
        LastSplitCount = 0;
        LastHitGuard = false;
        LastPeakDiff = MaxEdgeDiff();
        while (queue.Count > 0)
        {
            if (++guard > guardLimit)
            {
                LastHitGuard = true;
                break;
            }

            (int e0, double length) = queue.Dequeue();
            int e1 = _e2e[e0];
            bool isBoundary = e1 == -1;
            int f0 = e0 / 3;
            int f1 = isBoundary ? -1 : e1 / 3;
            int v0 = _f[f0][e0 % 3];
            int v0p = _f[f0][(e0 + 2) % 3];
            int v1 = _f[f0][(e0 + 1) % 3];
            if ((_v[v0] - _v[v1]).LengthSquared != length)
            {
                continue; // stale queue entry (edge already split / changed)
            }

            if (System.Math.Abs(diffs[e0].X) < 2 && System.Math.Abs(diffs[e0].Y) < 2)
            {
                continue;
            }

            int v1p = isBoundary ? -1 : _f[f1][(e1 + 2) % 3];

            LastSplitCount++;
            int curMax = System.Math.Max(System.Math.Abs(diffs[e0].X), System.Math.Abs(diffs[e0].Y));
            if (curMax > LastPeakDiff)
            {
                LastPeakDiff = curMax;
            }

            // New midpoint vertex.
            int vn = _v.Count;
            _v.Add((_v[v0] + _v[v1]) * 0.5);
            _n.Add(_n[v0]);
            _q.Add(_q[v0]);
            _o.Add((_o[v0] + _o[v1]) * 0.5);
            _scale.Add(_scale[v0]);
            _nonManifold.Add(false);
            _boundary.Add(isBoundary);
            _v2e.Add(-1);

            int eid = _fei[f0][e0 % 3];
            int eid01 = _fei[f0][(e0 + 1) % 3];
            int eid02 = _fei[f0][(e0 + 2) % 3];

            // Edge eid (v0..v1) becomes eid0 (v0..vn); add eid1 (vn..v1) and eid0p (vn..v0p).
            int eid0 = eid;
            _edgeValues[eid0] = Sorted(v0, vn);
            int eid1 = NewEdge(Sorted(vn, v1));
            int eid0p = NewEdge(Sorted(vn, v0p));

            int f2 = isBoundary ? -1 : NewFace();
            int f3 = NewFace();
            // Ensure diffs has slots for the new faces.
            while (diffs.Count < _f.Count * 3)
            {
                diffs.Add(new Vec2i(0, 0));
            }

            while (faceSpaces.Count < _f.Count)
            {
                faceSpaces.Add(new FaceOrient());
            }

            Vec2i d01 = diffs[e0];
            Vec2i d1p = diffs[(e0 / 3 * 3) + ((e0 + 1) % 3)];
            Vec2i dp0 = diffs[(e0 / 3 * 3) + ((e0 + 2) % 3)];
            var d0n = new Vec2i(d01.X / 2, d01.Y / 2);

            int o1 = e0 % 3;
            int o2 = isBoundary ? -1 : e1 % 3;

            FaceOrient orients1 = faceSpaces[f0].Clone();
            _f[f0] = new[] { vn, v0p, v0 };
            _fei[f0] = new[] { eid0p, eid02, eid0 };
            diffs[f0 * 3] = d01 + d1p - d0n;
            diffs[(f0 * 3) + 1] = dp0;
            diffs[(f0 * 3) + 2] = d0n;
            AnalyzeOrient(f0, new[] { 0, orients1.D[(o1 + 2) % 3], orients1.D[o1] });

            if (!isBoundary)
            {
                FaceOrient orients2 = faceSpaces[f1].Clone();
                int eid11 = _fei[f1][(e1 + 1) % 3];
                int eid12 = _fei[f1][(e1 + 2) % 3];

                Vec2i ds10 = diffs[e1];
                Vec2i ds0p = diffs[(e1 / 3 * 3) + ((e1 + 1) % 3)];
                Vec2i dsp1 = diffs[(e1 / 3 * 3) + ((e1 + 2) % 3)];
                int orient = 0;
                while (orient < 4 && !Lattice2D.Rshift90(d01, orient).Equals(ds10))
                {
                    orient += 1;
                }

                if (orient == 4)
                {
                    throw new InvalidOperationException(
                        $"subdivide: no rotation maps {d01} -> {ds10} (e0={e0}, e1={e1})");
                }

                Vec2i dsn0 = Lattice2D.Rshift90(d0n, orient);

                _f[f1] = new[] { vn, v0, v1p };
                int eid1p = NewEdge(Sorted(vn, v1p));
                _fei[f1] = new[] { eid0, eid11, eid1p };
                diffs[f1 * 3] = dsn0;
                diffs[(f1 * 3) + 1] = ds0p;
                diffs[(f1 * 3) + 2] = dsp1 + (ds10 - dsn0);
                AnalyzeOrient(f1, new[] { orients2.D[o2], orients2.D[(o2 + 1) % 3], 0 });

                faceSpaces[f2] = orients2.Clone();
                _fei[f2] = new[] { eid1p, eid12, eid1 };
                _f[f2] = new[] { vn, v1p, v1 };
                diffs[f2 * 3] = -dsp1 - (ds10 - dsn0);
                diffs[(f2 * 3) + 1] = dsp1;
                diffs[(f2 * 3) + 2] = ds10 - dsn0;
                AnalyzeOrient(f2, new[] { 0, orients2.D[(o2 + 2) % 3], orients2.D[o2] });
            }

            faceSpaces[f3] = orients1.Clone();
            _fei[f3] = new[] { eid1, eid01, eid0p };
            _f[f3] = new[] { vn, v1, v0p };
            diffs[f3 * 3] = d01 - d0n;
            diffs[(f3 * 3) + 1] = d1p;
            diffs[(f3 * 3) + 2] = d0n - (d01 + d1p);
            AnalyzeOrient(f3, new[] { orients1.D[o1], orients1.D[(o1 + 1) % 3], 0 });

            FixOrient(f0);
            if (!isBoundary)
            {
                FixOrient(f1);
                FixOrient(f2);
            }

            FixOrient(f3);

            // Rewire the dedge twins around the split (sE2E in the reference).
            int e0p = _e2e[DedgePrev(e0)];
            int e0n = _e2e[DedgeNext(e0)];
            SetTwin((3 * f0) + 0, (3 * f3) + 2);
            SetTwin((3 * f0) + 1, e0p);
            SetTwin((3 * f3) + 1, e0n);
            if (isBoundary)
            {
                SetTwin((3 * f0) + 2, -1);
                SetTwin((3 * f3) + 0, -1);
            }
            else
            {
                int e1p = _e2e[DedgePrev(e1)];
                int e1n = _e2e[DedgeNext(e1)];
                SetTwin((3 * f0) + 2, (3 * f1) + 0);
                SetTwin((3 * f1) + 1, e1n);
                SetTwin((3 * f1) + 2, (3 * f2) + 0);
                SetTwin((3 * f2) + 1, e1p);
                SetTwin((3 * f2) + 2, (3 * f3) + 0);
            }

            _v2e[v0] = (3 * f0) + 2;
            _v2e[vn] = (3 * f0) + 0;
            _v2e[v1] = (3 * f3) + 1;
            _v2e[v0p] = (3 * f0) + 1;
            if (!isBoundary)
            {
                _v2e[v1p] = (3 * f1) + 2;
            }

            Schedule(f0);
            if (!isBoundary)
            {
                Schedule(f2);
                Schedule(f1);
            }

            Schedule(f3);
        }
    }

    private (int X, int Y) Sorted(int a, int b) => a <= b ? (a, b) : (b, a);

    private int NewEdge((int X, int Y) value)
    {
        _edgeValues.Add(value);
        _edgeDiff.Add(new Vec2i(0, 0));
        _allow.Add(1); // edges created by subdivision are interior, not sharp
        _allow.Add(1);
        return _edgeDiff.Count - 1;
    }

    private int NewFace()
    {
        _f.Add(new[] { 0, 0, 0 });
        _fei.Add(new[] { 0, 0, 0 });
        _feo.Add(new[] { 0, 0, 0 });
        _e2e.Add(-1);
        _e2e.Add(-1);
        _e2e.Add(-1);
        return _f.Count - 1;
    }

    private static int DedgePrev(int e) => (e / 3 * 3) + ((e + 2) % 3);

    private static int DedgeNext(int e) => (e / 3 * 3) + ((e + 1) % 3);

    private void SetTwin(int a, int b)
    {
        _e2e[a] = b;
        if (b != -1)
        {
            _e2e[b] = a;
        }
    }
}

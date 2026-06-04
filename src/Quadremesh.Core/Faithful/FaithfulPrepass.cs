using Quadremesh.Core.Math;

namespace Quadremesh.Core.Faithful;

/// <summary>
/// Faithful re-port (plan 07, Phase 1e) of QuadriFlow's mesh pre-pass — the front end of
/// <c>Parametrizer::Load</c>/<c>Initialize</c> that produces the finest hierarchy mesh <c>mV[0]</c>/<c>mF</c>.
/// Replaces our full isotropic remesh (split+collapse+flip+relax) with the reference's NormalizeMesh
/// (unit-box rescale) + ComputeMeshStatus + split-only <c>subdivide</c> (priority-queue longest-edge
/// bisection to <c>target_len = min(scale/2, avg_edge*2)</c>). Deterministic, so it reproduces the
/// reference's finest mesh exactly. Quirks preserved verbatim for bit-fidelity (e.g. the
/// <c>rho[vn] = 0.5*rho[v1]</c> comma-operator in subdivide.cpp:74).
/// </summary>
internal sealed class FaithfulPrepass
{
    public Vector3[] V { get; private set; } = Array.Empty<Vector3>();

    public int[][] F { get; private set; } = Array.Empty<int[]>();

    public double Scale { get; private set; }

    private static int DedgePrev3(int e) => e % 3 == 0 ? e + 2 : e - 1;

    private static int DedgeNext3(int e) => e % 3 == 2 ? e - 2 : e + 1;

    /// <summary>Runs NormalizeMesh + ComputeMeshStatus + subdivide, yielding the finest mesh and grid scale.</summary>
    public static FaithfulPrepass Run(Vector3[] inputV, int[][] inputF, int targetFaces)
    {
        var p = new FaithfulPrepass();
        var v = new List<Vector3>(inputV);
        var f = new List<int[]>(inputF.Length);
        foreach (int[] face in inputF)
        {
            f.Add(new[] { face[0], face[1], face[2] });
        }

        NormalizeMesh(v);
        (double surfaceArea, double avgEdge, double maxEdge) = ComputeMeshStatus(v, f);
        double scale = System.Math.Sqrt(surfaceArea / targetFaces);
        double targetLen = System.Math.Min(scale / 2, avgEdge * 2);

        if (targetLen < maxEdge)
        {
            Subdivide(v, f, targetLen);
        }

        p.V = v.ToArray();
        p.F = f.ToArray();
        p.Scale = scale;
        return p;
    }

    private static void NormalizeMesh(List<Vector3> v)
    {
        var maxV = new[] { -1e30, -1e30, -1e30 };
        var minV = new[] { 1e30, 1e30, 1e30 };
        foreach (Vector3 p in v)
        {
            maxV[0] = System.Math.Max(maxV[0], p.X);
            maxV[1] = System.Math.Max(maxV[1], p.Y);
            maxV[2] = System.Math.Max(maxV[2], p.Z);
            minV[0] = System.Math.Min(minV[0], p.X);
            minV[1] = System.Math.Min(minV[1], p.Y);
            minV[2] = System.Math.Min(minV[2], p.Z);
        }

        double scale = System.Math.Max(System.Math.Max(maxV[0] - minV[0], maxV[1] - minV[1]), maxV[2] - minV[2]) * 0.5;
        double cx = (maxV[0] + minV[0]) * 0.5;
        double cy = (maxV[1] + minV[1]) * 0.5;
        double cz = (maxV[2] + minV[2]) * 0.5;
        for (int i = 0; i < v.Count; i++)
        {
            v[i] = new Vector3((v[i].X - cx) / scale, (v[i].Y - cy) / scale, (v[i].Z - cz) / scale);
        }
    }

    private static (double SurfaceArea, double AvgEdge, double MaxEdge) ComputeMeshStatus(
        List<Vector3> v, List<int[]> f)
    {
        double surfaceArea = 0.0;
        double avgEdge = 0.0;
        double maxEdge = 0.0;
        foreach (int[] face in f)
        {
            Vector3 v0 = v[face[0]];
            Vector3 v1 = v[face[1]];
            Vector3 v2 = v[face[2]];
            surfaceArea += 0.5 * Vector3.Cross(v1 - v0, v2 - v0).Length;
            for (int i = 0; i < 3; i++)
            {
                double len = (v[face[(i + 1) % 3]] - v[face[i]]).Length;
                avgEdge += len;
                if (len > maxEdge)
                {
                    maxEdge = len;
                }
            }
        }

        avgEdge /= f.Count * 3;
        return (surfaceArea, avgEdge, maxEdge);
    }

    private readonly record struct Edge(double Length, int Id);

    private sealed class EdgeComparer : IComparer<Edge>
    {
        // std::priority_queue<pair<double,int>> pops the LARGEST (length, id) first; .NET PriorityQueue
        // dequeues the MIN priority, so order larger-first as "smaller".
        public int Compare(Edge a, Edge b)
        {
            if (a.Length != b.Length)
            {
                return b.Length.CompareTo(a.Length);
            }

            return b.Id.CompareTo(a.Id);
        }
    }

    private static void Subdivide(List<Vector3> v, List<int[]> f, double maxLength)
    {
        // Initial directed-edge tables on the current mesh.
        DirectedEdgeMesh dm = DirectedEdgeMesh.Build(v.ToArray(), f.ToArray());
        var v2e = new List<int>(dm.V2E);
        var e2e = new List<int>(dm.E2E);
        var boundary = new List<bool>(dm.Boundary);
        var nonmanifold = new List<bool>(dm.NonManifold);
        var rho = new List<double>(v.Count);
        for (int i = 0; i < v.Count; i++)
        {
            rho.Add(1.0);
        }

        maxLength *= maxLength;

        var queue = new PriorityQueue<int, Edge>(new EdgeComparer());

        bool NeedSplit(double length, int va, int vb) =>
            length > maxLength ||
            length > System.Math.Max(maxLength * 0.75, System.Math.Min(rho[va], rho[vb]) * 1.0);

        for (int i = 0; i < e2e.Count; i++)
        {
            int v0 = f[i / 3][i % 3];
            int v1 = f[i / 3][(i + 1) % 3];
            if (nonmanifold[v0] || nonmanifold[v1])
            {
                continue;
            }

            double length = (v[v0] - v[v1]).LengthSquared;
            if (NeedSplit(length, v0, v1))
            {
                int other = e2e[i];
                if (other == -1 || other > i)
                {
                    queue.Enqueue(i, new Edge(length, i));
                }
            }
        }

        void Grow()
        {
            // Lists grow on demand; nothing to preallocate. Kept as a seam mirroring conservativeResize.
        }

        void Schedule(int face)
        {
            for (int i = 0; i < 3; i++)
            {
                int va = f[face][i];
                int vb = f[face][(i + 1) % 3];
                double length = (v[va] - v[vb]).LengthSquared;
                if (NeedSplit(length, va, vb))
                {
                    queue.Enqueue((face * 3) + i, new Edge(length, (face * 3) + i));
                }
            }
        }

        void SetE2E(int a, int b)
        {
            e2e[a] = b;
            if (b != -1)
            {
                e2e[b] = a;
            }
        }

        while (queue.Count > 0)
        {
            queue.TryDequeue(out int e0, out Edge edge);
            int e1 = e2e[e0];
            bool isBoundary = e1 == -1;
            int f0 = e0 / 3;
            int f1 = isBoundary ? -1 : e1 / 3;
            int v0 = f[f0][e0 % 3];
            int v0p = f[f0][(e0 + 2) % 3];
            int v1 = f[f0][(e0 + 1) % 3];
            if ((v[v0] - v[v1]).LengthSquared != edge.Length)
            {
                continue; // stale queue entry
            }

            int v1p = isBoundary ? -1 : f[f1][(e1 + 2) % 3];

            // New midpoint vertex.
            int vn = v.Count;
            v.Add((v[v0] + v[v1]) * 0.5);
            rho.Add(0.5 * rho[v1]); // subdivide.cpp:74 comma-operator quirk: 0.5f * (rho[v0], rho[v1])
            nonmanifold.Add(false);
            boundary.Add(isBoundary);
            v2e.Add(0);
            Grow();

            // New faces (f0 reused; f2 only when interior; f3 always).
            int f2 = isBoundary ? -1 : f.Count;
            if (!isBoundary)
            {
                f.Add(new int[3]);
            }

            int f3 = f.Count;
            f.Add(new int[3]);

            // Extend E2E to cover the 3 dedges of any new faces.
            int neededE2E = f.Count * 3;
            while (e2e.Count < neededE2E)
            {
                e2e.Add(-1);
            }

            // Update F.
            f[f0][0] = vn;
            f[f0][1] = v0p;
            f[f0][2] = v0;
            if (!isBoundary)
            {
                f[f1][0] = vn;
                f[f1][1] = v0;
                f[f1][2] = v1p;
                f[f2][0] = vn;
                f[f2][1] = v1p;
                f[f2][2] = v1;
            }

            f[f3][0] = vn;
            f[f3][1] = v1;
            f[f3][2] = v0p;

            // Update E2E.
            int e0p = e2e[DedgePrev3(e0)];
            int e0n = e2e[DedgeNext3(e0)];
            SetE2E((3 * f0) + 0, (3 * f3) + 2);
            SetE2E((3 * f0) + 1, e0p);
            SetE2E((3 * f3) + 1, e0n);
            if (isBoundary)
            {
                SetE2E((3 * f0) + 2, -1);
                SetE2E((3 * f3) + 0, -1);
            }
            else
            {
                int e1p = e2e[DedgePrev3(e1)];
                int e1n = e2e[DedgeNext3(e1)];
                SetE2E((3 * f0) + 2, (3 * f1) + 0);
                SetE2E((3 * f1) + 1, e1n);
                SetE2E((3 * f1) + 2, (3 * f2) + 0);
                SetE2E((3 * f2) + 1, e1p);
                SetE2E((3 * f2) + 2, (3 * f3) + 0);
            }

            // Update V2E.
            v2e[v0] = (3 * f0) + 2;
            v2e[vn] = (3 * f0) + 0;
            v2e[v1] = (3 * f3) + 1;
            v2e[v0p] = (3 * f0) + 1;
            if (!isBoundary)
            {
                v2e[v1p] = (3 * f1) + 2;
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
}

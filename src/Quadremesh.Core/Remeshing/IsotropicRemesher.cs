using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Remeshing;

/// <summary>
/// Stage-0 isotropic triangle remeshing (Botsch–Kobbelt): repeatedly split long edges, collapse short
/// edges, flip toward valence 6, then tangentially relax and re-project onto the original surface.
/// Produces a near-uniform triangle mesh at the requested resolution — the clean input the field
/// solvers expect. Boundaries (and non-manifold debris) are pinned/dropped rather than reshaped.
/// </summary>
public static class IsotropicRemesher
{
    private const int MaxSplitPasses = 16;
    private const int MaxCollapsePasses = 16;

    /// <summary>Runs the pre-pass and returns a new near-isotropic triangle mesh.</summary>
    public static TriangleMesh Run(TriangleMesh input, IsotropicRemeshOptions options)
    {
        ArgumentNullException.ThrowIfNull(input);
        ArgumentNullException.ThrowIfNull(options);
        if (input.TriangleCount == 0)
        {
            return input;
        }

        Bvh surface = Bvh.Build(input);
        double targetLength = ResolveTargetLength(input, options);
        double high = 4.0 / 3.0 * targetLength;
        double low = 4.0 / 5.0 * targetLength;

        var dyn = DynamicTriangleMesh.FromTriangleMesh(input);
        double? featureCos = options.FeatureAngleDegrees is { } deg
            ? System.Math.Cos(deg * System.Math.PI / 180.0)
            : null;

        for (int iter = 0; iter < options.Iterations; iter++)
        {
            SplitLongEdges(dyn, surface, high);
            CreaseInfo? crease = featureCos is { } cos ? BuildCreaseInfo(dyn, cos) : null;
            CollapseShortEdges(dyn, surface, low, high, crease);
            FlipToImproveValence(dyn, featureCos);
            TangentialRelaxation(dyn, surface, options.RelaxationWeight, crease);
        }

        return dyn.ToTriangleMesh();
    }

    private static double ResolveTargetLength(TriangleMesh input, IsotropicRemeshOptions options)
    {
        if (options.TargetEdgeLength is { } len and > 0)
        {
            return len;
        }

        if (options.TargetQuadCount is { } quads and > 0)
        {
            double area = 0.0;
            for (int t = 0; t < input.TriangleCount; t++)
            {
                (int a, int b, int c) = input.Triangle(t);
                area += MathUtils.TriangleArea(input.Positions[a], input.Positions[b], input.Positions[c]);
            }

            return System.Math.Sqrt(area / quads);
        }

        return MeanEdgeLength(input);
    }

    private static double MeanEdgeLength(TriangleMesh input)
    {
        double sum = 0.0;
        int count = 0;
        for (int t = 0; t < input.TriangleCount; t++)
        {
            (int a, int b, int c) = input.Triangle(t);
            sum += Vector3.Distance(input.Positions[a], input.Positions[b]);
            sum += Vector3.Distance(input.Positions[b], input.Positions[c]);
            sum += Vector3.Distance(input.Positions[c], input.Positions[a]);
            count += 3;
        }

        return count == 0 ? 1.0 : sum / count;
    }

    private static void SplitLongEdges(DynamicTriangleMesh dyn, Bvh surface, double high)
    {
        double highSq = high * high;
        for (int pass = 0; pass < MaxSplitPasses; pass++)
        {
            bool any = false;
            foreach ((int a, int b) in CollectEdges(dyn))
            {
                if (!EdgeAlive(dyn, a, b) || dyn.EdgeIsBoundary(a, b))
                {
                    continue;
                }

                if (Vector3.DistanceSquared(dyn.Position(a), dyn.Position(b)) > highSq)
                {
                    Vector3 mid = (dyn.Position(a) + dyn.Position(b)) * 0.5;
                    int m = dyn.Split(a, b, surface.ClosestPoint(mid));
                    if (m >= 0)
                    {
                        any = true;
                    }
                }
            }

            if (!any)
            {
                break;
            }
        }
    }

    private static void CollapseShortEdges(
        DynamicTriangleMesh dyn, Bvh surface, double low, double high, CreaseInfo? crease)
    {
        double lowSq = low * low;
        double highSq = high * high;
        for (int pass = 0; pass < MaxCollapsePasses; pass++)
        {
            bool any = false;
            foreach ((int a, int b) in CollectEdges(dyn))
            {
                if (!EdgeAlive(dyn, a, b) || dyn.EdgeIsBoundary(a, b))
                {
                    continue;
                }

                // Never collapse a corner, nor an edge that crosses a crease boundary (one endpoint on a
                // feature, the other off it) — that would drag the crease off the model's hard edge.
                if (crease is not null &&
                    (crease.IsCorner(a) || crease.IsCorner(b) || crease.IsFeature(a) != crease.IsFeature(b)))
                {
                    continue;
                }

                if (Vector3.DistanceSquared(dyn.Position(a), dyn.Position(b)) >= lowSq)
                {
                    continue;
                }

                Vector3 target = surface.ClosestPoint((dyn.Position(a) + dyn.Position(b)) * 0.5);
                if (WouldCreateLongEdge(dyn, a, b, target, highSq))
                {
                    continue;
                }

                if (dyn.Collapse(a, b, target))
                {
                    any = true;
                }
            }

            if (!any)
            {
                break;
            }
        }
    }

    /// <summary>True if merging a→b at <paramref name="target"/> would leave any incident edge longer than the high threshold.</summary>
    private static bool WouldCreateLongEdge(DynamicTriangleMesh dyn, int a, int b, Vector3 target, double highSq)
    {
        foreach (int n in dyn.Neighbors(a))
        {
            if (n != b && Vector3.DistanceSquared(target, dyn.Position(n)) > highSq)
            {
                return true;
            }
        }

        foreach (int n in dyn.Neighbors(b))
        {
            if (n != a && Vector3.DistanceSquared(target, dyn.Position(n)) > highSq)
            {
                return true;
            }
        }

        return false;
    }

    private static void FlipToImproveValence(DynamicTriangleMesh dyn, double? featureCos)
    {
        foreach ((int a, int b) in CollectEdges(dyn))
        {
            if (!EdgeAlive(dyn, a, b) || dyn.EdgeIsBoundary(a, b))
            {
                continue;
            }

            // Flipping a crease edge would erase the feature — leave it alone.
            if (featureCos is { } cos && IsSharpEdge(dyn, a, b, cos))
            {
                continue;
            }

            if (!TryApexes(dyn, a, b, out int c, out int d))
            {
                continue;
            }

            int va = dyn.Neighbors(a).Count;
            int vb = dyn.Neighbors(b).Count;
            int vc = dyn.Neighbors(c).Count;
            int vd = dyn.Neighbors(d).Count;

            int before = Deviation(va) + Deviation(vb) + Deviation(vc) + Deviation(vd);
            int after = Deviation(va - 1) + Deviation(vb - 1) + Deviation(vc + 1) + Deviation(vd + 1);
            if (after < before)
            {
                dyn.Flip(a, b);
            }
        }
    }

    private static void TangentialRelaxation(
        DynamicTriangleMesh dyn, Bvh surface, double weight, CreaseInfo? crease)
    {
        for (int v = 0; v < dyn.VertexCapacity; v++)
        {
            if (dyn.VertexRemoved(v) || dyn.IsBoundaryVertex(v))
            {
                continue;
            }

            // A feature corner is fully pinned; a clean crease vertex relaxes only along the crease so
            // it stays exactly on the hard edge instead of being pulled onto an adjacent face.
            bool onCrease = false;
            Vector3 creaseTangent = Vector3.Zero;
            if (crease is not null && crease.IsFeature(v))
            {
                if (crease.IsCorner(v))
                {
                    continue;
                }

                onCrease = true;
                creaseTangent = crease.Tangent(v);
            }

            HashSet<int> neighbors = dyn.Neighbors(v);
            if (neighbors.Count == 0)
            {
                continue;
            }

            Vector3 centroid = Vector3.Zero;
            foreach (int n in neighbors)
            {
                centroid += dyn.Position(n);
            }

            centroid /= neighbors.Count;

            Vector3 p = dyn.Position(v);
            Vector3 delta = centroid - p;
            if (onCrease)
            {
                // Slide along the crease only; do not surface-project (that would round the edge).
                Vector3 along = Vector3.Dot(delta, creaseTangent) * creaseTangent;
                dyn.SetPosition(v, p + (along * weight));
                continue;
            }

            Vector3 normal = VertexNormal(dyn, v);
            // Move toward the centroid but only within the tangent plane (keep the normal component).
            Vector3 tangential = delta - (Vector3.Dot(delta, normal) * normal);
            Vector3 moved = p + (tangential * weight);
            dyn.SetPosition(v, surface.ClosestPoint(moved));
        }
    }

    /// <summary>Per-vertex crease classification for the current dynamic mesh (built fresh each iteration).</summary>
    private sealed class CreaseInfo
    {
        private readonly bool[] _feature;
        private readonly bool[] _corner;
        private readonly Vector3[] _tangent;

        public CreaseInfo(bool[] feature, bool[] corner, Vector3[] tangent)
        {
            _feature = feature;
            _corner = corner;
            _tangent = tangent;
        }

        public bool IsFeature(int v) => v < _feature.Length && _feature[v];

        public bool IsCorner(int v) => v < _corner.Length && _corner[v];

        public Vector3 Tangent(int v) => _tangent[v];
    }

    /// <summary>True if the undirected edge (a,b)'s two faces meet at more than the crease angle.</summary>
    private static bool IsSharpEdge(DynamicTriangleMesh dyn, int a, int b, double cosThreshold)
    {
        Vector3 n0 = Vector3.Zero;
        Vector3 n1 = Vector3.Zero;
        int found = 0;
        foreach (int t in dyn.IncidentTriangles(a))
        {
            (int x, int y, int z) = dyn.Triangle(t);
            if ((x == b) || (y == b) || (z == b))
            {
                Vector3 n = MathUtils.TriangleNormalAreaWeighted(dyn.Position(x), dyn.Position(y), dyn.Position(z));
                if (n.TryNormalize(out Vector3 unit))
                {
                    if (found == 0)
                    {
                        n0 = unit;
                    }
                    else
                    {
                        n1 = unit;
                    }
                }

                found++;
            }
        }

        if (found < 2)
        {
            return found == 1; // boundary edge counts as a feature
        }

        return Vector3.Dot(n0, n1) < cosThreshold;
    }

    /// <summary>Builds the crease classification by dihedral angle over the current triangulation.</summary>
    private static CreaseInfo BuildCreaseInfo(DynamicTriangleMesh dyn, double cosThreshold)
    {
        int cap = dyn.VertexCapacity;

        // Per undirected edge, accumulate its (≤2) incident face normals.
        var n0 = new Dictionary<long, Vector3>();
        var n1 = new Dictionary<long, Vector3>();
        var count = new Dictionary<long, int>();
        for (int t = 0; t < dyn.TriangleCapacity; t++)
        {
            if (dyn.TriangleRemoved(t))
            {
                continue;
            }

            (int a, int b, int c) = dyn.Triangle(t);
            Vector3 n = MathUtils.TriangleNormalAreaWeighted(dyn.Position(a), dyn.Position(b), dyn.Position(c));
            n = n.TryNormalize(out Vector3 unit) ? unit : Vector3.UnitZ;
            AddFace(a, b, n);
            AddFace(b, c, n);
            AddFace(c, a, n);
        }

        void AddFace(int u, int w, Vector3 n)
        {
            long key = ((long)System.Math.Min(u, w) << 32) | (uint)System.Math.Max(u, w);
            if (count.TryGetValue(key, out int cnt))
            {
                count[key] = cnt + 1;
                n1[key] = n;
            }
            else
            {
                count[key] = 1;
                n0[key] = n;
            }
        }

        var sharpDirs = new List<Vector3>?[cap];
        foreach ((long key, int cnt) in count)
        {
            bool sharp = cnt == 1 || Vector3.Dot(n0[key], n1[key]) < cosThreshold;
            if (!sharp)
            {
                continue;
            }

            int u = (int)(key >> 32);
            int w = (int)(key & 0xFFFFFFFF);
            AddDir(u, w);
            AddDir(w, u);
        }

        void AddDir(int v, int other)
        {
            if ((dyn.Position(other) - dyn.Position(v)).TryNormalize(out Vector3 dir))
            {
                (sharpDirs[v] ??= new List<Vector3>()).Add(dir);
            }
        }

        var feature = new bool[cap];
        var corner = new bool[cap];
        var tangent = new Vector3[cap];
        for (int v = 0; v < cap; v++)
        {
            List<Vector3>? dirs = sharpDirs[v];
            if (dirs is null || dirs.Count == 0)
            {
                continue;
            }

            feature[v] = true;
            double bestDot = 1.0;
            int bi = -1, bj = -1;
            for (int i = 0; i < dirs.Count; i++)
            {
                for (int j = i + 1; j < dirs.Count; j++)
                {
                    double dot = Vector3.Dot(dirs[i], dirs[j]);
                    if (dot < bestDot)
                    {
                        bestDot = dot;
                        bi = i;
                        bj = j;
                    }
                }
            }

            if (bi >= 0 && bestDot < -0.5 && (dirs[bi] - dirs[bj]).TryNormalize(out Vector3 t))
            {
                tangent[v] = t;
            }
            else
            {
                corner[v] = true;
            }
        }

        return new CreaseInfo(feature, corner, tangent);
    }

    private static Vector3 VertexNormal(DynamicTriangleMesh dyn, int v)
    {
        Vector3 n = Vector3.Zero;
        foreach (int t in dyn.IncidentTriangles(v))
        {
            (int a, int b, int c) = dyn.Triangle(t);
            n += MathUtils.TriangleNormalAreaWeighted(dyn.Position(a), dyn.Position(b), dyn.Position(c));
        }

        return n.TryNormalize(out Vector3 unit) ? unit : Vector3.UnitZ;
    }

    private static bool TryApexes(DynamicTriangleMesh dyn, int a, int b, out int c, out int d)
    {
        c = Opposite(dyn, a, b);
        d = Opposite(dyn, b, a);
        return c >= 0 && d >= 0 && c != d;
    }

    /// <summary>The apex of the triangle on the (a→b) side of the edge, or -1 if none.</summary>
    private static int Opposite(DynamicTriangleMesh dyn, int a, int b)
    {
        foreach (int t in dyn.IncidentTriangles(a))
        {
            (int x, int y, int z) = dyn.Triangle(t);
            // Triangle contains directed edge a->b iff (a,b) appear consecutively in winding order.
            if ((x == a && y == b) || (y == a && z == b) || (z == a && x == b))
            {
                return x != a && x != b ? x : y != a && y != b ? y : z;
            }
        }

        return -1;
    }

    private static int Deviation(int valence)
    {
        int d = valence - 6;
        return d * d;
    }

    private static bool EdgeAlive(DynamicTriangleMesh dyn, int a, int b) =>
        !dyn.VertexRemoved(a) && !dyn.VertexRemoved(b) && dyn.Neighbors(a).Contains(b);

    /// <summary>Snapshots the current unique undirected edges (each as an ordered pair a &lt; b).</summary>
    private static List<(int A, int B)> CollectEdges(DynamicTriangleMesh dyn)
    {
        var seen = new HashSet<long>();
        var edges = new List<(int, int)>();
        for (int t = 0; t < dyn.TriangleCapacity; t++)
        {
            if (dyn.TriangleRemoved(t))
            {
                continue;
            }

            (int a, int b, int c) = dyn.Triangle(t);
            AddEdge(seen, edges, a, b);
            AddEdge(seen, edges, b, c);
            AddEdge(seen, edges, c, a);
        }

        return edges;
    }

    private static void AddEdge(HashSet<long> seen, List<(int, int)> edges, int u, int w)
    {
        int lo = System.Math.Min(u, w);
        int hi = System.Math.Max(u, w);
        if (seen.Add(((long)lo << 32) | (uint)hi))
        {
            edges.Add((lo, hi));
        }
    }
}

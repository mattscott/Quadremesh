using Quadremesh.Core.Math;

namespace Quadremesh.Core.Acceleration;

/// <summary>
/// A bounding-volume hierarchy over a static triangle set, supporting nearest-point queries. Used by
/// the isotropic remesher to project relaxed vertices back onto the original surface (Stage 0).
/// Build is one-shot (median split); the tree is immutable afterwards.
/// </summary>
public sealed class Bvh
{
    private const int LeafSize = 4;

    private readonly Vector3[] _positions;
    private readonly int[] _indices;       // 3 per triangle
    private readonly int[] _triangleOrder; // triangle ids permuted by the build
    private readonly Node[] _nodes;

    private Bvh(Vector3[] positions, int[] indices, int[] triangleOrder, Node[] nodes)
    {
        _positions = positions;
        _indices = indices;
        _triangleOrder = triangleOrder;
        _nodes = nodes;
    }

    /// <summary>Number of triangles indexed.</summary>
    public int TriangleCount => _indices.Length / 3;

    /// <summary>Builds a BVH over the triangles of <paramref name="mesh"/>.</summary>
    public static Bvh Build(Mesh.TriangleMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        return Build(mesh.Positions, mesh.Indices);
    }

    /// <summary>Builds a BVH over the given triangle soup.</summary>
    public static Bvh Build(Vector3[] positions, int[] indices)
    {
        ArgumentNullException.ThrowIfNull(positions);
        ArgumentNullException.ThrowIfNull(indices);

        int triCount = indices.Length / 3;
        var order = new int[triCount];
        var centroids = new Vector3[triCount];
        var triBounds = new Aabb[triCount];
        for (int t = 0; t < triCount; t++)
        {
            order[t] = t;
            Vector3 a = positions[indices[(t * 3) + 0]];
            Vector3 b = positions[indices[(t * 3) + 1]];
            Vector3 c = positions[indices[(t * 3) + 2]];
            triBounds[t] = Aabb.FromTriangle(a, b, c);
            centroids[t] = (a + b + c) / 3.0;
        }

        var nodes = new List<Node>(System.Math.Max(1, triCount));
        if (triCount > 0)
        {
            BuildRecursive(order, centroids, triBounds, 0, triCount, nodes);
        }
        else
        {
            nodes.Add(new Node { Bounds = Aabb.Empty, Start = 0, Count = 0, Left = -1, Right = -1 });
        }

        return new Bvh(positions, indices, order, nodes.ToArray());
    }

    /// <summary>Returns the closest point to <paramref name="query"/> on the triangle surface.</summary>
    public Vector3 ClosestPoint(Vector3 query) => ClosestPoint(query, out _);

    /// <summary>Returns the closest point on the surface and the squared distance to it.</summary>
    public Vector3 ClosestPoint(Vector3 query, out double distanceSquared)
    {
        Vector3 best = _positions[_indices[0]];
        double bestSq = double.PositiveInfinity;
        SearchClosest(0, query, ref best, ref bestSq);
        distanceSquared = bestSq;
        return best;
    }

    private void SearchClosest(int nodeIndex, Vector3 q, ref Vector3 best, ref double bestSq)
    {
        Node node = _nodes[nodeIndex];
        if (node.Bounds.DistanceSquared(q) >= bestSq)
        {
            return; // whole subtree is farther than the current best
        }

        if (node.IsLeaf)
        {
            for (int i = 0; i < node.Count; i++)
            {
                int t = _triangleOrder[node.Start + i];
                Vector3 a = _positions[_indices[(t * 3) + 0]];
                Vector3 b = _positions[_indices[(t * 3) + 1]];
                Vector3 c = _positions[_indices[(t * 3) + 2]];
                Vector3 p = ClosestPointOnTriangle(q, a, b, c);
                double dSq = Vector3.DistanceSquared(q, p);
                if (dSq < bestSq)
                {
                    bestSq = dSq;
                    best = p;
                }
            }

            return;
        }

        // Descend into the nearer child first to tighten the bound sooner.
        double dl = _nodes[node.Left].Bounds.DistanceSquared(q);
        double dr = _nodes[node.Right].Bounds.DistanceSquared(q);
        if (dl <= dr)
        {
            SearchClosest(node.Left, q, ref best, ref bestSq);
            SearchClosest(node.Right, q, ref best, ref bestSq);
        }
        else
        {
            SearchClosest(node.Right, q, ref best, ref bestSq);
            SearchClosest(node.Left, q, ref best, ref bestSq);
        }
    }

    private static int BuildRecursive(
        int[] order, Vector3[] centroids, Aabb[] triBounds, int start, int count, List<Node> nodes)
    {
        Aabb bounds = Aabb.Empty;
        for (int i = 0; i < count; i++)
        {
            bounds = bounds.Union(triBounds[order[start + i]]);
        }

        int nodeIndex = nodes.Count;
        nodes.Add(default);

        if (count <= LeafSize)
        {
            nodes[nodeIndex] = new Node { Bounds = bounds, Start = start, Count = count, Left = -1, Right = -1 };
            return nodeIndex;
        }

        // Split along the largest centroid extent, at the median.
        Aabb centroidBounds = Aabb.Empty;
        for (int i = 0; i < count; i++)
        {
            centroidBounds = centroidBounds.Include(centroids[order[start + i]]);
        }

        int axis = centroidBounds.LargestAxis();
        int mid = start + (count / 2);
        SortByAxis(order, centroids, start, count, axis, mid);

        int left = BuildRecursive(order, centroids, triBounds, start, mid - start, nodes);
        int right = BuildRecursive(order, centroids, triBounds, mid, start + count - mid, nodes);
        nodes[nodeIndex] = new Node { Bounds = bounds, Start = start, Count = count, Left = left, Right = right };
        return nodeIndex;
    }

    /// <summary>Partitions <paramref name="order"/>[start, start+count) so the element at <paramref name="nth"/> is the median along <paramref name="axis"/> (quickselect).</summary>
    private static void SortByAxis(int[] order, Vector3[] centroids, int start, int count, int axis, int nth)
    {
        int lo = start;
        int hi = start + count - 1;
        while (lo < hi)
        {
            int pivot = Partition(order, centroids, lo, hi, axis);
            if (pivot == nth)
            {
                return;
            }

            if (pivot < nth)
            {
                lo = pivot + 1;
            }
            else
            {
                hi = pivot - 1;
            }
        }
    }

    private static int Partition(int[] order, Vector3[] centroids, int lo, int hi, int axis)
    {
        double pivot = Axis(centroids[order[hi]], axis);
        int i = lo;
        for (int j = lo; j < hi; j++)
        {
            if (Axis(centroids[order[j]], axis) < pivot)
            {
                (order[i], order[j]) = (order[j], order[i]);
                i++;
            }
        }

        (order[i], order[hi]) = (order[hi], order[i]);
        return i;
    }

    private static double Axis(Vector3 v, int axis) => axis == 0 ? v.X : axis == 1 ? v.Y : v.Z;

    /// <summary>Closest point to <paramref name="p"/> on triangle (a, b, c). Ericson, Real-Time Collision Detection.</summary>
    public static Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = p - a;
        double d1 = Vector3.Dot(ab, ap);
        double d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0 && d2 <= 0)
        {
            return a;
        }

        Vector3 bp = p - b;
        double d3 = Vector3.Dot(ab, bp);
        double d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0 && d4 <= d3)
        {
            return b;
        }

        double vc = (d1 * d4) - (d3 * d2);
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            double v = d1 / (d1 - d3);
            return a + (ab * v);
        }

        Vector3 cp = p - c;
        double d5 = Vector3.Dot(ab, cp);
        double d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0 && d5 <= d6)
        {
            return c;
        }

        double vb = (d5 * d2) - (d1 * d6);
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            double w = d2 / (d2 - d6);
            return a + (ac * w);
        }

        double va = (d3 * d6) - (d5 * d4);
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + ((c - b) * w);
        }

        double denom = 1.0 / (va + vb + vc);
        double vv = vb * denom;
        double ww = vc * denom;
        return a + (ab * vv) + (ac * ww);
    }

    private struct Node
    {
        public Aabb Bounds;
        public int Start;
        public int Count;
        public int Left;
        public int Right;

        public readonly bool IsLeaf => Left < 0;
    }
}

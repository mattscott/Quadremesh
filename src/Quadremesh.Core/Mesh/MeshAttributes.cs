using Quadremesh.Core.Math;

namespace Quadremesh.Core.Mesh;

/// <summary>
/// Derived per-vertex / per-edge quantities the field solvers consume: vertex normals, smoothing
/// area weights, and cotangent edge weights. Computed from a <see cref="HalfEdgeMesh"/> and its
/// positions; all are tolerant of degenerate faces (they contribute nothing rather than NaNs).
/// </summary>
public static class MeshAttributes
{
    /// <summary>
    /// Area-weighted vertex normals via Newell's method (robust for non-planar polygons). Degenerate
    /// vertices receive <see cref="Vector3.Zero"/>.
    /// </summary>
    public static Vector3[] VertexNormals(HalfEdgeMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        Vector3[] pos = mesh.Positions;
        var normals = new Vector3[mesh.VertexCount];

        Span<int> corners = stackalloc int[16];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            int k = ReadFace(mesh, f, ref corners);
            Vector3 n = NewellNormal(pos, corners, k);
            for (int i = 0; i < k; i++)
            {
                normals[corners[i]] += n; // Newell magnitude ~ 2*area, so this is area-weighted
            }
        }

        for (int v = 0; v < normals.Length; v++)
        {
            normals[v] = normals[v].TryNormalize(out Vector3 unit) ? unit : Vector3.Zero;
        }

        return normals;
    }

    /// <summary>
    /// Barycentric vertex areas: each face contributes an equal share of its area to each corner.
    /// Used as the smoothing weight a_i in the field updates.
    /// </summary>
    public static double[] VertexAreas(HalfEdgeMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        Vector3[] pos = mesh.Positions;
        var areas = new double[mesh.VertexCount];

        Span<int> corners = stackalloc int[16];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            int k = ReadFace(mesh, f, ref corners);
            double area = 0.5 * NewellNormal(pos, corners, k).Length;
            double share = area / k;
            for (int i = 0; i < k; i++)
            {
                areas[corners[i]] += share;
            }
        }

        return areas;
    }

    /// <summary>
    /// Cotangent weights per undirected edge: w_ij = ½·(cot α + cot β), where α, β are the angles
    /// opposite the edge in its two adjacent triangles. Only triangular faces contribute; boundary
    /// edges get the single available term. Length equals <see cref="HalfEdgeMesh.EdgeCount"/>.
    /// </summary>
    public static double[] CotangentEdgeWeights(HalfEdgeMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        Vector3[] pos = mesh.Positions;
        var weights = new double[mesh.EdgeCount];

        for (int e = 0; e < mesh.EdgeCount; e++)
        {
            int h = mesh.EdgeHalfEdge(e);
            double sum = HalfEdgeCotangent(mesh, pos, h);
            int t = mesh.Twin(h);
            if (t >= 0)
            {
                sum += HalfEdgeCotangent(mesh, pos, t);
            }

            weights[e] = 0.5 * sum;
        }

        return weights;
    }

    /// <summary>Cotangent of the angle opposite half-edge <paramref name="h"/> within its triangle (0 for non-triangles).</summary>
    private static double HalfEdgeCotangent(HalfEdgeMesh mesh, Vector3[] pos, int h)
    {
        if (mesh.FaceSize(mesh.Face(h)) != 3)
        {
            return 0.0;
        }

        int a = mesh.Origin(h);
        int b = mesh.Head(h);
        int c = mesh.Head(mesh.Next(h)); // the third corner, opposite edge (a, b)
        return MathUtils.Cotangent(pos[c], pos[a], pos[b]);
    }

    private static int ReadFace(HalfEdgeMesh mesh, int f, ref Span<int> corners)
    {
        int size = mesh.FaceSize(f);
        if (size > corners.Length)
        {
            corners = new int[size];
        }

        return mesh.GetFaceVertices(f, corners);
    }

    private static Vector3 NewellNormal(Vector3[] pos, ReadOnlySpan<int> corners, int k)
    {
        double nx = 0, ny = 0, nz = 0;
        for (int i = 0; i < k; i++)
        {
            Vector3 cur = pos[corners[i]];
            Vector3 nxt = pos[corners[(i + 1) % k]];
            nx += (cur.Y - nxt.Y) * (cur.Z + nxt.Z);
            ny += (cur.Z - nxt.Z) * (cur.X + nxt.X);
            nz += (cur.X - nxt.X) * (cur.Y + nxt.Y);
        }

        return new Vector3(nx, ny, nz);
    }
}

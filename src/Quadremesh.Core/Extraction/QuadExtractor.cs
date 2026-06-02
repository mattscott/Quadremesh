using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Fields;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>Inputs to <see cref="QuadExtractor"/>: the prepared mesh plus its converged fields.</summary>
public sealed class ExtractionInput
{
    /// <summary>Connectivity of the (isotropic) input mesh.</summary>
    public required HalfEdgeMesh Mesh { get; init; }

    /// <summary>Per-vertex 4-RoSy orientation (P3).</summary>
    public required Vector3[] Orientation { get; init; }

    /// <summary>Per-vertex normal.</summary>
    public required Vector3[] Normal { get; init; }

    /// <summary>Per-vertex position-field anchor (P4).</summary>
    public required Vector3[] Anchor { get; init; }

    /// <summary>Per-vertex lattice spacing ℓ.</summary>
    public required double[] Scale { get; init; }

    /// <summary>Optional original surface; when set, output vertices are projected onto it.</summary>
    public Bvh? Surface { get; init; }
}

/// <summary>
/// Quad-dominant extraction (Instant Meshes Stage 3): collapse the position field's lattice into quad
/// vertices, build edges from unit lattice offsets, then trace minimal cycles via a per-vertex
/// rotation system into faces (4-cycles preferred; 3/5-cycles tolerated near singularities).
/// </summary>
public static class QuadExtractor
{
    // On a closed surface the rotation system tiles completely, so every traced cycle is a real face;
    // we keep them all (up to a safety cap that only guards against a pathological runaway trace).
    private const int MaxFaceSize = 64;

    /// <summary>Extracts a quad-dominant mesh from the converged fields.</summary>
    public static QuadMesh Extract(ExtractionInput input)
    {
        ArgumentNullException.ThrowIfNull(input);

        (int[] vertexToOut, int outCount) = CollapseToLatticeVertices(input);
        (Vector3[] outPos, Vector3[] outNormal, Vector3[] outOrient) =
            AccumulateOutputVertices(input, vertexToOut, outCount);
        List<int>[] adjacency = BuildEdges(input, vertexToOut, outCount);
        List<int[]> faces = TraceFaces(outPos, outNormal, outOrient, adjacency);
        faces = MergeTrianglePairs(faces);

        return Finalize(outPos, faces, input.Surface);
    }

    // --- Step 1: collapse input vertices that share a lattice cell ----------------------------

    private static (int[] VertexToOut, int OutCount) CollapseToLatticeVertices(ExtractionInput input)
    {
        HalfEdgeMesh mesh = input.Mesh;
        var dsu = new UnionFind(mesh.VertexCount);
        for (int e = 0; e < mesh.EdgeCount; e++)
        {
            (int a, int b) = mesh.Edge(e);
            if (Classify(input, a, b) == EdgeKind.Same)
            {
                dsu.Union(a, b);
            }
        }

        var rootToOut = new Dictionary<int, int>();
        var vertexToOut = new int[mesh.VertexCount];
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            int root = dsu.Find(v);
            if (!rootToOut.TryGetValue(root, out int outId))
            {
                outId = rootToOut.Count;
                rootToOut[root] = outId;
            }

            vertexToOut[v] = outId;
        }

        return (vertexToOut, rootToOut.Count);
    }

    private static (Vector3[] Pos, Vector3[] Normal, Vector3[] Orient) AccumulateOutputVertices(
        ExtractionInput input, int[] vertexToOut, int outCount)
    {
        var posAccum = new Vector3[outCount];
        var normalAccum = new Vector3[outCount];
        var count = new int[outCount];
        for (int v = 0; v < input.Mesh.VertexCount; v++)
        {
            int o = vertexToOut[v];
            posAccum[o] += input.Anchor[v];
            normalAccum[o] += input.Normal[v];
            count[o]++;
        }

        var pos = new Vector3[outCount];
        var normal = new Vector3[outCount];
        for (int o = 0; o < outCount; o++)
        {
            pos[o] = posAccum[o] / count[o];
            normal[o] = normalAccum[o].TryNormalize(out Vector3 n) ? n : Vector3.UnitZ;
        }

        // RoSy-average member orientations into a single cross direction per output vertex.
        var orientAccum = new Vector3[outCount];
        var hasOrient = new bool[outCount];
        for (int v = 0; v < input.Mesh.VertexCount; v++)
        {
            int o = vertexToOut[v];
            if (!hasOrient[o])
            {
                orientAccum[o] = input.Orientation[v];
                hasOrient[o] = true;
            }
            else
            {
                orientAccum[o] += Rosy.ClosestRepresentative(orientAccum[o], input.Orientation[v], input.Normal[v]);
            }
        }

        var orient = new Vector3[outCount];
        for (int o = 0; o < outCount; o++)
        {
            Vector3 t = TangentFrame.ProjectToTangent(orientAccum[o], normal[o]);
            orient[o] = t.TryNormalize(out Vector3 u) ? u : TangentFrame.AnyTangent(normal[o]);
        }

        return (pos, normal, orient);
    }

    // --- Step 2: build output edges from unit lattice offsets ---------------------------------

    private static List<int>[] BuildEdges(ExtractionInput input, int[] vertexToOut, int outCount)
    {
        var adjacency = new List<int>[outCount];
        for (int o = 0; o < outCount; o++)
        {
            adjacency[o] = new List<int>();
        }

        var seen = new HashSet<long>();
        HalfEdgeMesh mesh = input.Mesh;
        for (int e = 0; e < mesh.EdgeCount; e++)
        {
            (int a, int b) = mesh.Edge(e);
            int oa = vertexToOut[a];
            int ob = vertexToOut[b];
            if (oa == ob)
            {
                continue;
            }

            if (Classify(input, a, b) != EdgeKind.Unit)
            {
                continue; // only unit lattice steps are quad edges (skip diagonals / long spans)
            }

            long key = ((long)System.Math.Min(oa, ob) << 32) | (uint)System.Math.Max(oa, ob);
            if (seen.Add(key))
            {
                adjacency[oa].Add(ob);
                adjacency[ob].Add(oa);
            }
        }

        return adjacency;
    }

    // --- Step 3: trace faces via the rotation system ------------------------------------------

    private static List<int[]> TraceFaces(
        Vector3[] pos, Vector3[] normal, Vector3[] orient, List<int>[] adjacency)
    {
        int n = pos.Length;

        // Sort each vertex's neighbours counter-clockwise in its tangent plane (the rotation system).
        var sorted = new int[n][];
        var indexOf = new Dictionary<long, int>();
        for (int v = 0; v < n; v++)
        {
            Vector3 e1 = orient[v];
            Vector3 e2 = Vector3.Cross(normal[v], e1);
            List<int> nbrs = adjacency[v];
            var byAngle = new (double Angle, int Node)[nbrs.Count];
            for (int i = 0; i < nbrs.Count; i++)
            {
                Vector3 d = TangentFrame.ProjectToTangent(pos[nbrs[i]] - pos[v], normal[v]);
                byAngle[i] = (System.Math.Atan2(Vector3.Dot(d, e2), Vector3.Dot(d, e1)), nbrs[i]);
            }

            Array.Sort(byAngle, (x, y) => x.Angle.CompareTo(y.Angle));
            var order = new int[nbrs.Count];
            for (int i = 0; i < order.Length; i++)
            {
                order[i] = byAngle[i].Node;
                indexOf[((long)v << 32) | (uint)byAngle[i].Node] = i;
            }

            sorted[v] = order;
        }

        // Trace each directed half-edge once. next(u->v) leaves v toward the neighbour just clockwise
        // of u in v's CCW order, which walks the face on the left as a counter-clockwise cycle.
        var visited = new HashSet<long>();
        var faces = new List<int[]>();
        for (int u = 0; u < n; u++)
        {
            foreach (int v0 in adjacency[u])
            {
                if (!visited.Add(((long)u << 32) | (uint)v0))
                {
                    continue;
                }

                var cycle = new List<int>();
                int cu = u;
                int cv = v0;
                bool ok = true;
                while (true)
                {
                    cycle.Add(cu);
                    if (cycle.Count > MaxFaceSize)
                    {
                        ok = false;
                        break;
                    }

                    int[] order = sorted[cv];
                    int idx = indexOf[((long)cv << 32) | (uint)cu];
                    int nextNode = order[(idx - 1 + order.Length) % order.Length];
                    visited.Add(((long)cv << 32) | (uint)nextNode);

                    if (cv == u && nextNode == v0)
                    {
                        break; // closed the cycle
                    }

                    cu = cv;
                    cv = nextNode;
                }

                if (ok && cycle.Count is >= 3 and <= MaxFaceSize)
                {
                    faces.Add(cycle.ToArray());
                }
            }
        }

        return faces;
    }

    // --- Cleanup: merge adjacent triangle pairs into quads ------------------------------------

    private static List<int[]> MergeTrianglePairs(List<int[]> faces)
    {
        // Map each undirected edge to the (≤2) faces using it.
        var edgeFaces = new Dictionary<long, (int F0, int F1, int Count)>();
        for (int f = 0; f < faces.Count; f++)
        {
            int[] face = faces[f];
            for (int i = 0; i < face.Length; i++)
            {
                long key = EdgeKey(face[i], face[(i + 1) % face.Length]);
                edgeFaces.TryGetValue(key, out (int F0, int F1, int Count) slot);
                edgeFaces[key] = slot.Count switch
                {
                    0 => (f, -1, 1),
                    1 => (slot.F0, f, 2),
                    _ => (slot.F0, slot.F1, slot.Count + 1),
                };
            }
        }

        var consumed = new bool[faces.Count];
        var result = new List<int[]>(faces.Count);
        foreach ((long key, (int f0, int f1, int count)) in edgeFaces)
        {
            if (count != 2 || consumed[f0] || consumed[f1] || faces[f0].Length != 3 || faces[f1].Length != 3)
            {
                continue;
            }

            int p = (int)(key >> 32);
            int q = (int)(key & 0xFFFFFFFF);
            int[]? quad = MergeTriangles(faces[f0], faces[f1], p, q);
            if (quad is null)
            {
                continue;
            }

            consumed[f0] = true;
            consumed[f1] = true;
            result.Add(quad);
        }

        for (int f = 0; f < faces.Count; f++)
        {
            if (!consumed[f])
            {
                result.Add(faces[f]);
            }
        }

        return result;
    }

    /// <summary>Merges two triangles sharing edge {p, q} into a single quad (or null if inconsistent).</summary>
    private static int[]? MergeTriangles(int[] t1, int[] t2, int p, int q)
    {
        if (!FindDirected(t1, p, q, out int u, out int v, out int w))
        {
            return null;
        }

        // t2 must traverse the shared edge in the opposite direction (v -> u) to be consistently wound.
        if (!FindDirected(t2, p, q, out int u2, out int _, out int x) || u2 != v)
        {
            return null;
        }

        return new[] { u, x, v, w }; // boundary loop u -> x -> v -> w
    }

    private static bool FindDirected(int[] tri, int p, int q, out int u, out int v, out int third)
    {
        for (int i = 0; i < 3; i++)
        {
            int a = tri[i];
            int b = tri[(i + 1) % 3];
            if ((a == p && b == q) || (a == q && b == p))
            {
                u = a;
                v = b;
                third = tri[(i + 2) % 3];
                return true;
            }
        }

        u = v = third = -1;
        return false;
    }

    private static long EdgeKey(int a, int b) =>
        ((long)System.Math.Min(a, b) << 32) | (uint)System.Math.Max(a, b);

    // --- Step 4: finalise (project, compact, build) -------------------------------------------

    private static QuadMesh Finalize(Vector3[] outPos, List<int[]> faces, Bvh? surface)
    {
        // Keep only vertices referenced by a kept face, remapped to a dense range.
        var remap = new int[outPos.Length];
        Array.Fill(remap, -1);
        var positions = new List<Vector3>();
        var compactFaces = new List<int[]>(faces.Count);
        foreach (int[] face in faces)
        {
            var mapped = new int[face.Length];
            for (int i = 0; i < face.Length; i++)
            {
                int v = face[i];
                if (remap[v] < 0)
                {
                    Vector3 p = outPos[v];
                    remap[v] = positions.Count;
                    positions.Add(surface is null ? p : surface.ClosestPoint(p));
                }

                mapped[i] = remap[v];
            }

            compactFaces.Add(mapped);
        }

        return new QuadMesh(positions.ToArray(), compactFaces);
    }

    private enum EdgeKind
    {
        Same,  // both endpoints fall in the same lattice cell (collapse)
        Unit,  // one lattice step apart (a quad edge)
        Other, // a diagonal or multi-cell span (ignored)
    }

    /// <summary>
    /// Classifies an edge symmetrically, from both endpoints' lattice frames. Requiring agreement
    /// avoids wrong collapses/edges where the RoSy orientation differs by 90° between neighbours.
    /// </summary>
    private static EdgeKind Classify(ExtractionInput input, int a, int b)
    {
        (int k1, int k2) = Offset(input, a, b);
        (int j1, int j2) = Offset(input, b, a);
        if (k1 == 0 && k2 == 0 && j1 == 0 && j2 == 0)
        {
            return EdgeKind.Same;
        }

        if (System.Math.Abs(k1) + System.Math.Abs(k2) == 1 && System.Math.Abs(j1) + System.Math.Abs(j2) == 1)
        {
            return EdgeKind.Unit;
        }

        return EdgeKind.Other;
    }

    private static (int K1, int K2) Offset(ExtractionInput input, int a, int b)
    {
        Vector3 e1 = input.Orientation[a];
        Vector3 e2 = Vector3.Cross(input.Normal[a], e1);
        Vector3 d = input.Anchor[b] - input.Anchor[a];
        double inv = 1.0 / input.Scale[a];
        int k1 = (int)System.Math.Round(Vector3.Dot(d, e1) * inv);
        int k2 = (int)System.Math.Round(Vector3.Dot(d, e2) * inv);
        return (k1, k2);
    }
}

using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// Port of QuadriFlow's <c>Parametrizer::FixHoles</c> (parametrizer-flip.cpp:72–166) + the
/// <c>QuadEnergy</c> polygon quadrangulator (11–70). The integer-grid extractor leaves small boundary
/// loops wherever a residual parametrization foldover (a negative-area lattice cell FixFlip could not
/// clear — common where sharp-edge locks freeze the offsets near a crease) prevents a quad cell from
/// closing. This walks every boundary loop of the extracted quad mesh and, for loops short enough to be
/// such an artifact (&lt; 25 boundary edges), fills them with the minimal-corner-angle quadrangulation,
/// skipping any quad that would duplicate an existing directed edge. Closed inputs whose extraction is
/// already watertight (sphere/torus) have no boundary loops, so this is a no-op there.
/// Reimplemented clean-room from QuadriFlow (MIT, read for understanding only).
/// </summary>
internal static class QuadHoleFiller
{
    /// <summary>Boundary loops longer than this are treated as genuine open boundaries, not artifacts.</summary>
    private const int MaxLoopVertices = 25;

    private static Vector3[] _o = Array.Empty<Vector3>();
    private static Vector3[] _n = Array.Empty<Vector3>();

    // Diagnostics for the foldover/hole investigation.
    internal static List<int> LastLoopSizes { get; } = new();

    internal static int LastQuadsAdded { get; private set; }

    /// <summary>Fills small boundary-loop holes; returns the (possibly augmented) mesh.</summary>
    public static QuadMesh Fill(QuadMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);

        // The extractor emits pure quads; bail (unchanged) if anything else slipped through, since the
        // dedge bookkeeping below assumes 4 corners per face.
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            if (mesh.FaceSize(f) != 4)
            {
                return mesh;
            }
        }

        var faces = new List<int[]>(mesh.FaceCount);
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            ReadOnlySpan<int> fc = mesh.Face(f);
            faces.Add(new[] { fc[0], fc[1], fc[2], fc[3] });
        }

        _o = mesh.Positions;
        _n = ComputeVertexNormals(_o, faces);

        int[] e2e = BuildDedgeTwins(faces, _o.Length);

        // Quad_edges: every directed edge currently present. A candidate fill quad is rejected if it
        // would re-add one (that would fold the new quad back onto an existing face).
        var quadEdges = new HashSet<long>();
        foreach (int[] q in faces)
        {
            for (int j = 0; j < 4; j++)
            {
                quadEdges.Add(Key(q[j], q[(j + 1) % 4]));
            }
        }

        var detected = new bool[faces.Count * 4];
        int added = 0;
        LastLoopSizes.Clear();
        for (int i = 0; i < e2e.Length; i++)
        {
            if (detected[i] || e2e[i] != -1)
            {
                continue;
            }

            var loopEdges = new List<int>();
            int cur = i;
            while (!detected[cur])
            {
                detected[cur] = true;
                loopEdges.Add(cur);
                cur = NextInFace(cur);
                while (e2e[cur] != -1)
                {
                    cur = e2e[cur];
                    cur = NextInFace(cur);
                }
            }

            var loopVerts = new List<int>(loopEdges.Count);
            foreach (int le in loopEdges)
            {
                loopVerts.Add(faces[le / 4][le % 4]);
            }

            LastLoopSizes.Add(loopVerts.Count);
            if (loopVerts.Count < MaxLoopVertices)
            {
                added += FillLoop(loopVerts, quadEdges, faces);
            }
        }

        LastQuadsAdded = added;
        if (added == 0)
        {
            return mesh;
        }

        return new QuadMesh(_o, faces);
    }

    /// <summary>Port of <c>FixHoles(loop_vertices)</c>: split a boundary walk into even sub-loops, then fill each.</summary>
    private static int FillLoop(List<int> loopVertices, HashSet<long> quadEdges, List<int[]> faces)
    {
        // Split the walk at repeated vertices into simple even-length sub-loops (a figure-eight boundary
        // shares a pinch vertex; each lobe is filled separately).
        var loops = new List<List<int>>();
        var mapLoops = new Dictionary<int, int>();
        for (int i = 0; i < loopVertices.Count; i++)
        {
            int v = loopVertices[i];
            if (mapLoops.TryGetValue(v, out int j))
            {
                var sub = new List<int>();
                if (i - j > 3 && (i - j) % 2 == 0)
                {
                    for (int k = j; k < i; k++)
                    {
                        if (mapLoops.ContainsKey(loopVertices[k]))
                        {
                            sub.Add(loopVertices[k]);
                            mapLoops.Remove(loopVertices[k]);
                        }
                    }
                }

                loops.Add(sub);
            }

            mapLoops[v] = i;
        }

        if (mapLoops.Count >= 3)
        {
            var sub = new List<int>();
            for (int k = 0; k < loopVertices.Count; k++)
            {
                if (mapLoops.ContainsKey(loopVertices[k]))
                {
                    sub.Add(loopVertices[k]);
                    mapLoops.Remove(loopVertices[k]);
                }
            }

            loops.Add(sub);
        }

        int added = 0;
        foreach (List<int> loop in loops)
        {
            if (loop.Count == 0)
            {
                continue;
            }

            var quads = new List<int[]>();
            QuadEnergy(loop, quads);
            foreach (int[] p in quads)
            {
                bool clash = false;
                for (int j = 0; j < 4; j++)
                {
                    if (quadEdges.Contains(Key(p[j], p[(j + 1) % 4])))
                    {
                        clash = true;
                        break;
                    }
                }

                if (!clash)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        quadEdges.Add(Key(p[j], p[(j + 1) % 4]));
                    }

                    faces.Add(p);
                    added++;
                }
            }
        }

        return added;
    }

    /// <summary>
    /// Port of <c>QuadEnergy</c>: recursively quadrangulate the polygon <paramref name="loopVertices"/>
    /// minimising the sum of squared corner-angle deviations (a 90°-corner preference), appending the
    /// chosen quads to <paramref name="resQuads"/> and returning the energy.
    /// </summary>
    private static double QuadEnergy(List<int> loopVertices, List<int[]> resQuads)
    {
        int n = loopVertices.Count;
        if (n < 4)
        {
            return 0;
        }

        if (n == 4)
        {
            double energy = 0;
            for (int j = 0; j < 4; j++)
            {
                int v0 = loopVertices[j];
                int v2 = loopVertices[(j + 1) % 4];
                int v1 = loopVertices[(j + 3) % 4];

                // Coincident loop vertices (degenerate corner) → treat as a zero direction, mirroring
                // Eigen's non-throwing normalized(); the corner then contributes ~0 to the energy.
                _ = (_o[v1] - _o[v0]).TryNormalize(out Vector3 pt1);
                _ = (_o[v2] - _o[v0]).TryNormalize(out Vector3 pt2);
                Vector3 cross = Vector3.Cross(pt1, pt2);
                double sina = cross.Length;
                if (Vector3.Dot(cross, _n[v0]) < 0)
                {
                    sina = -sina;
                }

                double cosa = Vector3.Dot(pt1, pt2);
                double angle = System.Math.Atan2(sina, cosa) / System.Math.PI * 180.0;
                if (angle < 0)
                {
                    angle = 360 + angle;
                }

                energy += angle * angle;
            }

            resQuads.Add(new[] { loopVertices[0], loopVertices[3], loopVertices[2], loopVertices[1] });
            return energy;
        }

        double maxEnergy = 1e30;
        for (int seg1 = 2; seg1 < n; seg1 += 2)
        {
            for (int seg2 = seg1 + 1; seg2 < n; seg2 += 2)
            {
                var quads = new List<int[]>[4];
                for (int i = 0; i < 4; i++)
                {
                    quads[i] = new List<int[]>();
                }

                var center = new List<int> { loopVertices[0], loopVertices[1], loopVertices[seg1], loopVertices[seg2] };
                double energy = QuadEnergy(center, quads[0]);

                if (seg1 > 2)
                {
                    var v = loopVertices.GetRange(1, seg1 - 1);
                    v.Add(loopVertices[seg1]);
                    energy += QuadEnergy(v, quads[1]);
                }

                if (seg2 != seg1 + 1)
                {
                    var v = loopVertices.GetRange(seg1, seg2 - seg1);
                    v.Add(loopVertices[seg2]);
                    energy += QuadEnergy(v, quads[2]);
                }

                if (seg2 + 1 != n)
                {
                    var v = loopVertices.GetRange(seg2, n - seg2);
                    v.Add(loopVertices[0]);
                    energy += QuadEnergy(v, quads[3]);
                }

                if (maxEnergy > energy)
                {
                    maxEnergy = energy;
                    resQuads.Clear();
                    for (int i = 0; i < 4; i++)
                    {
                        resQuads.AddRange(quads[i]);
                    }
                }
            }
        }

        return maxEnergy;
    }

    private static int NextInFace(int e) => (e / 4 * 4) + ((e + 1) % 4);

    private static long Key(int a, int b) => ((long)a << 32) | (uint)b;

    /// <summary>Directed-dedge twin map (face*4+corner → opposite dedge, −1 on a boundary).</summary>
    private static int[] BuildDedgeTwins(List<int[]> faces, int vertexCount)
    {
        var dedgeOf = new Dictionary<long, int>(faces.Count * 4);
        for (int f = 0; f < faces.Count; f++)
        {
            int[] q = faces[f];
            for (int j = 0; j < 4; j++)
            {
                dedgeOf[Key(q[j], q[(j + 1) % 4])] = (f * 4) + j;
            }
        }

        var e2e = new int[faces.Count * 4];
        for (int f = 0; f < faces.Count; f++)
        {
            int[] q = faces[f];
            for (int j = 0; j < 4; j++)
            {
                e2e[(f * 4) + j] = dedgeOf.TryGetValue(Key(q[(j + 1) % 4], q[j]), out int twin) ? twin : -1;
            }
        }

        return e2e;
    }

    private static Vector3[] ComputeVertexNormals(Vector3[] pos, List<int[]> faces)
    {
        var normals = new Vector3[pos.Length];
        foreach (int[] q in faces)
        {
            // Newell's method over the quad gives a robust normal even for non-planar cells.
            var nrm = Vector3.Zero;
            for (int j = 0; j < 4; j++)
            {
                Vector3 a = pos[q[j]];
                Vector3 b = pos[q[(j + 1) % 4]];
                nrm += new Vector3(
                    (a.Y - b.Y) * (a.Z + b.Z),
                    (a.Z - b.Z) * (a.X + b.X),
                    (a.X - b.X) * (a.Y + b.Y));
            }

            for (int j = 0; j < 4; j++)
            {
                normals[q[j]] += nrm;
            }
        }

        for (int v = 0; v < normals.Length; v++)
        {
            double len = normals[v].Length;
            normals[v] = len > 1e-20 ? normals[v] / len : new Vector3(0, 0, 1);
        }

        return normals;
    }
}

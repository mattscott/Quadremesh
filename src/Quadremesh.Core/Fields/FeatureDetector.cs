using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Fields;

/// <summary>Per-mesh sharp-feature classification: which edges are creases, and the tangent to pin the field to.</summary>
public sealed class FeatureResult
{
    /// <summary>Whether each undirected edge is a sharp feature (dihedral above threshold, or a boundary).</summary>
    public required bool[] SharpEdge { get; init; }

    /// <summary>Whether each vertex lies on a feature (touches at least one sharp edge).</summary>
    public required bool[] OnFeature { get; init; }

    /// <summary>
    /// Whether each vertex is a feature corner/junction: it touches a number of sharp edges other than
    /// two, so there is no single crease tangent (a cube corner, a feature endpoint, a Y-junction).
    /// </summary>
    public required bool[] Corner { get; init; }

    /// <summary>
    /// Unit crease tangent per vertex, valid where <see cref="OnFeature"/> is set and <see cref="Corner"/>
    /// is not: the direction the field should align one axis to. Zero elsewhere.
    /// </summary>
    public required Vector3[] CreaseTangent { get; init; }

    /// <summary>Number of sharp edges.</summary>
    public int SharpEdgeCount { get; init; }

    /// <summary>Number of feature vertices.</summary>
    public int FeatureVertexCount { get; init; }
}

/// <summary>
/// Detects sharp features (creases) on a mesh by dihedral angle, plus open boundaries. These become
/// orientation-field constraints (the field aligns one cross axis to the crease) and, later, extraction
/// seams — so the output's quad edge loops follow the model's sharp edges (Instant Meshes / QuadriFlow
/// feature handling). Reimplemented clean-room.
/// </summary>
public static class FeatureDetector
{
    /// <summary>Default crease threshold: faces meeting at more than this normal angle form a sharp edge.</summary>
    public const double DefaultAngleDegrees = 30.0;

    /// <summary>Classifies the sharp features of <paramref name="mesh"/>.</summary>
    public static FeatureResult Detect(HalfEdgeMesh mesh, double angleDegrees = DefaultAngleDegrees)
    {
        ArgumentNullException.ThrowIfNull(mesh);

        Vector3[] faceNormal = ComputeFaceNormals(mesh);
        double cosThreshold = System.Math.Cos(angleDegrees * System.Math.PI / 180.0);

        var sharpEdge = new bool[mesh.EdgeCount];
        int sharpCount = 0;
        for (int e = 0; e < mesh.EdgeCount; e++)
        {
            int h = mesh.EdgeHalfEdge(e);
            int twin = mesh.Twin(h);
            bool sharp;
            if (twin < 0)
            {
                sharp = true; // open boundary is a feature
            }
            else
            {
                Vector3 n0 = faceNormal[mesh.Face(h)];
                Vector3 n1 = faceNormal[mesh.Face(twin)];
                sharp = Vector3.Dot(n0, n1) < cosThreshold;
            }

            sharpEdge[e] = sharp;
            if (sharp)
            {
                sharpCount++;
            }
        }

        // Per vertex: gather all sharp-edge directions. A vertex is a clean crease if some pair of those
        // directions runs nearly straight through it (a through-line); otherwise it is a corner/junction.
        // Requiring exactly two would mis-classify the jagged crease bands a remesher leaves behind.
        var sharpDirs = new List<Vector3>[mesh.VertexCount];
        for (int e = 0; e < mesh.EdgeCount; e++)
        {
            if (!sharpEdge[e])
            {
                continue;
            }

            (int v0, int v1) = mesh.Edge(e);
            Accumulate(v0, v1);
            Accumulate(v1, v0);

            void Accumulate(int v, int other)
            {
                if ((mesh.Positions[other] - mesh.Positions[v]).TryNormalize(out Vector3 dir))
                {
                    (sharpDirs[v] ??= new List<Vector3>()).Add(dir);
                }
            }
        }

        var onFeature = new bool[mesh.VertexCount];
        var corner = new bool[mesh.VertexCount];
        var creaseTangent = new Vector3[mesh.VertexCount];
        int featureVerts = 0;
        const double straightCos = -0.5; // the through-pair must span more than 120°
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            List<Vector3>? dirs = sharpDirs[v];
            if (dirs is null || dirs.Count == 0)
            {
                continue;
            }

            onFeature[v] = true;
            featureVerts++;

            // Find the most-opposite pair of sharp directions: that is the crease running through v.
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

            if (bi >= 0 && bestDot < straightCos && (dirs[bi] - dirs[bj]).TryNormalize(out Vector3 unit))
            {
                creaseTangent[v] = unit;
            }
            else
            {
                corner[v] = true; // a single dangling edge or a genuine junction — no through-tangent
            }
        }

        return new FeatureResult
        {
            SharpEdge = sharpEdge,
            OnFeature = onFeature,
            Corner = corner,
            CreaseTangent = creaseTangent,
            SharpEdgeCount = sharpCount,
            FeatureVertexCount = featureVerts,
        };
    }

    private static Vector3[] ComputeFaceNormals(HalfEdgeMesh mesh)
    {
        var normals = new Vector3[mesh.FaceCount];
        Span<int> corners = stackalloc int[8];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            int size = mesh.FaceSize(f);
            Span<int> buf = size <= corners.Length ? corners : new int[size];
            int k = mesh.GetFaceVertices(f, buf);

            // Newell's method: robust for non-planar polygons and degenerate triangles.
            var n = new Vector3(0, 0, 0);
            for (int i = 0; i < k; i++)
            {
                Vector3 cur = mesh.Positions[buf[i]];
                Vector3 next = mesh.Positions[buf[(i + 1) % k]];
                n += new Vector3(
                    (cur.Y - next.Y) * (cur.Z + next.Z),
                    (cur.Z - next.Z) * (cur.X + next.X),
                    (cur.X - next.X) * (cur.Y + next.Y));
            }

            normals[f] = n.TryNormalize(out Vector3 unit) ? unit : Vector3.UnitZ;
        }

        return normals;
    }
}

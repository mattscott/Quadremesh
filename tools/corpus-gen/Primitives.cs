using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using static System.Math;

namespace Quadremesh.CorpusGen;

/// <summary>
/// Deterministic procedural meshes for the test corpus. Used both to emit committed
/// <c>tests/corpus/*.obj</c> files and directly (in-memory) by unit tests. No randomness — every
/// generator is a pure function of its parameters.
/// </summary>
public static class Primitives
{
    /// <summary>Unit icosphere: an icosahedron subdivided <paramref name="subdivisions"/> times, projected to the sphere. Closed, manifold.</summary>
    public static TriangleMesh Icosphere(int subdivisions = 2, double radius = 1.0)
    {
        double t = (1.0 + Sqrt(5.0)) / 2.0;
        var verts = new List<Vector3>
        {
            new(-1, t, 0), new(1, t, 0), new(-1, -t, 0), new(1, -t, 0),
            new(0, -1, t), new(0, 1, t), new(0, -1, -t), new(0, 1, -t),
            new(t, 0, -1), new(t, 0, 1), new(-t, 0, -1), new(-t, 0, 1),
        };

        int[][] faces =
        {
            new[] { 0, 11, 5 }, new[] { 0, 5, 1 }, new[] { 0, 1, 7 }, new[] { 0, 7, 10 }, new[] { 0, 10, 11 },
            new[] { 1, 5, 9 }, new[] { 5, 11, 4 }, new[] { 11, 10, 2 }, new[] { 10, 7, 6 }, new[] { 7, 1, 8 },
            new[] { 3, 9, 4 }, new[] { 3, 4, 2 }, new[] { 3, 2, 6 }, new[] { 3, 6, 8 }, new[] { 3, 8, 9 },
            new[] { 4, 9, 5 }, new[] { 2, 4, 11 }, new[] { 6, 2, 10 }, new[] { 8, 6, 7 }, new[] { 9, 8, 1 },
        };

        var tris = new List<int[]>(faces);
        for (int s = 0; s < subdivisions; s++)
        {
            var midpoint = new Dictionary<long, int>();
            var next = new List<int[]>(tris.Count * 4);
            foreach (int[] f in tris)
            {
                int a = f[0], b = f[1], c = f[2];
                int ab = Midpoint(verts, midpoint, a, b);
                int bc = Midpoint(verts, midpoint, b, c);
                int ca = Midpoint(verts, midpoint, c, a);
                next.Add(new[] { a, ab, ca });
                next.Add(new[] { b, bc, ab });
                next.Add(new[] { c, ca, bc });
                next.Add(new[] { ab, bc, ca });
            }

            tris = next;
        }

        var positions = new Vector3[verts.Count];
        for (int i = 0; i < verts.Count; i++)
        {
            positions[i] = verts[i].Normalized() * radius;
        }

        return new TriangleMesh(positions, Flatten(tris));
    }

    /// <summary>Closed torus with the given major/minor radii and tessellation. Genus-1, manifold.</summary>
    public static TriangleMesh Torus(double major = 1.0, double minor = 0.35, int majorSegments = 48, int minorSegments = 24)
    {
        var positions = new Vector3[majorSegments * minorSegments];
        for (int i = 0; i < majorSegments; i++)
        {
            double u = 2.0 * PI * i / majorSegments;
            for (int j = 0; j < minorSegments; j++)
            {
                double v = 2.0 * PI * j / minorSegments;
                double rr = major + (minor * Cos(v));
                positions[(i * minorSegments) + j] = new Vector3(rr * Cos(u), rr * Sin(u), minor * Sin(v));
            }
        }

        var faces = new List<int[]>(majorSegments * minorSegments * 2);
        for (int i = 0; i < majorSegments; i++)
        {
            int ni = (i + 1) % majorSegments;
            for (int j = 0; j < minorSegments; j++)
            {
                int nj = (j + 1) % minorSegments;
                int a = (i * minorSegments) + j;
                int b = (ni * minorSegments) + j;
                int c = (ni * minorSegments) + nj;
                int d = (i * minorSegments) + nj;
                faces.Add(new[] { a, b, c });
                faces.Add(new[] { a, c, d });
            }
        }

        return new TriangleMesh(positions, Flatten(faces));
    }

    /// <summary>Open cylinder (a tube with no end caps), so it has two boundary loops. Tests boundary handling.</summary>
    public static TriangleMesh Cylinder(double radius = 0.5, double height = 2.0, int radialSegments = 32, int heightSegments = 8)
    {
        int rings = heightSegments + 1;
        var positions = new Vector3[radialSegments * rings];
        for (int i = 0; i < radialSegments; i++)
        {
            double theta = 2.0 * PI * i / radialSegments;
            for (int j = 0; j < rings; j++)
            {
                double z = (-height / 2.0) + (height * j / heightSegments);
                positions[(i * rings) + j] = new Vector3(radius * Cos(theta), radius * Sin(theta), z);
            }
        }

        var faces = new List<int[]>(radialSegments * heightSegments * 2);
        for (int i = 0; i < radialSegments; i++)
        {
            int ni = (i + 1) % radialSegments;
            for (int j = 0; j < heightSegments; j++)
            {
                int a = (i * rings) + j;
                int b = (ni * rings) + j;
                int c = (ni * rings) + j + 1;
                int d = (i * rings) + j + 1;
                faces.Add(new[] { a, b, c });
                faces.Add(new[] { a, c, d });
            }
        }

        return new TriangleMesh(positions, Flatten(faces));
    }

    /// <summary>Axis-aligned cube with eight shared corners (closed manifold). Hard-surface stand-in (sharp 90° creases).</summary>
    public static TriangleMesh Box(double size = 1.0)
    {
        double h = size / 2.0;
        var positions = new Vector3[]
        {
            new(-h, -h, -h), new(h, -h, -h), new(h, h, -h), new(-h, h, -h),
            new(-h, -h, h), new(h, -h, h), new(h, h, h), new(-h, h, h),
        };

        // Outward-facing triangles (CCW seen from outside).
        int[][] quads =
        {
            new[] { 0, 3, 2, 1 }, // -Z
            new[] { 4, 5, 6, 7 }, // +Z
            new[] { 0, 1, 5, 4 }, // -Y
            new[] { 2, 3, 7, 6 }, // +Y
            new[] { 0, 4, 7, 3 }, // -X
            new[] { 1, 2, 6, 5 }, // +X
        };

        var faces = new List<int[]>(12);
        foreach (int[] q in quads)
        {
            faces.Add(new[] { q[0], q[1], q[2] });
            faces.Add(new[] { q[0], q[2], q[3] });
        }

        return new TriangleMesh(positions, Flatten(faces));
    }

    /// <summary>Organic "blob": an icosphere displaced by smooth sinusoids. Stand-in for a scanned head until a real asset is added.</summary>
    public static TriangleMesh Blob(int subdivisions = 3)
    {
        TriangleMesh sphere = Icosphere(subdivisions);
        var positions = new Vector3[sphere.VertexCount];
        for (int i = 0; i < positions.Length; i++)
        {
            Vector3 p = sphere.Positions[i];
            double displacement = 1.0
                + (0.25 * Sin(3.0 * p.X) * Cos(2.0 * p.Y))
                + (0.15 * Sin(4.0 * p.Z));
            positions[i] = p * displacement;
        }

        return new TriangleMesh(positions, sphere.Indices);
    }

    /// <summary>A deliberately non-manifold mesh: an edge shared by three triangles plus a bow-tie vertex. Must load without throwing.</summary>
    public static TriangleMesh NonManifold()
    {
        var positions = new Vector3[]
        {
            new(0, 0, 0),   // 0  shared edge endpoint
            new(1, 0, 0),   // 1  shared edge endpoint
            new(0.5, 1, 0), // 2  fan blade
            new(0.5, -1, 0), // 3 fan blade
            new(0.5, 0, 1), // 4  fan blade
            new(3, 0, 0),   // 5  bow-tie centre
            new(4, 1, 0),   // 6
            new(4, -1, 0),  // 7
            new(2, 1, 0),   // 8
            new(2, -1, 0),  // 9
        };

        int[] indices =
        {
            0, 1, 2,   // three faces all sharing undirected edge (0,1)
            0, 1, 3,
            0, 1, 4,
            5, 6, 7,   // bow-tie: two triangles meeting only at vertex 5
            5, 8, 9,
        };

        return new TriangleMesh(positions, indices);
    }

    /// <summary>All corpus meshes keyed by file stem.</summary>
    public static IReadOnlyDictionary<string, TriangleMesh> All() => new Dictionary<string, TriangleMesh>
    {
        ["sphere"] = Icosphere(),
        ["torus"] = Torus(),
        ["cylinder"] = Cylinder(),
        ["hard-surface"] = Box(),
        ["head"] = Blob(),
        ["non-manifold"] = NonManifold(),
    };

    private static int Midpoint(List<Vector3> verts, Dictionary<long, int> cache, int a, int b)
    {
        long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
        if (cache.TryGetValue(key, out int existing))
        {
            return existing;
        }

        int index = verts.Count;
        verts.Add((verts[a] + verts[b]) * 0.5);
        cache[key] = index;
        return index;
    }

    private static int[] Flatten(List<int[]> faces)
    {
        var indices = new int[faces.Count * 3];
        for (int f = 0; f < faces.Count; f++)
        {
            indices[(f * 3) + 0] = faces[f][0];
            indices[(f * 3) + 1] = faces[f][1];
            indices[(f * 3) + 2] = faces[f][2];
        }

        return indices;
    }
}

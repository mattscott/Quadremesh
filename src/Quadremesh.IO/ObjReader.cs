using System.Globalization;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.IO;

/// <summary>
/// A tolerant Wavefront OBJ reader. Parses <c>v</c> and <c>f</c> records (tris, quads, and n-gons),
/// resolves 1-based and negative (relative) indices, and ignores everything else (normals, UVs,
/// groups, materials). Malformed or out-of-range faces are skipped rather than throwing, so dirty
/// input loads best-effort (P1 non-manifold/defective acceptance).
/// </summary>
public static class ObjReader
{
    /// <summary>Reads an OBJ file as a general polygon mesh, preserving original face sizes.</summary>
    public static QuadMesh ReadPolygons(string path)
    {
        using var reader = new StreamReader(path);
        return ReadPolygons(reader);
    }

    /// <summary>Reads OBJ text from a reader as a general polygon mesh.</summary>
    public static QuadMesh ReadPolygons(TextReader reader)
    {
        (List<Vector3> positions, List<int[]> faces) = Parse(reader);
        return new QuadMesh(positions.ToArray(), faces);
    }

    /// <summary>
    /// Reads an OBJ file as a triangle mesh, fan-triangulating any polygon with more than three sides.
    /// </summary>
    public static TriangleMesh ReadTriangles(string path)
    {
        using var reader = new StreamReader(path);
        return ReadTriangles(reader);
    }

    /// <summary>Reads OBJ text from a reader as a triangle mesh (fan-triangulating polygons).</summary>
    public static TriangleMesh ReadTriangles(TextReader reader)
    {
        (List<Vector3> positions, List<int[]> faces) = Parse(reader);
        var indices = new List<int>(faces.Count * 3);
        foreach (int[] face in faces)
        {
            // Fan triangulation: (0,1,2), (0,2,3), ... Valid for convex faces, adequate for input prep.
            for (int k = 2; k < face.Length; k++)
            {
                indices.Add(face[0]);
                indices.Add(face[k - 1]);
                indices.Add(face[k]);
            }
        }

        return new TriangleMesh(positions.ToArray(), indices.ToArray());
    }

    private static (List<Vector3> Positions, List<int[]> Faces) Parse(TextReader reader)
    {
        var positions = new List<Vector3>();
        var faces = new List<int[]>();
        var faceScratch = new List<int>(8);

        string? line;
        while ((line = reader.ReadLine()) is not null)
        {
            ReadOnlySpan<char> span = line.AsSpan().Trim();
            if (span.IsEmpty || span[0] == '#')
            {
                continue;
            }

            if (span.StartsWith("v ") || span.StartsWith("v\t"))
            {
                if (TryParseVertex(span[1..], out Vector3 v))
                {
                    positions.Add(v);
                }
            }
            else if (span.StartsWith("f ") || span.StartsWith("f\t"))
            {
                ParseFace(span[1..], positions.Count, faceScratch);
                if (faceScratch.Count >= 3)
                {
                    faces.Add(faceScratch.ToArray());
                }
            }

            // All other records (vn, vt, g, o, s, mtllib, usemtl, ...) are intentionally ignored.
        }

        return (positions, faces);
    }

    private static bool TryParseVertex(ReadOnlySpan<char> span, out Vector3 v)
    {
        v = Vector3.Zero;
        Span<double> xyz = stackalloc double[3];
        int count = 0;
        foreach (Range r in Tokenize(span))
        {
            if (count == 3)
            {
                break; // ignore optional w / vertex colors
            }

            if (!double.TryParse(span[r], NumberStyles.Float, CultureInfo.InvariantCulture, out xyz[count]))
            {
                return false;
            }

            count++;
        }

        if (count < 3)
        {
            return false;
        }

        v = new Vector3(xyz[0], xyz[1], xyz[2]);
        return true;
    }

    private static void ParseFace(ReadOnlySpan<char> span, int vertexCount, List<int> output)
    {
        output.Clear();
        foreach (Range r in Tokenize(span))
        {
            ReadOnlySpan<char> token = span[r];
            int slash = token.IndexOf('/');
            ReadOnlySpan<char> vertexPart = slash >= 0 ? token[..slash] : token;
            if (!int.TryParse(vertexPart, NumberStyles.Integer, CultureInfo.InvariantCulture, out int raw) || raw == 0)
            {
                output.Clear(); // malformed corner — drop the whole face
                return;
            }

            // OBJ indices are 1-based; negative indices count back from the current vertex count.
            int resolved = raw > 0 ? raw - 1 : vertexCount + raw;
            if ((uint)resolved >= (uint)vertexCount)
            {
                output.Clear(); // out-of-range corner — drop the whole face (defect, not fatal)
                return;
            }

            output.Add(resolved);
        }
    }

    /// <summary>Enumerates whitespace-delimited token ranges within <paramref name="span"/>.</summary>
    private static List<Range> Tokenize(ReadOnlySpan<char> span)
    {
        var ranges = new List<Range>();
        int i = 0;
        while (i < span.Length)
        {
            while (i < span.Length && char.IsWhiteSpace(span[i]))
            {
                i++;
            }

            if (i >= span.Length)
            {
                break;
            }

            int start = i;
            while (i < span.Length && !char.IsWhiteSpace(span[i]))
            {
                i++;
            }

            ranges.Add(new Range(start, i));
        }

        return ranges;
    }
}

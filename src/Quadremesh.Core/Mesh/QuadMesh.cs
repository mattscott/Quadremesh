using Quadremesh.Core.Math;

namespace Quadremesh.Core.Mesh;

/// <summary>
/// A polygon mesh stored in compressed (CSR) form: one index run per face. Faces are usually quads,
/// but tris and the occasional pentagon are permitted so the same type can hold both quad-dominant
/// (P5) and pure-quad (P6) output. This is the library's output POCO.
/// </summary>
public sealed class QuadMesh
{
    private readonly int[] _faceOffsets;   // length FaceCount + 1; face f spans [off[f], off[f+1])
    private readonly int[] _faceVertices;  // flattened corner indices

    /// <summary>Vertex positions, indexed by vertex id.</summary>
    public Vector3[] Positions { get; }

    /// <summary>
    /// Constructs a polygon mesh from a list of faces (each an array of vertex indices).
    /// </summary>
    public QuadMesh(Vector3[] positions, IReadOnlyList<int[]> faces)
    {
        ArgumentNullException.ThrowIfNull(positions);
        ArgumentNullException.ThrowIfNull(faces);

        _faceOffsets = new int[faces.Count + 1];
        int total = 0;
        for (int f = 0; f < faces.Count; f++)
        {
            total += faces[f].Length;
        }

        _faceVertices = new int[total];
        int cursor = 0;
        for (int f = 0; f < faces.Count; f++)
        {
            int[] face = faces[f];
            if (face.Length < 3)
            {
                throw new ArgumentException($"Face {f} has fewer than three vertices.", nameof(faces));
            }

            foreach (int vi in face)
            {
                if ((uint)vi >= (uint)positions.Length)
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(faces), vi, "Face references a vertex index out of range.");
                }

                _faceVertices[cursor++] = vi;
            }

            _faceOffsets[f + 1] = cursor;
        }

        Positions = positions;
    }

    /// <summary>Internal CSR constructor (offsets/vertices already validated by the caller).</summary>
    internal QuadMesh(Vector3[] positions, int[] faceOffsets, int[] faceVertices)
    {
        Positions = positions;
        _faceOffsets = faceOffsets;
        _faceVertices = faceVertices;
    }

    /// <summary>Number of vertices.</summary>
    public int VertexCount => Positions.Length;

    /// <summary>Number of faces.</summary>
    public int FaceCount => _faceOffsets.Length - 1;

    /// <summary>Number of corners (sides) of face <paramref name="face"/>.</summary>
    public int FaceSize(int face) => _faceOffsets[face + 1] - _faceOffsets[face];

    /// <summary>The vertex indices of face <paramref name="face"/>, in winding order.</summary>
    public ReadOnlySpan<int> Face(int face) =>
        _faceVertices.AsSpan(_faceOffsets[face], FaceSize(face));

    /// <summary>Fraction of faces that are quads, in [0, 1]. Used by P5/P6 acceptance tests.</summary>
    public double QuadFraction
    {
        get
        {
            if (FaceCount == 0)
            {
                return 0.0;
            }

            int quads = 0;
            for (int f = 0; f < FaceCount; f++)
            {
                if (FaceSize(f) == 4)
                {
                    quads++;
                }
            }

            return (double)quads / FaceCount;
        }
    }
}

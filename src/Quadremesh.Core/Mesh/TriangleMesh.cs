using Quadremesh.Core.Math;

namespace Quadremesh.Core.Mesh;

/// <summary>
/// A plain triangle mesh: vertex positions plus a flat index buffer (three indices per triangle).
/// This is the library's input POCO — readers produce it, the pipeline consumes it.
/// </summary>
public sealed class TriangleMesh
{
    /// <summary>Vertex positions, indexed by vertex id.</summary>
    public Vector3[] Positions { get; }

    /// <summary>Triangle corner indices into <see cref="Positions"/>; length is a multiple of three.</summary>
    public int[] Indices { get; }

    /// <summary>
    /// Constructs a triangle mesh, validating that the index buffer is well-formed.
    /// </summary>
    /// <exception cref="ArgumentException">If <paramref name="indices"/> length is not a multiple of three.</exception>
    /// <exception cref="ArgumentOutOfRangeException">If any index is outside <paramref name="positions"/>.</exception>
    public TriangleMesh(Vector3[] positions, int[] indices)
    {
        ArgumentNullException.ThrowIfNull(positions);
        ArgumentNullException.ThrowIfNull(indices);
        if (indices.Length % 3 != 0)
        {
            throw new ArgumentException("Triangle index buffer length must be a multiple of three.", nameof(indices));
        }

        for (int i = 0; i < indices.Length; i++)
        {
            if ((uint)indices[i] >= (uint)positions.Length)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(indices), indices[i], "Triangle index is out of range of the vertex array.");
            }
        }

        Positions = positions;
        Indices = indices;
    }

    /// <summary>Number of vertices.</summary>
    public int VertexCount => Positions.Length;

    /// <summary>Number of triangles.</summary>
    public int TriangleCount => Indices.Length / 3;

    /// <summary>Returns the three vertex indices of triangle <paramref name="tri"/>.</summary>
    public (int A, int B, int C) Triangle(int tri)
    {
        int o = tri * 3;
        return (Indices[o], Indices[o + 1], Indices[o + 2]);
    }
}

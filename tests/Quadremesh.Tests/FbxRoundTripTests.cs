using Quadremesh.CorpusGen;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;
using Xunit;

namespace Quadremesh.Tests;

/// <summary>
/// Binary FBX writer -> reader round trip. The reader fan-triangulates polygons and stores positions as
/// float, so topology is checked by counts and positions within single-precision tolerance.
/// </summary>
public class FbxRoundTripTests
{
    private const double Tolerance = 1e-5;

    [Fact]
    public void Quad_cube_round_trips_through_fbx_as_twelve_triangles()
    {
        var positions = new Vector3[]
        {
            new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0),
            new(0, 0, 1), new(1, 0, 1), new(1, 1, 1), new(0, 1, 1),
        };
        var faces = new List<int[]>
        {
            new[] { 0, 3, 2, 1 }, new[] { 4, 5, 6, 7 },
            new[] { 0, 1, 5, 4 }, new[] { 2, 3, 7, 6 },
            new[] { 0, 4, 7, 3 }, new[] { 1, 2, 6, 5 },
        };
        var original = new QuadMesh(positions, faces);

        TriangleMesh reloaded = WriteThenRead(original);

        Assert.Equal(8, reloaded.VertexCount);
        Assert.Equal(12, reloaded.TriangleCount); // each quad fans into two triangles
        for (int v = 0; v < positions.Length; v++)
        {
            Assert.True(Vector3.Distance(positions[v], reloaded.Positions[v]) < Tolerance, $"vertex {v} moved");
        }
    }

    [Fact]
    public void Triangle_mesh_keeps_vertices_and_indices_through_fbx()
    {
        TriangleMesh source = Primitives.Icosphere();
        var faces = new List<int[]>();
        for (int t = 0; t < source.TriangleCount; t++)
        {
            faces.Add(new[] { source.Indices[t * 3], source.Indices[(t * 3) + 1], source.Indices[(t * 3) + 2] });
        }

        TriangleMesh reloaded = WriteThenRead(new QuadMesh(source.Positions, faces));

        Assert.Equal(source.VertexCount, reloaded.VertexCount);
        Assert.Equal(source.Indices, reloaded.Indices); // triangles pass through the fan unchanged
        for (int v = 0; v < source.VertexCount; v++)
        {
            Assert.True(Vector3.Distance(source.Positions[v], reloaded.Positions[v]) < Tolerance, $"vertex {v} moved");
        }
    }

    [Fact]
    public void Reader_rejects_a_stream_that_is_not_binary_fbx()
    {
        using var stream = new MemoryStream("v 0 0 0\n"u8.ToArray());
        Assert.ThrowsAny<Exception>(() => FbxReader.ReadTriangles(stream));
    }

    private static TriangleMesh WriteThenRead(QuadMesh mesh)
    {
        using var buffer = new MemoryStream();
        FbxWriter.Write(buffer, mesh);
        buffer.Position = 0;
        return FbxReader.ReadTriangles(buffer);
    }
}

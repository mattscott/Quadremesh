using Quadremesh.CorpusGen;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;
using Xunit;

namespace Quadremesh.Tests;

public class ObjRoundTripTests
{
    [Fact]
    public void Triangle_mesh_survives_obj_round_trip_with_identical_topology()
    {
        TriangleMesh original = Primitives.Icosphere();

        var sw = new StringWriter();
        ObjWriter.Write(sw, original);
        TriangleMesh reloaded = ObjReader.ReadTriangles(new StringReader(sw.ToString()));

        Assert.Equal(original.VertexCount, reloaded.VertexCount);
        Assert.Equal(original.TriangleCount, reloaded.TriangleCount);
        Assert.Equal(original.Indices, reloaded.Indices); // topology preserved exactly

        for (int v = 0; v < original.VertexCount; v++)
        {
            Assert.Equal(original.Positions[v], reloaded.Positions[v]); // "R" formatting round-trips
        }
    }

    [Fact]
    public void Quad_mesh_preserves_face_sizes_through_obj()
    {
        // A cube as six quad faces — exercises n-gon (non-triangle) read/write.
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

        var sw = new StringWriter();
        ObjWriter.Write(sw, original);
        QuadMesh reloaded = ObjReader.ReadPolygons(new StringReader(sw.ToString()));

        Assert.Equal(original.FaceCount, reloaded.FaceCount);
        Assert.Equal(1.0, reloaded.QuadFraction); // every face still a quad
        for (int f = 0; f < original.FaceCount; f++)
        {
            Assert.Equal(original.Face(f).ToArray(), reloaded.Face(f).ToArray());
        }
    }

    [Fact]
    public void Obj_reader_resolves_negative_relative_indices()
    {
        const string obj = """
            v 0 0 0
            v 1 0 0
            v 0 1 0
            f -3 -2 -1
            """;

        TriangleMesh mesh = ObjReader.ReadTriangles(new StringReader(obj));

        Assert.Equal(1, mesh.TriangleCount);
        Assert.Equal((0, 1, 2), mesh.Triangle(0));
    }

    [Fact]
    public void Obj_reader_skips_malformed_and_out_of_range_faces()
    {
        const string obj = """
            # tolerant parsing: only the first face is valid
            v 0 0 0
            v 1 0 0
            v 0 1 0
            f 1 2 3
            f 1 2 99
            f 1 2
            f 1/2/3 2/3/4 3/4/5
            """;

        TriangleMesh mesh = ObjReader.ReadTriangles(new StringReader(obj));

        // Valid: "f 1 2 3" and the "v/vt/vn" face (resolves to 1 2 3). The 99 and 2-corner faces drop.
        Assert.Equal(2, mesh.TriangleCount);
    }

    [Fact]
    public void Non_manifold_obj_loads_and_builds_without_throwing()
    {
        var sw = new StringWriter();
        ObjWriter.Write(sw, Primitives.NonManifold());

        TriangleMesh reloaded = ObjReader.ReadTriangles(new StringReader(sw.ToString()));
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(reloaded);

        Assert.True(mesh.Defects.NonManifoldEdges > 0);
    }
}

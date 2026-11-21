using System.Numerics;
using Quadremesh.Core.Mesh;
using NVector3 = System.Numerics.Vector3;
using QVector3 = Quadremesh.Core.Math.Vector3;

namespace Quadremesh.IO;

/// <summary>
/// Reads a binary FBX into a <see cref="TriangleMesh"/> (positions + triangle indices). The parser is
/// clean-room (<see cref="FbxBinaryParser"/>); polygons are fan-triangulated, mesh-to-world transforms
/// are applied, and all mesh parts are merged into one mesh. Normals, UVs, and materials are ignored —
/// the remesher only needs geometry.
/// </summary>
public static class FbxReader
{
    /// <summary>Reads and merges all FBX mesh parts in a file into one triangle mesh.</summary>
    public static TriangleMesh ReadTriangles(string path)
    {
        ArgumentNullException.ThrowIfNull(path);
        using FileStream stream = File.OpenRead(path);
        return ReadTriangles(stream);
    }

    /// <summary>Reads and merges all FBX mesh parts in a stream into one triangle mesh.</summary>
    public static TriangleMesh ReadTriangles(Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        FbxNode root = FbxBinaryParser.Parse(stream);
        var scene = new FbxScene(root);

        var positions = new List<QVector3>();
        var indices = new List<int>();
        foreach (FbxNode objects in root.ChildrenNamed("Objects"))
        {
            foreach (FbxNode geo in objects.ChildrenNamed("Geometry"))
            {
                AppendGeometry(geo, scene.MeshToWorld(geo.AsLong(0)), positions, indices);
            }
        }

        if (positions.Count == 0)
        {
            throw new InvalidDataException("FBX has no mesh geometry.");
        }

        return new TriangleMesh(positions.ToArray(), indices.ToArray());
    }

    private static void AppendGeometry(FbxNode geo, Matrix4x4 meshToWorld, List<QVector3> positions, List<int> indices)
    {
        double[]? vertices = geo.Child("Vertices")?.AsDoubleArray(0);
        int[]? polygonVertexIndex = geo.Child("PolygonVertexIndex")?.AsIntArray(0);
        if (vertices is null || polygonVertexIndex is null)
        {
            return;
        }

        int baseIndex = positions.Count;
        int controlPointCount = vertices.Length / 3;
        for (int i = 0; i < controlPointCount; i++)
        {
            var raw = new NVector3((float)vertices[i * 3], (float)vertices[(i * 3) + 1], (float)vertices[(i * 3) + 2]);
            NVector3 world = NVector3.Transform(raw, meshToWorld);
            positions.Add(new QVector3(world.X, world.Y, world.Z));
        }

        // FBX packs polygons as a flat index list; the last index of each polygon is bitwise-NOT.
        int polygonStart = 0;
        for (int pv = 0; pv < polygonVertexIndex.Length; pv++)
        {
            if (polygonVertexIndex[pv] >= 0)
            {
                continue; // not the last vertex of this polygon yet
            }

            for (int t = polygonStart + 1; t + 1 <= pv; t++)
            {
                indices.Add(baseIndex + Decode(polygonVertexIndex[polygonStart]));
                indices.Add(baseIndex + Decode(polygonVertexIndex[t]));
                indices.Add(baseIndex + Decode(polygonVertexIndex[t + 1]));
            }

            polygonStart = pv + 1;
        }
    }

    private static int Decode(int rawIndex) => rawIndex < 0 ? ~rawIndex : rawIndex;
}

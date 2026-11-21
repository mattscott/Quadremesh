using System.Buffers.Binary;
using System.Text;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.IO;

/// <summary>
/// Writes a <see cref="QuadMesh"/> as a binary FBX file (version 7400, 32-bit offsets, uncompressed
/// arrays). Clean-room: it emits the documented Kaydara record format — a header, an Objects/Geometry
/// node (Vertices + PolygonVertexIndex with the bitwise-NOT polygon-end marker) plus a Model and an
/// "OO" Connection so DCCs attach the mesh, and the file footer. Round-trips through
/// <see cref="FbxReader"/>; n-gons are preserved (no triangulation on write).
/// </summary>
public static class FbxWriter
{
    private const uint Version = 7400;
    private const long GeometryId = 1_000_000L;
    private const long ModelId = 2_000_000L;

    /// <summary>Writes <paramref name="mesh"/> as a binary FBX file at <paramref name="path"/>.</summary>
    public static void Write(string path, QuadMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(path);
        ArgumentNullException.ThrowIfNull(mesh);
        using FileStream stream = File.Create(path);
        Write(stream, mesh);
    }

    /// <summary>Writes <paramref name="mesh"/> as binary FBX to <paramref name="stream"/>.</summary>
    public static void Write(Stream stream, QuadMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(mesh);
        FbxNode root = BuildTree(mesh);
        FbxBinaryWriter.WriteDocument(stream, root.Children, Version);
    }

    private static FbxNode BuildTree(QuadMesh mesh)
    {
        var verts = new double[mesh.VertexCount * 3];
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            Vector3 p = mesh.Positions[v];
            verts[v * 3] = p.X;
            verts[(v * 3) + 1] = p.Y;
            verts[(v * 3) + 2] = p.Z;
        }

        var pvi = new List<int>();
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            ReadOnlySpan<int> face = mesh.Face(f);
            for (int k = 0; k < face.Length; k++)
            {
                // Last index of each polygon is bitwise-NOT to mark the polygon end.
                pvi.Add(k == face.Length - 1 ? ~face[k] : face[k]);
            }
        }

        var root = new FbxNode("(root)");

        var header = new FbxNode("FBXHeaderExtension");
        header.Children.Add(Scalar("FBXHeaderVersion", 1003));
        header.Children.Add(Scalar("FBXVersion", (int)Version));
        header.Children.Add(Scalar("Creator", "Quadremesh"));
        root.Children.Add(header);

        var objects = new FbxNode("Objects");

        var geo = new FbxNode("Geometry");
        geo.Properties.Add(GeometryId);
        geo.Properties.Add("Quadremesh\0Geometry");
        geo.Properties.Add("Mesh");
        geo.Children.Add(DoubleArray("Vertices", verts));
        geo.Children.Add(IntArray("PolygonVertexIndex", pvi.ToArray()));
        geo.Children.Add(Scalar("GeometryVersion", 124));
        objects.Children.Add(geo);

        var model = new FbxNode("Model");
        model.Properties.Add(ModelId);
        model.Properties.Add("Quadremesh\0Model");
        model.Properties.Add("Mesh");
        model.Children.Add(Scalar("Version", 232));
        objects.Children.Add(model);

        root.Children.Add(objects);

        var connections = new FbxNode("Connections");
        connections.Children.Add(Connect(GeometryId, ModelId)); // geometry -> model
        connections.Children.Add(Connect(ModelId, 0L));         // model -> scene root
        root.Children.Add(connections);

        return root;
    }

    private static FbxNode Connect(long child, long parent)
    {
        var c = new FbxNode("C");
        c.Properties.Add("OO");
        c.Properties.Add(child);
        c.Properties.Add(parent);
        return c;
    }

    private static FbxNode Scalar(string name, int value)
    {
        var n = new FbxNode(name);
        n.Properties.Add(value);
        return n;
    }

    private static FbxNode Scalar(string name, string value)
    {
        var n = new FbxNode(name);
        n.Properties.Add(value);
        return n;
    }

    private static FbxNode DoubleArray(string name, double[] data)
    {
        var n = new FbxNode(name);
        n.Properties.Add(data);
        return n;
    }

    private static FbxNode IntArray(string name, int[] data)
    {
        var n = new FbxNode(name);
        n.Properties.Add(data);
        return n;
    }
}

/// <summary>
/// Serializes an <see cref="FbxNode"/> tree to the binary FBX record format. Buffers into a seekable
/// <see cref="MemoryStream"/> so node end-offsets and property-list lengths can be back-patched, then
/// copies the result to the target stream. Arrays are written uncompressed (encoding 0).
/// </summary>
internal static class FbxBinaryWriter
{
    private static ReadOnlySpan<byte> Magic => "Kaydara FBX Binary  "u8;

    // The 16-byte file-end magic written by mainstream FBX exporters (importers key on it; not validated by our reader).
    private static ReadOnlySpan<byte> EndMagic => new byte[]
    {
        0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5, 0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3, 0x75, 0x8f, 0x29, 0x0b,
    };

    public static void WriteDocument(Stream target, IReadOnlyList<FbxNode> topLevel, uint version)
    {
        using var buffer = new MemoryStream();

        buffer.Write(Magic);
        buffer.WriteByte(0x00);
        buffer.WriteByte(0x1a);
        buffer.WriteByte(0x00);
        WriteUInt32(buffer, version);

        foreach (FbxNode node in topLevel)
        {
            WriteNode(buffer, node);
        }

        WriteNullRecord(buffer);
        WriteFooter(buffer, version);

        buffer.Position = 0;
        buffer.CopyTo(target);
    }

    private static void WriteNode(MemoryStream s, FbxNode node)
    {
        long endOffsetPos = s.Position;
        WriteUInt32(s, 0);                              // EndOffset (back-patched)
        WriteUInt32(s, (uint)node.Properties.Count);
        long propLenPos = s.Position;
        WriteUInt32(s, 0);                              // PropertyListLength (back-patched)
        byte[] name = Encoding.ASCII.GetBytes(node.Name);
        s.WriteByte((byte)name.Length);
        s.Write(name);

        long propsStart = s.Position;
        foreach (object prop in node.Properties)
        {
            WriteProperty(s, prop);
        }

        Backpatch(s, propLenPos, (uint)(s.Position - propsStart));

        if (node.Children.Count > 0)
        {
            foreach (FbxNode child in node.Children)
            {
                WriteNode(s, child);
            }

            WriteNullRecord(s); // a nested list is terminated by a null record
        }

        Backpatch(s, endOffsetPos, (uint)s.Position);
    }

    private static void WriteProperty(MemoryStream s, object prop)
    {
        switch (prop)
        {
            case int i:
                s.WriteByte((byte)'I');
                WriteInt32(s, i);
                break;
            case long l:
                s.WriteByte((byte)'L');
                WriteInt64(s, l);
                break;
            case float f:
                s.WriteByte((byte)'F');
                WriteUInt32(s, BitConverter.SingleToUInt32Bits(f));
                break;
            case double d:
                s.WriteByte((byte)'D');
                WriteUInt64(s, BitConverter.DoubleToUInt64Bits(d));
                break;
            case bool b:
                s.WriteByte((byte)'C');
                s.WriteByte((byte)(b ? 1 : 0));
                break;
            case string str:
                s.WriteByte((byte)'S');
                byte[] bytes = Encoding.Latin1.GetBytes(str); // Latin1 preserves the \0\1 name separator
                WriteUInt32(s, (uint)bytes.Length);
                s.Write(bytes);
                break;
            case double[] da:
                WriteDoubleArray(s, da);
                break;
            case int[] ia:
                WriteIntArray(s, ia);
                break;
            default:
                throw new NotSupportedException($"cannot serialize FBX property of type {prop.GetType()}.");
        }
    }

    private static void WriteDoubleArray(MemoryStream s, double[] data)
    {
        s.WriteByte((byte)'d');
        WriteUInt32(s, (uint)data.Length);
        WriteUInt32(s, 0);                        // encoding: raw
        WriteUInt32(s, (uint)(data.Length * 8));  // payload length
        Span<byte> b = stackalloc byte[8];
        foreach (double v in data)
        {
            BinaryPrimitives.WriteDoubleLittleEndian(b, v);
            s.Write(b);
        }
    }

    private static void WriteIntArray(MemoryStream s, int[] data)
    {
        s.WriteByte((byte)'i');
        WriteUInt32(s, (uint)data.Length);
        WriteUInt32(s, 0);                        // encoding: raw
        WriteUInt32(s, (uint)(data.Length * 4));  // payload length
        Span<byte> b = stackalloc byte[4];
        foreach (int v in data)
        {
            BinaryPrimitives.WriteInt32LittleEndian(b, v);
            s.Write(b);
        }
    }

    private static void WriteNullRecord(MemoryStream s)
    {
        Span<byte> zero = stackalloc byte[13]; // 3 x uint32 + 1 byte, all zero (32-bit offsets)
        zero.Clear();
        s.Write(zero);
    }

    private static void WriteFooter(MemoryStream s, uint version)
    {
        // 16-byte footer id: importers do not validate it, so zeros are accepted.
        Span<byte> footerId = stackalloc byte[16];
        footerId.Clear();
        s.Write(footerId);

        while (s.Position % 16 != 0)
        {
            s.WriteByte(0);
        }

        WriteUInt32(s, version);
        Span<byte> zeros = stackalloc byte[120];
        zeros.Clear();
        s.Write(zeros);
        s.Write(EndMagic);
    }

    private static void Backpatch(MemoryStream s, long position, uint value)
    {
        long save = s.Position;
        s.Position = position;
        WriteUInt32(s, value);
        s.Position = save;
    }

    private static void WriteUInt32(MemoryStream s, uint value)
    {
        Span<byte> b = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32LittleEndian(b, value);
        s.Write(b);
    }

    private static void WriteInt32(MemoryStream s, int value)
    {
        Span<byte> b = stackalloc byte[4];
        BinaryPrimitives.WriteInt32LittleEndian(b, value);
        s.Write(b);
    }

    private static void WriteInt64(MemoryStream s, long value)
    {
        Span<byte> b = stackalloc byte[8];
        BinaryPrimitives.WriteInt64LittleEndian(b, value);
        s.Write(b);
    }

    private static void WriteUInt64(MemoryStream s, ulong value)
    {
        Span<byte> b = stackalloc byte[8];
        BinaryPrimitives.WriteUInt64LittleEndian(b, value);
        s.Write(b);
    }
}

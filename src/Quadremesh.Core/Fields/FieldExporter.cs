using System.Globalization;
using System.Text;
using Quadremesh.Core.Math;

namespace Quadremesh.Core.Fields;

/// <summary>
/// Debug visualisation of an orientation field as a Wavefront OBJ of line segments — one cross per
/// vertex (the direction o_i and its 90° rotation). Open it alongside the surface to eyeball field
/// smoothness and where the singularities sit (P3 deliverable).
/// </summary>
public static class FieldExporter
{
    /// <summary>Writes the cross field to an OBJ file of <c>l</c> line segments.</summary>
    public static void WriteOrientationField(
        string path, Vector3[] position, Vector3[] normal, Vector3[] orientation, double scale)
    {
        using var writer = new StreamWriter(path) { NewLine = "\n" };
        WriteOrientationField(writer, position, normal, orientation, scale);
    }

    /// <summary>Writes the cross field as OBJ text.</summary>
    public static void WriteOrientationField(
        TextWriter writer, Vector3[] position, Vector3[] normal, Vector3[] orientation, double scale)
    {
        ArgumentNullException.ThrowIfNull(writer);
        ArgumentNullException.ThrowIfNull(position);
        ArgumentNullException.ThrowIfNull(normal);
        ArgumentNullException.ThrowIfNull(orientation);

        writer.WriteLine("# Quadremesh orientation field (4-RoSy cross per vertex)");
        var sb = new StringBuilder();
        int vertexId = 1; // OBJ is 1-based

        for (int i = 0; i < position.Length; i++)
        {
            Vector3 p = position[i];
            Vector3 u = orientation[i] * scale;
            Vector3 v = TangentFrame.Rotate90(orientation[i], normal[i]) * scale;

            // Four endpoints of the cross centred at p.
            WriteVertex(sb, p - u);
            WriteVertex(sb, p + u);
            WriteVertex(sb, p - v);
            WriteVertex(sb, p + v);

            // Two segments through the centre.
            sb.Append('l').Append(' ').Append(vertexId).Append(' ').Append(vertexId + 1).Append('\n');
            sb.Append('l').Append(' ').Append(vertexId + 2).Append(' ').Append(vertexId + 3).Append('\n');
            vertexId += 4;
        }

        writer.Write(sb.ToString());
    }

    private static void WriteVertex(StringBuilder sb, Vector3 p) =>
        sb.Append("v ")
          .Append(p.X.ToString("R", CultureInfo.InvariantCulture)).Append(' ')
          .Append(p.Y.ToString("R", CultureInfo.InvariantCulture)).Append(' ')
          .Append(p.Z.ToString("R", CultureInfo.InvariantCulture)).Append('\n');
}

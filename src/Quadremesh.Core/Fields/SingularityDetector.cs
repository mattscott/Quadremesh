using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Fields;

/// <summary>Per-face singularity classification of an orientation field.</summary>
public sealed class SingularityResult
{
    /// <summary>Index numerator per face: the net field rotation in quarter-turns (0 = regular, ±1 = a ¼ singularity).</summary>
    public required int[] FaceIndex { get; init; }

    /// <summary>Number of faces with a non-zero index.</summary>
    public int SingularCount { get; init; }

    /// <summary>Sum of all face index numerators. By discrete Poincaré–Hopf this equals 4·χ (4·Euler characteristic).</summary>
    public int IndexSum { get; init; }
}

/// <summary>
/// Detects orientation-field singularities. A face is singular when the cross field cannot be combed
/// consistently around it: summing the per-edge 90° matchings around the face gives a non-zero
/// holonomy. These become the irregular vertices of the output mesh; QuadriFlow (P6) reduces them.
/// </summary>
public static class SingularityDetector
{
    /// <summary>Classifies every face of <paramref name="mesh"/> under the field <paramref name="orientation"/>.</summary>
    public static SingularityResult Detect(HalfEdgeMesh mesh, Vector3[] orientation, Vector3[] normal)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentNullException.ThrowIfNull(orientation);
        ArgumentNullException.ThrowIfNull(normal);

        var faceIndex = new int[mesh.FaceCount];
        int singular = 0;
        int sum = 0;

        int[] corners = new int[8];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            int size = mesh.FaceSize(f);
            if (size > corners.Length)
            {
                corners = new int[size];
            }

            int k = mesh.GetFaceVertices(f, corners);
            if (k < 3)
            {
                continue;
            }

            // Accumulate the 90° matchings around the face loop; the holonomy mod 4 is the index.
            int holonomy = 0;
            for (int i = 0; i < k; i++)
            {
                int a = corners[i];
                int b = corners[(i + 1) % k];
                holonomy += Rosy.MatchingIndex(orientation[a], orientation[b], normal[b]);
            }

            holonomy %= 4;
            if (holonomy < 0)
            {
                holonomy += 4;
            }

            int index = holonomy <= 2 ? holonomy : holonomy - 4; // smallest-magnitude representative
            faceIndex[f] = index;
            if (index != 0)
            {
                singular++;
                sum += index;
            }
        }

        return new SingularityResult { FaceIndex = faceIndex, SingularCount = singular, IndexSum = sum };
    }
}

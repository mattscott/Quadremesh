using Quadremesh.Core.Math;

namespace Quadremesh.Core.Fields;

/// <summary>
/// 4-RoSy (cross-field) arithmetic. A direction <c>d</c> at a vertex represents the set
/// {d, R90 d, R180 d, R270 d} about that vertex's normal, since a quad's edge directions are
/// indistinguishable under 90° rotation. These helpers pick the representative of one cross that
/// best matches another, which is the heart of both the smoothing update and singularity detection.
/// </summary>
public static class Rosy
{
    /// <summary>
    /// Returns the representative of cross <paramref name="dj"/> (rotated 0/90/180/270° about
    /// <paramref name="nj"/>) that is most aligned with reference direction <paramref name="di"/>.
    /// </summary>
    public static Vector3 ClosestRepresentative(Vector3 di, Vector3 dj, Vector3 nj)
    {
        Vector3 perp = Vector3.Cross(nj, dj);
        Vector3 best = dj;
        double bestDot = Vector3.Dot(di, dj);

        double d1 = Vector3.Dot(di, perp);
        if (d1 > bestDot)
        {
            bestDot = d1;
            best = perp;
        }

        double d2 = -Vector3.Dot(di, dj);
        if (d2 > bestDot)
        {
            bestDot = d2;
            best = -dj;
        }

        double d3 = -d1;
        if (d3 > bestDot)
        {
            best = -perp;
        }

        return best;
    }

    /// <summary>
    /// The integer rotation k in {0, 1, 2, 3} such that R(k·90°) applied to <paramref name="dj"/> about
    /// <paramref name="nj"/> best matches <paramref name="di"/>. Used to accumulate holonomy around a
    /// face for singularity detection.
    /// </summary>
    public static int MatchingIndex(Vector3 di, Vector3 dj, Vector3 nj)
    {
        Vector3 perp = Vector3.Cross(nj, dj);
        double d0 = Vector3.Dot(di, dj);
        double d1 = Vector3.Dot(di, perp);

        // Dots for the four representatives are (d0, d1, -d0, -d1).
        int best = 0;
        double bestDot = d0;
        if (d1 > bestDot)
        {
            bestDot = d1;
            best = 1;
        }

        if (-d0 > bestDot)
        {
            bestDot = -d0;
            best = 2;
        }

        if (-d1 > bestDot)
        {
            best = 3;
        }

        return best;
    }
}

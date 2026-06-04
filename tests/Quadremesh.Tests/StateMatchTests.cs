using Quadremesh.Core.Math;
using Xunit;
using Xunit.Abstractions;

namespace Quadremesh.Tests;

/// <summary>
/// Phase-0 validation harness (plan 07 §4): diff our per-stage state against dumps from the instrumented
/// QuadriFlow reference. This is the gate the bottom-up field re-port (Phases 1-4) is validated against —
/// goldens alone are insufficient because intermediate states are PROVEN to regress them.
///
/// Comparisons are modulo the stage's invariance: orientation (4-RoSy) matches modulo 90° rotation about
/// the normal; positions modulo lattice translation; hierarchy modulo a node permutation recovered from
/// mToLower. Until the oracle dumps exist (the other half of Phase 0) AND the matching faithful stage is
/// ported, these SKIP with a message naming exactly what's missing. Opt-in via QUADREMESH_STATEMATCH=1.
///
/// Dump layout (written by the instrumented oracle): tests/golden/quadriflow/state/&lt;mesh&gt;/&lt;stage&gt;.bin
/// e.g. sphere/orientation.bin (a 3xN double matrix = mQ[0]).
/// </summary>
public class StateMatchTests
{
    private readonly ITestOutputHelper _output;

    public StateMatchTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void OrientationMatchesReference(string mesh)
    {
        if (Environment.GetEnvironmentVariable("QUADREMESH_STATEMATCH") != "1")
        {
            _output.WriteLine($"SKIP {mesh}: set QUADREMESH_STATEMATCH=1.");
            return;
        }

        string? root = RepoRoot();
        Assert.NotNull(root);
        string dump = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "orientation.bin");
        if (!File.Exists(dump))
        {
            _output.WriteLine(
                $"SKIP {mesh}: no reference dump at {dump}. Needs the instrumented oracle (Phase 0 harness, " +
                "reference side) to emit qflow::Save(fp, hierarchy.mQ[0]) after optimize_orientations.");
            return;
        }

        // Once Phase 1-2 land: run our faithful field on the SAME pre-pass mesh ref dumped, then:
        //   Vector3[] refQ = Read(dump); Vector3[] ourQ = ...; Vector3[] n = ...;
        //   double maxDeg = MaxOrientationErrorDegrees(refQ, ourQ, n);
        //   Assert.True(maxDeg < tol, $"orientation field off by {maxDeg:F2} deg");
        _output.WriteLine(
            $"PENDING {mesh}: dump present but the faithful field (Phase 1-2) is not wired yet. " +
            "The comparator below is ready; this becomes a live gate when the faithful orientation solve lands.");
    }

    /// <summary>
    /// Max per-vertex angular deviation (degrees) between two 4-RoSy fields, modulo 90° rotation about the
    /// normal. The four representatives of a cross at vertex v are {q, n×q, -q, -(n×q)}; the error is the
    /// angle to the best-aligned representative. 0 ⇒ identical fields up to the quad symmetry.
    /// </summary>
    internal static double MaxOrientationErrorDegrees(Vector3[] reference, Vector3[] ours, Vector3[] normals)
    {
        if (reference.Length != ours.Length || ours.Length != normals.Length)
        {
            throw new ArgumentException("Field length mismatch.");
        }

        double worst = 0.0;
        for (int v = 0; v < reference.Length; v++)
        {
            Vector3 q = ours[v];
            Vector3 perp = Vector3.Cross(normals[v], q); // +90° representative (n×q), per the audit
            double best = System.Math.Max(
                System.Math.Max(Vector3.Dot(reference[v], q), Vector3.Dot(reference[v], perp)),
                System.Math.Max(Vector3.Dot(reference[v], -q), Vector3.Dot(reference[v], -perp)));
            best = System.Math.Clamp(best, -1.0, 1.0);
            double deg = System.Math.Acos(best) * 180.0 / System.Math.PI;
            worst = System.Math.Max(worst, deg);
        }

        return worst;
    }

    private static string? RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            if (File.Exists(Path.Combine(dir.FullName, "Quadremesh.sln")))
            {
                return dir.FullName;
            }

            dir = dir.Parent;
        }

        return null;
    }
}

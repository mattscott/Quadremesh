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

    /// <summary>
    /// Verifies the reader against a REAL reference dump: loads the full hierarchy state and asserts the
    /// cross-array structural invariants (per-level counts agree; coarsening maps line up; fields are
    /// per-vertex; singularities index valid faces). This proves the C# harness ingests the reference
    /// bytes correctly — the foundation every Phase 1-4 gate builds on. SKIPs if the dump is absent
    /// (regenerate via the instrumented oracle: quadriflow -mcf -seed 0 -dump &lt;dir&gt; -i &lt;mesh&gt; -f 5000).
    /// </summary>
    [Theory]
    [InlineData("sphere")]
    [InlineData("torus")]
    public void ReferenceDumpLoads(string mesh)
    {
        string? root = RepoRoot();
        Assert.NotNull(root);
        string dump = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dump))
        {
            _output.WriteLine($"SKIP {mesh}: no dump at {dump}. Regenerate with the instrumented oracle (-dump).");
            return;
        }

        RefHierarchyDump d = RefHierarchyDump.Load(dump);
        int faces = d.F.GetLength(1);
        _output.WriteLine(
            $"{mesh}: scale={d.Scale:F5} levels={d.LevelCount} faces={faces} finestV={d.V[0].Length} " +
            $"coarsestV={d.V[^1].Length} singularities={d.Singularities.Count}");

        Assert.True(d.LevelCount > 1, "expected a multi-level hierarchy");
        // mV/mN/mA/mQ/mO/mAdj are per-level (LevelCount); mToLower/mToUpper are inter-level maps
        // (LevelCount-1 of them — one per downsample step).
        Assert.Equal(d.LevelCount - 1, d.ToLower.Length);
        Assert.Equal(d.LevelCount - 1, d.ToUpper.Length);
        for (int l = 0; l < d.LevelCount; l++)
        {
            int n = d.V[l].Length;
            Assert.Equal(n, d.N[l].Length);
            Assert.Equal(n, d.A[l].Length);
            Assert.Equal(n, d.Q[l].Length);
            Assert.Equal(n, d.O[l].Length);
            Assert.Equal(n, d.Adj[l].Length);

            // mToLower maps each node at level l to a parent at level l+1; mToUpper is 2 × nCoarse.
            if (l + 1 < d.LevelCount)
            {
                Assert.Equal(n, d.ToLower[l].Length);
                int coarse = d.V[l + 1].Length;
                Assert.Equal(coarse, d.ToUpper[l].GetLength(1));
                foreach (int parent in d.ToLower[l])
                {
                    Assert.InRange(parent, 0, coarse - 1);
                }
            }
        }

        // Orientation field is unit-length in the tangent plane; sanity-check it's normalized.
        foreach (Vector3 q in d.Q[0])
        {
            Assert.InRange(q.Length, 0.99, 1.01);
        }

        // Singularities index real faces with QuadriFlow's valence codes (1 ⇒ valence 3, 3 ⇒ valence 5).
        foreach ((int face, int code) in d.Singularities)
        {
            Assert.InRange(face, 0, faces - 1);
            Assert.True(code == 1 || code == 3, $"unexpected singularity code {code}");
        }
    }

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
        string dump = Path.Combine(root!, "tests", "golden", "quadriflow", "state", mesh, "state.bin");
        if (!File.Exists(dump))
        {
            _output.WriteLine($"SKIP {mesh}: no reference dump at {dump} (regenerate via instrumented oracle).");
            return;
        }

        // Once Phase 1-2 land: run our faithful field on the SAME pre-pass mesh ref built, then:
        //   RefHierarchyDump d = RefHierarchyDump.Load(dump);
        //   double maxDeg = MaxOrientationErrorDegrees(d.Q[0], ourQ, d.N[0]);
        //   Assert.True(maxDeg < tol, $"orientation field off by {maxDeg:F2} deg");
        _output.WriteLine(
            $"PENDING {mesh}: dump present and loadable, but the faithful field (Phase 1-2) is not wired yet. " +
            "MaxOrientationErrorDegrees is ready; this becomes a live gate when the faithful orientation solve lands.");
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

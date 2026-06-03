using Quadremesh.Core.Extraction;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Xunit;

namespace Quadremesh.Tests;

public class QuadMeshCleanupTests
{
    /// <summary>Builds an N×M torus quad grid (periodic both ways): every vertex is valence 4.</summary>
    private static QuadMesh TorusGrid(int n, int m)
    {
        var positions = new Vector3[n * m];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                positions[(i * m) + j] = new Vector3(i, j, 0);
            }
        }

        var faces = new List<int[]>(n * m);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                int a = (i * m) + j;
                int b = (((i + 1) % n) * m) + j;
                int c = (((i + 1) % n) * m) + ((j + 1) % m);
                int d = (i * m) + ((j + 1) % m);
                faces.Add(new[] { a, b, c, d });
            }
        }

        return new QuadMesh(positions, faces);
    }

    [Fact]
    public void FixValence_leaves_a_clean_all_quad_torus_untouched()
    {
        QuadMesh grid = TorusGrid(4, 5);
        QuadMesh cleaned = QuadMeshCleanup.FixValence(grid);

        // No doublets, pinches, or high-valence vertices: topology must be preserved exactly.
        Assert.Equal(grid.VertexCount, cleaned.VertexCount);
        Assert.Equal(grid.FaceCount, cleaned.FaceCount);
        Assert.Equal(1.0, cleaned.QuadFraction);

        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(cleaned);
        Assert.Equal(0, he.Defects.NonManifoldEdges);
        Assert.Equal(0, he.Defects.BoundaryEdges);
        Assert.Equal(0, he.VertexCount - he.EdgeCount + he.FaceCount); // torus χ = 0
    }

    [Fact]
    public void FixValence_collapses_a_two_quad_doublet_pillow()
    {
        // A "pillow": two quads sharing all four edges. Every vertex is valence 2 (all doublets), so
        // the valence-2 pass must collapse it rather than leave the degenerate pair.
        var positions = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0),
        };
        var faces = new List<int[]>
        {
            new[] { 0, 1, 2, 3 },
            new[] { 0, 3, 2, 1 },
        };

        QuadMesh cleaned = QuadMeshCleanup.FixValence(new QuadMesh(positions, faces));

        Assert.True(cleaned.FaceCount < 2, $"doublet pillow not collapsed: {cleaned.FaceCount} faces");
    }
}

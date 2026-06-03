using Quadremesh.Core.Extraction;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Xunit;

namespace Quadremesh.Tests;

/// <summary>
/// Unit tests for <see cref="QuadHoleFiller"/> (the FixHoles port): even boundary loops left by the
/// integer-grid extractor must be quadrangulated and closed; already-watertight meshes must be left
/// untouched.
/// </summary>
public class QuadHoleFillerTests
{
    [Fact]
    public void Watertight_mesh_is_unchanged()
    {
        // A cube: 8 verts, 6 quads, closed.
        QuadMesh cube = Cube();
        QuadMesh result = QuadHoleFiller.Fill(cube);
        Assert.Same(cube, result);
        Assert.Equal(6, result.FaceCount);
    }

    [Fact]
    public void Single_quad_hole_is_filled()
    {
        // A cube with one face removed -> a single 4-edge boundary loop on an otherwise closed surface.
        QuadMesh holed = CubeMinusTop();
        Assert.Equal(5, holed.FaceCount);
        Assert.Equal(4, BoundaryEdges(holed));

        QuadMesh filled = QuadHoleFiller.Fill(holed);
        Assert.Equal(0, BoundaryEdges(filled));
        Assert.Equal(1.0, filled.QuadFraction);
    }

    [Fact]
    public void Hexagonal_hole_is_filled_with_quads()
    {
        // A 2x1x1 box with both top quads removed -> a 6-edge boundary loop on a closed surface;
        // QuadEnergy must tile it with two quads, leaving no internal boundary (no winding clash).
        QuadMesh holed = BoxMinusTwoTop();
        Assert.Equal(6, BoundaryEdges(holed));

        QuadMesh filled = QuadHoleFiller.Fill(holed);
        Assert.Equal(0, BoundaryEdges(filled));
        Assert.Equal(1.0, filled.QuadFraction);
    }

    private static int BoundaryEdges(QuadMesh mesh)
    {
        HalfEdgeMesh he = HalfEdgeMesh.FromQuadMesh(mesh);
        int n = 0;
        for (int e = 0; e < he.EdgeCount; e++)
        {
            if (he.Twin(he.EdgeHalfEdge(e)) < 0)
            {
                n++;
            }
        }

        return n;
    }

    private static QuadMesh Cube()
    {
        var p = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0),
            new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(1, 1, 1), new Vector3(0, 1, 1),
        };
        var f = new List<int[]>
        {
            new[] { 0, 3, 2, 1 }, // bottom (z=0)
            new[] { 4, 5, 6, 7 }, // top (z=1)
            new[] { 0, 1, 5, 4 },
            new[] { 1, 2, 6, 5 },
            new[] { 2, 3, 7, 6 },
            new[] { 3, 0, 4, 7 },
        };
        return new QuadMesh(p, f);
    }

    private static QuadMesh CubeMinusTop()
    {
        var p = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0),
            new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(1, 1, 1), new Vector3(0, 1, 1),
        };
        var f = new List<int[]>
        {
            new[] { 0, 3, 2, 1 }, // bottom
            new[] { 0, 1, 5, 4 },
            new[] { 1, 2, 6, 5 },
            new[] { 2, 3, 7, 6 },
            new[] { 3, 0, 4, 7 },
            // top (4,5,6,7) removed -> 4-edge hole
        };
        return new QuadMesh(p, f);
    }

    private static QuadMesh BoxMinusTwoTop()
    {
        var p = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(2, 0, 0), // 0,1,2
            new Vector3(0, 1, 0), new Vector3(1, 1, 0), new Vector3(2, 1, 0), // 3,4,5
            new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(2, 0, 1), // 6,7,8
            new Vector3(0, 1, 1), new Vector3(1, 1, 1), new Vector3(2, 1, 1), // 9,10,11
        };
        var f = new List<int[]>
        {
            new[] { 0, 3, 4, 1 }, new[] { 1, 4, 5, 2 },   // bottom
            new[] { 0, 1, 7, 6 }, new[] { 1, 2, 8, 7 },   // front (y=0)
            new[] { 3, 9, 10, 4 }, new[] { 4, 10, 11, 5 }, // back (y=1)
            new[] { 0, 6, 9, 3 },                          // left (x=0)
            new[] { 2, 5, 11, 8 },                         // right (x=2)
            // top quads (6,7,10,9) and (7,8,11,10) removed -> 6-edge hole rim 6-7-8-11-10-9
        };
        return new QuadMesh(p, f);
    }
}

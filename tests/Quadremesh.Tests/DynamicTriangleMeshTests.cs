using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.Core.Remeshing;
using Quadremesh.CorpusGen;
using Xunit;

namespace Quadremesh.Tests;

public class DynamicTriangleMeshTests
{
    private static DynamicTriangleMesh Sphere() =>
        DynamicTriangleMesh.FromTriangleMesh(Primitives.Icosphere(subdivisions: 1));

    private static void AssertClosedManifoldSphere(DynamicTriangleMesh dyn)
    {
        HalfEdgeMesh he = HalfEdgeMesh.FromTriangleMesh(dyn.ToTriangleMesh());
        Assert.True(he.Defects.IsEdgeManifold);
        Assert.True(he.Defects.IsClosed);
        Assert.Equal(0, he.Defects.IsolatedVertices);
        Assert.Equal(2, he.VertexCount - he.EdgeCount + he.FaceCount); // genus-0
    }

    [Fact]
    public void Build_from_closed_sphere_is_manifold()
    {
        DynamicTriangleMesh dyn = Sphere();
        Assert.Equal(0, dyn.DroppedTriangles);
        AssertClosedManifoldSphere(dyn);
    }

    [Fact]
    public void Build_drops_non_manifold_triangles()
    {
        DynamicTriangleMesh dyn = DynamicTriangleMesh.FromTriangleMesh(Primitives.NonManifold());
        Assert.True(dyn.DroppedTriangles > 0); // the 3-fan / bow-tie cannot all be kept manifold
    }

    [Fact]
    public void Split_adds_one_vertex_and_two_triangles_and_stays_manifold()
    {
        DynamicTriangleMesh dyn = Sphere();
        int v0 = dyn.LiveVertexCount;
        int f0 = dyn.LiveTriangleCount;

        (int a, int b, int _) = dyn.Triangle(0);
        Vector3 mid = (dyn.Position(a) + dyn.Position(b)) * 0.5;
        int m = dyn.Split(a, b, mid);

        Assert.True(m >= 0);
        Assert.Equal(v0 + 1, dyn.LiveVertexCount);
        Assert.Equal(f0 + 2, dyn.LiveTriangleCount);
        AssertClosedManifoldSphere(dyn);
    }

    [Fact]
    public void Flip_preserves_counts_and_manifoldness()
    {
        DynamicTriangleMesh dyn = Sphere();
        int v0 = dyn.LiveVertexCount;
        int f0 = dyn.LiveTriangleCount;

        bool flipped = false;
        for (int t = 0; t < dyn.TriangleCapacity && !flipped; t++)
        {
            if (dyn.TriangleRemoved(t))
            {
                continue;
            }

            (int a, int b, int _) = dyn.Triangle(t);
            flipped = dyn.Flip(a, b);
        }

        Assert.True(flipped);
        Assert.Equal(v0, dyn.LiveVertexCount);
        Assert.Equal(f0, dyn.LiveTriangleCount);
        AssertClosedManifoldSphere(dyn);
    }

    [Fact]
    public void Collapse_removes_one_vertex_and_two_triangles_and_stays_manifold()
    {
        DynamicTriangleMesh dyn = Sphere();
        int v0 = dyn.LiveVertexCount;
        int f0 = dyn.LiveTriangleCount;

        bool collapsed = false;
        for (int t = 0; t < dyn.TriangleCapacity && !collapsed; t++)
        {
            if (dyn.TriangleRemoved(t))
            {
                continue;
            }

            (int a, int b, int _) = dyn.Triangle(t);
            Vector3 target = (dyn.Position(a) + dyn.Position(b)) * 0.5;
            collapsed = dyn.Collapse(a, b, target);
        }

        Assert.True(collapsed);
        Assert.Equal(v0 - 1, dyn.LiveVertexCount);
        Assert.Equal(f0 - 2, dyn.LiveTriangleCount);
        AssertClosedManifoldSphere(dyn);
    }

    [Fact]
    public void Boundary_edges_are_not_collapsed()
    {
        DynamicTriangleMesh dyn = DynamicTriangleMesh.FromTriangleMesh(Primitives.Cylinder());

        // Find a boundary edge and confirm collapse refuses it.
        bool testedBoundary = false;
        for (int t = 0; t < dyn.TriangleCapacity && !testedBoundary; t++)
        {
            if (dyn.TriangleRemoved(t))
            {
                continue;
            }

            (int a, int b, int c) = dyn.Triangle(t);
            foreach ((int u, int w) in new[] { (a, b), (b, c), (c, a) })
            {
                if (dyn.EdgeIsBoundary(u, w))
                {
                    Vector3 target = (dyn.Position(u) + dyn.Position(w)) * 0.5;
                    Assert.False(dyn.Collapse(u, w, target));
                    Assert.False(dyn.Collapse(w, u, target));
                    testedBoundary = true;
                    break;
                }
            }
        }

        Assert.True(testedBoundary);
    }
}

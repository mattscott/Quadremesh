using Quadremesh.CorpusGen;
using Quadremesh.Core.Mesh;
using Xunit;

namespace Quadremesh.Tests;

public class HalfEdgeMeshTests
{
    [Fact]
    public void Sphere_is_closed_edge_manifold_with_euler_characteristic_two()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere());

        Assert.True(mesh.Defects.IsEdgeManifold);
        Assert.True(mesh.Defects.IsClosed);
        Assert.Equal(0, mesh.Defects.IsolatedVertices);

        // Genus-0 closed surface: V - E + F = 2.
        Assert.Equal(2, mesh.VertexCount - mesh.EdgeCount + mesh.FaceCount);
    }

    [Fact]
    public void Torus_is_closed_with_euler_characteristic_zero()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Torus());

        Assert.True(mesh.Defects.IsEdgeManifold);
        Assert.True(mesh.Defects.IsClosed);

        // Genus-1 closed surface: V - E + F = 0.
        Assert.Equal(0, mesh.VertexCount - mesh.EdgeCount + mesh.FaceCount);
    }

    [Fact]
    public void Open_cylinder_has_boundary_edges_but_is_manifold()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Cylinder());

        Assert.True(mesh.Defects.IsEdgeManifold);
        Assert.False(mesh.Defects.IsClosed);
        Assert.True(mesh.Defects.BoundaryEdges > 0);
    }

    [Fact]
    public void Valence_sum_equals_twice_edge_count()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere());

        long valenceSum = 0;
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            valenceSum += mesh.Valence(v);
        }

        Assert.Equal(2L * mesh.EdgeCount, valenceSum);
    }

    [Fact]
    public void Neighbors_are_symmetric_and_unique()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere());

        for (int v = 0; v < mesh.VertexCount; v++)
        {
            var seen = new HashSet<int>();
            foreach (int n in mesh.Neighbors(v))
            {
                Assert.True(seen.Add(n), "neighbour list should be unique");
                Assert.NotEqual(v, n);
                Assert.Contains(v, mesh.Neighbors(n).ToArray()); // adjacency is symmetric
            }
        }
    }

    [Fact]
    public void Face_traversal_returns_three_corners_per_triangle()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere());

        Span<int> corners = stackalloc int[8];
        for (int f = 0; f < mesh.FaceCount; f++)
        {
            Assert.Equal(3, mesh.FaceSize(f));
            Assert.Equal(3, mesh.GetFaceVertices(f, corners));
        }
    }

    [Fact]
    public void Non_manifold_input_loads_without_throwing_and_is_flagged()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.NonManifold());

        Assert.False(mesh.Defects.IsEdgeManifold);
        Assert.True(mesh.Defects.NonManifoldEdges > 0);
    }
}

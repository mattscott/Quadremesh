using Quadremesh.Core.Fields;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;
using Quadremesh.CorpusGen;
using Xunit;

namespace Quadremesh.Tests;

public class FeatureDetectorTests
{
    [Fact]
    public void Box_creases_form_loops_with_eight_corners()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Box());
        FeatureResult features = FeatureDetector.Detect(mesh);

        // A cube has 12 sharp edges meeting at 8 corners (each a 3-way junction).
        Assert.True(features.SharpEdgeCount > 0);
        int corners = features.Corner.Count(c => c);
        Assert.Equal(8, corners);

        // Every cube vertex lies on a feature; the 8 corners are junctions, the rest are clean creases.
        Assert.Equal(8, features.FeatureVertexCount);
    }

    [Fact]
    public void Sphere_has_no_sharp_edges()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Icosphere(subdivisions: 3));
        FeatureResult features = FeatureDetector.Detect(mesh);

        Assert.Equal(0, features.SharpEdgeCount);
        Assert.Equal(0, features.FeatureVertexCount);
    }

    [Fact]
    public void Cylinder_boundary_loops_are_features_with_tangents_along_the_rim()
    {
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(Primitives.Cylinder());
        FeatureResult features = FeatureDetector.Detect(mesh);

        // The open top and bottom rims are detected as features; the smooth wall is not creased.
        Assert.True(features.FeatureVertexCount > 0);

        // A rim vertex's crease tangent must be perpendicular to the cylinder axis (+Z): the rim is
        // a horizontal circle, so its tangent has (near) zero Z component.
        int checkedTangents = 0;
        for (int v = 0; v < mesh.VertexCount; v++)
        {
            if (features.OnFeature[v] && !features.Corner[v])
            {
                Assert.True(
                    System.Math.Abs(features.CreaseTangent[v].Z) < 0.2,
                    $"rim tangent at {v} not horizontal: {features.CreaseTangent[v]}");
                checkedTangents++;
            }
        }

        Assert.True(checkedTangents > 0, "no clean rim crease tangents found");
    }

    [Fact]
    public void Crease_tangent_is_unit_and_follows_a_planar_v_groove()
    {
        // Two quads meeting at a 90° fold along the shared edge x-axis: that edge is a crease, and the
        // crease tangent must run along it (±X).
        var positions = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(2, 0, 0), // the crease line (y=z=0)
            new Vector3(0, 1, 0), new Vector3(1, 1, 0), new Vector3(2, 1, 0), // +Y wing (flat z=0)
            new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(2, 0, 1), // +Z wing (folds up)
        };
        var tris = new[]
        {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4, // +Y wing
            0, 6, 7, 0, 7, 1, 1, 7, 8, 1, 8, 2, // +Z wing
        };
        HalfEdgeMesh mesh = HalfEdgeMesh.FromTriangleMesh(new TriangleMesh(positions, tris));
        FeatureResult features = FeatureDetector.Detect(mesh);

        // Vertex 1 is the interior crease vertex (between 0 and 2 along the fold).
        Assert.True(features.OnFeature[1]);
        Assert.False(features.Corner[1]);
        Assert.Equal(1.0, features.CreaseTangent[1].Length, 6);
        Assert.True(System.Math.Abs(features.CreaseTangent[1].X) > 0.9, $"tangent not along X: {features.CreaseTangent[1]}");
    }
}

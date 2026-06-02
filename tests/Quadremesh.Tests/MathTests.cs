using Quadremesh.Core.Math;
using Xunit;

namespace Quadremesh.Tests;

public class MathTests
{
    [Fact]
    public void Dot_and_cross_are_consistent_for_unit_axes()
    {
        Assert.Equal(0.0, Vector3.Dot(Vector3.UnitX, Vector3.UnitY), 12);
        Assert.Equal(Vector3.UnitZ, Vector3.Cross(Vector3.UnitX, Vector3.UnitY));
        Assert.Equal(-Vector3.UnitZ, Vector3.Cross(Vector3.UnitY, Vector3.UnitX));
    }

    [Fact]
    public void Normalized_yields_unit_length()
    {
        var v = new Vector3(3, 4, 12); // length 13
        Assert.Equal(13.0, v.Length, 12);
        Assert.Equal(1.0, v.Normalized().Length, 12);
    }

    [Fact]
    public void TryNormalize_fails_on_zero_vector()
    {
        Assert.False(Vector3.Zero.TryNormalize(out Vector3 result));
        Assert.Equal(Vector3.Zero, result);
    }

    [Fact]
    public void Triangle_area_of_unit_right_triangle_is_one_half()
    {
        double area = MathUtils.TriangleArea(Vector3.Zero, Vector3.UnitX, Vector3.UnitY);
        Assert.Equal(0.5, area, 12);
    }
}

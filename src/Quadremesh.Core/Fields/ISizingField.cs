namespace Quadremesh.Core.Fields;

/// <summary>
/// The target quad edge length ℓ_i at each vertex — the lattice spacing the position field snaps to.
/// Uniform unless adaptive (curvature-driven) sizing is enabled (finalised in P7).
/// </summary>
public interface ISizingField
{
    /// <summary>Target lattice spacing at finest-level vertex <paramref name="vertex"/>.</summary>
    double LengthAt(int vertex);
}

/// <summary>A constant target edge length everywhere.</summary>
public sealed class UniformSizingField : ISizingField
{
    private readonly double _length;

    /// <summary>Creates a uniform sizing field with the given spacing.</summary>
    public UniformSizingField(double length)
    {
        if (!(length > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(length), length, "Sizing length must be positive.");
        }

        _length = length;
    }

    /// <inheritdoc/>
    public double LengthAt(int vertex) => _length;
}

namespace Quadremesh.Core.Remeshing;

/// <summary>Options for the <see cref="IsotropicRemesher"/> Stage-0 pre-pass.</summary>
public sealed class IsotropicRemeshOptions
{
    /// <summary>
    /// Desired triangle edge length. Takes precedence over <see cref="TargetQuadCount"/>. If neither is
    /// set, the input's mean edge length is used (the pre-pass then only regularises, not re-scales).
    /// </summary>
    public double? TargetEdgeLength { get; set; }

    /// <summary>
    /// Desired output quad count; the edge length is derived as sqrt(surfaceArea / count). Ignored when
    /// <see cref="TargetEdgeLength"/> is set.
    /// </summary>
    public int? TargetQuadCount { get; set; }

    /// <summary>Number of split/collapse/flip/relax sweeps. More iterations = more uniform.</summary>
    public int Iterations { get; set; } = 10;

    /// <summary>Tangential relaxation step factor in [0, 1] (fraction of the way to the 1-ring centroid).</summary>
    public double RelaxationWeight { get; set; } = 1.0;
}

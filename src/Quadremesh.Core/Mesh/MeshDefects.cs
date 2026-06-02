namespace Quadremesh.Core.Mesh;

/// <summary>
/// Connectivity defects discovered while building a <see cref="HalfEdgeMesh"/>. The builder never
/// throws on dirty input; it records what it found here so callers can decide how to proceed
/// (P1 acceptance: non-manifold inputs load best-effort, defects flagged not fatal).
/// </summary>
public readonly struct MeshDefects
{
    /// <summary>Number of undirected edges incident to more than two faces, or traversed twice in the same direction.</summary>
    public int NonManifoldEdges { get; init; }

    /// <summary>Number of boundary edges (incident to exactly one face).</summary>
    public int BoundaryEdges { get; init; }

    /// <summary>Number of vertices not referenced by any face.</summary>
    public int IsolatedVertices { get; init; }

    /// <summary>True if the mesh has no non-manifold edges.</summary>
    public bool IsEdgeManifold => NonManifoldEdges == 0;

    /// <summary>True if the mesh has no boundary (closed surface).</summary>
    public bool IsClosed => BoundaryEdges == 0;

    /// <inheritdoc/>
    public override string ToString() =>
        $"non-manifold edges: {NonManifoldEdges}, boundary edges: {BoundaryEdges}, isolated vertices: {IsolatedVertices}";
}

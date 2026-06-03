namespace Quadremesh.Core.Extraction;

/// <summary>
/// A union-find where each node carries a relative 90° rotation to its parent, so the whole forest can
/// be combed into one consistent frame. QuadriFlow merges all non-singular faces this way; the
/// accumulated <see cref="Orient"/> then rotates every face into the root's frame, leaving the
/// orientation singularities (deliberately not merged) as the cone points. Ports the reference's
/// <c>DisajointOrientTree</c> (read for understanding only).
/// </summary>
public sealed class DisjointOrientTree
{
    private readonly int[] _parent;
    private readonly int[] _orient; // rotation (mod 4) relative to parent
    private readonly int[] _rank;

    /// <summary>Creates <paramref name="count"/> singleton nodes.</summary>
    public DisjointOrientTree(int count)
    {
        _parent = new int[count];
        _orient = new int[count];
        _rank = new int[count];
        for (int i = 0; i < count; i++)
        {
            _parent[i] = i;
        }
    }

    /// <summary>Representative of <paramref name="j"/> (with path compression that folds in rotations).</summary>
    public int Parent(int j)
    {
        if (_parent[j] == j)
        {
            return j;
        }

        int root = Parent(_parent[j]);
        _orient[j] = (_orient[j] + _orient[_parent[j]]) % 4;
        _parent[j] = root;
        return root;
    }

    /// <summary>Accumulated rotation of <paramref name="j"/> relative to its root.</summary>
    public int Orient(int j)
    {
        if (_parent[j] == j)
        {
            return _orient[j];
        }

        return (_orient[j] + Orient(_parent[j])) % 4;
    }

    /// <summary>
    /// Merges the sets of <paramref name="v0"/> and <paramref name="v1"/> given their edge-relative
    /// orientations, recording the rotation that makes the two frames consistent.
    /// </summary>
    public void Merge(int v0, int v1, int orient0, int orient1)
    {
        int p0 = Parent(v0);
        int p1 = Parent(v1);
        if (p0 == p1)
        {
            return;
        }

        int op0 = Orient(v0);
        int op1 = Orient(v1);
        if (_rank[p1] < _rank[p0])
        {
            _rank[p0] += _rank[p1];
            _parent[p1] = p0;
            _orient[p1] = ((orient1 - orient0 + op0 - op1) % 4 + 8) % 4;
        }
        else
        {
            _rank[p1] += _rank[p0];
            _parent[p0] = p1;
            _orient[p0] = ((orient0 - orient1 + op1 - op0) % 4 + 8) % 4;
        }
    }
}

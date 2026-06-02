namespace Quadremesh.Core.Acceleration;

/// <summary>
/// A disjoint-set (union-find) structure with path compression and union by rank. Used by extraction
/// to collapse all input vertices that fall in the same quad-lattice cell into one output vertex.
/// </summary>
public sealed class UnionFind
{
    private readonly int[] _parent;
    private readonly int[] _rank;

    /// <summary>Creates <paramref name="count"/> singleton sets.</summary>
    public UnionFind(int count)
    {
        _parent = new int[count];
        _rank = new int[count];
        for (int i = 0; i < count; i++)
        {
            _parent[i] = i;
        }
    }

    /// <summary>Returns the representative of the set containing <paramref name="x"/>.</summary>
    public int Find(int x)
    {
        while (_parent[x] != x)
        {
            _parent[x] = _parent[_parent[x]]; // path halving
            x = _parent[x];
        }

        return x;
    }

    /// <summary>Merges the sets containing <paramref name="a"/> and <paramref name="b"/>.</summary>
    public void Union(int a, int b)
    {
        int ra = Find(a);
        int rb = Find(b);
        if (ra == rb)
        {
            return;
        }

        if (_rank[ra] < _rank[rb])
        {
            (ra, rb) = (rb, ra);
        }

        _parent[rb] = ra;
        if (_rank[ra] == _rank[rb])
        {
            _rank[ra]++;
        }
    }

    /// <summary>True if <paramref name="a"/> and <paramref name="b"/> are in the same set.</summary>
    public bool Connected(int a, int b) => Find(a) == Find(b);
}

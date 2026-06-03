using Quadremesh.Core.Extraction;
using Quadremesh.Core.Math;
using Xunit;

namespace Quadremesh.Tests;

public class LatticeMathTests
{
    [Fact]
    public void Rshift90_rotates_through_the_four_orientations()
    {
        var e = new Vec2i(1, 0);
        Assert.Equal(new Vec2i(1, 0), Lattice2D.Rshift90(e, 0));
        Assert.Equal(new Vec2i(0, 1), Lattice2D.Rshift90(e, 1));
        Assert.Equal(new Vec2i(-1, 0), Lattice2D.Rshift90(e, 2));
        Assert.Equal(new Vec2i(0, -1), Lattice2D.Rshift90(e, 3));
    }

    [Fact]
    public void Rshift90_is_not_normalized_matching_the_reference()
    {
        // Rshift90 mirrors QuadriFlow's rshift90 exactly: the amount is NOT reduced mod 4. The
        // parity/sign tests run on the raw amount, so amount 4 is a 180° turn (not the identity) and
        // amount −2 is the identity. subdivide_edgeDiff produces such out-of-range orients and depends
        // on this faithful behaviour, so the contract is "callers pass the value the reference would".
        var e = new Vec2i(1, 0);
        Assert.Equal(new Vec2i(-1, 0), Lattice2D.Rshift90(e, 4));  // &1==0, >=2 -> negate
        Assert.Equal(e, Lattice2D.Rshift90(e, -2));                // &1==0, !>=2 -> identity
    }

    [Fact]
    public void Rank_is_zero_for_an_identical_cross_and_one_for_a_quarter_turn()
    {
        Vector3 n = Vector3.UnitZ;
        Vector3 q = Vector3.UnitX;
        Assert.Equal(0, Lattice2D.Rank(q, n, q, n));

        Vector3 qRotated = Vector3.Cross(n, q); // q turned 90° about n
        Assert.Equal(1, Lattice2D.Rank(q, n, qRotated, n));
    }

    [Fact]
    public void Orient_tree_makes_merged_edges_consistent()
    {
        // Invariant from the reference: after Merge(v0, v1, o0, o1),
        //   o1 + Orient(v0) ≡ o0 + Orient(v1)  (mod 4).
        var tree = new DisjointOrientTree(4);
        tree.Merge(0, 1, 1, 3);
        tree.Merge(1, 2, 2, 0);
        tree.Merge(2, 3, 3, 1);

        AssertConsistent(tree, 0, 1, 1, 3);
        AssertConsistent(tree, 1, 2, 2, 0);
        AssertConsistent(tree, 2, 3, 3, 1);
    }

    private static void AssertConsistent(DisjointOrientTree tree, int v0, int v1, int o0, int o1)
    {
        Assert.Equal((o1 + tree.Orient(v0)) % 4, (o0 + tree.Orient(v1)) % 4);
    }
}

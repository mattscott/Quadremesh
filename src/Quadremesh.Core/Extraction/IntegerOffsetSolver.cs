using Quadremesh.Core.Acceleration;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// The QuadriFlow integer-consistency solve. Local rounding of the position field's per-edge lattice
/// offsets is mutually inconsistent — around a face the offsets don't sum to zero, which forces
/// non-quad faces and extra singularities. This solver combs the field, measures each face's integer
/// "divergence", and routes integer corrections through a <see cref="MinCostFlow"/> on the dual graph
/// so that every (non-singular) face closes up: Σ offsets = 0. The result is globally consistent
/// integer offsets the extractor can collapse into clean quads.
/// </summary>
public sealed class IntegerOffsetSolver
{
    private const int CostScale = 1000;

    private readonly HalfEdgeMesh _mesh;
    private readonly Vector3[] _anchor;
    private readonly Vector3[] _normal;
    private readonly Vector3[] _rep;       // combed orientation per vertex
    private readonly double[] _scale;

    private readonly int[,] _round;        // [edge, dim] canonical-direction integer offset
    private readonly double[,] _frac;      // [edge, dim] fractional residual

    private IntegerOffsetSolver(
        HalfEdgeMesh mesh, Vector3[] anchor, Vector3[] normal, Vector3[] rep, double[] scale)
    {
        _mesh = mesh;
        _anchor = anchor;
        _normal = normal;
        _rep = rep;
        _scale = scale;
        _round = new int[mesh.EdgeCount, 2];
        _frac = new double[mesh.EdgeCount, 2];
        ComputeEdgeOffsets();
    }

    /// <summary>The combed orientation used for the solve (for the extractor's face tracing).</summary>
    public Vector3[] CombedOrientation => _rep;

    /// <summary>Globally consistent integer offset per edge, dimension 0 (canonical direction).</summary>
    public int[] EdgeK1 { get; private set; } = Array.Empty<int>();

    /// <summary>Globally consistent integer offset per edge, dimension 1.</summary>
    public int[] EdgeK2 { get; private set; } = Array.Empty<int>();

    /// <summary>Combs the field and runs the integer solve for both lattice dimensions.</summary>
    public static IntegerOffsetSolver Solve(
        HalfEdgeMesh mesh, Vector3[] orientation, Vector3[] normal, Vector3[] anchor, double[] scale)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentNullException.ThrowIfNull(orientation);
        ArgumentNullException.ThrowIfNull(normal);
        ArgumentNullException.ThrowIfNull(anchor);
        ArgumentNullException.ThrowIfNull(scale);

        Vector3[] rep = OrientationComb.Comb(mesh, orientation, normal);
        var solver = new IntegerOffsetSolver(mesh, anchor, normal, rep, scale);
        solver.EdgeK1 = solver.SolveDimension(0);
        solver.EdgeK2 = solver.SolveDimension(1);
        return solver;
    }

    /// <summary>Maximum absolute per-face divergence for the given offsets (0 = fully consistent).</summary>
    public int MaxFaceDivergence(int[] k1, int[] k2)
    {
        int max = 0;
        for (int f = 0; f < _mesh.FaceCount; f++)
        {
            if (_mesh.FaceSize(f) != 3)
            {
                continue;
            }

            int d1 = Divergence(f, k1);
            int d2 = Divergence(f, k2);
            max = System.Math.Max(max, System.Math.Max(System.Math.Abs(d1), System.Math.Abs(d2)));
        }

        return max;
    }

    /// <summary>The locally-rounded (pre-solve) offsets, for measuring the improvement.</summary>
    public (int[] K1, int[] K2) RoundedOffsets()
    {
        var k1 = new int[_mesh.EdgeCount];
        var k2 = new int[_mesh.EdgeCount];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            k1[e] = _round[e, 0];
            k2[e] = _round[e, 1];
        }

        return (k1, k2);
    }

    private void ComputeEdgeOffsets()
    {
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            int h = _mesh.EdgeHalfEdge(e);
            int o = _mesh.Origin(h);
            int d = _mesh.Head(h);
            Vector3 e1 = _rep[o];
            Vector3 e2 = Vector3.Cross(_normal[o], e1);
            Vector3 delta = (_anchor[d] - _anchor[o]) / _scale[o];
            double t1 = Vector3.Dot(delta, e1);
            double t2 = Vector3.Dot(delta, e2);
            int r1 = (int)System.Math.Round(t1);
            int r2 = (int)System.Math.Round(t2);
            _round[e, 0] = r1;
            _round[e, 1] = r2;
            _frac[e, 0] = t1 - r1;
            _frac[e, 1] = t2 - r2;
        }
    }

    private int[] SolveDimension(int dim)
    {
        int faceCount = _mesh.FaceCount;
        var flow = new MinCostFlow();
        var faceNode = new int[faceCount];
        for (int f = 0; f < faceCount; f++)
        {
            faceNode[f] = flow.AddNode();
        }

        int ground = flow.AddNode();

        long groundBalance = 0;
        for (int f = 0; f < faceCount; f++)
        {
            if (_mesh.FaceSize(f) != 3)
            {
                continue;
            }

            int div = Divergence(f, dim);
            flow.SetSupply(faceNode[f], div);
            groundBalance -= div;
        }

        flow.SetSupply(ground, groundBalance);

        // One bidirectional dual arc per edge. Raising the canonical offset c_e by 1 adds +1 to the
        // sum of the face on its canonical (+) side and −1 to the other: model as flow (+face → −face).
        var arcPlus = new int[_mesh.EdgeCount];
        var arcMinus = new int[_mesh.EdgeCount];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            int h = _mesh.EdgeHalfEdge(e);
            int twin = _mesh.Twin(h);
            int facePlus = _mesh.Face(h);
            int faceMinus = twin >= 0 ? _mesh.Face(twin) : -1;
            int nodePlus = faceNode[facePlus];
            int nodeMinus = faceMinus >= 0 ? faceNode[faceMinus] : ground;

            int cost = System.Math.Max(1, (int)System.Math.Round(CostScale * (1.0 - (2.0 * System.Math.Abs(_frac[e, dim])))));
            arcPlus[e] = flow.AddEdge(nodePlus, nodeMinus, 1_000_000, cost);
            arcMinus[e] = flow.AddEdge(nodeMinus, nodePlus, 1_000_000, cost);
        }

        flow.Solve();

        var result = new int[_mesh.EdgeCount];
        for (int e = 0; e < _mesh.EdgeCount; e++)
        {
            long net = flow.Flow(arcPlus[e]) - flow.Flow(arcMinus[e]);
            result[e] = _round[e, dim] - (int)net;
        }

        return result;
    }

    private int Divergence(int face, int dim)
    {
        int div = 0;
        int h = _mesh.FaceHalfEdge(face);
        do
        {
            int e = HalfEdgeToEdge(h, out int sign);
            div += sign * _round[e, dim];
            h = _mesh.Next(h);
        }
        while (h != _mesh.FaceHalfEdge(face));

        return div;
    }

    private int Divergence(int face, int[] offsets)
    {
        int div = 0;
        int h = _mesh.FaceHalfEdge(face);
        do
        {
            int e = HalfEdgeToEdge(h, out int sign);
            div += sign * offsets[e];
            h = _mesh.Next(h);
        }
        while (h != _mesh.FaceHalfEdge(face));

        return div;
    }

    /// <summary>The undirected edge of half-edge <paramref name="h"/>, plus +1/−1 for its canonical direction.</summary>
    private int HalfEdgeToEdge(int h, out int sign)
    {
        int o = _mesh.Origin(h);
        int d = _mesh.Head(h);
        ReadOnlySpan<int> neighbors = _mesh.Neighbors(o);
        ReadOnlySpan<int> edges = _mesh.NeighborEdges(o);
        for (int k = 0; k < neighbors.Length; k++)
        {
            if (neighbors[k] == d)
            {
                int e = edges[k];
                sign = _mesh.Origin(_mesh.EdgeHalfEdge(e)) == o ? 1 : -1;
                return e;
            }
        }

        sign = 1;
        return 0;
    }
}

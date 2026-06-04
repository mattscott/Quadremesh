namespace Quadremesh.Core.Acceleration;

/// <summary>
/// Standard MT19937 (32-bit Mersenne Twister), matching <c>std::mt19937</c>. The reference QuadriFlow seeds
/// it with <c>rng_seed = 0</c> to shuffle the Balance candidate lists; replicating it bit-for-bit (with
/// <see cref="MsvcShuffle"/>) is the last piece needed for a flow solution identical to the <c>-mcf</c> dump.
/// </summary>
internal sealed class Mt19937
{
    private const int N = 624;
    private const int M = 397;
    private const uint MatrixA = 0x9908B0DFu;
    private const uint UpperMask = 0x80000000u;
    private const uint LowerMask = 0x7FFFFFFFu;

    private readonly uint[] _mt = new uint[N];
    private int _index;

    public Mt19937(uint seed)
    {
        _mt[0] = seed;
        for (uint i = 1; i < N; i++)
        {
            _mt[i] = (1812433253u * (_mt[i - 1] ^ (_mt[i - 1] >> 30))) + i;
        }

        _index = N;
    }

    /// <summary>Next 32-bit output (matches <c>std::mt19937::operator()</c>).</summary>
    public uint Next()
    {
        if (_index >= N)
        {
            Generate();
        }

        uint y = _mt[_index++];
        y ^= y >> 11;
        y ^= (y << 7) & 0x9D2C5680u;
        y ^= (y << 15) & 0xEFC60000u;
        y ^= y >> 18;
        return y;
    }

    private void Generate()
    {
        for (int i = 0; i < N; i++)
        {
            uint x = (_mt[i] & UpperMask) | (_mt[(i + 1) % N] & LowerMask);
            uint xa = x >> 1;
            if ((x & 1u) != 0)
            {
                xa ^= MatrixA;
            }

            _mt[i] = _mt[(i + M) % N] ^ xa;
        }

        _index = 0;
    }
}

/// <summary>
/// Bit-faithful port of MSVC's <c>std::shuffle</c> (<c>_Random_shuffle1</c> + <c>_Rng_from_urng</c>, STL
/// 14.16): forward Fisher–Yates where each draw is a rejection-sampled <c>mt19937() % index</c>. QuadriFlow's
/// flow result depends on the exact permutation, so this must match MSVC's, not just any uniform shuffle.
/// </summary>
internal static class MsvcShuffle
{
    /// <summary>Shuffles <paramref name="list"/> in place exactly as MSVC's <c>std::shuffle(.., g)</c> would.</summary>
    public static void Shuffle<T>(System.Collections.Generic.IList<T> list, Mt19937 g)
    {
        int n = list.Count;
        for (int targetIndex = 1; targetIndex < n; targetIndex++)
        {
            int off = (int)RngFunc(g, targetIndex + 1);
            if (off != targetIndex)
            {
                (list[targetIndex], list[off]) = (list[off], list[targetIndex]);
            }
        }
    }

    // _Rng_from_urng::operator()(_Index) for mt19937: _Bits=32, _Bmask=0xFFFFFFFF, _Udiff=ulong.
    // Returns a uniform value in [0, index) via rejection sampling on full 32-bit draws.
    private static long RngFunc(Mt19937 g, long index)
    {
        var idx = (ulong)index;
        while (true)
        {
            ulong ret = 0;
            ulong mask = 0;
            while (mask < idx - 1)
            {
                ret = (ret << 32) | g.Next();
                mask = (mask << 32) | 0xFFFFFFFFul;
            }

            if ((ret / idx) < (mask / idx) || (mask % idx) == idx - 1)
            {
                return (long)(ret % idx);
            }
        }
    }
}

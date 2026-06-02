using Quadremesh.Core.Fields;
using Quadremesh.Core.Math;
using Quadremesh.Core.Mesh;

namespace Quadremesh.Core.Extraction;

/// <summary>
/// Combs a 4-RoSy field: picks one concrete representative direction per vertex so that across a
/// spanning tree of the mesh neighbouring directions are aligned (no 90° flip). On the resulting
/// combed field the position lattice frames vary only continuously between neighbours, which is what
/// lets the integer solve treat the two lattice axes as (locally) separable.
/// </summary>
public static class OrientationComb
{
    /// <summary>Returns a combed representative direction per vertex (unit, in the tangent plane).</summary>
    public static Vector3[] Comb(HalfEdgeMesh mesh, Vector3[] orientation, Vector3[] normal)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentNullException.ThrowIfNull(orientation);
        ArgumentNullException.ThrowIfNull(normal);

        int n = mesh.VertexCount;
        var rep = new Vector3[n];
        var visited = new bool[n];

        // BFS spanning forest (handles multiple connected components). Each child takes the
        // representative of its parent's direction that is closest to it.
        var queue = new Queue<int>();
        for (int seed = 0; seed < n; seed++)
        {
            if (visited[seed])
            {
                continue;
            }

            rep[seed] = orientation[seed];
            visited[seed] = true;
            queue.Enqueue(seed);

            while (queue.Count > 0)
            {
                int u = queue.Dequeue();
                foreach (int v in mesh.Neighbors(u))
                {
                    if (visited[v])
                    {
                        continue;
                    }

                    rep[v] = Rosy.ClosestRepresentative(rep[u], orientation[v], normal[v]);
                    visited[v] = true;
                    queue.Enqueue(v);
                }
            }
        }

        return rep;
    }
}

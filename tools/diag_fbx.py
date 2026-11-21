"""Quick mesh-health diagnostic for a (Quadremesh/UVBake-written) binary FBX: degenerate triangles,
coincident vertices, non-manifold edges. Reuses the minimal parser from render_fbx.py."""
import struct
import sys
import zlib
from collections import defaultdict

import numpy as np

data = open(sys.argv[1], "rb").read()
assert data[:20] == b"Kaydara FBX Binary  "
version = struct.unpack_from("<I", data, 23)[0]
off = 8 if version >= 7500 else 4


def read_off(p):
    return (struct.unpack_from("<q" if off == 8 else "<I", data, p)[0], p + off)


def read_props(p, n):
    out = []
    for _ in range(n):
        t = chr(data[p]); p += 1
        if t == "Y": out.append(struct.unpack_from("<h", data, p)[0]); p += 2
        elif t == "C": out.append(data[p]); p += 1
        elif t == "I": out.append(struct.unpack_from("<i", data, p)[0]); p += 4
        elif t == "F": out.append(struct.unpack_from("<f", data, p)[0]); p += 4
        elif t == "D": out.append(struct.unpack_from("<d", data, p)[0]); p += 8
        elif t == "L": out.append(struct.unpack_from("<q", data, p)[0]); p += 8
        elif t in "fdlib":
            length, enc, comp = struct.unpack_from("<III", data, p); p += 12
            payload = data[p:p + comp]; p += comp
            if enc == 1: payload = zlib.decompress(payload)
            out.append(np.frombuffer(payload, np.dtype("<" + {"f":"f","d":"d","l":"q","i":"i","b":"b"}[t])))
        elif t in "SR":
            ln = struct.unpack_from("<I", data, p)[0]; p += 4
            out.append(data[p:p + ln]); p += ln
        p = p
    return out, p


def read_node(p):
    end, p = read_off(p); nprops, p = read_off(p); _, p = read_off(p)
    nl = data[p]; p += 1
    if end == 0: return None, p
    name = data[p:p + nl].decode("latin1"); p += nl
    props, p = read_props(p, nprops)
    kids = []
    while p < end:
        k, p = read_node(p)
        if k is None: break
        kids.append(k)
    return {"name": name, "props": props, "children": kids}, end


pos = 27; top = []
while pos + 3 * off + 1 <= len(data):
    n, pos = read_node(pos)
    if n is None: break
    top.append(n)

V, quads = [], []
for objs in (n for n in top if n["name"] == "Objects"):
    for geo in (c for c in objs["children"] if c["name"] == "Geometry"):
        vc = next((c for c in geo["children"] if c["name"] == "Vertices"), None)
        ic = next((c for c in geo["children"] if c["name"] == "PolygonVertexIndex"), None)
        if not vc or not ic: continue
        v = np.asarray(vc["props"][0], np.float64).reshape(-1, 3); V.append(v)
        f = []
        for idx in np.asarray(ic["props"][0], np.int64):
            if idx < 0: f.append(int(~idx)); quads.append(f); f = []
            else: f.append(int(idx))
V = np.vstack(V)

# triangulate (fan) and measure
tris = []
for f in quads:
    for t in range(1, len(f) - 1):
        tris.append((f[0], f[t], f[t + 1]))
tris = np.array(tris)
p0, p1, p2 = V[tris[:, 0]], V[tris[:, 1]], V[tris[:, 2]]
area = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0), axis=1)
med = np.median(area)
degen = np.sum(area < 1e-9 * med)
tiny = np.sum(area < 1e-4 * med)

# coincident vertices
uniq = {}
dup = 0
for i, v in enumerate(map(tuple, np.round(V, 6))):
    if v in uniq: dup += 1
    else: uniq[v] = i

# non-manifold / boundary edges (by position index, undirected)
edges = defaultdict(int)
for a, b, c in tris:
    for u, w in ((a, b), (b, c), (c, a)):
        edges[(min(u, w), max(u, w))] += 1
boundary = sum(1 for v in edges.values() if v == 1)
nonman = sum(1 for v in edges.values() if v > 2)

bb = V.max(0) - V.min(0)
print(f"verts={len(V)}  quads={len(quads)}  tris={len(tris)}")
print(f"bbox={bb.round(3)}")
print(f"triangle area: median={med:.3e}  min={area.min():.3e}")
print(f"degenerate tris (area<1e-9*median)= {degen}   near-degenerate (<1e-4*median)= {tiny}")
print(f"coincident vertices (rounded 1e-6)= {dup}")
print(f"edges: total={len(edges)}  boundary(open)={boundary}  non-manifold(>2 faces)={nonman}")

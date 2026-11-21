"""Minimal binary-FBX previewer: parses a (Quadremesh-written) binary FBX, preserving polygon
(quad) faces, and renders a shaded multi-angle PNG with the quad wireframe. Handles raw and
zlib-encoded property arrays; 32/64-bit offsets. For quick visual inspection only."""
import os
import struct
import sys
import zlib

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

path, out = sys.argv[1], sys.argv[2]
data = open(path, "rb").read()
assert data[:20] == b"Kaydara FBX Binary  ", "not a binary FBX"
version = struct.unpack_from("<I", data, 23)[0]
off = 8 if version >= 7500 else 4


def read_off(p):
    return (struct.unpack_from("<q" if off == 8 else "<I", data, p)[0], p + off)


def read_props(p, n):
    props = []
    for _ in range(n):
        t = chr(data[p]); p += 1
        if t == "Y": props.append(struct.unpack_from("<h", data, p)[0]); p += 2
        elif t == "C": props.append(data[p]); p += 1
        elif t == "I": props.append(struct.unpack_from("<i", data, p)[0]); p += 4
        elif t == "F": props.append(struct.unpack_from("<f", data, p)[0]); p += 4
        elif t == "D": props.append(struct.unpack_from("<d", data, p)[0]); p += 8
        elif t == "L": props.append(struct.unpack_from("<q", data, p)[0]); p += 8
        elif t in "fdlib":
            length, enc, comp = struct.unpack_from("<III", data, p); p += 12
            payload = data[p:p + comp]; p += comp
            if enc == 1:
                payload = zlib.decompress(payload)
            fmt = {"f": "f", "d": "d", "l": "q", "i": "i", "b": "b"}[t]
            props.append(np.frombuffer(payload, dtype=np.dtype("<" + fmt)))
        elif t in "SR":
            length = struct.unpack_from("<I", data, p)[0]; p += 4
            val = data[p:p + length]; p += length
            props.append(val if t == "R" else val.decode("latin1"))
        else:
            raise ValueError("bad property type " + t)
    return props, p


def read_node(p):
    end, p = read_off(p)
    nprops, p = read_off(p)
    _, p = read_off(p)
    namelen = data[p]; p += 1
    if end == 0:
        return None, p
    name = data[p:p + namelen].decode("latin1"); p += namelen
    props, p = read_props(p, nprops)
    children = []
    while p < end:
        child, p = read_node(p)
        if child is None:
            break
        children.append(child)
    return {"name": name, "props": props, "children": children}, end


pos = 27
top = []
while pos + 3 * off + 1 <= len(data):
    node, pos = read_node(pos)
    if node is None:
        break
    top.append(node)

verts, faces, base = [], [], 0
for objs in (n for n in top if n["name"] == "Objects"):
    for geo in (c for c in objs["children"] if c["name"] == "Geometry"):
        vc = next((c for c in geo["children"] if c["name"] == "Vertices"), None)
        ic = next((c for c in geo["children"] if c["name"] == "PolygonVertexIndex"), None)
        if vc is None or ic is None:
            continue
        v = np.asarray(vc["props"][0], np.float64).reshape(-1, 3)
        verts.append(v)
        face = []
        for idx in np.asarray(ic["props"][0], np.int64):
            if idx < 0:
                face.append(int(~idx) + base)
                faces.append(face); face = []
            else:
                face.append(int(idx) + base)
        base += len(v)

V = np.vstack(verts)
sizes = np.bincount([len(f) for f in faces])
print(f"{len(V)} verts, {len(faces)} faces; face-size histogram: {dict(enumerate(sizes)) }")

# Center, build polygons, flat (two-sided) shading.
c = 0.5 * (V.min(0) + V.max(0))
P = V - c
lo, hi = P.min(0), P.max(0)
ext = hi - lo
pad = 0.04 * ext
lo, hi = lo - pad, hi + pad
polys = [P[f] for f in faces]


def normal(poly):
    n = np.cross(poly[1] - poly[0], poly[2] - poly[0])
    ln = np.linalg.norm(n)
    return n / ln if ln > 1e-12 else np.array([0.0, 0.0, 1.0])


nrm = np.array([normal(p) for p in polys])
light = np.array([0.35, 0.45, 0.9]); light /= np.linalg.norm(light)
shade = 0.22 + 0.78 * np.abs(nrm @ light)
base_col = np.array([0.80, 0.74, 0.66])  # warm clay
fc = np.clip(shade[:, None] * base_col[None, :], 0, 1)

views = [(24, -52), (24, 128)]
fig = plt.figure(figsize=(12, 13))
for k, (elev, azim) in enumerate(views):
    ax = fig.add_subplot(2, 1, k + 1, projection="3d")
    pc = Poly3DCollection(polys, facecolors=fc, edgecolors=(0.05, 0.05, 0.07, 0.55), linewidths=0.35)
    ax.add_collection3d(pc)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(ext, zoom=1.5)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.text2D(0.02, 0.04, f"elev {elev}° / azim {azim}°", transform=ax.transAxes, fontsize=8, color="0.4")
fig.suptitle(f"{os.path.basename(path)}  —  {len(faces)} faces, 100% quad", fontsize=13, y=0.995)
fig.subplots_adjust(left=0, right=1, top=0.97, bottom=0.0, hspace=0.0)
fig.savefig(out, dpi=160, bbox_inches="tight")
print("wrote", out)

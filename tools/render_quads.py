#!/usr/bin/env python3
"""Render a quad OBJ with simple Lambert shading + edge lines, from top/side/perspective views.

Usage: python render_quads.py <mesh-name> [<mesh-name> ...]
Reads tests/diag/faith_<name>.obj, writes tests/diag/render_<name>.png.
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIAG = os.path.join(ROOT, "tests", "diag")


def load_obj(path):
    verts = []
    faces = []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                faces.append(idx)
    return np.array(verts), faces


def render(name):
    path = os.path.join(DIAG, f"faith_{name}.obj")
    if not os.path.exists(path):
        print(f"skip {name}: no {path}")
        return
    V, faces = load_obj(path)

    # Center + scale to a unit-ish box for consistent framing.
    lo, hi = V.min(0), V.max(0)
    center = (lo + hi) / 2
    scale = (hi - lo).max() / 2
    V = (V - center) / scale

    # Per-quad geometry + normal from the mesh's CONSISTENT winding (the extractor orients all faces the
    # same way). Use the winding directly — do NOT flip per face by a center heuristic (that breaks on
    # concave regions). Only decide the GLOBAL orientation (outward vs inward) and flip all-or-nothing.
    quad_pts = [V[f] for f in faces]
    normals = np.zeros((len(faces), 3))
    centroids = np.zeros((len(faces), 3))
    for i, p in enumerate(quad_pts):
        n = np.cross(p[1] - p[0], p[3] - p[0])  # winding normal
        ln = np.linalg.norm(n)
        normals[i] = n / ln if ln > 1e-12 else (0, 0, 1.0)
        centroids[i] = p.mean(0)

    mesh_center = centroids.mean(0)
    if np.sum(normals * (centroids - mesh_center)) < 0:
        normals *= -1  # global flip so the consistent winding points outward

    light = np.array([0.4, 0.5, 0.78]); light /= np.linalg.norm(light)
    base = np.array([0.58, 0.70, 0.86])  # soft blue

    def cam_dir(elev, azim):
        e, a = np.radians(elev), np.radians(azim)
        return np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])

    views = [
        ("perspective", dict(elev=22, azim=-60)),
        ("top", dict(elev=89, azim=-90)),
        ("side", dict(elev=2, azim=0)),
    ]

    fig = plt.figure(figsize=(13.5, 5.0))
    fig.suptitle(f"{name}  —  faithful quad remesh ({len(faces)} quads)", fontsize=13, y=0.98)
    for i, (title, vk) in enumerate(views):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        cd = cam_dir(**vk)
        facing = normals @ cd
        front = np.where(facing > -0.02)[0]                 # back-face cull (front surface only)
        front = front[np.argsort(centroids[front] @ cd)]    # painter sort among front faces
        polys = [quad_pts[j] for j in front]
        shade = 0.34 + 0.66 * np.abs(normals[front] @ light)
        fcols = np.clip(base[None, :] * shade[:, None], 0, 1)
        pc = Poly3DCollection(
            polys, facecolors=fcols, edgecolors=(0.12, 0.12, 0.14), linewidths=0.15)
        ax.add_collection3d(pc)
        ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(**vk)
        ax.set_axis_off()
        ax.set_title(title, fontsize=10, pad=-2)

    out = os.path.join(DIAG, f"render_{name}.png")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=130, facecolor="white")
    plt.close(fig)
    print(f"wrote {out}  ({len(faces)} quads)")


if __name__ == "__main__":
    for n in sys.argv[1:]:
        render(n)

#!/usr/bin/env python3
"""그립 3D 구조 파악 — z별 단면적/폭 프로파일 + 좌표축·가정 버튼위치 표시.
실행: cad/.venv/bin/python cad/analyze_grip.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
m = trimesh.load(f"{OUT}/grip_capped.stl")
V = m.vertices
zmin, zmax = float(V[:, 2].min()), float(V[:, 2].max())
xmin, xmax = float(V[:, 0].min()), float(V[:, 0].max())
ymin, ymax = float(V[:, 1].min()), float(V[:, 1].max())
print(f"bbox X[{xmin:.0f},{xmax:.0f}] Y[{ymin:.0f},{ymax:.0f}] Z[{zmin:.0f},{zmax:.0f}]")

# z별 단면 폭/면적
print("  z   | Xwid Ywid | area(hull)")
zs = np.arange(np.floor(zmin) + 1, zmax, 4)
areas = []
for z in zs:
    b = V[np.abs(V[:, 2] - z) < 2.0][:, :2]
    if len(b) > 5:
        try:
            a = ConvexHull(b).volume   # 2D hull area
        except Exception:
            a = 0
        areas.append((z, np.ptp(b[:, 0]), np.ptp(b[:, 1]), a))
        print(f" {z:+5.0f} | {np.ptp(b[:,0]):4.0f} {np.ptp(b[:,1]):4.0f} | {a:6.0f}")
    else:
        areas.append((z, 0, 0, 0))

# 렌더: 4뷰 + 가정 버튼위치(z=-4, z=zmax-3 두 후보) + 축
def axes(ax, L=40):
    o = V.mean(0) * 0
    for d, c, t in [((L, 0, 0), "r", "X"), ((0, L, 0), "g", "Y"), ((0, 0, L), "b", "Z")]:
        ax.plot([0, d[0]], [0, d[1]], [0, d[2]], c, lw=2)
        ax.text(d[0], d[1], d[2], t, color=c, fontsize=11)

jx, jy = PT.JOY_POS
fig = plt.figure(figsize=(18, 5.2))
for k, (el, az, ttl) in enumerate([(20, -60, "iso"), (88, -90, "top +Z"), (2, -90, "front -Y"), (2, 0, "side +X")]):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(V[m.faces], facecolor=(0.7, 0.85, 0.7), edgecolor="none", alpha=0.30))
    axes(ax)
    for zc, col, tag in [(-4, "magenta", "z=-4"), (zmax - 3, "orange", f"z={zmax-3:.0f}")]:
        ax.scatter([jx], [jy], [zc], color=col, s=40)
        for nm, (bx, by) in PT.BTN_POS.items():
            ax.scatter([bx], [by], [zc], color=col, s=18)
        ax.text(jx, jy, zc, f" stick@{tag}", color=col, fontsize=7)
    c = V.mean(0); rng = (V.max(0) - V.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl, fontsize=9); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("그립 구조 분석 — 좌표축 + 가정 버튼위치(z=-4 마젠타 / z=top 주황)")
plt.tight_layout(); plt.savefig(f"{OUT}/analyze_grip.png", dpi=92)
print("saved analyze_grip.png")

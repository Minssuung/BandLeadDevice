#!/usr/bin/env python3
"""V3 전체 조립 한 컷 — 그립+캐리어+만능보드+버튼캡+트리거.
실행: cad/.venv/bin/python cad/full_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
PARTS = [
    ("grip_body_v3.stl", (0.55, 0.8, 0.6), 0.18, "그립①"),
    ("carrier_v3.stl", (0.3, 0.55, 0.95), 0.55, "캐리어②"),
    ("perfboard_v3.stl", (0.15, 0.6, 0.25), 0.95, "만능보드"),
    ("button_caps_v3.stl", (0.95, 0.5, 0.2), 1.0, "버튼캡"),
    ("trigger_lever_v3.stl", (0.9, 0.3, 0.3), 1.0, "트리거③"),
]
meshes = [(trimesh.load(f"{OUT}/{f}"), c, a, n) for f, c, a, n in PARTS]
allv = np.vstack([m.vertices for m, *_ in meshes])
ctr = allv.mean(0); rng = (allv.max(0) - allv.min(0)).max() / 2

fig = plt.figure(figsize=(19, 5.6))
views = [(20, -60, "iso", 0), (3, -90, "front", 0), (3, 0, "side", 0), (16, -60, "분해(캐리어+보드 위로)", 30)]
for k, (el, az, ttl, dz) in enumerate(views):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    for m, col, al, nm in meshes:
        off = np.array([0, 0, dz]) if (dz and nm in ("캐리어②", "만능보드", "버튼캡")) else np.zeros(3)
        ax.add_collection3d(Poly3DCollection((m.vertices + off)[m.faces], facecolor=col, edgecolor="none", alpha=al))
    ax.set_xlim(ctr[0]-rng, ctr[0]+rng); ax.set_ylim(ctr[1]-rng, ctr[1]+rng); ax.set_zlim(ctr[2]-rng, ctr[2]+rng)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=10); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
handles = [plt.Line2D([0], [0], marker="s", ls="", mfc=c, mec="k", ms=10, label=n) for _, c, _, n in meshes]
fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=9)
fig.suptitle("Bandi V2 리드디바이스 — 전체 조립 (PCB 중심)")
plt.tight_layout(rect=[0, 0.05, 1, 1]); plt.savefig(f"{OUT}/full_v3.png", dpi=95)
print("saved full_v3.png")

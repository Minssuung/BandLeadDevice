#!/usr/bin/env python3
"""V3 전체 조립본 — 모든 파트를 한 좌표계로 합쳐 렌더 + 단일 STL.
실행: cad/.venv/bin/python cad/combine_v3.py
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
    ("controller_v3.stl", (0.80, 0.85, 0.90), 0.18, "그립 형상"),
    ("carrier_v3.stl",     (0.35, 0.55, 0.95), 0.95, "헤드캐리어"),
    ("button_caps.stl",    (0.95, 0.50, 0.25), 1.0, "버튼캡 A/B/menu"),
    ("trigger_lever_v3.stl", (0.20, 0.45, 0.90), 1.0, "트리거 레버(검지)"),
    ("trigger_mount_v3.stl", (0.30, 0.70, 0.40), 1.0, "트리거 보스+홀"),
    ("imu_cradle_v3.stl",  (0.60, 0.40, 0.85), 0.95, "IMU 크래들"),
]
loaded = []
merged = []
for fn, col, alpha, lab in PARTS:
    p = f"{OUT}/{fn}"
    if not os.path.exists(p):
        print("skip(없음):", fn); continue
    m = trimesh.load(p)
    loaded.append((m, col, alpha, lab))
    if "controller_v3" not in fn:
        merged.append(m)
    print(f"  {lab:18} {fn}")

if merged:
    asm = trimesh.util.concatenate(merged)
    asm.export(f"{OUT}/assembly_v3.stl")
    print("saved assembly_v3.stl (형상 제외 부품 합본)")

allv = np.vstack([m.vertices for m, *_ in loaded])
c = allv.mean(0); rng = (allv.max(0) - allv.min(0)).max() / 2
fig = plt.figure(figsize=(18, 5.6))
for k, (el, az, ttl) in enumerate([(20, -62, "iso"), (3, -90, "front -Y"), (3, 0, "side +X"), (88, -90, "top")]):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    for m, col, alpha, lab in loaded:
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=col, edgecolor="none", alpha=alpha))
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
# 범례
handles = [plt.Line2D([0], [0], marker="s", ls="", mfc=col, mec="k", ms=9, label=lab) for _, col, _, lab in loaded]
fig.legend(handles=handles, loc="lower center", ncol=len(loaded), fontsize=8)
fig.suptitle("V3 전체 조립본 — 모든 입력 메커니즘 배치")
plt.tight_layout(rect=[0, 0.05, 1, 1]); plt.savefig(f"{OUT}/assembly_v3.png", dpi=95)
print("saved assembly_v3.png")

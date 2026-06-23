#!/usr/bin/env python3
"""만능보드(PCB) — 40×44×1.5, 캐리어 스탠드오프에 마운트. 조이스틱+택트3 실장.
실행: cad/.venv/bin/python cad/perfboard_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
BW, BL, BT = 40.0, 44.0, 1.5          # 보드 가로×세로×두께
BC = (0, -2)                          # 보드 중심 XY
BZ = -7.75                            # 보드 중심 z (top -7, bottom -8.5)
JOY = (0, -9); gx, gy = PT.JOY_MOUNT_GRID
BTN = {"A": (-14, 14), "B": (0, 14), "menu": (14, 14)}
POSTS = [(-18, -8), (18, -8), (-18, 6), (18, 6)]   # 캐리어 스탠드오프와 동일(버튼홀 침범 회피)

board = cq.Workplane("XY", origin=(BC[0], BC[1], BZ - BT / 2)).box(BW, BL, BT, centered=(True, True, False))
# 앞중앙 노치 — 트리거 보스/홀바 자리 비움 (조이스틱 마운트 ±13.5는 보존)
board = board.cut(cq.Workplane("XY", origin=(0, -21, BZ)).box(20, 12, 5))
# 스탠드오프 마운트홀 (Ø2.2)
for (px, py) in POSTS:
    board = board.faces(">Z").workplane().moveTo(px, py).hole(2.2)
# 조이스틱 마운트홀 4 (27×20) + 스틱 클리어 (Ø12)
board = board.faces(">Z").workplane().moveTo(*JOY).hole(12)
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        board = board.faces(">Z").workplane().moveTo(JOY[0] + dx, JOY[1] + dy).hole(2.2)
# 택트 핀 통로 (각 버튼 4핀 ~ 단순화: 5×5 관통)
for nm, (bx, by) in BTN.items():
    board = board.faces(">Z").workplane().moveTo(bx, by).rect(5, 5).cutThruAll()
cq.exporters.export(board, f"{OUT}/perfboard_v3.stl")
print(f"perfboard {BW}×{BL}×{BT} @ z{BZ}, 마운트홀4 + 조이스틱4 + 택트3")

# ── 렌더: 캐리어 + 보드 + 부품 마커 ──
def cqmesh(p):
    m = trimesh.load(p)
    return m

carrier = trimesh.load(f"{OUT}/carrier_v3.stl")
bm = trimesh.load(f"{OUT}/perfboard_v3.stl")
fig = plt.figure(figsize=(16, 6.5))
for k, (el, az, ttl) in enumerate([(20, -60, "iso (캐리어+보드)"), (4, -90, "front 단면감"), (90, -90, "top (배치)")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(carrier.vertices[carrier.faces], facecolor=(.3, .55, .95), edgecolor="none", alpha=.25))
    ax.add_collection3d(Poly3DCollection(bm.vertices[bm.faces], facecolor=(.2, .7, .3), edgecolor="none", alpha=.9))
    ax.scatter(*JOY, BZ, c="orange", s=40); ax.text(*JOY, BZ, " 조이스틱", color="orange", fontsize=8)
    for nm, (bx, by) in BTN.items():
        ax.scatter(bx, by, BZ, c="red", s=20)
    for (px, py) in POSTS:
        ax.scatter(px, py, BZ, c="k", s=15)
    allp = np.vstack([carrier.vertices, bm.vertices]); c = allp.mean(0); r = (allp.max(0) - allp.min(0)).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("만능보드(초록) 캐리어(파랑) 장착 — 조이스틱+버튼3 실장, 스탠드오프4")
plt.tight_layout(); plt.savefig(f"{OUT}/perfboard_v3.png", dpi=92)
print("saved perfboard_v3.png")

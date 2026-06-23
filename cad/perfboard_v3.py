#!/usr/bin/env python3
"""만능보드(PCB) — 택트2 납땜 + 배선 junction. 조이스틱 뒤, 캐리어 스탠드오프에 마운트.
조이스틱·홀·리프트·IMU·MAX485는 보드에 안 올라감(자기 자리 + 점퍼선).
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

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
BW, BL, BT = 40.0, 18.0, 1.5          # 보드 가로×세로×두께 (조이스틱 뒤 스트립)
BC = (0, 16)                          # 보드 중심 XY (y7..25, 큰 조이스틱 뒤)
BZ = -7.75                            # 보드 중심 z (top -7, bottom -8.5)
BTN = {"torque": (-11, 16), "kbd": (11, 16)}    # 택트2 (캐리어 버튼홀과 정렬)
POSTS = [(-16, 10), (16, 10), (-16, 22), (16, 22)]  # 캐리어 스탠드오프와 동일

board = cq.Workplane("XY", origin=(BC[0], BC[1], BZ - BT / 2)).box(BW, BL, BT, centered=(True, True, False))

def hole(b, x, y, d):    # 월드 절대좌표 cut
    return b.cut(cq.Workplane("XY", origin=(x, y, BZ + 2)).circle(d / 2).extrude(-(BT + 4)))

for (px, py) in POSTS:                       # 스탠드오프 마운트홀 Ø2.2
    board = hole(board, px, py, 2.2)
# 택트는 보드 표면에 얹고 핀만 그리드에 납땜 → 컷 없음. 핀 위치만 Ø1로 표시
for nm, (bx, by) in BTN.items():
    for dx, dy in [(-2.5, -2.5), (2.5, -2.5), (-2.5, 2.5), (2.5, 2.5)]:   # 6×6 택트 4핀
        board = hole(board, bx + dx, by + dy, 1.0)
cq.exporters.export(board, f"{OUT}/perfboard_v3.stl")
import pymeshfix
_m = trimesh.load(f"{OUT}/perfboard_v3.stl")
if not _m.is_watertight:
    _v, _f = pymeshfix.clean_from_arrays(np.asarray(_m.vertices), np.asarray(_m.faces))
    _m = trimesh.Trimesh(_v, _f); trimesh.repair.fix_normals(_m); _m.export(f"{OUT}/perfboard_v3.stl")
print(f"perfboard {BW}×{BL}×{BT} @ y{BC[1]} z{BZ} | 택트2+junction | watertight:", trimesh.load(f"{OUT}/perfboard_v3.stl").is_watertight)

# ── 렌더: 캐리어 + 보드 + 부품 마커 ──
carrier = trimesh.load(f"{OUT}/carrier_v3.stl")
bm = trimesh.load(f"{OUT}/perfboard_v3.stl")
gx, gy = 27, 20; JOY = (0, -9)
fig = plt.figure(figsize=(16, 6.5))
for k, (el, az, ttl) in enumerate([(20, -60, "iso (캐리어+보드)"), (4, -90, "front"), (90, -90, "top (배치)")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(carrier.vertices[carrier.faces], facecolor=(.3, .55, .95), edgecolor="none", alpha=.25))
    ax.add_collection3d(Poly3DCollection(bm.vertices[bm.faces], facecolor=(.2, .7, .3), edgecolor="none", alpha=.9))
    ax.scatter(*JOY, BZ, c="orange", s=50); ax.text(*JOY, BZ, " 조이스틱(보드밖)", color="orange", fontsize=7)
    for (dx, dy) in [(-gx/2,-gy/2),(-gx/2,gy/2),(gx/2,-gy/2),(gx/2,gy/2)]:
        ax.scatter(JOY[0]+dx, JOY[1]+dy, BZ, c="darkorange", s=18)   # 조이스틱 스탠드오프
    for nm, (bx, by) in BTN.items():
        ax.scatter(bx, by, BZ, c="red", s=25)
    for (px, py) in POSTS:
        ax.scatter(px, py, BZ, c="k", s=15)
    allp = np.vstack([carrier.vertices, bm.vertices]); c = allp.mean(0); r = (allp.max(0) - allp.min(0)).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("만능보드(초록)=택트2+junction / 조이스틱(주황)=자체 스탠드오프, 보드 밖 / 버튼2(빨강)")
plt.tight_layout(); plt.savefig(f"{OUT}/perfboard_v3.png", dpi=92)
print("saved perfboard_v3.png")

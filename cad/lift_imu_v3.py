#!/usr/bin/env python3
"""V3 Stage2c — 측면 리프트(SS-5GL) + IMU 마운트(대각 M2 스탠드오프 2개).
IMU: 보드 34×30×10, 대각선 Ø2 홀 2개에 M2 스탠드오프 보스로 고정(케이스 없음).
리프트: SS-5GL 본체 손잡이 내부, 레버만 측면 돌출.
실행: cad/.venv/bin/python cad/lift_imu_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
IMU_AT = (0.0, 24.6, -55.0)        # 손잡이 중심선 위
IMU_TILT = PT.IMU_TILT_DEG         # 18
W34, W30, TH = PT.IMU_BOARD        # 34(세로) × 30(가로) × 10(두께)
HD = PT.IMU_HOLE_DIA               # 2
HOLES = PT.IMU_MOUNT_HOLES         # [(3,6.5),(31,23.5)] (가로34, 세로30) from 좌하단
STAND = 4.0                        # 스탠드오프 높이
LIFT_AT = (8, 18, -34)
SS = (20.0, 6.5, 10.2)

# ── IMU: 평면 빌드(34→X,30→Y,두께→Z) → 회전(34→Z세로, 넓은면→±X) + 18° + 배치 ──
board = cq.Workplane("XY").box(W34, W30, TH)            # 중심기준
for u, v in HOLES:
    board = board.faces(">Z").workplane().pushPoints([(u - W34 / 2, v - W30 / 2)]).hole(HD)
posts = None
for u, v in HOLES:
    p = (cq.Workplane("XY", origin=(u - W34 / 2, v - W30 / 2, -TH / 2 - STAND)).circle(2.3).extrude(STAND)
         .faces(">Z").workplane().circle(0.85).cutBlind(STAND + 2))   # M2 파일럿
    posts = p if posts is None else posts.union(p)


def orient(s):
    return s.rotate((0, 0, 0), (0, 1, 0), -90).rotate((0, 0, 0), (1, 0, 0), IMU_TILT).translate(IMU_AT)

board, posts = orient(board), orient(posts)
cq.exporters.export(posts, f"{OUT}/imu_mount_v3.stl")
cq.exporters.export(board, f"{OUT}/imu_board_v3.stl")
print(f"IMU 보드 {W34}×{W30}×{TH}, 홀 {HOLES} Ø{HD}, 스탠드오프2 @ {IMU_AT}, {IMU_TILT:.0f}°")

# ── 리프트 SS-5GL ──
body = cq.Workplane("XY", origin=LIFT_AT).box(SS[1], SS[2], SS[0])
lever = cq.Workplane("XY", origin=(LIFT_AT[0] + SS[1] / 2 + 6, LIFT_AT[1], LIFT_AT[2])).box(12, 3, 1.2)
pocket = cq.Workplane("XY", origin=LIFT_AT).box(SS[1] + 0.6, SS[2] + 0.6, SS[0] + 0.6)
cq.exporters.export(pocket, f"{OUT}/lift_pocket_v3.stl")
print(f"리프트 SS-5GL @ {LIFT_AT} (레버 +X 돌출)")


def mesh(wp, tol=0.3):
    vv, tt = wp.val().tessellate(tol)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt))

bm, pm, ssb, ssl = mesh(board), mesh(posts), mesh(body), mesh(lever)
grip = trimesh.load(f"{OUT}/controller_v3.stl")

# 홀 월드좌표 (라벨용)
def hole_world(u, v):
    s = cq.Workplane("XY").box(.1, .1, .1).translate((u - W34 / 2, v - W30 / 2, 0))
    s = orient(s)
    bb = s.val().BoundingBox()
    return np.array([(bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2, (bb.zmin + bb.zmax) / 2])

fig = plt.figure(figsize=(17, 6.2))
# iso 배치
ax = fig.add_subplot(1, 3, 1, projection="3d")
ax.add_collection3d(Poly3DCollection(grip.vertices[grip.faces], facecolor=(.82, .86, .92), edgecolor="none", alpha=.12))
for m, col in [(bm, (.95, .55, .25)), (pm, (.2, .4, .85)), (ssb, (.4, .7, .4)), (ssl, (.9, .3, .3))]:
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=col, edgecolor="none", alpha=.9))
P = grip.vertices; c = P.mean(0); r = (P.max(0) - P.min(0)).max() / 2
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(16, -68); ax.set_title("배치(IMU 주황+보스 파랑·리프트 초록)", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
# IMU 치수/홀 확인 (정면 보기로 보드 평면)
ax = fig.add_subplot(1, 3, 2, projection="3d")
ax.add_collection3d(Poly3DCollection(grip.vertices[grip.faces], facecolor=(.85, .88, .92), edgecolor="none", alpha=.1))
ax.add_collection3d(Poly3DCollection(bm.vertices[bm.faces], facecolor=(.95, .55, .25), edgecolor="k", lw=.2, alpha=.85))
ax.add_collection3d(Poly3DCollection(pm.vertices[pm.faces], facecolor=(.2, .4, .85), edgecolor="none", alpha=.9))
for u, v in HOLES:
    hp = hole_world(u, v); ax.scatter(*hp, c="k", s=25); ax.text(*hp, f" ({u:.0f},{v:.0f})", fontsize=7)
ax.text(*(np.array(IMU_AT) + [12, 0, 0]), "넓은면\n→측면", fontsize=7, color="orange")
cc = np.array(IMU_AT); rr = 24
ax.set_xlim(cc[0]-rr, cc[0]+rr); ax.set_ylim(cc[1]-rr, cc[1]+rr); ax.set_zlim(cc[2]-rr, cc[2]+rr)
ax.view_init(8, -55); ax.set_title(f"IMU {W34}×{W30}×{TH} + 대각 홀Ø{HD}", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
# 리프트 단면
ax = fig.add_subplot(1, 3, 3)
for m, col, lab in [(grip, (.7, .7, .7), "그립"), (ssb, "green", "SS-5GL"), (ssl, "red", "레버")]:
    s = m.section(plane_origin=[0, 0, LIFT_AT[2]], plane_normal=[0, 0, 1])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 1], color=col, lw=1.2)
        ax.plot([], [], color=col, label=lab)
ax.set_aspect("equal"); ax.set_xlabel("X(측면→)"); ax.set_ylabel("Y"); ax.set_title("리프트 SS-5GL", fontsize=9); ax.legend(fontsize=8); ax.grid(alpha=.3)
fig.suptitle(f"V3 Stage2c — IMU 대각 M2 마운트({W34}×{W30}×{TH}) + 측면 리프트")
plt.tight_layout(); plt.savefig(f"{OUT}/lift_imu_v3.png", dpi=92)
print("saved lift_imu_v3.png")

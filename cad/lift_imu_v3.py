#!/usr/bin/env python3
"""V3 Stage2c — 측면 리프트(SS-5GL) + IMU 크래들(30°).
리프트: 스위치 포켓 + 레버 통로(중지). IMU: 34×30×10 보드를 11mm 슬롯에 30° 세로 크래들.
실행: cad/.venv/bin/python cad/lift_imu_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
from trimesh import transformations as TF
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
IMU_AT = (0, 16, -60)
IMU_TILT = PT.IMU_TILT_DEG        # 30
LIFT_AT = (7, 17, -34)            # 손잡이 내부(+X쪽), 레버만 측면 돌출
SS = (20.0, 6.5, 10.2)            # SS-5GL 본체 L×W×H (근사), 장축=Z 배치

# ── IMU 크래들 (보드 34×30×10, 11mm 슬롯, 30° 기울임) ──
bL, bW, bH = PT.IMU_BOARD          # 34,30,10
slot = PT.IMU_SLOT_W               # 11
# 보드: 폭30(X) × 두께10(Y) × 길이34(Z)
board = cq.Workplane("XY").box(bW, bH, bL)
# 크래들: 바닥판 + 양벽(11mm 슬롯)
base = cq.Workplane("XY", origin=(0, 0, -bL / 2 - 1.5)).box(bW + 6, slot + 6, 3)
w1 = cq.Workplane("XY", origin=(0, slot / 2 + 1, -bL / 2 + 8)).box(bW + 6, 2, 20)
w2 = cq.Workplane("XY", origin=(0, -slot / 2 - 1, -bL / 2 + 8)).box(bW + 6, 2, 20)
cradle = base.union(w1).union(w2)


def place(wp, tilt, at):
    return wp.rotate((0, 0, 0), (1, 0, 0), tilt).translate(at)

board_p = place(board, IMU_TILT, IMU_AT)
cradle_p = place(cradle, IMU_TILT, IMU_AT)
cq.exporters.export(cradle_p, f"{OUT}/imu_cradle_v3.stl")
print("IMU 크래들 vol:", round(cradle_p.val().Volume()), "| 보드 30° @", IMU_AT)

# ── 리프트 SS-5GL: 포켓 + 레버 ──
body = cq.Workplane("XY", origin=LIFT_AT).box(SS[1], SS[2], SS[0])      # X6.5 × Y10 × Z20(장축)
lever = cq.Workplane("XY", origin=(LIFT_AT[0] + SS[1] / 2 + 6, LIFT_AT[1], LIFT_AT[2])).box(12, 3, 1.2)  # +X 측면 돌출(중지)
# 포켓(그립에서 파낼 형상) = 본체+여유
pocket = cq.Workplane("XY", origin=LIFT_AT).box(SS[1] + 0.6, SS[2] + 0.6, SS[0] + 0.6)
cq.exporters.export(pocket, f"{OUT}/lift_pocket_v3.stl")
print("리프트 SS-5GL @", LIFT_AT, "(레버 +X 측면 돌출)")


def mesh(wp, tol=0.3):
    vv, tt = wp.val().tessellate(tol)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt))

bm, cm, ssb, ssl = mesh(board_p), mesh(cradle_p), mesh(body), mesh(lever)
try:
    grip = trimesh.load(f"{OUT}/controller_v3.stl")
except Exception:
    grip = None

fig = plt.figure(figsize=(17, 6.2))
# iso 배치
ax = fig.add_subplot(1, 3, 1, projection="3d")
if grip is not None:
    ax.add_collection3d(Poly3DCollection(grip.vertices[grip.faces], facecolor=(.8, .85, .9), edgecolor="none", alpha=.12))
for m, col in [(cm, (.3, .55, .95)), (bm, (.95, .55, .25)), (ssb, (.4, .7, .4)), (ssl, (.9, .3, .3))]:
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=col, edgecolor="none", alpha=.9))
ax.view_init(16, -68); ax.set_title("배치 (IMU 주황/크래들 파랑 · 리프트 초록)", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
P = grip.vertices if grip is not None else bm.vertices; c = P.mean(0); r = (P.max(0) - P.min(0)).max() / 2
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
# IMU 단면 (x=0, 30° 확인)
ax = fig.add_subplot(1, 3, 2)
for m, col, lab in [(grip, (.7, .7, .7), "그립"), (cm, "royalblue", "크래들"), (bm, "orange", "IMU 보드")]:
    if m is None:
        continue
    s = m.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], color=col, lw=1.2)
        ax.plot([], [], color=col, label=lab)
ax.text(IMU_AT[1], IMU_AT[2], f"  {IMU_TILT:.0f}°", fontsize=9, color="orange")
ax.set_aspect("equal"); ax.invert_xaxis(); ax.set_xlabel("Y"); ax.set_ylabel("Z"); ax.set_title("IMU 30° 크래들 단면", fontsize=9); ax.legend(fontsize=8); ax.grid(alpha=.3)
# 리프트 단면 (z = LIFT_AT z)
ax = fig.add_subplot(1, 3, 3)
for m, col, lab in [(grip, (.7, .7, .7), "그립"), (ssb, "green", "SS-5GL"), (ssl, "red", "레버")]:
    if m is None:
        continue
    s = m.section(plane_origin=[0, 0, LIFT_AT[2]], plane_normal=[0, 0, 1])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 1], color=col, lw=1.2)
        ax.plot([], [], color=col, label=lab)
ax.set_aspect("equal"); ax.set_xlabel("X(측면→)"); ax.set_ylabel("Y"); ax.set_title("리프트 SS-5GL (레버 측면돌출)", fontsize=9); ax.legend(fontsize=8); ax.grid(alpha=.3)
fig.suptitle("V3 Stage2c — 측면 리프트(SS-5GL) + IMU 크래들 30°")
plt.tight_layout(); plt.savefig(f"{OUT}/lift_imu_v3.png", dpi=92)
print("saved lift_imu_v3.png")

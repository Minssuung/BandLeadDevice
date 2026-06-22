#!/usr/bin/env python3
"""V3 Stage2d-1 — 그립 본체①: 속 비운 셸 + 헤드 개구 + 내부 마운트/포켓.
형상(헤드+손잡이) → shell(벽3, 상단 개구) → 트리거 보스+홀포켓 / 리프트 포켓+레버창 / IMU 스탠드오프.
실행: cad/.venv/bin/python cad/grip_body_v3.py
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
HEAD = (56, 58, 24); HEAD_R = 11
HANDLE_D = 38; HANDLE_L = 92; HANDLE_TILT = 18; HANDLE_OFF = (0, 10, -10)
WALL = 3.0
# 입력 위치 (다른 스크립트와 동일)
C = (0, -25, -6)                 # 트리거 피벗
HALL = (0, -19.6, -14.6)
LIFT_AT = (8, 18, -34); SS = (20.0, 6.5, 10.2)
IMU_AT = (0, 24.6, -55); IMU_TILT = PT.IMU_TILT_DEG
W34, W30, TH = PT.IMU_BOARD; STAND = 4.0

# ── 형상 ──
head = (cq.Workplane("XY").box(*HEAD, centered=(True, True, False)).translate((0, 0, -HEAD[2])).edges("|Z").fillet(HEAD_R))
handle = (cq.Workplane("XY").circle(HANDLE_D / 2).extrude(-HANDLE_L).edges("<Z").fillet(HANDLE_D / 2 * 0.85)
          .rotate((0, 0, 0), (1, 0, 0), HANDLE_TILT).translate(HANDLE_OFF))
ctrl = head.union(handle)

# ── 속 비움 (상단 개구) ──
try:
    body = ctrl.faces(">Z").shell(-WALL)
    print("shell OK")
except Exception as e:
    print("shell 실패 → inner-cut:", e)
    ih = (cq.Workplane("XY").box(HEAD[0]-2*WALL, HEAD[1]-2*WALL, HEAD[2]-WALL, centered=(True, True, False)).translate((0, 0, -HEAD[2])).edges("|Z").fillet(HEAD_R-WALL))
    ihandle = (cq.Workplane("XY").circle(HANDLE_D/2-WALL).extrude(-HANDLE_L).rotate((0, 0, 0), (1, 0, 0), HANDLE_TILT).translate(HANDLE_OFF))
    body = ctrl.cut(ih.union(ihandle))
    body = body.faces(">Z").workplane().rect(HEAD[0]-2*WALL, HEAD[1]-2*WALL).cutBlind(-(HEAD[2]-WALL))

# ── 트리거 피벗 보스 2 + 홀 포켓 ──
for sx in (-8, 8):
    boss = (cq.Workplane("YZ", origin=(sx, C[1], C[2])).circle(4).extrude(2.5 if sx < 0 else -2.5)
            .cut(cq.Workplane("YZ", origin=(sx, C[1], C[2])).circle(PT.TRIG_PIVOT_DIA / 2).extrude(3 if sx < 0 else -3)))
    body = body.union(boss)
body = body.cut(cq.Workplane("XZ", origin=(HALL[0], HALL[1] + 2, HALL[2])).rect(4.6, 3.4).extrude(-3.5))  # AH49E 포켓

# ── 리프트 SS-5GL 포켓 + 레버창 ──
body = body.cut(cq.Workplane("XY", origin=LIFT_AT).box(SS[1] + 0.6, SS[2] + 0.6, SS[0] + 0.6))
body = body.cut(cq.Workplane("XY", origin=(LIFT_AT[0] + 6, LIFT_AT[1], LIFT_AT[2])).box(16, 4, 6))      # 레버 통로(측면)

# ── IMU 스탠드오프 보스 2 (대각 홀) ──
def orient(s):
    return s.rotate((0, 0, 0), (0, 1, 0), -90).rotate((0, 0, 0), (1, 0, 0), IMU_TILT).translate(IMU_AT)
for u, v in PT.IMU_MOUNT_HOLES:
    p = (cq.Workplane("XY", origin=(u - W34 / 2, v - W30 / 2, -TH / 2 - STAND)).circle(2.3).extrude(STAND)
         .faces(">Z").workplane().circle(0.85).cutBlind(STAND + 2))
    body = body.union(orient(p))

cq.exporters.export(body, f"{OUT}/grip_body_v3.step")
cq.exporters.export(body, f"{OUT}/grip_body_v3.stl")
print("grip_body vol:", round(body.val().Volume()))

# ── 렌더: iso + 앞 반단면(cavity) ──
vv, tt = body.val().tessellate(0.4)
V = np.array([(p.x, p.y, p.z) for p in vv]); T = np.array(tt)
m = trimesh.Trimesh(V, T)
fig = plt.figure(figsize=(15, 6.5))
ax = fig.add_subplot(1, 2, 1, projection="3d")
ax.add_collection3d(Poly3DCollection(V[T], facecolor=(0.6, 0.75, 0.9), edgecolor="none", alpha=.6))
c = V.mean(0); r = (V.max(0) - V.min(0)).max() / 2
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(18, -62); ax.set_title("그립 본체①", fontsize=10); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
ax = fig.add_subplot(1, 2, 2, projection="3d")
cent = V[T].mean(1)
ax.add_collection3d(Poly3DCollection(V[T][cent[:, 0] < 1.0], facecolor=(0.6, 0.75, 0.9), edgecolor="none", alpha=.7))
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(8, -88); ax.set_title("X<0 반단면 (cavity·포켓·보스)", fontsize=10); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("V3 Stage2d-1 — 그립 본체①: 셸 + 헤드개구 + 트리거보스/홀 + 리프트포켓 + IMU스탠드오프")
plt.tight_layout(); plt.savefig(f"{OUT}/grip_body_v3.png", dpi=92)
print("saved grip_body_v3.png")

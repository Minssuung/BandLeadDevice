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
IMU_AT = (-4, 24.6, -55); IMU_TILT = PT.IMU_TILT_DEG   # x: -X벽 쪽으로 붙임
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

# ── 트리거 피벗 보스 2 (앞벽 부착) + 핀홀 + 홀 포켓 ──
for sx in (-9, 9):
    body = body.union(cq.Workplane("XY", origin=(sx, C[1], C[2])).box(4, 7, 9))   # 앞벽까지 닿는 블록
body = body.cut(cq.Workplane("YZ", origin=(-12, C[1], C[2])).circle(PT.TRIG_PIVOT_DIA / 2).extrude(24))  # 핀홀 Φ3 (X축)
body = body.cut(cq.Workplane("XZ", origin=(HALL[0], HALL[1] + 2, HALL[2])).rect(4.6, 3.4).extrude(-3.5))  # AH49E 포켓

# ── 리프트 SS-5GL: 벽부착 브래킷 → 포켓 + 레버창 + 나사홀 ──
lbrk = cq.Workplane("XY", origin=(LIFT_AT[0] - 2, LIFT_AT[1], LIFT_AT[2])).box(24, 14, 26).intersect(ctrl)
body = body.union(lbrk)                                                                       # 벽 부착 솔리드
body = body.cut(cq.Workplane("XY", origin=LIFT_AT).box(SS[1] + 0.6, SS[2] + 0.6, SS[0] + 0.6))  # 스위치 포켓
body = body.cut(cq.Workplane("XY", origin=(LIFT_AT[0] + 8, LIFT_AT[1], LIFT_AT[2])).box(18, 4, 6))  # 레버창(측면)
for dz in (-6, 6):                                                                            # 스위치 고정 나사홀
    body = body.cut(cq.Workplane("YZ", origin=(LIFT_AT[0] - SS[1] / 2, LIFT_AT[1], LIFT_AT[2] + dz)).circle(0.9).extrude(-5))

# ── IMU 백킹 플레이트 (-X벽 부착, 손잡이형상 intersect로 벽에 확실히 붙음) + 나사홀2 ──
ca, sa = np.cos(np.radians(IMU_TILT)), np.sin(np.radians(IMU_TILT))
plate = cq.Workplane("XY", origin=(-14, IMU_AT[1], IMU_AT[2])).box(10, 30, 36)   # x -19..-9
plate = plate.rotate(IMU_AT, (IMU_AT[0] + 1, IMU_AT[1], IMU_AT[2]), IMU_TILT)
plate = plate.intersect(ctrl)                                                     # 손잡이 형상으로 트림 → 벽 부착
for u, v in PT.IMU_MOUNT_HOLES:
    ly, lz = v - W30 / 2, u - W34 / 2
    wy = IMU_AT[1] + ly * ca - lz * sa
    wz = IMU_AT[2] + ly * sa + lz * ca
    plate = plate.cut(cq.Workplane("YZ", origin=(IMU_AT[0] - 5, wy, wz)).circle(0.85).extrude(-8))  # M2 파일럿
body = body.union(plate)

# ── 조립 완성용 디테일 ──
# 배선 출구 Ø8 (손잡이 바닥 → 외부 MCU 케이블)
body = body.cut(cq.Workplane("XY", origin=(0, 35, -88)).circle(4.0).extrude(-18))
# 트리거 토션스프링 레그 포스트 (앞벽 부착, 슬롯 밖)
body = body.union(cq.Workplane("XY", origin=(12, C[1], C[2] - 2)).box(2, 7, 6))
# 트리거 레버 통로 슬롯 (앞벽, 레버 폭만 — 보스는 슬롯 밖에 남게)
body = body.cut(cq.Workplane("XY", origin=(0, -27.5, -16)).box(13, 7, 26))

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

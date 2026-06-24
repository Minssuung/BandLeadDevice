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
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
HEAD = (56, 58, 24); HEAD_R = 11
HANDLE_D = 38; HANDLE_L = 112; HANDLE_TILT = 18; HANDLE_OFF = (0, 10, -10)   # 92→112: 트리거 아래 IMU 전용 구간 확보
WALL = 3.0
# 입력 위치 (다른 스크립트와 동일)
C = (0, -25, -9)                 # 트리거 피벗 (캐리어 안 닿게 -9)
HALL = (0, -17, -18)             # AH49E (자석 뒤, 보스 연결바에)
LIFT_AT = (11.5, 17.8, -34); SS = (20.0, 6.5, 10.2)   # +X 내벽(표면17)에 붙되 벽 2mm 남게 x=11.5
IMU_AT = (-4, 31, -75); IMU_TILT = PT.IMU_TILT_DEG   # -X벽, 트리거 아래 전용구간(손잡이 길어져 확보). 트리거(z-60 끝)와 분리
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

# ── 트리거 피벗 보스 2 (앞벽 부착, 길게 z-20..-4) + 핀홀 + 홀 마운트 바 ──
for sx in (-9, 9):
    body = body.union(cq.Workplane("XY", origin=(sx, C[1], -12)).box(4, 7, 16))   # z-20..-4, 앞벽 부착
body = body.cut(cq.Workplane("YZ", origin=(-12, C[1], C[2])).circle(PT.TRIG_PIVOT_DIA / 2).extrude(24))  # 핀홀 Φ3 (X축)
# 홀 마운트 바: 두 보스를 잇고(연결) AH49E 포켓 -Y면. 자석 overshoot/충돌 안 하게 뒤로(앞면 -20.5)
hbar = cq.Workplane("XY", origin=(0, -17.5, HALL[2])).box(22, 6, 6)               # Y-20.5..-14.5, x±11
hbar = hbar.cut(cq.Workplane("XZ", origin=(0, -20.5, HALL[2])).rect(4.6, 3.4).extrude(-3))  # AH49E 포켓(-Y면, 앞 -20.5)
body = body.union(hbar)

# ── 리프트 = 앞면 2단 트리거 (중지) → 작은 택트(6×6) 직접 누름. 손잡이 곡률(18°) 따라 앞면에 배치 (SS-5GL 대체) ──
# 손잡이가 깊을수록 앞면이 +Y로 휘므로, 피벗 기준 18° 틸트해야 레버가 앞면에 붙고 손가락이 닿음
LIFT_PIV = (0, -1, -36)                            # 피벗 (앞면 z-36, front y-1.5), X핀. 곡률 18° 틸트 기준
TACT_AT = (0, 6.8, -50)                            # 택트 중심(레버 뒤·손잡이 안, 틸트 후 실위치) — 참고용


def _lt(wp):                                        # LIFT_PIV 기준 손잡이 곡률(18°) 틸트
    return wp.rotate(LIFT_PIV, (LIFT_PIV[0] + 1, LIFT_PIV[1], LIFT_PIV[2]), HANDLE_TILT)


for sx in (-8, 8):                                                                            # 피벗 보스 2 (intersect로 앞면 밖 안나감)
    body = body.union(_lt(cq.Workplane("XY", origin=(sx, -1, -38)).box(4, 10, 16)).intersect(ctrl))
body = body.cut(_lt(cq.Workplane("YZ", origin=(-11, -1, -36)).circle(PT.TRIG_PIVOT_DIA / 2).extrude(22)))  # 피벗 핀홀 Φ3
body = body.cut(_lt(cq.Workplane("XY", origin=(0, -1, -47)).box(11, 12, 32)))                 # 레버 통로 슬롯(곡률 따라, 앞면 손가락 접근 + 당김 스윙 클리어)
body = body.union(_lt(cq.Workplane("XY", origin=(0, 4, -52)).box(12, 8, 11)).intersect(ctrl))  # 택트 홀더(레버 뒤, 손잡이 안)
body = body.cut(_lt(cq.Workplane("XY", origin=(0, 1.25, -52)).box(6.8, 5, 6.6)))             # 택트 포켓(뒤=택트몸체, 앞=플런저+패드 진입 열림)
body = body.cut(_lt(cq.Workplane("XY", origin=(0, 7, -58)).box(3.5, 4, 6)))                   # 와이어 출구(아래)
body = body.union(_lt(cq.Workplane("XY", origin=(7, 1, -34)).box(2, 3, 5)))                   # 토션스프링 그립 다리 포스트

# ── IMU 백킹 플레이트 (-X벽 부착, 손잡이형상 intersect로 벽에 확실히 붙음) + 나사홀2 ──
ca, sa = np.cos(np.radians(IMU_TILT)), np.sin(np.radians(IMU_TILT))
plate = cq.Workplane("XY", origin=(-14, IMU_AT[1], IMU_AT[2])).box(10, 30, 36)   # x -19..-9 (택트가 작아 충돌 없어 원래 마운트로 복귀)
plate = plate.rotate(IMU_AT, (IMU_AT[0] + 1, IMU_AT[1], IMU_AT[2]), IMU_TILT)
plate = plate.intersect(ctrl)                                                     # 손잡이 형상으로 트림 → 벽 부착
for u, v in PT.IMU_MOUNT_HOLES:
    ly, lz = v - W30 / 2, u - W34 / 2
    wy = IMU_AT[1] + ly * ca - lz * sa
    wz = IMU_AT[2] + ly * sa + lz * ca
    plate = plate.cut(cq.Workplane("YZ", origin=(IMU_AT[0] - 5, wy, wz)).circle(0.85).extrude(-8))  # M2 파일럿
body = body.union(plate)

# ── 허브 만능보드 트레이 (그립 하부; 스커트(-14)·조이스틱모듈(-13) 아래, 홀바 뒤) ──
# 보드 44×36 @ (0,5). 받침 ledge(z-16.85) + 위립(z-15.1)으로 Z 샌드위치(흔들림 방지). 틸트인 삽입(앞부터 넣고 뒤 내림)
for (ox, oy, w, d) in [(-23, 4.5, 6, 35), (23, 4.5, 6, 35), (0, 23, 48, 6)]:
    led = cq.Workplane("XY", origin=(ox, oy, -17.45)).box(w, d, 1.2).intersect(ctrl)    # 받침 1.2(top -16.85 동일, 아래로 두껍게-프린트가능)
    lip = cq.Workplane("XY", origin=(ox, oy, -14.5)).box(w, d, 1.2).intersect(ctrl)     # 위립 1.2(bottom -15.1 동일, 위로 두껍게)
    body = body.union(led).union(lip)

# ── 조립 완성용 디테일 ──
# 배선 출구 Ø8 (실제 손잡이축 통해 바닥중심(0,35,-96.5) 관통 — 원점회전 드리프트 방지)
wire = (cq.Workplane("XY", origin=(0, 0, -15)).circle(4.0).extrude(30)
        .rotate((0, 0, 0), (1, 0, 0), HANDLE_TILT).translate((0, 43, -111)))
body = body.cut(wire)
# 트리거 토션스프링(0.4×ID3.2 일자) — 코일은 핀 x5~7 틈에, 그립 다리는 +X보스 위 포스트에 (슬롯·레버 밖, 조이스틱 앞)
body = body.union(cq.Workplane("XY", origin=(7.5, -22, C[2] + 2.75)).box(2, 3, 4.5))   # 그립 다리 포스트 (x6.5..8.5, 보스 부착, z-9..-4.5)
# 트리거 레버 통로 슬롯 (앞벽, +Y로 더 확장해 당김 전구간 클리어)
body = body.cut(cq.Workplane("XY", origin=(0, -26, -16)).box(13, 10, 26))

# ── 클램쉘: 시임(x=0) 나사보스(축X) — 우=파일럿(M2셀프탭), 좌=클리어. 부품 회피 시임벽 지점 ──
# (y,z): 정크션뒤 + 핸들하부 앞·뒤. 헤드=캐리어가, 앞상부=트리거핀이 체결. IMU구간 회피
# 나사 다리 = 벽~벽 솔리드 bridge(cavity 가로지름) → 나사가 솔리드 안으로만 지나감(노출X). 좌 외부 카운터싱크(머리 flush)
# 벽쪽(앞·뒤) 위치 — 가운데는 케이블 길로 비움. 나사를 Ø7 솔리드 튜브로 감싸 cavity 노출 없음. 정크션뒤+하부 앞·뒤
SEAM_BOSS = [(31, -28), (24, -103), (58, -103)]   # 하부2는 길어진 손잡이 바닥 근처로 내림(IMU z-75 아래)
for (by, bz) in SEAM_BOSS:
    tube = cq.Workplane("YZ", origin=(0, by, bz)).circle(3.5).extrude(40, both=True).intersect(ctrl)  # Ø7 솔리드 튜브(벽~벽, 나사 감쌈)
    body = body.union(tube)
    body = body.cut(cq.Workplane("YZ", origin=(0, by, bz)).circle(0.85).extrude(8))       # 우 파일럿 Ø1.7 블라인드(x0..8만, 오른쪽 바깥은 막힘)
    body = body.cut(cq.Workplane("YZ", origin=(0, by, bz)).circle(1.5).extrude(-22))      # 좌 클리어 Ø3 (튜브 안, 옆면 외부까지 = 나사 입구)
    body = body.cut(cq.Workplane("YZ", origin=(-6, by, bz)).circle(2.4).extrude(-24))     # 좌 외부 카운터싱크 Ø4.8 (머리 묻힘)

cq.exporters.export(body, f"{OUT}/grip_body_v3.step")
cq.exporters.export(body, f"{OUT}/grip_body_v3.stl")   # 조립체(보스O, 창X) — assemble이 창 자른 뒤 좌우 분리함
print("grip_body vol:", round(body.val().Volume()))
# 좌우 분리는 assemble_v3.py에서 창(캐리어 스냅) 자른 뒤 수행 → 출력용 반쪽에 창 포함됨

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

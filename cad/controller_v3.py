#!/usr/bin/env python3
"""V3 파라메트릭 컨트롤러 — Stage1: 전체 형상(헤드+손잡이) + 모든 입력 배치.
깨끗한 솔리드로 재설계. 모든 버튼 메커니즘이 얹힐 기준 형상/좌표 확정용.
좌표: 헤드 윗면=z0, +X 오른쪽, +Y 뒤(손목쪽), -Y 앞(트리거쪽), Z 위.
실행: cad/.venv/bin/python cad/controller_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"

# ── 전체 치수 (mm) ──
HEAD = (56, 58, 24)            # 헤드 X×Y×Z (엄지 입력부)
HEAD_R = 11                     # 헤드 모서리 라운드
HANDLE_D = 38                   # 손잡이 단면 지름 (IMU 30mm폭 + 벽 수납 위해 34→38)
HANDLE_L = 92                   # 손잡이 길이
HANDLE_TILT = 18               # 손잡이 뒤로 기울임(°)
HANDLE_OFF = (0, 10, -10)      # 손잡이 부착 위치(헤드 하부 뒤쪽)

# ── 입력 배치 (헤드 윗면 z=0 기준 XY) ──
JOY = (0, -9)                   # 조이스틱(엄지, 앞쪽)
BTN = {"A": (-20, -2), "B": (-19, 13), "menu": (16, 16)}   # 택트(엄지)
TRIG_AT = (0, -HEAD[1] / 2 + 2, -15)    # 트리거(검지) — 헤드 앞면 하단
LIFT_AT = (HANDLE_D / 2 + 6, 16, -34)   # 리프트(중지) — 손잡이 측면
IMU_AT = (0, 14, -64)                    # IMU 크래들 — 손잡이 하부

# ── 헤드 ──
head = (cq.Workplane("XY").box(HEAD[0], HEAD[1], HEAD[2], centered=(True, True, False))
        .translate((0, 0, -HEAD[2])).edges("|Z").fillet(HEAD_R))

# ── 손잡이 (기울인 캡슐, 바닥 라운드) ──
handle = (cq.Workplane("XY").circle(HANDLE_D / 2).extrude(-HANDLE_L)
          .edges("<Z").fillet(HANDLE_D / 2 * 0.85))
handle = handle.rotate((0, 0, 0), (1, 0, 0), HANDLE_TILT).translate(HANDLE_OFF)

ctrl = head.union(handle)
cq.exporters.export(ctrl, f"{OUT}/controller_v3.step")
print("controller_v3 vol:", round(ctrl.val().Volume()), "bbox:", [round(x, 0) for x in ctrl.val().BoundingBox().DiagonalLength * np.array([1])])
bb = ctrl.val().BoundingBox()
print(f"bbox X[{bb.xmin:.0f},{bb.xmax:.0f}] Y[{bb.ymin:.0f},{bb.ymax:.0f}] Z[{bb.zmin:.0f},{bb.zmax:.0f}]")

# ── 렌더 (형상 + 입력 위치 마커) ──
vv, tt = ctrl.val().tessellate(0.4)
P = np.array([(p.x, p.y, p.z) for p in vv]); T = np.array(tt)
marks = [("stick", (*JOY, 0), "green"), *[(k, (*v, 0), "red") for k, v in BTN.items()],
         ("trigger", TRIG_AT, "blue"), ("lift", LIFT_AT, "purple"), ("IMU", IMU_AT, "orange")]
fig = plt.figure(figsize=(18, 5.4))
for k, (el, az, ttl) in enumerate([(22, -60, "iso"), (4, -90, "front -Y (트리거면)"), (4, 0, "side +X (리프트면)"), (88, -90, "top (엄지면)")]):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(P[T], facecolor=(0.6, 0.75, 0.9), edgecolor="none", alpha=0.5))
    for nm, pos, col in marks:
        ax.scatter(*[[c] for c in pos], color=col, s=30)
        ax.text(*pos, nm, color=col, fontsize=7)
    c = P.mean(0); rng = (P.max(0) - P.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl, fontsize=9); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("V3 컨트롤러 Stage1 — 형상 + 입력배치 (엄지:스틱+A/B/menu · 검지:트리거 · 중지:리프트)")
plt.tight_layout(); plt.savefig(f"{OUT}/controller_v3.png", dpi=92)
print("saved controller_v3.png")

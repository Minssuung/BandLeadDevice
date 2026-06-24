#!/usr/bin/env python3
"""파트 ③ 트리거 모듈 — 검지 트리거 레버 + 홀센서 마운트 (파라메트릭).
레버가 Φ3 피벗으로 회전, 끝쪽 자석(Φ5×2)이 하우징 고정 AH49E 앞을 1~3mm 갭으로 지나감.
실행: cad/.venv/bin/python cad/trigger.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
piv = PT.TRIG_PIVOT_DIA              # 3 (M3 핀)
md, mt = PT.MAGNET_DIA, PT.MAGNET_THK   # 5, 2
gap = sum(PT.TRIG_GAP) / 2           # (1,3) → 2mm
clr = PT.FDM_CLEAR                   # 0.3

# --- 레버(움직이는 부품) ---
L, Wy, Tz = 30.0, 9.0, 7.0           # 길이/폭(Y,피벗축)/두께(Z)
PIVOT_X = 6.0                        # 피벗 위치
MAG_X = PIVOT_X + 8.0                # 자석 = 피벗서 8mm
lever = cq.Workplane("XY").box(L, Wy, Tz, centered=(False, True, True))   # x:0..L
# 피벗 보어 Φ(3+clr) — Y축 관통
pivbore = (cq.Workplane("XZ", origin=(PIVOT_X, -Wy, 0)).circle((piv + clr) / 2).extrude(2 * Wy))
lever = lever.cut(pivbore)
# 자석 포켓 Φ(5+0.2)×(2+0.2) — 윗면(+Z)
mag = (cq.Workplane("XY", origin=(MAG_X, 0, Tz / 2)).circle((md + 0.2) / 2).extrude(-(mt + 0.2)))
lever = lever.cut(mag)
# 토션스프링 레그 앵커 Φ1.3 (피벗 옆)
leg = (cq.Workplane("XZ", origin=(PIVOT_X, -Wy, -2.4)).circle(0.65).extrude(2 * Wy))
lever = lever.cut(leg)
# 핑거 끝 라운드
lever = lever.edges("|Y and >X").fillet(2.0)
cq.exporters.export(lever, f"{OUT}/trigger_lever.step")
cq.exporters.export(lever, f"{OUT}/trigger_lever.stl")
print("lever vol:", round(lever.val().Volume()))

# --- 홀센서 마운트(하우징 고정) ---
BW, BD, BT = 9.0, Wy, 4.0            # 마운트 블록
BZ0 = Tz / 2 + gap                   # 레버 윗면 + 갭 = 블록 밑면
mount = cq.Workplane("XY").box(BW, BD, BT, centered=(True, True, False)).translate((MAG_X, 0, BZ0))
# AH49E 포켓(4.4×3.2×3) — 자석 마주보는 밑면
hall = (cq.Workplane("XY", origin=(MAG_X, 0, BZ0)).rect(4.4, 3.2).extrude(3.0))
mount = mount.cut(hall)
# 하우징 고정용 M2 관통홀 ×2
for sx in (-BW / 2 + 1.6, BW / 2 - 1.6):
    mount = mount.faces(">Z").workplane().moveTo(MAG_X + sx, 0).circle(1.1).cutThruAll()
cq.exporters.export(mount, f"{OUT}/trigger_hallmount.step")
cq.exporters.export(mount, f"{OUT}/trigger_hallmount.stl")
print("hallmount vol:", round(mount.val().Volume()))

# --- 렌더 (레버 파랑 + 마운트 주황 + 갭/피벗 표시) ---
def tess(wp, tol=0.3):
    vv, tt = wp.val().tessellate(tol)
    return np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt)
LV, LT = tess(lever); MVm, MTm = tess(mount)
allp = np.vstack([LV, MVm])
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(18, -60, "iso"), (0, 0, "side +X (회전평면)"), (90, -90, "top")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(LV[LT], facecolor=(0.35, 0.6, 0.9), edgecolor="k", linewidths=.1, alpha=.95))
    ax.add_collection3d(Poly3DCollection(MVm[MTm], facecolor=(0.95, 0.6, 0.2), edgecolor="k", linewidths=.1, alpha=.9))
    ax.plot([PIVOT_X, PIVOT_X], [-Wy, Wy], [0, 0], "r-", lw=2)          # 피벗축
    ax.text(PIVOT_X, 0, -Tz, "pivot Φ3", color="red", fontsize=7)
    ax.text(MAG_X, 0, Tz / 2 + gap / 2, f"gap {gap:.0f}", color="green", fontsize=7)
    ax.text(MAG_X, 0, Tz / 2, "Φ5×2 자석", color="blue", fontsize=7)
    ax.text(MAG_X, 0, BZ0 + BT, "AH49E", color="darkorange", fontsize=7)
    c = allp.mean(0); rng = (allp.max(0) - allp.min(0)).max() / 2 + 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("파트3 트리거 모듈 v1 — 레버(피벗Φ3·자석Φ5×2) + 홀마운트(AH49E, 갭2mm)")
plt.tight_layout(); plt.savefig(f"{OUT}/trigger.png", dpi=88)
print("saved trigger.png")

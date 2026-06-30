#!/usr/bin/env python3
"""리프트 = 앞면 2단 트리거 레버 (중지). 당기면 뒤 nub이 작은 택트(6×6) 누름. 토션스프링이 레버 복귀(Meta Quest 그립버튼식).
실행: cad/.venv/bin/python cad/trigger_lift_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
from trimesh import transformations as TF
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
C = np.array([0.0, 5.0, -32.0])      # 피벗 (그립 LIFT_PIV와 동일, 손잡이 안쪽 깊은곳), 축 X
SS = np.array([0.0, 8.0, -50.0])     # SS-5GL 중심 (그립 SS_AT, 피벗 아래). 액추에이터 -Y(레버쪽)
PULL = 6.0                           # 당김 각

# ── 레버 (피벗 안쪽, 날 앞으로). 허브 가운데 갭에 토션스프링 코일을 가둠(케이지). 18° 틸트 ──
COIL_LEN = 4.8                        # 토션스프링 코일 축방향 두께(실측)
GAP = COIL_LEN + 0.4                  # 케이지 갭 5.2 (코일+여유)
FL = 4.4                             # 양 플랜지 두께 (넓혀 보스까지 0.5mm 남기고 공간 채움 — 축방향 유격 제거)
HW = GAP / 2 + FL                     # 허브 반폭 7.0 → 허브 x-7.0..7.0 (보스 안쪽 x7.5까지 0.5)
hub = cq.Workplane("YZ", origin=(-HW, C[1], C[2])).circle(4).extrude(2 * HW)           # Ø8 허브, x-7..7
arm = cq.Workplane("XY", origin=(0, 0, -45)).box(6, 4, 28)                             # 중지 패드 날 x±3 (앞 브리지가 양 플랜지 연결), y-2..2, z-59..-31
lever = hub.union(arm)
# 코일 케이지 갭: 허브 가운데(x±2.6) 뒤쪽 개방 포켓 — 코일이 핀 감고 두 플랜지 사이에 갇힘. 앞 브리지(y<2.5)+날이 양쪽 연결
lever = lever.cut(cq.Workplane("XY", origin=(0, C[1] + 2, C[2])).box(GAP, 9, 6))       # x±2.6, y2.5..11.5, z-35..-29 (코일 자리, 뒤로 개방)
lever = lever.cut(cq.Workplane("YZ", origin=(-7.5, C[1], C[2])).circle((PT.TRIG_PIVOT_DIA + 0.4) / 2).extrude(15))  # 피벗 보어 Φ3.4(x-7.5..7.5, 넓은 허브 관통)
lever = lever.edges("|X and <Z").fillet(1.5)
# 토션스프링 레버다리 슬롯: 갭 가운데(중앙정렬) 날-윗면에 코일 다리(아래로) 키 — 슬롯 벽이 토크 받음(코일은 케이지가 축방향 잡음)
lever = lever.cut(cq.Workplane("XY", origin=(0, C[1] - 3.5, C[2] - 4.5)).box(1.8, 4, 2.2))  # x±0.9 중앙, y앞아래, z핀아래(레버다리 끼움)
lever = lever.rotate((C[0], C[1], C[2]), (C[0] + 1, C[1], C[2]), 18)                   # 손잡이 곡률 따라 18° 틸트
cq.exporters.export(lever, f"{OUT}/trigger_lift_v3.stl")
cq.exporters.export(lever, f"{OUT}/trigger_lift_v3.step")   # Fusion 편집용
print("lift lever vol:", round(lever.val().Volume()))


def mesh(wp, tol=0.3):
    vv, tt = wp.val().tessellate(tol)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt))

lm = mesh(lever)
g = trimesh.load(f"{OUT}/grip_body_v3.stl")
# ── 검증: 당김 전구간 그립과 간섭(걸림) + nub이 택트 누르나 ──
print("=== 리프트 레버 스윙 vs 그립 ===")
for deg in [0, 4, 8, 12]:
    lr = lm.copy(); lr.apply_transform(TF.rotation_matrix(np.radians(deg), [1, 0, 0], C))
    it = trimesh.boolean.intersection([g, lr], engine="manifold")
    v = 0 if (it is None or it.is_empty) else it.volume
    print(f"  {deg:2d}°: 그립간섭 {round(v)} mm³")
# nub 끝(뒤) y, SS 버튼 y1.6과 비교
nub_rest = lm.bounds[1][1]   # 레버 max y (nub 뒤끝)
nr = lm.copy(); nr.apply_transform(TF.rotation_matrix(np.radians(PULL), [1, 0, 0], C))
nub_pull = nr.bounds[1][1]
print(f"nub 뒤끝 y: rest {nub_rest:.1f} → pull {nub_pull:.1f} (택트 플런저 y≈4, pull때 닿아야)")

# ── 렌더 ──
fig = plt.figure(figsize=(14, 6))
ax = fig.add_subplot(1, 2, 1)
s = g.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
if s:
    for e in s.entities:
        pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], color=(.7, .7, .7), lw=.8)
for m, col, lab in [(lm, "royalblue", "rest"), (nr, "orange", f"pull {PULL:.0f}°")]:
    sl = m.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if sl:
        for e in sl.entities:
            pp = sl.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], col, lw=1.4)
        ax.plot([], [], col, label=lab)
ax.add_patch(plt.Rectangle((SS[1] - 3, SS[2] - 3), 6, 6, fill=False, ec="green", lw=1.3))   # 택트 6×6
ax.text(SS[1] + 2, SS[2] + 6, "택트(6×6)", color="green", fontsize=8, ha="center")
ax.scatter(*C[1:], c="k", s=25); ax.text(C[1], C[2] + 1, "피벗", fontsize=8)
ax.set_aspect("equal"); ax.set_xlabel("Y(앞←)"); ax.set_ylabel("Z(위)"); ax.invert_xaxis()
ax.set_title("리프트 2단 트리거 (당기면 nub이 택트 누름)", fontsize=10); ax.legend(fontsize=8); ax.grid(alpha=.3)
ax = fig.add_subplot(1, 2, 2, projection="3d")
ax.add_collection3d(Poly3DCollection(g.vertices[g.faces], facecolor=(.8, .85, .9), edgecolor="none", alpha=.12))
ax.add_collection3d(Poly3DCollection(lm.vertices[lm.faces], facecolor=(.3, .5, .95), edgecolor="none", alpha=.95))
c = np.array([0, 0, -42]); r = 22
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(14, -72); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("리프트 2단 트리거 (중지) — 택트(6×6) 누름, Meta Quest 그립버튼식")
plt.tight_layout(); plt.savefig(f"{OUT}/trigger_lift_v3.png", dpi=92); print("saved trigger_lift_v3.png")

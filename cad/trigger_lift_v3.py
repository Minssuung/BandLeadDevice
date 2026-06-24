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
C = np.array([0.0, -1.0, -36.0])     # 피벗 (그립 LIFT_PIV와 동일), 축 X
SS = np.array([0.0, 6.8, -50.0])     # 택트(6×6) 중심 (그립 TACT_AT, 틸트 후). 플런저 -Y(레버쪽)
PULL = 6.0                           # 당김 각 (택트 트래블 작아 실사용 ~5° 내)

# ── 레버 (직접식: nub 없이 패드 뒷면이 택트 직접 누름). 수직으로 만들고 18° 틸트 → 손잡이 앞면 곡률 따라감 ──
hub = cq.Workplane("YZ", origin=(-5, C[1], C[2])).circle(4).extrude(7)                 # 허브 x-5..2 (+X쪽 줄여 토션스프링 코일 자리)
arm = cq.Workplane("XY", origin=(0, -3, -49)).box(6, 4, 24)                            # 중지 패드 암 (피벗서 수직, y-5..-1, x±3=택트포켓에 맞춤)
lever = hub.union(arm)
lever = lever.cut(cq.Workplane("YZ", origin=(-6, C[1], C[2])).circle((PT.TRIG_PIVOT_DIA + 0.4) / 2).extrude(12))  # 피벗 보어 Φ3.4
lever = lever.edges("|X and <Z").fillet(1.5)
lever = lever.rotate((C[0], C[1], C[2]), (C[0] + 1, C[1], C[2]), 18)                   # 손잡이 곡률 따라 18° 틸트
cq.exporters.export(lever, f"{OUT}/trigger_lift_v3.stl")
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

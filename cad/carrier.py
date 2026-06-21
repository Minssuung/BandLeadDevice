#!/usr/bin/env python3
"""파트 ② 헤드 캐리어 — 그립 캐비티 맞춤 플레이트 + 부품 하우징(파라메트릭).
그립 단면(z=Zc)을 따 외벽-5mm 인셋 → 플레이트 → KY-023 돔홀+마운트홀 + 택트 포켓.
실행: cad/.venv/bin/python cad/carrier.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy.spatial import ConvexHull
import open3d as o3d
import cadquery as cq
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
Zc = -4.0       # 캐리어 평면 (WIP: z=-4 캐비티 맞춤)
TH = 3.5        # 플레이트 두께
INSET = PT.WALL # 외벽 -5mm

# 1) 그립 단면 외형 (z≈Zc 점들의 볼록외곽)
V = np.asarray(o3d.io.read_triangle_mesh(f"{OUT}/grip_solid.stl").vertices)
band = V[np.abs(V[:, 2] - Zc) < 3.0][:, :2]
hull = band[ConvexHull(band).vertices]
pts = [(float(x), float(y)) for x, y in hull]
print("outline pts:", len(pts))

# 2) 플레이트 = 외형 인셋 후 압출
plate = (cq.Workplane("XY", origin=(0, 0, Zc))
         .polyline(pts).close().offset2D(-INSET).extrude(TH))
print("plate vol:", plate.val().Volume())

# 3) 부품 하우징
jx, jy = PT.JOY_POS
gx, gy = PT.JOY_MOUNT_GRID
# KY-023 돔 클리어런스 (Ø18 관통)
plate = plate.faces(">Z").workplane().moveTo(jx, jy).circle(PT.JOY_DOME_DIA / 2).cutThruAll()
# KY-023 마운트홀 27×20 그리드 (M2: Ø2.2)
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        plate = plate.faces(">Z").workplane().moveTo(jx + dx, jy + dy).circle(1.1).cutThruAll()
# 택트 6.4□ 포켓 (A/B/메뉴) 관통
for name, (bx, by) in PT.BTN_POS.items():
    plate = plate.faces(">Z").workplane().moveTo(bx, by).rect(PT.TACT_POCKET, PT.TACT_POCKET).cutThruAll()

cq.exporters.export(plate, f"{OUT}/head_carrier.step")
cq.exporters.export(plate, f"{OUT}/head_carrier.stl")
print("exported head_carrier.step/.stl")

# 4) 렌더 (캐리어 파랑 + 부품위치 마커 + 그립 점군 회색 컨텍스트)
vv, tt = plate.val().tessellate(0.3)
PP = np.array([(p.x, p.y, p.z) for p in vv]); TT = np.array(tt)
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(70, -90, "top"), (18, -65, "iso"), (8, -90, "front")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.scatter(V[::20, 0], V[::20, 1], V[::20, 2], s=0.3, color=(.8, .8, .8), alpha=.12)
    ax.add_collection3d(Poly3DCollection(PP[TT], facecolor=(0.35, 0.6, 0.9), edgecolor="k", linewidths=.1, alpha=.95))
    for nm, (bx, by) in PT.BTN_POS.items():
        ax.text(bx, by, Zc + TH, nm, color="red", fontsize=8)
    ax.text(jx, jy, Zc + TH, "stick", color="green", fontsize=8)
    c = PP.mean(0); rng = (PP.max(0) - PP.min(0)).max() / 2 + 5
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(Zc-30, Zc+30)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("파트2 헤드캐리어 v1 (KY-023 돔홀+마운트 / 택트 포켓 3)")
plt.tight_layout(); plt.savefig(f"{OUT}/head_carrier.png", dpi=88)
print("saved head_carrier.png")

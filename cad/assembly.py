#!/usr/bin/env python3
"""스냅핏 조립 v1 — 캐리어 ②(플랜지+스커트+훅) 가 그립 ① 헤드 개구부에 드롭인,
스커트 훅이 그립 림의 창(window)에 걸림. 그립에 창 4개 뚫고 조립 렌더.
실행: cad/.venv/bin/python cad/assembly.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import cadquery as cq
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
Zc = -4.0
SLIDE = PT.FDM_CLEAR          # 0.3 슬라이드 클리어런스
INSET_HOLE = 4.0              # grip_shell 개구 인셋과 동일

gm = trimesh.load(f"{OUT}/grip_capped.stl")
V = gm.vertices
zmax = float(V[:, 2].max())
band = V[np.abs(V[:, 2] - Zc) < 3.0][:, :2]
H = band[ConvexHull(band).vertices]
P = Polygon(H)
print(f"rim_top(zmax)={zmax:.1f} | hull area={P.area:.0f}")


def ring(poly):
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    cs = np.array(poly.exterior.coords)[:-1]
    return [(float(x), float(y)) for x, y in cs]


# 아웃라인: 플랜지>개구>스커트
flange_out = P.buffer(-1.0)
skirt_out = P.buffer(-(INSET_HOLE + SLIDE))   # 개구보다 작아 들어감
skirt_in = skirt_out.buffer(-2.0)             # 스커트 벽 2mm
FT = 3.0                                       # 플랜지 두께
SK = (zmax - FT) - (Zc + 1.0)                  # 스커트 길이 → z=-4 근처까지
skirt_top = zmax - FT

# 캐리어 = 플랜지 + 스커트벽
carrier = cq.Workplane("XY", origin=(0, 0, skirt_top)).polyline(ring(flange_out)).close().extrude(FT)
wall = (cq.Workplane("XY", origin=(0, 0, skirt_top)).polyline(ring(skirt_out)).close().extrude(-SK)
        .cut(cq.Workplane("XY", origin=(0, 0, skirt_top + 0.5)).polyline(ring(skirt_in)).close().extrude(-SK - 1)))
carrier = carrier.union(wall)

# 버튼/스틱 컷 (플랜지)
jx, jy = PT.JOY_POS; gx, gy = PT.JOY_MOUNT_GRID
carrier = carrier.faces(">Z").workplane().moveTo(jx, jy).circle(PT.JOY_DOME_DIA / 2).cutThruAll()
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        carrier = carrier.faces(">Z").workplane().moveTo(jx + dx, jy + dy).circle(1.1).cutThruAll()
for nm, (bx, by) in PT.BTN_POS.items():
    carrier = carrier.faces(">Z").workplane().moveTo(bx, by).rect(PT.TACT_POCKET, PT.TACT_POCKET).cutThruAll()

# 스냅 훅 위치 = 스커트 4변 중점, 훅 z = 바닥 근처
cx, cy = float(P.centroid.x), float(P.centroid.y)
minx, miny, maxx, maxy = skirt_out.bounds
HOOK_Z = Zc + 3.0
LIP = 1.4; HKW = 6.0; HKH = 2.5
hooks = [(maxx, cy, +1, 0), (minx, cy, -1, 0), (cx, maxy, 0, +1), (cx, miny, 0, -1)]
for (hx, hy, dx, dy) in hooks:
    # 바깥쪽으로 돌출하는 립
    if dx:
        lip = cq.Workplane("XY", origin=(hx, hy, HOOK_Z)).box(LIP * 2, HKW, HKH, centered=(True, True, False)).translate((dx * LIP, 0, 0))
    else:
        lip = cq.Workplane("XY", origin=(hx, hy, HOOK_Z)).box(HKW, LIP * 2, HKH, centered=(True, True, False)).translate((0, dy * LIP, 0))
    carrier = carrier.union(lip)
cq.exporters.export(carrier, f"{OUT}/carrier_v2.step")
cq.exporters.export(carrier, f"{OUT}/carrier_v2.stl")
print("carrier_v2 vol:", round(carrier.val().Volume()))

# 그립 림에 창(window) 4개 뚫기 (boolean)
grip = trimesh.load(f"{OUT}/grip_shell.stl")
boxes = []
for (hx, hy, dx, dy) in hooks:
    b = trimesh.creation.box(extents=(HKW + 2, HKW + 2, HKH + 2))
    b.apply_translation([hx + dx * 2.5, hy + dy * 2.5, HOOK_Z + HKH / 2])
    boxes.append(b)
grip_win = trimesh.boolean.difference([grip] + boxes, engine="manifold")
grip_win.export(f"{OUT}/grip_assembled.stl")
print("grip+windows watertight:", grip_win.is_watertight, "vol:", round(grip_win.volume))

# 렌더 (그립 반투명 초록 + 캐리어 파랑)
cvv, ctt = carrier.val().tessellate(0.3)
CV = np.array([(p.x, p.y, p.z) for p in cvv]); CT = np.array(ctt)
GV, GT = grip_win.vertices, grip_win.faces
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(20, -60, "iso"), (4, -90, "front (드롭인)"), (4, 0, "side")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(GV[GT], facecolor=(0.55, 0.8, 0.6), edgecolor="none", alpha=0.25))
    ax.add_collection3d(Poly3DCollection(CV[CT], facecolor=(0.3, 0.55, 0.95), edgecolor="k", linewidths=.1, alpha=.95))
    allp = np.vstack([GV, CV]); c = allp.mean(0); rng = (allp.max(0) - allp.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("스냅핏 조립 v1 — 그립(초록 반투명) + 캐리어(파랑) 드롭인+훅")
plt.tight_layout(); plt.savefig(f"{OUT}/assembly.png", dpi=90)
print("saved assembly.png")

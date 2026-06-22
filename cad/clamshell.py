#!/usr/bin/env python3
"""클램셸 조립 — PART_Z(-2)에서 상/하 분할.
캐리어②=실제 돔 윗면 캡 + 하향 립훅(캔틸레버) + 버튼홀. 그립①=하단 몸통 shell + 창.
훅이 그립 벽(창 위)에 걸려 잠김. 조립렌더 + 잠김 수치검증.
실행: cad/.venv/bin/python cad/clamshell.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import cadquery as cq
import pymeshfix
from scipy import ndimage
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, LineString
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
PART_Z = -2.0
WALL = 3.0
CLR = PT.FDM_CLEAR            # 0.3
PITCH = 0.6
# 훅/창 (PART_Z 아래)
HKW = 6.0; LIP_OUT = 2.5; SLOT_W = 1.2
LIP_LEN = 9.0                                  # 립벽 길이 (PART_Z → PART_Z-9)
CATCH_Z = PART_Z - 4.5                          # 립 캐치 평평윗면
LIP_BOT = PART_Z - 7.5                          # 립 아래(램프)
WIN_BOT, WIN_TOP = PART_Z - 8.0, PART_Z - 4.0   # 창 (윗변 위로 그립벽=캐치)


def repair(m):
    if not m.is_volume:
        v, f = pymeshfix.clean_from_arrays(np.asarray(m.vertices), np.asarray(m.faces))
        m = trimesh.Trimesh(v, f); trimesh.repair.fix_normals(m)
    return m


gm = trimesh.load(f"{OUT}/grip_capped.stl")
V = gm.vertices
zmax = float(V[:, 2].max())
band = V[np.abs(V[:, 2] - PART_Z) < 1.5][:, :2]
Pz = Polygon(band[ConvexHull(band).vertices])
cx, cy = float(Pz.centroid.x), float(Pz.centroid.y)
lip_out = Pz.buffer(-(WALL + CLR))           # 립벽 외곽(그립 개구 안쪽) — 훅 기준
lip_in = lip_out.buffer(-2.0)


def ring(poly):
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    return [(float(x), float(y)) for x, y in np.array(poly.exterior.coords)[:-1]]


def bnd(dx, dy):
    it = LineString([(cx, cy), (cx + dx * 1e3, cy + dy * 1e3)]).intersection(lip_out.boundary)
    pt = max(it.geoms, key=lambda p: (p.x - cx) * dx + (p.y - cy) * dy) if it.geom_type == "MultiPoint" else it
    return float(pt.x), float(pt.y)


hooks = [(*bnd(dx, dy), dx, dy) for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]]
print("PART_Z", PART_Z, "| hooks", [(round(h[0], 1), round(h[1], 1)) for h in hooks])

# ===== 그립 ① = 하단 몸통 shell + 창 =====
vg = gm.voxelized(PITCH).fill()
occ = np.asarray(vg.matrix); T = vg.transform
o, s = T[:3, 3], np.array([T[0, 0], T[1, 1], T[2, 2]])
nx, ny, nz = occ.shape
wv = max(1, int(round(WALL / PITCH)))
shell = occ & ~ndimage.binary_erosion(occ, iterations=wv)
K = np.arange(nz)
WZ = o[2] + s[2] * K
shell[:, :, WZ > PART_Z] = False                 # 윗부분 잘라 top 개구
grip = trimesh.voxel.ops.matrix_to_marching_cubes(shell); grip.apply_transform(T)
trimesh.smoothing.filter_taubin(grip, iterations=10)
wins = []
for (hx, hy, dx, dy) in hooks:
    ext = (8, HKW + 1.5, WIN_TOP - WIN_BOT) if dx else (HKW + 1.5, 8, WIN_TOP - WIN_BOT)
    b = trimesh.creation.box(extents=ext); b.apply_translation([hx, hy, (WIN_BOT + WIN_TOP) / 2]); wins.append(b)
grip = trimesh.boolean.difference([repair(grip)] + wins, engine="manifold")
grip.export(f"{OUT}/grip_assembled.stl")
print("그립① watertight:", grip.is_watertight, "vol:", round(grip.volume))

# ===== 캐리어 ② = 돔 윗면 캡 + 립훅 + 버튼홀 =====
topbox = trimesh.creation.box(extents=(140, 180, (zmax - PART_Z) + 20))
topbox.apply_translation([cx, cy, PART_Z + ((zmax - PART_Z) + 20) / 2])
cap = trimesh.boolean.intersection([repair(gm), topbox], engine="manifold")
print("캡 watertight:", cap.is_watertight, "z:", round(cap.bounds[0][2], 1), "~", round(cap.bounds[1][2], 1))

# 립벽 + 캔틸레버 훅 (cadquery)
lipwall = (cq.Workplane("XY", origin=(0, 0, PART_Z)).polyline(ring(lip_out)).close().extrude(-LIP_LEN)
           .cut(cq.Workplane("XY", origin=(0, 0, PART_Z + 0.5)).polyline(ring(lip_in)).close().extrude(-LIP_LEN - 1)))


def make_lip(hx, hy, dx, dy):
    if dx:
        b = cq.Workplane("XY", origin=(hx + dx * LIP_OUT / 2, hy, (LIP_BOT + CATCH_Z) / 2)).box(LIP_OUT, HKW, CATCH_Z - LIP_BOT)
        oe = hx + dx * LIP_OUT
        cutter = cq.Workplane("XY", origin=(oe, hy, LIP_BOT)).box(LIP_OUT * 2.2, HKW * 1.3, LIP_OUT * 2.2).rotate((oe, hy - 1, LIP_BOT), (oe, hy + 1, LIP_BOT), 45)
    else:
        b = cq.Workplane("XY", origin=(hx, hy + dy * LIP_OUT / 2, (LIP_BOT + CATCH_Z) / 2)).box(HKW, LIP_OUT, CATCH_Z - LIP_BOT)
        oe = hy + dy * LIP_OUT
        cutter = cq.Workplane("XY", origin=(hx, oe, LIP_BOT)).box(HKW * 1.3, LIP_OUT * 2.2, LIP_OUT * 2.2).rotate((hx - 1, oe, LIP_BOT), (hx + 1, oe, LIP_BOT), 45)
    return b.cut(cutter)


for (hx, hy, dx, dy) in hooks:
    # 슬롯으로 텅 자유화
    H = (PART_Z + 0.5) - (LIP_BOT - 2)
    if dx:
        for sy in (hy - HKW / 2 - SLOT_W / 2, hy + HKW / 2 + SLOT_W / 2):
            lipwall = lipwall.cut(cq.Workplane("XY", origin=(hx, sy, LIP_BOT - 2)).box(9, SLOT_W, H, centered=(True, True, False)))
    else:
        for sx in (hx - HKW / 2 - SLOT_W / 2, hx + HKW / 2 + SLOT_W / 2):
            lipwall = lipwall.cut(cq.Workplane("XY", origin=(sx, hy, LIP_BOT - 2)).box(SLOT_W, 9, H, centered=(True, True, False)))
    lipwall = lipwall.union(make_lip(hx, hy, dx, dy))
cq.exporters.export(lipwall, f"{OUT}/_lipwall.stl")
lipmesh = repair(trimesh.load(f"{OUT}/_lipwall.stl"))

carrier = trimesh.boolean.union([cap, lipmesh], engine="manifold")
# 버튼홀 (수직 관통)
cuts = []
jx, jy = PT.JOY_POS; gx, gy = PT.JOY_MOUNT_GRID
cyl = trimesh.creation.cylinder(radius=PT.JOY_DOME_DIA / 2, height=60); cyl.apply_translation([jx, jy, PART_Z + 25]); cuts.append(cyl)
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        c = trimesh.creation.cylinder(radius=1.1, height=60); c.apply_translation([jx + dx, jy + dy, PART_Z + 25]); cuts.append(c)
for nm, (bx, by) in PT.BTN_POS.items():
    b = trimesh.creation.box(extents=(PT.TACT_POCKET, PT.TACT_POCKET, 60)); b.apply_translation([bx, by, PART_Z + 25]); cuts.append(b)
carrier = trimesh.boolean.difference([carrier] + cuts, engine="manifold")
carrier.export(f"{OUT}/carrier_v2.stl")
print("캐리어② watertight:", carrier.is_watertight, "vol:", round(carrier.volume))

# ===== 잠김 검증 =====
hx0, hy0 = hooks[0][0], hooks[0][1]
probe = np.array([[hx0 + 1.5, hy0, PART_Z - 3.0], [hx0 + 1.5, hy0, PART_Z - 6.0]])
sv = grip.contains(probe)
RETAIN = bool(sv[0]) and (not bool(sv[1]))
print(f"잠김검증: 창위(z={PART_Z-3}) 그립={bool(sv[0])}(막힘需True) | 립자리(z={PART_Z-6}) 그립={bool(sv[1])}(구멍需False) => 잠김 {RETAIN}")

# ===== 렌더 =====
GV, GT = grip.vertices, grip.faces
CV, CT = carrier.vertices, carrier.faces
fig = plt.figure(figsize=(18, 5.2))
for k, (el, az, ttl) in enumerate([(18, -60, "iso"), (2, -90, "front -Y"), (2, 0, "side +X"), (0, -90, "분해(위로)")]):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    off = np.array([0, 0, 18]) if k == 3 else np.zeros(3)
    ax.add_collection3d(Poly3DCollection(GV[GT], facecolor=(0.55, 0.8, 0.6), edgecolor="none", alpha=0.5))
    ax.add_collection3d(Poly3DCollection((CV + off)[CT], facecolor=(0.3, 0.55, 0.95), edgecolor="none", alpha=0.9))
    allp = np.vstack([GV, CV + off]); c = allp.mean(0); rng = (allp.max(0) - allp.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl, fontsize=9); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle(f"클램셸 조립 (PART_Z={PART_Z}) — 캐리어(파랑)+그립(초록) | 잠김={RETAIN}")
plt.tight_layout(); plt.savefig(f"{OUT}/clamshell.png", dpi=92)
print("saved clamshell.png")

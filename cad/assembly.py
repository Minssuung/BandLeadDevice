#!/usr/bin/env python3
"""스냅핏 조립 v2 — 진짜 잠기는 캔틸레버 훅 + 트리거 배치.
캐리어②: 플랜지+스커트 + (텅 슬롯으로 자유로워진 캔틸레버 + 리드인램프 + 캐치립)×4.
그립①: 림에 창4 (창 윗변이 립 캐치를 막아 빠짐 방지). 트리거③ 앞면 추정배치.
실행: cad/.venv/bin/python cad/assembly.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
from trimesh import transformations as TF
import cadquery as cq
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, LineString
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
Zc = -4.0
SLIDE = PT.FDM_CLEAR          # 0.3
INSET_HOLE = 4.0

# 훅/창 파라미터
HKW = 6.0          # 훅 폭
LIP_OUT = 1.6      # 립 돌출(반경)
CATCH_Z = 0.0      # 립 캐치(평평한 윗면) z
LIP_BOT = -3.0     # 립 아래(램프 시작)
SLOT_W = 1.2       # 텅 자유화 슬롯
TONGUE_TOP = 2.0   # 텅 루트(이 위는 붙어있음)
WIN_BOT, WIN_TOP = -3.6, 0.6   # 그립 창 z범위(윗변 0.6>캐치0 → 0.6mm play 후 잠김)

gm = trimesh.load(f"{OUT}/grip_capped.stl")
V = gm.vertices
zmax = float(V[:, 2].max())
band = V[np.abs(V[:, 2] - Zc) < 3.0][:, :2]
H = band[ConvexHull(band).vertices]
P = Polygon(H)


def ring(poly):
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    return [(float(x), float(y)) for x, y in np.array(poly.exterior.coords)[:-1]]


flange_out = P.buffer(-1.0)
skirt_out = P.buffer(-(INSET_HOLE + SLIDE))
skirt_in = skirt_out.buffer(-2.0)
FT = 3.0
skirt_top = zmax - FT
SK = skirt_top - (Zc + 1.0)

# 캐리어 본체
carrier = cq.Workplane("XY", origin=(0, 0, skirt_top)).polyline(ring(flange_out)).close().extrude(FT)
wall = (cq.Workplane("XY", origin=(0, 0, skirt_top)).polyline(ring(skirt_out)).close().extrude(-SK)
        .cut(cq.Workplane("XY", origin=(0, 0, skirt_top + 0.5)).polyline(ring(skirt_in)).close().extrude(-SK - 1)))
carrier = carrier.union(wall)
# 버튼/스틱 컷
jx, jy = PT.JOY_POS; gx, gy = PT.JOY_MOUNT_GRID
carrier = carrier.faces(">Z").workplane().moveTo(jx, jy).circle(PT.JOY_DOME_DIA / 2).cutThruAll()
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        carrier = carrier.faces(">Z").workplane().moveTo(jx + dx, jy + dy).circle(1.1).cutThruAll()
for nm, (bx, by) in PT.BTN_POS.items():
    carrier = carrier.faces(">Z").workplane().moveTo(bx, by).rect(PT.TACT_POCKET, PT.TACT_POCKET).cutThruAll()

# 훅 위치 = 무게중심에서 각 방향 실제 외곽선 교점
cx, cy = float(P.centroid.x), float(P.centroid.y)


def bnd(dx, dy):
    ray = LineString([(cx, cy), (cx + dx * 1e3, cy + dy * 1e3)])
    it = ray.intersection(skirt_out.boundary)
    pt = max(it.geoms, key=lambda p: (p.x - cx) * dx + (p.y - cy) * dy) if it.geom_type == "MultiPoint" else it
    return float(pt.x), float(pt.y)


hooks = [(*bnd(dx, dy), dx, dy) for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]]
print("훅 위치:", [(round(h[0], 1), round(h[1], 1)) for h in hooks])


def make_lip(hx, hy, dx, dy):
    """캐치(평평 윗면 z=0) + 리드인 램프 립."""
    if dx:
        ccx = hx + dx * LIP_OUT / 2
        b = cq.Workplane("XY", origin=(ccx, hy, (LIP_BOT + CATCH_Z) / 2)).box(LIP_OUT, HKW, CATCH_Z - LIP_BOT)
        oe = hx + dx * LIP_OUT
        cutter = (cq.Workplane("XY", origin=(oe, hy, LIP_BOT)).box(LIP_OUT * 2.2, HKW * 1.3, LIP_OUT * 2.2)
                  .rotate((oe, hy - 1, LIP_BOT), (oe, hy + 1, LIP_BOT), 45))
    else:
        ccy = hy + dy * LIP_OUT / 2
        b = cq.Workplane("XY", origin=(hx, ccy, (LIP_BOT + CATCH_Z) / 2)).box(HKW, LIP_OUT, CATCH_Z - LIP_BOT)
        oe = hy + dy * LIP_OUT
        cutter = (cq.Workplane("XY", origin=(hx, oe, LIP_BOT)).box(HKW * 1.3, LIP_OUT * 2.2, LIP_OUT * 2.2)
                  .rotate((hx - 1, oe, LIP_BOT), (hx + 1, oe, LIP_BOT), 45))
    return b.cut(cutter)


def cut_slots(c, hx, hy, dx, dy):
    H = TONGUE_TOP - (LIP_BOT - 2)
    if dx:
        for sy in (hy - HKW / 2 - SLOT_W / 2, hy + HKW / 2 + SLOT_W / 2):
            c = c.cut(cq.Workplane("XY", origin=(hx, sy, LIP_BOT - 2)).box(9, SLOT_W, H, centered=(True, True, False)))
    else:
        for sx in (hx - HKW / 2 - SLOT_W / 2, hx + HKW / 2 + SLOT_W / 2):
            c = c.cut(cq.Workplane("XY", origin=(sx, hy, LIP_BOT - 2)).box(SLOT_W, 9, H, centered=(True, True, False)))
    return c


for (hx, hy, dx, dy) in hooks:
    carrier = cut_slots(carrier, hx, hy, dx, dy)
    carrier = carrier.union(make_lip(hx, hy, dx, dy))
cq.exporters.export(carrier, f"{OUT}/carrier_v2.step")
cq.exporters.export(carrier, f"{OUT}/carrier_v2.stl")
print("carrier_v2 vol:", round(carrier.val().Volume()))

# 그립 림에 창 4
grip = trimesh.load(f"{OUT}/grip_shell.stl")
wins = []
for (hx, hy, dx, dy) in hooks:
    ext = (8, HKW + 1.5, WIN_TOP - WIN_BOT) if dx else (HKW + 1.5, 8, WIN_TOP - WIN_BOT)
    b = trimesh.creation.box(extents=ext)
    b.apply_translation([hx, hy, (WIN_BOT + WIN_TOP) / 2])
    wins.append(b)
grip_win = trimesh.boolean.difference([grip] + wins, engine="manifold")
grip_win.export(f"{OUT}/grip_assembled.stl")
print("grip+windows watertight:", grip_win.is_watertight)

# 트리거 ③ 앞면 추정 배치 (보고 수정)
trig_meshes = []
try:
    gb = gm.bounds
    TRIG_T = [cx + 6, gb[0][1] + 4, -2]          # 헤드 앞쪽(-Y), 추정
    M = TF.translation_matrix(TRIG_T) @ TF.euler_matrix(np.radians(-90), 0, np.radians(-90))
    for f in ("trigger_lever.stl", "trigger_hallmount.stl"):
        t = trimesh.load(f"{OUT}/{f}"); t.apply_transform(M); trig_meshes.append(t)
    print("trigger 배치(추정):", np.round(TRIG_T, 1))
except Exception as e:
    print("trigger 배치 skip:", e)

# 렌더 (그립 반투명 + 캐리어 파랑 + 트리거 주황 + 훅 줌)
cvv, ctt = carrier.val().tessellate(0.25)
CV = np.array([(p.x, p.y, p.z) for p in cvv]); CT = np.array(ctt)
GV, GT = grip_win.vertices, grip_win.faces
hx0, hy0 = hooks[0][0], hooks[0][1]
fig = plt.figure(figsize=(18, 5.2))
views = [(20, -60, "iso 3파트", None), (3, -90, "front", None), (3, 0, "side", None),
         (6, -55, "훅 줌(캐치)", (hx0, hy0))]
for k, (el, az, ttl, zoom) in enumerate(views):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(GV[GT], facecolor=(0.55, 0.8, 0.6), edgecolor="none", alpha=0.22))
    ax.add_collection3d(Poly3DCollection(CV[CT], facecolor=(0.3, 0.55, 0.95), edgecolor="k", linewidths=.08, alpha=.95))
    for t in trig_meshes:
        ax.add_collection3d(Poly3DCollection(t.vertices[t.faces], facecolor=(0.95, 0.6, 0.2), edgecolor="none", alpha=.95))
    if zoom is None:
        allp = np.vstack([GV, CV]); c = allp.mean(0); rng = (allp.max(0) - allp.min(0)).max() / 2
        ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    else:
        zx, zy = zoom; rng = 9
        ax.set_xlim(zx-rng, zx+rng); ax.set_ylim(zy-rng, zy+rng); ax.set_zlim(-7, 5)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl, fontsize=9); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("스냅핏 조립 v2 — 캔틸레버 훅(잠김) + 트리거 배치(추정)")
plt.tight_layout(); plt.savefig(f"{OUT}/assembly.png", dpi=92)
print("saved assembly.png")

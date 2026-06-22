#!/usr/bin/env python3
"""V3 Stage2d-2/3 — 헤드 캐리어② 스냅 결합 + 3파트 조립.
캐리어: 플랜지+스커트+캔틸레버 훅 + 조이스틱 돔/보스 + 버튼홀. 그립 헤드벽에 창4. 조립+잠김검증.
실행: cad/.venv/bin/python cad/assemble_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
import pymeshfix
from trimesh import transformations as TF
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
HEAD = (56, 58); HEAD_R = 11; WALL = 3.0; CLR = PT.FDM_CLEAR
JOY = (0, -9); BTN = {"A": (-20, -2), "B": (-19, 13), "menu": (16, 16)}
FT = 3.0; SK = 11.0
HKW = 6.0; LIP_OUT = 2.0; SLOT_W = 1.2
CATCH_Z = -8.0; LIP_BOT = -11.0; WIN_BOT, WIN_TOP = -12.0, -7.5
sx, sy = HEAD[0] / 2 - (WALL + CLR), HEAD[1] / 2 - (WALL + CLR)   # 스커트 외곽 반치수
hooks = [(sx, 0, 1, 0), (-sx, 0, -1, 0), (0, sy, 0, 1), (0, -sy, 0, -1)]


def repair(m):
    if not m.is_volume:
        v, f = pymeshfix.clean_from_arrays(np.asarray(m.vertices), np.asarray(m.faces))
        m = trimesh.Trimesh(v, f); trimesh.repair.fix_normals(m)
    return m

# ── 캐리어 ② ──
flange = (cq.Workplane("XY").box(HEAD[0], HEAD[1], FT, centered=(True, True, False)).translate((0, 0, -FT)).edges("|Z").fillet(HEAD_R))
sko = cq.Workplane("XY", origin=(0, 0, -FT)).box(2 * sx, 2 * sy, SK, centered=(True, True, False)).translate((0, 0, -SK)).edges("|Z").fillet(max(1, HEAD_R - WALL))
ski = cq.Workplane("XY", origin=(0, 0, -FT + 0.5)).box(2 * sx - 4, 2 * sy - 4, SK + 1, centered=(True, True, False)).translate((0, 0, -SK - 1))
carrier = flange.union(sko.cut(ski))
# 조이스틱 돔 + 4 보스
jx, jy = JOY; gx, gy = PT.JOY_MOUNT_GRID
carrier = carrier.faces(">Z").workplane().moveTo(jx, jy).hole(PT.JOY_DOME_DIA)
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        carrier = carrier.union(cq.Workplane("XY", origin=(jx + dx, jy + dy, -FT)).circle(2.4).extrude(-9).faces("<Z").workplane().circle(0.9).cutBlind(6))
# 버튼 구멍
for nm, (bx, by) in BTN.items():
    carrier = carrier.faces(">Z").workplane().moveTo(bx, by).hole(8.0)

def make_lip(hx, hy, dx, dy):
    if dx:
        b = cq.Workplane("XY", origin=(hx + dx * LIP_OUT / 2, hy, (LIP_BOT + CATCH_Z) / 2)).box(LIP_OUT, HKW, CATCH_Z - LIP_BOT)
        oe = hx + dx * LIP_OUT
        cut = cq.Workplane("XY", origin=(oe, hy, LIP_BOT)).box(LIP_OUT * 2.2, HKW * 1.3, LIP_OUT * 2.2).rotate((oe, hy - 1, LIP_BOT), (oe, hy + 1, LIP_BOT), 45)
    else:
        b = cq.Workplane("XY", origin=(hx, hy + dy * LIP_OUT / 2, (LIP_BOT + CATCH_Z) / 2)).box(HKW, LIP_OUT, CATCH_Z - LIP_BOT)
        oe = hy + dy * LIP_OUT
        cut = cq.Workplane("XY", origin=(hx, oe, LIP_BOT)).box(HKW * 1.3, LIP_OUT * 2.2, LIP_OUT * 2.2).rotate((hx - 1, oe, LIP_BOT), (hx + 1, oe, LIP_BOT), 45)
    return b.cut(cut)

for (hx, hy, dx, dy) in hooks:
    H = (-FT + 0.5) - (LIP_BOT - 2)
    if dx:
        for s in (hy - HKW / 2 - SLOT_W / 2, hy + HKW / 2 + SLOT_W / 2):
            carrier = carrier.cut(cq.Workplane("XY", origin=(hx, s, LIP_BOT - 2)).box(9, SLOT_W, H, centered=(True, True, False)))
    else:
        for s in (hx - HKW / 2 - SLOT_W / 2, hx + HKW / 2 + SLOT_W / 2):
            carrier = carrier.cut(cq.Workplane("XY", origin=(s, hy, LIP_BOT - 2)).box(SLOT_W, 9, H, centered=(True, True, False)))
    carrier = carrier.union(make_lip(hx, hy, dx, dy))
cq.exporters.export(carrier, f"{OUT}/carrier_v3.step")
cq.exporters.export(carrier, f"{OUT}/carrier_v3.stl")
print("carrier vol:", round(carrier.val().Volume()))

# ── 그립 헤드벽에 창4 ──
grip = cq.importers.importStep(f"{OUT}/grip_body_v3.step")
for (hx, hy, dx, dy) in hooks:
    if dx:
        win = cq.Workplane("XY", origin=(hx, hy, (WIN_BOT + WIN_TOP) / 2)).box(8, HKW + 1.5, WIN_TOP - WIN_BOT)
    else:
        win = cq.Workplane("XY", origin=(hx, hy, (WIN_BOT + WIN_TOP) / 2)).box(HKW + 1.5, 8, WIN_TOP - WIN_BOT)
    grip = grip.cut(win)
cq.exporters.export(grip, f"{OUT}/grip_body_v3.stl")
print("그립+창 exported")

# ── 잠김 검증 ──
def cqmesh(wp, tol=0.4):
    vv, tt = wp.val().tessellate(tol)
    return repair(trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt)))
gm = cqmesh(grip); cm = cqmesh(carrier)
hx0, hy0 = hooks[0][0], hooks[0][1]
probe = np.array([[hx0 + 1.5, hy0, -5.0], [hx0 + 1.5, hy0, -10.0]])
sv = gm.contains(probe)
RETAIN = bool(sv[0]) and (not bool(sv[1]))
print(f"잠김검증: 창위(z=-5) 그립={bool(sv[0])} | 립자리(z=-10) 그립={bool(sv[1])} => 잠김 {RETAIN}")

# ── 트리거 레버 로드 ──
try:
    lev = trimesh.load(f"{OUT}/trigger_lever_v3.stl")
except Exception:
    lev = None

# ── 렌더: iso / front / side / exploded ──
GV, GT = gm.vertices, gm.faces; CV, CT = cm.vertices, cm.faces
fig = plt.figure(figsize=(18, 5.4))
for k, (el, az, ttl, dz) in enumerate([(18, -60, "iso", 0), (3, -90, "front", 0), (3, 0, "side", 0), (12, -60, "분해", 26)]):
    ax = fig.add_subplot(1, 4, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(GV[GT], facecolor=(0.55, 0.78, 0.62), edgecolor="none", alpha=0.5))
    ax.add_collection3d(Poly3DCollection((CV + [0, 0, dz])[CT], facecolor=(0.3, 0.55, 0.95), edgecolor="none", alpha=0.92))
    if lev is not None:
        ax.add_collection3d(Poly3DCollection(lev.vertices[lev.faces], facecolor=(0.95, 0.55, 0.2), edgecolor="none", alpha=0.95))
    allp = np.vstack([GV, CV]); c = allp.mean(0); r = (allp.max(0) - allp.min(0)).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle(f"V3 Stage2d — 3파트 조립 (그립①초록+캐리어②파랑+트리거③주황) | 캐리어 잠김={RETAIN}")
plt.tight_layout(); plt.savefig(f"{OUT}/assemble_v3.png", dpi=92)
print("saved assemble_v3.png")

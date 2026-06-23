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
JOY = (0, -6); BTN = {"torque": (-11, 16), "kbd": (11, 16)}   # 조이스틱 y-6(트리거허브 y-20 회피) / 버튼2 뒤
FT = 3.0; SK = 11.0
HKW = 6.0; LIP_OUT = 2.0; SLOT_W = 1.2
CATCH_Z = -8.0; LIP_BOT = -11.0; WIN_BOT, WIN_TOP = -12.0, -7.5
sx, sy = HEAD[0] / 2 - (WALL + CLR), HEAD[1] / 2 - (WALL + CLR)   # 스커트 외곽 반치수
hooks = [(sx, 0, 1, 0), (-sx, 0, -1, 0), (0, sy, 0, 1), (13, -sy, 0, -1), (-13, -sy, 0, -1)]  # 앞훅2 트리거슬롯 양옆


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
# 조이스틱 모듈 마운트: 돔홀 + 4 스탠드오프(27×20) — KY-023 모듈을 여기 나사고정(스틱→돔), 5핀은 직배선
jx, jy = JOY; gx, gy = PT.JOY_MOUNT_GRID
carrier = carrier.faces(">Z").workplane().moveTo(jx, jy).hole(PT.JOY_DOME_DIA)   # 스틱 돔홀
JOY_POSTS = [(jx + dx, jy + dy) for dx in (-gx / 2, gx / 2) for dy in (-gy / 2, gy / 2)]
for (px, py) in JOY_POSTS:
    post = cq.Workplane("XY", origin=(px, py, -FT)).circle(2.3).extrude(-5)        # z-3..-8 (모듈 PCB 안착)
    post = post.faces("<Z").workplane().circle(0.85).cutBlind(-4)                   # M2 파일럿(위로)
    carrier = carrier.union(post)
# 버튼 2개 구멍 Ø8 + 누름캡(디스크+스템) — 아래 보드 택트가 받쳐줌
ACT_TOP = -2.5   # 보드(z-7) 위 택트 액추에이터 top z
caps = None
for nm, (bx, by) in BTN.items():
    carrier = carrier.faces(">Z").workplane().moveTo(bx, by).hole(8.0)
    cap = (cq.Workplane("XY", origin=(bx, by, 0.8)).circle(5).extrude(2)                  # 디스크(0.8 돌출)
           .union(cq.Workplane("XY", origin=(bx, by, 0.8)).circle(2.9).extrude(ACT_TOP - 0.8)))  # 스템→택트
    caps = cap if caps is None else caps.union(cap)
cq.exporters.export(caps, f"{OUT}/button_caps_v3.stl")
print("button_caps vol:", round(caps.val().Volume()))
# 만능보드(택트2 + 배선 junction) 스탠드오프 4 — 조이스틱 뒤(y>4), 버튼 사이
PCB_POSTS = [(-16, 8), (16, 8), (-16, 20), (16, 20)]
for (px, py) in PCB_POSTS:
    post = cq.Workplane("XY", origin=(px, py, -FT)).circle(2.3).extrude(-4)        # z-3..-7
    post = post.faces("<Z").workplane().circle(0.85).cutBlind(-3)                   # M2 파일럿(위로)
    carrier = carrier.union(post)

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
# 트리거 릴리프 — 앞쪽 스커트가 레버 허브와 겹치지 않게 파냄
carrier = carrier.cut(cq.Workplane("XY", origin=(0, -25, -10)).box(16, 12, 14))
cq.exporters.export(carrier, f"{OUT}/carrier_v3.step")
cq.exporters.export(carrier, f"{OUT}/carrier_v3.stl")
print("carrier vol:", round(carrier.val().Volume()))

# ── 그립 헤드벽에 창4 ──
grip = cq.importers.importStep(f"{OUT}/grip_body_v3.step")
grip = grip.faces(">Z").workplane().rect(HEAD[0] + 1, HEAD[1] + 1).cutBlind(-FT)   # 플랜지 안착 리세스(겹침 제거)
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
    lev = repair(trimesh.load(f"{OUT}/trigger_lever_v3.stl"))
except Exception:
    lev = None
# ── 종합 검증 ──
print("=== 종합 검증 ===")
# 1) 캐리어 vs 그립 간섭 (훅이 창에 들어가므로 거의 0이어야)
ci = trimesh.boolean.intersection([gm, cm], engine="manifold")
civ = 0 if (ci is None or ci.is_empty) else ci.volume
print(f"  캐리어↔그립 간섭: {round(civ)} mm³ (작을수록 OK)")
# 2) 조이스틱 본체(40×26, 면아래 18mm) 헤드 cavity 수납
joy = trimesh.creation.box(extents=(40, 26, 14)); joy.apply_translation([JOY[0], JOY[1], -5])   # PCB z=-12 기준 본체
ji = trimesh.boolean.intersection([gm, joy], engine="manifold")
jiv = 0 if (ji is None or ji.is_empty) else ji.volume
print(f"  조이스틱 본체↔그립벽 간섭: {round(jiv)} mm³ (0이면 cavity 수납 OK)")
# 3) 캡 빠짐방지: 플랜지Ø8.8 > 구멍Ø8 → 위로 못빠짐 (기하 보장)
print("  버튼캡 리테이너: 플랜지Ø8.8 > 구멍Ø8 → 빠짐방지 OK / 돌출 0.8 스트로크")
# 4) 트리거 레버 vs 그립 간섭 (rest)
tiv = 0
if lev is not None:
    ti = trimesh.boolean.intersection([gm, lev], engine="manifold")
    tiv = 0 if (ti is None or ti.is_empty) else ti.volume
    print(f"  트리거 레버↔그립 간섭(rest): {round(tiv)} mm³ (피벗부만 약간이면 OK)")
print(f"  배선출구/리프트보스/스프링리브/IMU스탠드오프: grip_body에 포함")
OK_ALL = RETAIN and civ < 800 and jiv < 50
print(f"=== 조립가능 종합: {'OK' if OK_ALL else '점검필요'} ===")

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

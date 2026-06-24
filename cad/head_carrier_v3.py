#!/usr/bin/env python3
"""V3 Stage2a — 헤드 캐리어: 조이스틱 마운트 + A/B/menu 버튼캡(눌림 스트로크).
버튼 = 디스크(손가락)+스템(관통)+리테이너 플랜지(빠짐방지) → 아래 택트 6×6 작동.
실행: cad/.venv/bin/python cad/head_carrier_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
HEAD = (56, 58, 24); HEAD_R = 11
PLATE_T = 3.0
JOY = (0, -9)
BTN = {"A": (-20, -2), "B": (-19, 13), "menu": (16, 16)}

# 버튼캡 파라미터
HOLE_D = 8.0        # 플레이트 구멍
CAP_D = 10.0        # 캡 디스크(손가락)
STEM_D = 5.8        # 스템(구멍 관통)
FLANGE_D = 9.0      # 리테이너(구멍보다 큼 → 빠짐방지)
TRAVEL = 0.8        # 눌림 스트로크
# z 배치: 플레이트 0~-3, 택트 액추에이터 top=-4.5, 캡 스템 bottom=-5(0.5갭)
ACT_TOP = -4.5
TACT_BODY_TOP = -5.5

# ── 캐리어 플레이트 ──
plate = (cq.Workplane("XY").box(HEAD[0], HEAD[1], PLATE_T, centered=(True, True, False))
         .translate((0, 0, -PLATE_T)).edges("|Z").fillet(HEAD_R))

# ── 조이스틱: Ø18 돔홀 + 4 마운트 보스(아래로) ──
jx, jy = JOY; gx, gy = PT.JOY_MOUNT_GRID
plate = plate.faces(">Z").workplane().moveTo(jx, jy).hole(PT.JOY_DOME_DIA)
for dx in (-gx / 2, gx / 2):
    for dy in (-gy / 2, gy / 2):
        boss = (cq.Workplane("XY", origin=(jx + dx, jy + dy, -PLATE_T)).circle(2.5).extrude(-(14 - PLATE_T))
                .faces("<Z").workplane().circle(0.9).cutBlind(-8))   # M2 파일럿
        plate = plate.union(boss)

# ── 택트 버튼 ×3: 구멍 + 택트마운트 보스 + 캡(별도) ──
caps = []
for nm, (bx, by) in BTN.items():
    plate = plate.faces(">Z").workplane().moveTo(bx, by).hole(HOLE_D)          # 손가락 구멍
    # 택트 6.4□ 포켓 보스 (액추에이터 위로)
    block = (cq.Workplane("XY", origin=(bx, by, TACT_BODY_TOP - 4)).box(9, 9, 4, centered=(True, True, False))
             .faces(">Z").workplane().rect(PT.TACT_POCKET, PT.TACT_POCKET).cutBlind(-3.5))
    # 보스를 플레이트 밑면과 리브로 연결
    rib = cq.Workplane("XY", origin=(bx, by, TACT_BODY_TOP)).box(2, 9, abs(TACT_BODY_TOP) - PLATE_T, centered=(True, True, False)).translate((0, 0, PLATE_T - abs(TACT_BODY_TOP)))
    plate = plate.union(block)
    # 캡(별도 출력물): 디스크(돌출) + 스템(액추에이터까지) + 리테이너 플랜지
    cz0 = TRAVEL   # rest 시 디스크 밑면이 플레이트 위로 TRAVEL 돌출 → 누르면 flush
    cap = (cq.Workplane("XY", origin=(bx, by, cz0)).circle(CAP_D / 2).extrude(2.0))                      # 디스크
    cap = cap.union(cq.Workplane("XY", origin=(bx, by, cz0)).circle(STEM_D / 2).extrude(ACT_TOP - cz0))  # 스템 → 액추에이터(-4.5)
    cap = cap.union(cq.Workplane("XY", origin=(bx, by, -PLATE_T)).circle(FLANGE_D / 2).extrude(-1.0))     # 플랜지(플레이트 밑, 빠짐방지)
    caps.append((nm, cap))

carrier = plate
cq.exporters.export(carrier, f"{OUT}/carrier_v3.step")
cq.exporters.export(carrier, f"{OUT}/carrier_v3.stl")
allcaps = caps[0][1]
for _, c in caps[1:]:
    allcaps = allcaps.union(c)
cq.exporters.export(allcaps, f"{OUT}/button_caps.stl")
print("carrier vol:", round(carrier.val().Volume()), "| caps:", len(caps))

# ── 렌더: top / iso / 단면(버튼 A 눌림 스트로크) ──
def mesh(wp, tol=0.3):
    vv, tt = wp.val().tessellate(tol)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt))

cm = mesh(carrier); capm = [(nm, mesh(c)) for nm, c in caps]
# 택트 placeholder 메시(6×6×3.5 + 액추에이터)
def tact_at(bx, by):
    b = trimesh.creation.box(extents=(6, 6, 3.5)); b.apply_translation([bx, by, TACT_BODY_TOP - 3.5 / 2])
    a = trimesh.creation.cylinder(radius=1.75, height=1.0); a.apply_translation([bx, by, ACT_TOP - 0.5])
    return [b, a]

fig = plt.figure(figsize=(18, 5.4))
# top
ax = fig.add_subplot(1, 3, 1, projection="3d")
ax.add_collection3d(Poly3DCollection(cm.vertices[cm.faces], facecolor=(0.6, 0.75, 0.9), edgecolor="none", alpha=.6))
for nm, m in capm:
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=(0.95, 0.5, 0.3), edgecolor="none"))
ax.view_init(88, -90); ax.set_title("top (버튼캡 주황)", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 0.5))
P = cm.vertices; c = P.mean(0); r = (P.max(0) - P.min(0)).max() / 2
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(-r, r)
# iso
ax = fig.add_subplot(1, 3, 2, projection="3d")
ax.add_collection3d(Poly3DCollection(cm.vertices[cm.faces], facecolor=(0.6, 0.75, 0.9), edgecolor="none", alpha=.45))
for nm, m in capm:
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=(0.95, 0.5, 0.3), edgecolor="none"))
ax.view_init(20, -60); ax.set_title("iso", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
# 단면 (버튼 A의 y평면)
ax = fig.add_subplot(1, 3, 3)
bx, by = BTN["A"]
parts_sec = [("carrier", cm, "royalblue"), ("cap", capm[0][1], "orangered")] + [(f"tact{i}", t, "green") for i, t in enumerate(tact_at(bx, by))]
for nm, m, col in parts_sec:
    sec = m.section(plane_origin=[0, by, 0], plane_normal=[0, 1, 0])
    if sec is None:
        continue
    for e in sec.entities:
        pp = sec.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 2], col, lw=1.3)
    ax.plot([], [], col, label=nm)
ax.annotate("", xy=(bx + 7, 0), xytext=(bx + 7, TRAVEL), arrowprops=dict(arrowstyle="<->", color="k"))
ax.text(bx + 7.3, TRAVEL / 2, f"돌출=스트로크 {TRAVEL}", fontsize=8)
ax.set_xlim(bx - 9, bx + 12); ax.set_ylim(-11, 4); ax.set_aspect("equal")
ax.set_title(f"버튼 A 단면 — 캡(주황)→택트(초록)", fontsize=9); ax.legend(loc="lower left", fontsize=7); ax.grid(alpha=.3)
fig.suptitle("V3 Stage2a — 헤드캐리어: 조이스틱 마운트 + 버튼캡 눌림 메커니즘")
plt.tight_layout(); plt.savefig(f"{OUT}/carrier_v3.png", dpi=92)
print("saved carrier_v3.png")

#!/usr/bin/env python3
"""만능보드(PCB) = 중앙 배선 허브. 그립 하부 헤드(z-16)에 트레이로 드롭인.
모든 부품(조이스틱5·홀3·리프트2·IMU/485·버튼2) 선이 여기 모임 → 바닥 케이블로 컨트롤러 밖.
부품은 안 올라감(맨몸 택트도 캐리어로 감) — 보드는 배선 그리드 + 케이블 단자.
실행: cad/.venv/bin/python cad/perfboard_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
BW, BL, BT = 44.0, 36.0, 1.5          # 허브보드 가로×세로×두께 (크게)
BC = (0, 5)                           # 중심 XY (x±22, y-13..23 — 홀바 뒤·트리거 회피)
BZ = -16.0                            # z (top -15.25, bottom -16.75; 스커트-14·조이스틱-13 아래)

board = cq.Workplane("XY", origin=(BC[0], BC[1], BZ - BT / 2)).box(BW, BL, BT, centered=(True, True, False))
# 케이블 출구 슬롯(뒤쪽) + 배선 그리드 표시는 실제 만능기판이 가짐(여기선 외형만)
cq.exporters.export(board, f"{OUT}/perfboard_v3.stl")
import pymeshfix
_m = trimesh.load(f"{OUT}/perfboard_v3.stl")
if not _m.is_watertight:
    _v, _f = pymeshfix.clean_from_arrays(np.asarray(_m.vertices), np.asarray(_m.faces))
    _m = trimesh.Trimesh(_v, _f); trimesh.repair.fix_normals(_m); _m.export(f"{OUT}/perfboard_v3.stl")
print(f"hub board {BW}×{BL}×{BT} @ y{BC[1]} z{BZ} | watertight:", trimesh.load(f"{OUT}/perfboard_v3.stl").is_watertight)

# ── 렌더: 그립 + 허브보드 (그립 하부에 트레이로 안착) ──
g = trimesh.load(f"{OUT}/grip_body_v3.stl")
bm = trimesh.load(f"{OUT}/perfboard_v3.stl")
fig = plt.figure(figsize=(16, 6.5))
for k, (el, az, ttl) in enumerate([(20, -60, "iso (그립+허브보드)"), (3, -90, "front 단면감"), (90, -90, "top (배치)")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(g.vertices[g.faces], facecolor=(.6, .8, .6), edgecolor="none", alpha=.18))
    ax.add_collection3d(Poly3DCollection(bm.vertices[bm.faces], facecolor=(.15, .55, .25), edgecolor="none", alpha=.95))
    allp = np.vstack([g.vertices, bm.vertices]); c = allp.mean(0); r = (allp.max(0) - allp.min(0)).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("허브 만능보드(초록) — 그립 하부 헤드(z-16)에 드롭인. 모든 부품 선이 여기 모여 바닥 케이블로")
plt.tight_layout(); plt.savefig(f"{OUT}/perfboard_v3.png", dpi=92)
print("saved perfboard_v3.png")

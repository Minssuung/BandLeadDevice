#!/usr/bin/env python3
"""V4 미관 폼 — 헤드→손잡이를 loft로 매끄럽게 모핑(경계 블렌딩).
실행: cad/.venv/bin/python cad/controller_v4.py
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
HEAD = (56, 58); HEAD_R = 14
TILT = 18; off = (0, 10, -10)
ca, sa = np.cos(np.radians(TILT)), np.sin(np.radians(TILT))


def axis_pt(s):  # 손잡이 축 위 점(파라미터 s)
    return np.array([0, off[1] + sa * s, off[2] - ca * s])

# 헤드 프리즘 (z 0..-18, 둥근 사각)
head = (cq.Workplane("XY").box(HEAD[0], HEAD[1], 18, centered=(True, True, False)).translate((0, 0, -18)).edges("|Z").fillet(HEAD_R))

# 로프트: 헤드바닥(둥근사각) → 중간 → 손잡이 타원 (매끄러운 전이)
sections = []
specs = [(8, HEAD[0], HEAD[1], HEAD_R), (20, 48, 46, 16), (34, 40, 39, 19), (50, 38, 38, 19)]
for s, wx, wy, r in specs:
    p = axis_pt(s)
    wp = cq.Workplane("XY", origin=(0, p[1], p[2]))
    sections.append(wp.rect(wx, wy).vertices().fillet2D(r) if False else wp)
# loft은 2D 와이어 시퀀스로 — 각 단면을 rect+fillet2D
blendwp = cq.Workplane("XY")
wires = []
for i, (s, wx, wy, r) in enumerate(specs):
    p = axis_pt(s)
    w = (cq.Workplane("XY", origin=(0, p[1], p[2])).rect(wx, wy)
         .val())
    wires.append((p, wx, wy, r))
# cadquery loft: place each section as a sketch on its own offset, then loft
res = cq.Workplane("XY")
secs = []
for (s, wx, wy, r) in specs:
    p = axis_pt(s)
    secs.append(cq.Workplane("XY").transformed(offset=cq.Vector(0, p[1], p[2])).rect(wx, wy))
try:
    lofted = secs[0]
    for sct in secs[1:]:
        lofted = lofted.add(sct)
    lofted = lofted.loft(ruled=False)
    ctrl = head.union(lofted)
    ok = "loft OK"
except Exception as e:
    ctrl = head; ok = f"loft 실패: {e}"
print(ok)
cq.exporters.export(ctrl, f"{OUT}/controller_v4.step")
cq.exporters.export(ctrl, f"{OUT}/controller_v4.stl")
bb = ctrl.val().BoundingBox()
print(f"v4 bbox X[{bb.xmin:.0f},{bb.xmax:.0f}] Y[{bb.ymin:.0f},{bb.ymax:.0f}] Z[{bb.zmin:.0f},{bb.zmax:.0f}]")

before = trimesh.load(f"{OUT}/controller_v3.stl")
vv, tt = ctrl.val().tessellate(0.4)
av = np.array([(p.x, p.y, p.z) for p in vv]); at = np.array(tt)
fig = plt.figure(figsize=(16, 7))
for col, (vF, tF, name, fc) in enumerate([(before.vertices, before.faces, "BEFORE v3", (.6, .75, .9)),
                                          (av, at, "AFTER v4 loft", (.55, .8, .62))]):
    for row, (el, az, ttl) in enumerate([(18, -62, "iso"), (3, 0, "side")]):
        ax = fig.add_subplot(2, 2, row * 2 + col + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(vF[tF], facecolor=fc, edgecolor="none", alpha=0.7))
        c = vF.mean(0); rr = (vF.max(0) - vF.min(0)).max() / 2
        ax.set_xlim(c[0]-rr, c[0]+rr); ax.set_ylim(c[1]-rr, c[1]+rr); ax.set_zlim(c[2]-rr, c[2]+rr)
        ax.view_init(el, az); ax.set_title(f"{name}·{ttl}", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("미관 — loft 블렌딩")
plt.tight_layout(); plt.savefig(f"{OUT}/controller_v4.png", dpi=92)
print("saved controller_v4.png")

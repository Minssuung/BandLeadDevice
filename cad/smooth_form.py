#!/usr/bin/env python3
"""미관 — 메시 모폴로지 스무딩으로 헤드↔손잡이 경계 블렌딩 + 전체 라운드 (시각 목표).
복셀 채움 → closing(오목경계 메움/블렌딩) → opening(볼록모서리 라운드) → marching cubes → Taubin.
실행: cad/.venv/bin/python cad/smooth_form.py
"""
import numpy as np
import trimesh
import cadquery as cq
from scipy import ndimage
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
PITCH = 0.7
# 현재 외형 솔리드 (grip_body_v3.py와 동일 파라미터: 손잡이 112, 틸트18 등) — stale STL 대신 직접 빌드
HEAD = (56, 58, 24); HEAD_R = 11; HANDLE_D = 38; HANDLE_L = 112; TILT = 18; OFF = (0, 10, -10)
_head = cq.Workplane("XY").box(*HEAD, centered=(True, True, False)).translate((0, 0, -HEAD[2])).edges("|Z").fillet(HEAD_R)
_handle = (cq.Workplane("XY").circle(HANDLE_D / 2).extrude(-HANDLE_L).edges("<Z").fillet(HANDLE_D / 2 * 0.85)
           .rotate((0, 0, 0), (1, 0, 0), TILT).translate(OFF))
_ctrl = _head.union(_handle)
_vv, _tt = _ctrl.val().tessellate(0.5)
m = trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in _vv]), np.array(_tt))
m.export(f"{OUT}/controller_v3.stl")   # 현재 외형 솔리드(각짐, before) 갱신
# 팜 스웰(메타퀘스트식 손바닥 볼륨) — 손잡이 뒤(+Y) 중하단에 유기적 융기. closing이 손잡이와 블렌딩
swell = trimesh.creation.icosphere(subdivisions=3, radius=1.0)
swell.apply_scale([16, 16, 27]); swell.apply_translation([0, 38, -67])
m_form = trimesh.util.concatenate([m, swell])
vg = m_form.voxelized(PITCH).fill()
occ = np.pad(np.asarray(vg.matrix), 8, constant_values=0)


def ball(rmm):
    r = int(round(rmm / PITCH))
    zz, yy, xx = np.ogrid[-r:r+1, -r:r+1, -r:r+1]
    return (xx*xx + yy*yy + zz*zz) <= r*r

a = ndimage.binary_closing(occ, ball(5))    # 경계(오목) 블렌딩
a = ndimage.binary_opening(a, ball(3.0))    # 모서리(볼록) 라운드 — 퀘스트식 유기적 헤드
mc = trimesh.voxel.ops.matrix_to_marching_cubes(a)
mc.apply_translation([-8, -8, -8]); mc.apply_transform(vg.transform)
trimesh.smoothing.filter_taubin(mc, iterations=18)
mc.export(f"{OUT}/controller_smooth.stl")
print("smoothed watertight:", mc.is_watertight, "vol", round(mc.volume))

before = m
fig = plt.figure(figsize=(16, 7))
for col, (mm, name, fc) in enumerate([(before, "BEFORE (각짐)", (.6, .75, .9)), (mc, "AFTER (메타퀘스트 스타일)", (.55, .8, .62))]):
    for row, (el, az, ttl) in enumerate([(18, -62, "iso"), (3, 0, "side(팜스웰)")]):
        ax = fig.add_subplot(2, 2, row * 2 + col + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(mm.vertices[mm.faces], facecolor=fc, edgecolor="none", alpha=0.75))
        c = mm.vertices.mean(0); r = (mm.vertices.max(0) - mm.vertices.min(0)).max() / 2
        ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
        ax.view_init(el, az); ax.set_title(f"{name}·{ttl}", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("미관 목표 — 메타퀘스트 스타일 (헤드 + 앞으로 기운 에르고 손잡이 + 팜스웰 + 유기적 블렌딩)")
plt.tight_layout(); plt.savefig(f"{OUT}/controller_smooth.png", dpi=92)
print("saved controller_smooth.png")

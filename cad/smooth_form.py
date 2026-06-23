#!/usr/bin/env python3
"""미관 — 메시 모폴로지 스무딩으로 헤드↔손잡이 경계 블렌딩 + 전체 라운드 (시각 목표).
복셀 채움 → closing(오목경계 메움/블렌딩) → opening(볼록모서리 라운드) → marching cubes → Taubin.
실행: cad/.venv/bin/python cad/smooth_form.py
"""
import numpy as np
import trimesh
from scipy import ndimage
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
PITCH = 0.7
m = trimesh.load(f"{OUT}/controller_v3.stl")
vg = m.voxelized(PITCH).fill()
occ = np.pad(np.asarray(vg.matrix), 8, constant_values=0)


def ball(rmm):
    r = int(round(rmm / PITCH))
    zz, yy, xx = np.ogrid[-r:r+1, -r:r+1, -r:r+1]
    return (xx*xx + yy*yy + zz*zz) <= r*r

a = ndimage.binary_closing(occ, ball(5))    # 경계(오목) 블렌딩
a = ndimage.binary_opening(a, ball(2.5))    # 모서리(볼록) 라운드
mc = trimesh.voxel.ops.matrix_to_marching_cubes(a)
mc.apply_translation([-8, -8, -8]); mc.apply_transform(vg.transform)
trimesh.smoothing.filter_taubin(mc, iterations=18)
mc.export(f"{OUT}/controller_smooth.stl")
print("smoothed watertight:", mc.is_watertight, "vol", round(mc.volume))

before = m
fig = plt.figure(figsize=(16, 7))
for col, (mm, name, fc) in enumerate([(before, "BEFORE (각짐)", (.6, .75, .9)), (mc, "AFTER (스무딩)", (.55, .8, .62))]):
    for row, (el, az, ttl) in enumerate([(18, -62, "iso"), (3, 0, "side")]):
        ax = fig.add_subplot(2, 2, row * 2 + col + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(mm.vertices[mm.faces], facecolor=fc, edgecolor="none", alpha=0.75))
        c = mm.vertices.mean(0); r = (mm.vertices.max(0) - mm.vertices.min(0)).max() / 2
        ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
        ax.view_init(el, az); ax.set_title(f"{name}·{ttl}", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("미관 목표 — 메시 스무딩(헤드↔손잡이 블렌딩 + 전체 라운드)")
plt.tight_layout(); plt.savefig(f"{OUT}/controller_smooth.png", dpi=92)
print("saved controller_smooth.png")

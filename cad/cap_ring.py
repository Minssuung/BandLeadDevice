#!/usr/bin/env python3
"""링 제거 자리(함몰/빈공간)를 벽으로 메움 — 복셀 채움 + morphological closing.
그립(watertight) → 복셀 solid → binary_closing(ball R) → marching cubes → 매끈 솔리드.
실행: cad/.venv/bin/python cad/cap_ring.py [반경mm=8]
"""
import sys
import numpy as np
import trimesh
from scipy import ndimage
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
R_MM = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
PITCH = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
SMOOTH = int(sys.argv[3]) if len(sys.argv) > 3 else 25

m = trimesh.load(f"{OUT}/grip_solid.stl")
vg = m.voxelized(pitch=PITCH).fill()
occ = np.asarray(vg.matrix)
r = max(1, int(round(R_MM / PITCH)))
zz, yy, xx = np.ogrid[-r:r+1, -r:r+1, -r:r+1]
ball = (xx*xx + yy*yy + zz*zz) <= r*r
# 패딩(닫기가 경계에서 잘리지 않게)
pad = r + 2
occ = np.pad(occ, pad, mode="constant", constant_values=0)
closed = ndimage.binary_closing(occ, structure=ball)
print(f"closing R={R_MM}mm (vox {r}) | 채워진 보셀: {int(closed.sum()-occ.sum())}")

mc = trimesh.voxel.ops.matrix_to_marching_cubes(closed)
# 패딩 보정 후 원좌표로 변환
mc.apply_translation([-pad, -pad, -pad])
mc.apply_transform(vg.transform)
trimesh.smoothing.filter_taubin(mc, iterations=8)
print("capped watertight:", mc.is_watertight, "| vol:", round(mc.volume), "| ext:", np.round(mc.extents, 1))
mc.export(f"{OUT}/grip_capped.stl")
print("saved grip_capped.stl")

MV, MT = mc.vertices, mc.faces
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(18, -65, "iso"), (12, -90, "front/ring부"), (0, 0, "side +X")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(MV[MT], facecolor=(0.55, 0.78, 0.6), edgecolor="none"))
    c = MV.mean(0); rng = (MV.max(0) - MV.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle(f"링자리 캡핑 (closing R={R_MM}mm)")
plt.tight_layout(); plt.savefig(f"{OUT}/grip_capped.png", dpi=88)
print("saved grip_capped.png")

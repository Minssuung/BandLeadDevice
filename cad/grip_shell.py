#!/usr/bin/env python3
"""그립 본체 속비움(shell) + 헤드 상단 개구부 + 안착 레지.
복셀: solid → erosion으로 cavity → shell. 헤드 footprint 위쪽을 열어 캐리어 드롭인 구멍.
실행: cad/.venv/bin/python cad/grip_shell.py [wall=3] [insetLedge=4]
"""
import sys
import numpy as np
import trimesh
from scipy import ndimage
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
from matplotlib.path import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
WALL = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
LEDGE = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0   # 캐리어 안착 레지 폭
PITCH = 0.6
Zc = -4.0        # 캐리어 안착 평면 (이 위로 개구)

m = trimesh.load(f"{OUT}/grip_capped.stl")
vg = m.voxelized(PITCH).fill()
occ = np.asarray(vg.matrix)
T = vg.transform
origin = T[:3, 3]; scale = np.array([T[0, 0], T[1, 1], T[2, 2]])
nx, ny, nz = occ.shape

# 1) shell = solid − 안쪽 erosion
wv = max(1, int(round(WALL / PITCH)))
inner = ndimage.binary_erosion(occ, iterations=wv)
shell = occ & ~inner
print(f"wall {WALL}mm ({wv}vox) | solid {occ.sum()} → shell {shell.sum()} (cavity {inner.sum()})")

# 2) 헤드 footprint (z≈Zc 단면 볼록외곽, 레지폭만큼 인셋) 위쪽을 개구
V = m.vertices
band = V[np.abs(V[:, 2] - Zc) < 3.0][:, :2]
hull = band[ConvexHull(band).vertices]
poly = Polygon(hull).buffer(-LEDGE)
pth = Path(np.array(poly.exterior.coords))
I, J, K = np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij")
WX = origin[0] + scale[0] * I; WY = origin[1] + scale[1] * J; WZ = origin[2] + scale[2] * K
inside = pth.contains_points(np.column_stack([WX.ravel(), WY.ravel()])).reshape(occ.shape)
openmask = inside & (WZ > Zc + 0.5)
final = shell & ~openmask
print(f"open footprint area≈{poly.area:.0f}mm² | 개구 후 voxel {final.sum()}")

# 3) mesh화
mc = trimesh.voxel.ops.matrix_to_marching_cubes(final)
mc.apply_transform(T)
trimesh.smoothing.filter_taubin(mc, iterations=12)
mc.export(f"{OUT}/grip_shell.stl")
print("shell watertight:", mc.is_watertight, "| vol:", round(mc.volume), "| ext:", np.round(mc.extents, 1))
print("saved grip_shell.stl")

# 4) 렌더 (top=개구 / iso / front-half 컷=cavity)
MV, MT = mc.vertices, mc.faces
cent = MV[MT].mean(axis=1)
fig = plt.figure(figsize=(17, 6))
specs = [(88, -90, "top (개구부)", None), (20, -60, "iso", None), (3, -90, "front 반단면(cavity)", cent[:, 1] < 1.0)]
for k, (el, az, ttl, mask) in enumerate(specs):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    faces = MT[mask] if mask is not None else MT
    ax.add_collection3d(Poly3DCollection(MV[faces], facecolor=(0.55, 0.78, 0.6), edgecolor="none"))
    c = MV.mean(0); rng = (MV.max(0) - MV.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle(f"그립 shell + 헤드개구 (wall {WALL}mm, 레지 {LEDGE}mm)")
plt.tight_layout(); plt.savefig(f"{OUT}/grip_shell.png", dpi=88)
print("saved grip_shell.png")

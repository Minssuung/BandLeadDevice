#!/usr/bin/env python3
"""그립 솔리드 v2 — 구멍 없는 watertight.
Poisson(트림 없음=구멍 없음) → 박스와 manifold boolean-intersection으로 외삽 blob만 깔끔 제거.
실행: cad/.venv/bin/python cad/poisson2.py
"""
import numpy as np
import open3d as o3d
import trimesh
import cadquery as cq
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SRC = "/home/minsung/Downloads/touch-accessory-guidelines-2.0/Touch for Quest and Rift S/Left Controller for Quest and Rift S.stp"
OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
RING = {4, 5}

# 1) de-ringed 표면 → 점군(법선)
r = cq.importers.importStep(SRC)
solids = r.solids().vals()
comp = cq.Compound.makeCompound([s for i, s in enumerate(solids) if i not in RING])
v, t = comp.tessellate(0.25)
V = np.array([(p.x, p.y, p.z) for p in v]); T = np.array(t)
m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(T))
m.compute_triangle_normals(); m.compute_vertex_normals()
pcd = m.sample_points_uniformly(number_of_points=200000, use_triangle_normal=True)

# 2) Poisson(닫힌면) → pymeshfix로 watertight manifold 복구
import pymeshfix
pm, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, linear_fit=True)
PV = np.asarray(pm.vertices); PT_ = np.asarray(pm.triangles)
vc, fc = pymeshfix.clean_from_arrays(PV, PT_)        # 구멍/비매니폴드 정리 → 닫힌 표면
poiss = trimesh.Trimesh(vc, fc, process=True)
trimesh.repair.fix_normals(poiss)
print("meshfix watertight:", poiss.is_watertight, "is_volume:", poiss.is_volume, "tris", len(poiss.faces))

# 3) 입력 bbox + 2mm 박스와 intersection (외삽 blob만 잘림, 본체는 보존)
mn, mx = V.min(0) - 2.0, V.max(0) + 2.0
ext = mx - mn
box = trimesh.creation.box(extents=ext, transform=trimesh.transformations.translation_matrix((mn + mx) / 2))
grip = trimesh.boolean.intersection([poiss, box], engine="manifold")
print("grip watertight:", grip.is_watertight, "| vol:", round(grip.volume, 0),
      "| bbox:", np.round(grip.extents, 1))
grip.export(f"{OUT}/grip_solid.stl")
grip.export(f"{OUT}/grip_solid.ply")
print("saved grip_solid.stl/.ply")

# 4) 렌더
MV, MT = grip.vertices, grip.faces
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(18, -65, "iso"), (0, -90, "front -Y"), (0, 0, "side +X")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(MV[MT], facecolor=(0.5, 0.7, 0.9), edgecolor="none"))
    c = MV.mean(0); rng = (MV.max(0) - MV.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("그립 솔리드 v2 (트림X + boolean-crop, watertight)")
plt.tight_layout(); plt.savefig(f"{OUT}/grip_solid_v2.png", dpi=90)
print("saved grip_solid_v2.png")

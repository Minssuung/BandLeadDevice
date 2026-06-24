#!/usr/bin/env python3
"""옵션 B(보정): de-ringed 실제 표면 → Poisson 재구성 → watertight 솔리드 메시.
실제 Quest 형상 보존(링 제외분이 자동 메움). 출력 STL = 프린트/후속 boolean용.
실행: cad/.venv/bin/python cad/poisson.py
"""
import numpy as np
import open3d as o3d
import cadquery as cq
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SRC = "/home/minsung/Downloads/touch-accessory-guidelines-2.0/Touch for Quest and Rift S/Left Controller for Quest and Rift S.stp"
OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
RING = {4, 5}

# 1) de-ringed 표면 → 삼각망
r = cq.importers.importStep(SRC)
solids = r.solids().vals()
comp = cq.Compound.makeCompound([s for i, s in enumerate(solids) if i not in RING])
v, t = comp.tessellate(0.3)
V = np.array([(p.x, p.y, p.z) for p in v]); T = np.array(t)
print("tess verts/tris:", len(V), len(T))
m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(T))
m.compute_triangle_normals(); m.compute_vertex_normals()

# 2) 표면 샘플 점군(법선 포함)
pcd = m.sample_points_uniformly(number_of_points=120000, use_triangle_normal=True)

# 3) Poisson 재구성
mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, linear_fit=True)
dens = np.asarray(dens)
mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.04))   # 외삽 blob 제거
bb = m.get_axis_aligned_bounding_box()
bb = o3d.geometry.AxisAlignedBoundingBox(bb.min_bound - 3, bb.max_bound + 3)
mesh = mesh.crop(bb)
mesh.remove_degenerate_triangles(); mesh.remove_duplicated_vertices(); mesh.remove_unreferenced_vertices()
# 최대 연결성분만 남김(잔여 조각 제거) — fill_holes는 큰 경계를 이어붙여 폭발하므로 미사용
tc, nt, _ = mesh.cluster_connected_triangles()
tc = np.asarray(tc); nt = np.asarray(nt)
mesh.remove_triangles_by_mask(tc != int(nt.argmax()))
mesh.remove_unreferenced_vertices(); mesh.compute_vertex_normals()

print("watertight:", mesh.is_watertight(), "| tris:", len(mesh.triangles),
      "| bbox:", np.round(mesh.get_axis_aligned_bounding_box().get_extent(), 1))
o3d.io.write_triangle_mesh(f"{OUT}/grip_solid.stl", mesh)
o3d.io.write_triangle_mesh(f"{OUT}/grip_solid.ply", mesh)
print("saved grip_solid.stl/.ply")

# 4) 렌더 (재구성 파랑 vs 원본 점군 빨강)
MV = np.asarray(mesh.vertices); MT = np.asarray(mesh.triangles)
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(18, -65, "iso"), (0, -90, "front -Y"), (0, 0, "side +X")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.scatter(V[::10, 0], V[::10, 1], V[::10, 2], s=0.4, color=(.85, .4, .4), alpha=.18)
    ax.add_collection3d(Poly3DCollection(MV[MT], facecolor=(0.5, 0.68, 0.9), edgecolor="none", alpha=.85))
    c = MV.mean(0); rng = (MV.max(0) - MV.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("Poisson 재구성 솔리드(파랑) vs 원본 표면점(빨강)")
plt.tight_layout(); plt.savefig(f"{OUT}/grip_solid.png", dpi=85)
print("saved grip_solid.png")

#!/usr/bin/env python3
"""IMU 보드 방향 점검 — 34/30/10 변을 라벨링해 현재 배치 확인.
실행: cad/.venv/bin/python cad/imu_orient.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
import parts as PT
bL, bW, bH = PT.IMU_BOARD          # 34, 30, 10
IMU_AT = (0, 16, -60); TILT = PT.IMU_TILT_DEG

# 현재 코드와 동일 배치: box(bW=30 →X, bH=10 →Y, bL=34 →Z), 30° about X
board = trimesh.creation.box(extents=(bW, bH, bL))
R = trimesh.transformations.rotation_matrix(np.radians(TILT), [1, 0, 0])
board.apply_transform(R); board.apply_translation(IMU_AT)
# 변 라벨용: 각 축 끝점
def edge_mid(axis_local, length):
    v = R[:3, :3] @ (np.array(axis_local) * length / 2)
    return np.array(IMU_AT) + v
labels = [("34(긴변)", edge_mid([0, 0, 1], bL), "red"),
          ("30(넓은변)", edge_mid([1, 0, 0], bW), "blue"),
          ("10(두께)", edge_mid([0, 1, 0], bH), "green")]
grip = trimesh.load(f"{OUT}/controller_v3.stl")

fig = plt.figure(figsize=(15, 6.5))
for k, (el, az, ttl) in enumerate([(3, -90, "front (-Y 정면)"), (3, 0, "side (+X 측면)"), (18, -65, "iso")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(grip.vertices[grip.faces], facecolor=(.85, .88, .92), edgecolor="none", alpha=.12))
    ax.add_collection3d(Poly3DCollection(board.vertices[board.faces], facecolor=(.95, .55, .25), edgecolor="k", linewidths=.2, alpha=.9))
    for t, p, col in labels:
        ax.text(*p, t, color=col, fontsize=9, weight="bold")
    c = np.array([0, 14, -55]); r = 30
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=10); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle(f"현재 IMU 배치 — 긴변34=Z(세로), 넓은면34×30=앞뒤(±Y), 두께10=슬롯  /  {TILT:.0f}° 기울임")
plt.tight_layout(); plt.savefig(f"{OUT}/imu_orient.png", dpi=95)
print("saved imu_orient.png | 긴변34→Z, 넓은면→±Y, 두께10→Y슬롯, tilt", TILT)

#!/usr/bin/env python3
"""택트(중지) 영역 점검 — 좌우 반쪽 시임면 + 앞면 + 레버 배치.
실행: cad/.venv/bin/python cad/ss_inspect.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"] = "Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
gr = trimesh.load(f"{OUT}/grip_right_v3.stl")
gl = trimesh.load(f"{OUT}/grip_left_v3.stl")
lift = trimesh.load(f"{OUT}/trigger_lift_v3.stl")
C = np.array([0, 6, -50]); R = 18   # SS 영역 중심


def view(ax, meshes_cols, el, az, ttl):
    for m, col, al in meshes_cols:
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=col, edgecolor=(0, 0, 0, .15), lw=.1, alpha=al))
    ax.set_xlim(C[0]-R, C[0]+R); ax.set_ylim(C[1]-R, C[1]+R); ax.set_zlim(C[2]-R, C[2]+R)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))


fig = plt.figure(figsize=(17, 6))
# 1) +X 반쪽(grip_right) 시임면서 봄 (-X방향 시점) → 포켓 + 파일럿
view(fig.add_subplot(1, 3, 1, projection="3d"), [(gr, (.6, .75, .9), 1)], 12, 95,
     "+X반쪽(grip_right) — 시임쪽서 본 택트포켓")
# 2) -X 반쪽(grip_left) 시임면서 봄 (+X방향 시점) → 포켓 + IMU 플레이트
view(fig.add_subplot(1, 3, 2, projection="3d"), [(gl, (.9, .8, .65), 1)], 12, -95,
     "-X반쪽(grip_left) — 택트포켓 + IMU 플레이트·홀")
# 3) 레버(따로) 배치 — +X반쪽 + 리프트레버
view(fig.add_subplot(1, 3, 3, projection="3d"), [(gr, (.6, .75, .9), .25), (lift, (.95, .5, .2), 1)], 10, -70,
     "리프트 레버(주황, 따로 STL)가 끼워진 모습")
fig.suptitle("택트(중지) 영역 — 중앙 포켓(좌우반쪽 캡처), 앞은 레버통로, 레버는 별도 부품")
plt.tight_layout(); plt.savefig(f"{OUT}/ss_inspect.png", dpi=98); print("saved ss_inspect.png")

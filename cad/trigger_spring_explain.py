#!/usr/bin/env python3
"""트리거 토션스프링 설치 설명 — 코일 위치·그립포스트·레버다리·프리로드.
실행: cad/.venv/bin/python cad/trigger_spring_explain.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
g = trimesh.load(f"{OUT}/grip_body_v3.stl")
lev = trimesh.load(f"{OUT}/trigger_lever_v3.stl")
C = (0, -25, -9)          # 피벗
POST = (7.5, -22, -6)     # 그립 다리 포스트
COIL = (6, -25, -9)       # 코일(핀 위, x5~7 틈)

fig = plt.figure(figsize=(15, 6.5))
# side (YZ평면, x=6 단면 — 코일 있는 면)
ax = fig.add_subplot(1, 2, 1)
for m, col, lw in [(g, (.7, .7, .7), .8), (lev, "royalblue", 1.4)]:
    s = m.section(plane_origin=[6, 0, 0], plane_normal=[1, 0, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], color=col, lw=lw)
ax.scatter(C[1], C[2], c="k", s=40); ax.text(C[1] - .5, C[2] + 1, "피벗 Φ3핀", fontsize=8)
ax.add_patch(plt.Circle((C[1], C[2]), 2.0, fill=False, ec="purple", lw=1.6)); ax.text(C[1], C[2] - 4.5, "코일\n(ID3.2)", color="purple", fontsize=8, ha="center")
ax.scatter(POST[1], POST[2], c="red", s=50); ax.text(POST[1] + .5, POST[2], "그립 다리\n포스트", color="red", fontsize=8)
ax.annotate("", xy=(POST[1], POST[2]), xytext=(C[1], C[2]), arrowprops=dict(arrowstyle="->", color="red", lw=1.3))
ax.annotate("", xy=(-25.5, -20), xytext=(C[1], C[2]), arrowprops=dict(arrowstyle="->", color="blue", lw=1.3))
ax.text(-27, -19, "레버 다리\n(암 따라↓)", color="blue", fontsize=8, ha="center")
ax.text(-34, -2, "다리 둘을 살짝 벌려 끼우면\n그게 복귀 프리로드", color="green", fontsize=8)
ax.set_aspect("equal"); ax.set_xlabel("Y(앞←)"); ax.set_ylabel("Z(위)"); ax.invert_xaxis()
ax.set_title("트리거 스프링 (x=6 단면) — 코일·포스트·다리", fontsize=10); ax.grid(alpha=.3)
# iso 줌
ax = fig.add_subplot(1, 2, 2, projection="3d")
ax.add_collection3d(Poly3DCollection(g.vertices[g.faces], facecolor=(.6, .75, .9), edgecolor="none", alpha=.5))
ax.add_collection3d(Poly3DCollection(lev.vertices[lev.faces], facecolor=(.3, .5, .95), edgecolor="none", alpha=.9))
ax.scatter(*POST, c="red", s=50); ax.scatter(*COIL, c="purple", s=40)
c = np.array([4, -22, -10]); r = 16
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(16, -60); ax.set_title("줌 (레버=파랑, 포스트=빨강, 코일=보라)", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("토션스프링(0.4×ID3.2) 설치 — 코일 핀에, 그립다리→포스트, 레버다리→암, 벌려끼워 프리로드")
plt.tight_layout(); plt.savefig(f"{OUT}/trigger_spring_explain.png", dpi=95)
print("saved trigger_spring_explain.png")

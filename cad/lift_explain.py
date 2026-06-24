#!/usr/bin/env python3
"""리프트(SS-5GL) 영역 설명 — 포켓·레버창·나사홀이 뭔지 + 나사홀 방향 확인.
실행: cad/.venv/bin/python cad/lift_explain.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
g = trimesh.load(f"{OUT}/grip_body_v3.stl")
LIFT = (11.5, 17.8, -34); SS = (20.0, 6.5, 10.2)

# 나사홀 위치/방향 확인 (contains: 빔=False)
print("=== 리프트 나사홀 확인 ===")
for dz in (-6, 6):
    p_hole = (LIFT[0] - SS[1] / 2 - 2, LIFT[1], LIFT[2] + dz)   # 포켓 뒤(-X) 나사홀 영역
    print(f"  나사홀 z{LIFT[2]+dz} (브래킷속 {tuple(round(x,1) for x in p_hole)}): contains={bool(g.contains([p_hole])[0])} (False=구멍)")
print("  포켓 중심(스위치 자리):", bool(g.contains([LIFT])[0]), "(False=빔)")
print("  외부 표면쪽(x=20, 뚫렸나):", bool(g.contains([(20, LIFT[1], LIFT[2])])[0]))

# z = LIFT z 단면
fig = plt.figure(figsize=(16, 6))
ax = fig.add_subplot(1, 2, 1)
s = g.section(plane_origin=[0, 0, LIFT[2]], plane_normal=[0, 0, 1])
if s:
    for e in s.entities:
        pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 1], "gray", lw=1)
ax.add_patch(plt.Rectangle((LIFT[0] - SS[1] / 2, LIFT[1] - SS[2] / 2), SS[1], SS[2], fill=False, ec="green", lw=1.5))
ax.text(LIFT[0], LIFT[1] + 6, "SS-5GL 본체 포켓", color="green", fontsize=8, ha="center")
ax.annotate("나사홀(스위치 고정)", xy=(LIFT[0] - SS[1] / 2 - 2, LIFT[1]), xytext=(0, 30),
            arrowprops=dict(arrowstyle="->", color="red"), color="red", fontsize=8)
ax.annotate("레버창(측면, 레버 나옴)", xy=(19, LIFT[1]), xytext=(24, 5),
            arrowprops=dict(arrowstyle="->", color="blue"), color="blue", fontsize=8)
ax.set_aspect("equal"); ax.set_xlabel("X(측면→)"); ax.set_ylabel("Y"); ax.set_title(f"리프트 단면 z={LIFT[2]}", fontsize=10); ax.grid(alpha=.3)
# iso 줌
ax = fig.add_subplot(1, 2, 2, projection="3d")
ax.add_collection3d(Poly3DCollection(g.vertices[g.faces], facecolor=(.6, .75, .9), edgecolor="none", alpha=.55))
c = np.array([12, 16, -34]); r = 18
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(12, -50); ax.set_title("리프트 영역 줌", fontsize=10); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("리프트(SS-5GL) — 네모홀=스위치포켓, 나사홀=스위치 고정 M2, 측면창=레버 통로")
plt.tight_layout(); plt.savefig(f"{OUT}/lift_explain.png", dpi=95)
print("saved lift_explain.png")

#!/usr/bin/env python3
"""트리거 설명 — 검지 핑거패드 위치 + 피벗 핀홀/자석포켓 확인.
실행: cad/.venv/bin/python cad/trigger_explain.py
"""
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
lev = trimesh.load(f"{OUT}/trigger_lever_v3.stl")
grip = trimesh.load(f"{OUT}/grip_body_v3.stl")
C = [0, -25, -9]

# 구멍 존재 확인 (contains: 구멍이면 False)
chk = {
    "피벗 핀홀 중심(0,-25,-9)": (0, -25, -9),
    "핀홀 옆 허브(솔리드)": (3, -25, -9),
    "자석 포켓(0,-24.5,-18)": (0, -24.5, -18),
}
print("=== 트리거 레버 구멍 확인 (False=구멍/빔, True=솔리드) ===")
for nm, p in chk.items():
    print(f"  {nm}: {bool(lev.contains([p])[0])}")
bz = lev.bounds
print(f"레버 bbox Y[{bz[0][1]:.0f},{bz[1][1]:.0f}] Z[{bz[0][2]:.0f},{bz[1][2]:.0f}]  (핑거패드=Y최소·Z최소 끝)")

fig = plt.figure(figsize=(17, 6))
# 1) 레버 단독 + 라벨
ax = fig.add_subplot(1, 3, 1, projection="3d")
ax.add_collection3d(Poly3DCollection(lev.vertices[lev.faces], facecolor=(.3, .55, .95), edgecolor="k", lw=.1, alpha=.9))
ax.text(0, C[1], C[2], "  피벗 핀홀 Φ3.4\n  (관통)", color="red", fontsize=8)
ax.text(0, -23.5, -18, "  자석포켓 Ø5.2", color="green", fontsize=8)
ax.text(0, bz[0][1], bz[0][2], "검지 핑거패드\n(여기 당김)", color="darkorange", fontsize=9, weight="bold")
c = lev.vertices.mean(0); r = (lev.vertices.max(0) - lev.vertices.min(0)).max() / 2 + 2
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(16, -70); ax.set_title("트리거 레버 (구멍 2개 + 핑거패드)", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
# 2) 단면 (x=0) — 핀홀·포켓 보임
ax = fig.add_subplot(1, 3, 2)
for m, col in [(grip, (.7, .7, .7)), (lev, "royalblue")]:
    s = m.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], color=col, lw=1.3)
ax.scatter(C[1], C[2], c="red", s=60, zorder=5); ax.text(C[1] + 1, C[2], "핀홀 Φ3", color="red", fontsize=8)
ax.scatter(-23.5, -18, c="green", s=40, zorder=5); ax.text(-23.5 + 1, -18, "자석", color="green", fontsize=8)
ax.annotate("검지 당김", xy=(bz[0][1], bz[0][2] + 2), xytext=(bz[0][1] - 6, bz[0][2] + 8),
            arrowprops=dict(arrowstyle="->", color="darkorange"), color="darkorange", fontsize=9)
ax.set_aspect("equal"); ax.invert_xaxis(); ax.set_xlabel("Y(앞←)"); ax.set_ylabel("Z(위)")
ax.set_title("단면 — 레버(파랑)+그립(회색): 핀홀·자석·핑거패드", fontsize=9); ax.grid(alpha=.3)
# 3) 그립 안 배치 (핑거패드가 앞으로 돌출)
ax = fig.add_subplot(1, 3, 3, projection="3d")
ax.add_collection3d(Poly3DCollection(grip.vertices[grip.faces], facecolor=(.8, .85, .9), edgecolor="none", alpha=.12))
ax.add_collection3d(Poly3DCollection(lev.vertices[lev.faces], facecolor=(.95, .55, .2), edgecolor="none", alpha=.95))
c2 = np.array([0, -15, -15]); r2 = 26
ax.set_xlim(c2[0]-r2, c2[0]+r2); ax.set_ylim(c2[1]-r2, c2[1]+r2); ax.set_zlim(c2[2]-r2, c2[2]+r2)
ax.view_init(10, -78); ax.set_title("그립 장착 — 핑거패드 앞으로 돌출(검지)", fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("검지 트리거 — 핑거패드(당김부) + 핀홀·자석포켓 (스위치 아닌 홀센서식)")
plt.tight_layout(); plt.savefig(f"{OUT}/trigger_explain.png", dpi=95)
print("saved trigger_explain.png")

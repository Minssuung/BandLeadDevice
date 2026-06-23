#!/usr/bin/env python3
"""V3 Stage2b — 검지 트리거: 레버(Φ3 피벗) + 자석Φ5×2 + AH49E 홀 + 복귀.
당기면 자석이 홀에 가까워져(갭 3→1mm) 아날로그 그리퍼 센싱. rest↔pull 동작 렌더.
실행: cad/.venv/bin/python cad/trigger_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadquery as cq
import trimesh
from trimesh import transformations as TF
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parts as PT

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
C = np.array([0.0, -25.0, -9.0])    # 피벗 중심, 축 = X (캐리어 안 닿게 -9로 낮춤)
PULL = 16.0                          # 당김 각(°)
MAG = np.array([0.0, -23.5, -18.0])  # 자석 위치(레버 rear면, 허브 아래)
HALL = np.array([0.0, -17.0, -18.0]) # AH49E 위치(그립 고정, 자석 뒤)

# ── 레버 (월드좌표, rest) ──
# 허브 +X쪽 축소(x-6..+3) → 보스(x7)와 틈 x5~7 확보(토션스프링 코일 ID3.2/OD4 자리)
hub = cq.Workplane("YZ", origin=(-6, C[1], C[2])).circle(5).extrude(9)
arm = cq.Workplane("XY", origin=(0, -25.5, -16)).box(10, 4, 22)
lever = hub.union(arm)
# 피벗 보어 Φ3.4 — union 후에 뚫어야 arm이 안 메움 (X축 관통)
lever = lever.cut(cq.Workplane("YZ", origin=(-7, C[1], C[2])).circle((PT.TRIG_PIVOT_DIA + 0.4) / 2).extrude(14))
# 자석 포켓 Φ5.2×2.2 — rear(+Y)면에서 -Y로 파냄(자석이 +Y=홀쪽을 향함)
lever = lever.cut(cq.Workplane("XZ", origin=(MAG[0], -23.5, MAG[2])).circle((PT.MAGNET_DIA + 0.2) / 2).extrude(PT.MAGNET_THK + 0.2))
lever = lever.edges("|X and <Z").fillet(1.8)
cq.exporters.export(lever, f"{OUT}/trigger_lever_v3.stl")
print("lever vol:", round(lever.val().Volume()))

# ── 피벗 보스 2 + 핀 + 홀 마운트 (그립측, 별도) ──
mount = cq.Workplane("XY")
for sx in (-8, 8):   # 레버 양옆 보스
    b = (cq.Workplane("YZ", origin=(sx, C[1], C[2])).circle(4).extrude(2.5 if sx < 0 else -2.5)
         .cut(cq.Workplane("YZ", origin=(sx, C[1], C[2])).circle(PT.TRIG_PIVOT_DIA / 2).extrude(3 if sx < 0 else -3)))
    mount = mount.union(b)
# AH49E 홀 포켓 블록 (그립 고정, 자석 마주봄)
hallblk = (cq.Workplane("XY", origin=(HALL[0], HALL[1] - 1, HALL[2])).box(8, 4, 6)
           .cut(cq.Workplane("XZ", origin=(HALL[0], HALL[1] + 1, HALL[2])).rect(4.4, 3.2).extrude(-3)))
mount = mount.union(hallblk)
cq.exporters.export(mount, f"{OUT}/trigger_mount_v3.stl")
print("mount(보스+홀) vol:", round(mount.val().Volume()))


# ── 동작 계산: 자석을 C 기준 X축으로 회전 → 홀과의 갭 ──
def rotX(p, deg, c):
    a = np.radians(deg); R = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    return c + R @ (p - c)


# 당김 방향(갭 줄어드는 쪽) 자동 선택
g_p = np.linalg.norm(rotX(MAG, +PULL, C) - HALL); g_m = np.linalg.norm(rotX(MAG, -PULL, C) - HALL)
SGN = +1 if g_p < g_m else -1
mag_rest, mag_pull = MAG, rotX(MAG, SGN * PULL, C)
gap_rest = np.linalg.norm(mag_rest - HALL) - PT.MAGNET_THK / 2
gap_pull = np.linalg.norm(mag_pull - HALL) - PT.MAGNET_THK / 2
print(f"갭: rest {gap_rest:.1f}mm → pull {gap_pull:.1f}mm (목표 {PT.TRIG_GAP})")


def mesh(wp, tol=0.3):
    vv, tt = wp.val().tessellate(tol)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in vv]), np.array(tt))

lm = mesh(lever); mm = mesh(mount)
lm_pull = lm.copy(); lm_pull.apply_transform(TF.rotation_matrix(np.radians(SGN * PULL), [1, 0, 0], C))
try:
    grip = trimesh.load(f"{OUT}/controller_v3.stl")
except Exception:
    grip = None

# ── 렌더: side(YZ 스윙평면) rest vs pull + iso ──
fig = plt.figure(figsize=(16, 6.5))
# side
ax = fig.add_subplot(1, 2, 1)
if grip is not None:
    s = grip.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], color=(.7, .7, .7), lw=.8)
for m, col, lab in [(lm, "royalblue", "레버 rest"), (lm_pull, "orange", f"레버 pull {PULL:.0f}°")]:
    s = m.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], col, lw=1.4)
        ax.plot([], [], col, label=lab)
sh = mm.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
if sh:
    for e in sh.entities:
        pp = sh.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], "green", lw=1.2)
    ax.plot([], [], "green", label="피벗보스+홀마운트")
ax.scatter(*HALL[1:], c="darkgreen", s=40); ax.text(HALL[1] + .5, HALL[2], "AH49E", color="darkgreen", fontsize=8)
for mp, c2, t in [(mag_rest, "blue", f"자석 갭{gap_rest:.1f}"), (mag_pull, "red", f"갭{gap_pull:.1f}")]:
    ax.scatter(*mp[1:], c=c2, s=30); ax.plot([mp[1], HALL[1]], [mp[2], HALL[2]], c2, ls=":", lw=.8)
ax.scatter(*C[1:], c="k", s=25); ax.text(C[1], C[2] + .6, "피벗 Φ3", fontsize=8)
ax.set_aspect("equal"); ax.set_xlabel("Y(앞←)"); ax.set_ylabel("Z(위)"); ax.invert_xaxis()
ax.set_title("검지 트리거 동작 (당기면 자석→홀 갭 ↓)", fontsize=10); ax.legend(loc="lower right", fontsize=8); ax.grid(alpha=.3)
# iso
ax = fig.add_subplot(1, 2, 2, projection="3d")
if grip is not None:
    ax.add_collection3d(Poly3DCollection(grip.vertices[grip.faces], facecolor=(.8, .85, .9), edgecolor="none", alpha=.12))
for m, col in [(lm, (.3, .5, .95)), (mm, (.3, .7, .4))]:
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolor=col, edgecolor="none", alpha=.95))
P = lm.vertices; c = np.array([0, -20, -14]); r = 26
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.view_init(16, -72); ax.set_title("배치(그립 앞면)", fontsize=10); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("V3 Stage2b — 검지 트리거: 레버 피벗 + 자석Φ5×2 + AH49E 홀 (아날로그 그리퍼)")
plt.tight_layout(); plt.savefig(f"{OUT}/trigger_v3.png", dpi=92)
print("saved trigger_v3.png")

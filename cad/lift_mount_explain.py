#!/usr/bin/env python3
"""리프트 마운트 설명 — SS-5GL 내부고정(크래들+18°틸트, 시임서 나사) + 레버 피벗.
실행: cad/.venv/bin/python cad/lift_mount_explain.py
"""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"] = "Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
g = trimesh.load(f"{OUT}/grip_body_v3.stl")
SS = (0, 7, -52); PIV = (0, -1, -36); th = math.radians(18)


def tilt(dy, dz):  # SS 기준 X축 18° 회전 → (y,z)
    return (SS[1] + dy * math.cos(th) - dz * math.sin(th), SS[2] + dy * math.sin(th) + dz * math.cos(th))


fig = plt.figure(figsize=(15, 6.5))
# 패널1: y≈7 수평단면(XZ) — 크래들(솔리드)·중앙포켓·시임서 나사(블라인드 파일럿)·머리 카운터보어
ax = fig.add_subplot(1, 2, 1)
s = g.section(plane_origin=[0, 7, 0], plane_normal=[0, 1, 0])
if s:
    for e in s.entities:
        pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 2], color="gray", lw=1)
ax.text(0, -38, "크래들=손잡이 안 솔리드 충전\n(나사 물릴 재료)", color="saddlebrown", fontsize=8, ha="center")
ax.add_patch(plt.Rectangle((-3.55, -62.6), 7.1, 20.6, fill=False, ec="green", lw=1.5))
ax.text(0, -52, "SS-5GL\n포켓\n(시임 중앙)", color="green", fontsize=7.5, ha="center", va="center")
for dz in (4.75, -4.75):
    _, z = tilt(0, dz)
    ax.annotate("", xy=(8.5, z), xytext=(-0.5, z), arrowprops=dict(arrowstyle="->", color="red", lw=1.6))   # 시임서 +X로
    ax.plot([-6.5, -3.55], [z, z], color="blue", lw=3, solid_capstyle="butt")                                # -X 머리 카운터보어
ax.text(9, -47.5, "나사: 시임(-X)서\n+X로 박음\n→ +X반쪽 파일럿\n(블라인드, 바깥X)", color="red", fontsize=7.5, va="center")
ax.text(-13, -60, "머리 카운터보어\n(-X반쪽이 덮음)", color="blue", fontsize=7.5)
ax.axvline(0, color="purple", ls=":", lw=.8); ax.text(0.5, -40, "시임", color="purple", fontsize=7.5)
ax.set_aspect("equal"); ax.set_xlabel("X(옆 ←→)"); ax.set_ylabel("Z(위↑)")
ax.set_title("SS-5GL 내부고정 (y≈7 단면) — 외부구멍 0, 시임서 조립", fontsize=10); ax.grid(alpha=.3)
# 패널2: x=0 단면 — 레버 피벗 + 토션스프링
ax = fig.add_subplot(1, 2, 2)
s = g.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
if s:
    for e in s.entities:
        pp = s.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], color="gray", lw=1)
try:
    lv = trimesh.load(f"{OUT}/trigger_lift_v3.stl"); sl = lv.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if sl:
        for e in sl.entities:
            pp = sl.vertices[e.points]; ax.plot(pp[:, 1], pp[:, 2], "royalblue", lw=1.3)
        ax.plot([], [], "royalblue", label="리프트 레버")
except Exception:
    pass
ax.scatter(PIV[1], PIV[2], c="k", s=40); ax.annotate("피벗: Φ3 핀이\n양쪽 보스(x±8) 관통\n→ 레버 끼움", xy=(PIV[1], PIV[2]), xytext=(15, -25), arrowprops=dict(arrowstyle="->"), fontsize=8)
ax.annotate("토션스프링\n(핀에 코일, 레버 복귀)", xy=(2, -31), xytext=(18, -42), arrowprops=dict(arrowstyle="->", color="darkorange"), color="darkorange", fontsize=8)
ax.set_aspect("equal"); ax.set_xlabel("Y(앞←)"); ax.set_ylabel("Z(위↑)"); ax.invert_xaxis()
ax.set_title("리프트 레버 피벗 (x=0 단면)", fontsize=10); ax.legend(fontsize=8); ax.grid(alpha=.3)
fig.suptitle("리프트 마운트 — SS-5GL 내부고정(크래들+18°틸트, 시임서 나사2, 외부구멍0) + 레버 피벗·토션스프링")
plt.tight_layout(); plt.savefig(f"{OUT}/lift_mount_explain.png", dpi=95); print("saved lift_mount_explain.png")

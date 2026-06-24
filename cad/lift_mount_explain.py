#!/usr/bin/env python3
"""리프트 마운트 설명 — SS-5GL 2나사(옆 관통) + 2단 트리거 레버 피벗.
실행: cad/.venv/bin/python cad/lift_mount_explain.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"] = "Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
g = trimesh.load(f"{OUT}/grip_body_v3.stl")
SS = (0, 7, -52); PIV = (0, -1, -36)

fig = plt.figure(figsize=(15, 6.5))
# 패널1: y=7 수평단면(XZ) — SS-5GL 포켓 + 옆 나사2 (이 단면이라 옆 관통홀 보임)
ax = fig.add_subplot(1, 2, 1)
s = g.section(plane_origin=[0, 7, 0], plane_normal=[0, 1, 0])
if s:
    for e in s.entities:
        pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 2], color="gray", lw=1)
ax.add_patch(plt.Rectangle((-3.55, -62.6), 7.1, 20.6, fill=False, ec="green", lw=1.5))
ax.text(0, -40, "SS-5GL 포켓\n(중앙=시임, 좌우반쪽이 캡처)", color="green", fontsize=8, ha="center")
for z in (-47.25, -56.75):
    ax.annotate("", xy=(7, z), xytext=(-7, z), arrowprops=dict(arrowstyle="<->", color="red", lw=1.4))
ax.text(9, -52, "나사 2개\n(Ø2.4, 피치9.5)\n옆면 관통\n→스위치 고정", color="red", fontsize=8, va="center")
ax.text(-15, -38, "시임(x=0)", color="purple", fontsize=8); ax.axvline(0, color="purple", ls=":", lw=.7)
ax.set_aspect("equal"); ax.set_xlabel("X(옆 ←→)"); ax.set_ylabel("Z(위↑)")
ax.set_title("SS-5GL 마운트 (y=7 단면) — 옆으로 나사 2개 관통", fontsize=10); ax.grid(alpha=.3)
# 패널2: x=0 단면 — 레버 피벗 + 보스 + 스프링 포스트
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
fig.suptitle("리프트 마운트 — SS-5GL 옆나사2 + 레버 Φ3핀 피벗 + 토션스프링")
plt.tight_layout(); plt.savefig(f"{OUT}/lift_mount_explain.png", dpi=95); print("saved lift_mount_explain.png")

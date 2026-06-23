#!/usr/bin/env python3
"""버튼 단면 설명 — 캐리어 가이드홀 + 누름캡 + 택트(보드 표면) + 만능보드 적층.
실행: cad/.venv/bin/python cad/button_explain.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
cc = trimesh.load(f"{OUT}/carrier_v3.stl")
caps = trimesh.load(f"{OUT}/button_caps_v3.stl")
pb = trimesh.load(f"{OUT}/perfboard_v3.stl")
BX = -11.0   # 왼쪽 버튼 x

fig, ax = plt.subplots(figsize=(9, 8))
# y=16 단면 (X-Z 평면) — 버튼 줄
for m, col, lab in [(cc, (.3, .55, .95), "캐리어"), (caps, (.95, .5, .2), "누름캡"), (pb, (.2, .7, .3), "만능보드")]:
    s = m.section(plane_origin=[0, 16, 0], plane_normal=[0, 1, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 2], color=col, lw=1.8)
        ax.plot([], [], color=col, lw=2, label=lab)
# 택트(물리부품, 미모델) 6×6×3.5 + 액추에이터 — 보드(z-7) 위에 그림
for bx in (-11, 11):
    ax.add_patch(plt.Rectangle((bx - 3, -7), 6, 3.5, fc="0.6", ec="k", lw=1))         # 택트 본체
    ax.add_patch(plt.Rectangle((bx - 1.75, -3.5), 3.5, 0.9, fc="0.4", ec="k", lw=1))  # 액추에이터
ax.text(11 + 4, -5.2, "택트 6×6\n(보드 표면 납땜)", fontsize=9, va="center")
# 라벨/치수
ax.annotate("Ø6.2 가이드홀\n(스템 Ø5.8 꽉맞음)", xy=(BX + 3.1, -1.5), xytext=(BX + 9, 3),
            arrowprops=dict(arrowstyle="->"), fontsize=9)
ax.annotate("디스크 Ø10\n(턱 — 위로 안빠짐)", xy=(BX, 2.5), xytext=(BX - 22, 5),
            arrowprops=dict(arrowstyle="->"), fontsize=9)
ax.annotate("스템→액추에이터\n(누르면 택트 ON)", xy=(BX, -2.5), xytext=(BX - 26, -3),
            arrowprops=dict(arrowstyle="->"), fontsize=9)
ax.text(-30, -8.8, "조립: 보드 먼저 → 캡 떨어뜨려 끼움. 디스크(위)+택트(아래)가 캡을 가둠", fontsize=8, color="navy")
ax.axhline(0, color="0.85", lw=.5)
ax.set_aspect("equal"); ax.set_xlabel("X(mm)"); ax.set_ylabel("Z(mm) 위↑")
ax.set_xlim(-34, 26); ax.set_ylim(-11, 7)
ax.legend(loc="upper right", fontsize=9); ax.grid(alpha=.3)
ax.set_title("버튼 단면 (y=16) — 캐리어 가이드홀 + 누름캡 + 택트(보드 표면) 적층", fontsize=11)
plt.tight_layout(); plt.savefig(f"{OUT}/button_explain.png", dpi=100)
print("saved button_explain.png")

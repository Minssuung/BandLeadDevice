#!/usr/bin/env python3
"""버튼 단면 — 캐리어 홀더에 택트 고정(플랜지 위 + 리테이너 링 아래 사이 갇힘) + 누름캡.
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
BX = 11.0   # 버튼 x (kbd)

fig, ax = plt.subplots(figsize=(9.5, 8))
for m, col, lab in [(cc, (.3, .55, .95), "캐리어(홀더)"), (caps, (.95, .5, .2), "누름캡")]:
    s = m.section(plane_origin=[0, 16, 0], plane_normal=[0, 1, 0])
    if s:
        for e in s.entities:
            pp = s.vertices[e.points]; ax.plot(pp[:, 0], pp[:, 2], color=col, lw=1.8)
        ax.plot([], [], color=col, lw=2, label=lab)
# 택트(물리부품) 6×6×3.5: body z-6.5..-3, 액추에이터 z-3..-2, 핀 아래로
for bx in (-11, 11):
    ax.add_patch(plt.Rectangle((bx-3, -6.5), 6, 3.5, fc="0.6", ec="k", lw=1))        # body
    ax.add_patch(plt.Rectangle((bx-1.75, -3), 3.5, 1.0, fc="0.4", ec="k", lw=1))     # 액추에이터(위로)
    for px in (bx-2, bx+2):
        ax.plot([px, px], [-6.5, -9], "k", lw=1.2)                                   # 핀(아래로→허브)
ax.text(11+4.5, -5, "택트 6×6\n(밑서 삽입)", fontsize=9, va="center")
ax.text(11+4.5, -8.5, "핀↓ 허브보드", fontsize=8, color="navy", va="center")
ax.annotate("플랜지(Ø6.2 턱)\n택트 위로 못감", xy=(BX-3.1, -3), xytext=(BX+5, 0.5), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
ax.annotate("리테이너 링(z-6.5)\n택트 아래로 못감\n(누를때 안빠짐)", xy=(BX-2.6, -6.6), xytext=(BX-22, -8.5), arrowprops=dict(arrowstyle="->", color="red"), color="red", fontsize=8.5)
ax.annotate("캡 디스크(턱)\n+스템→액추에이터", xy=(BX, 2), xytext=(BX-24, 3.5), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
ax.text(-26, -10.3, "택트는 플랜지(위)+링(아래) 사이에 갇힘 → 눌러도 빠지지 않음. 밑서 끼우면 링 스냅통과", fontsize=8, color="darkred")
ax.axhline(0, color="0.85", lw=.5)
ax.set_aspect("equal"); ax.set_xlabel("X(mm)"); ax.set_ylabel("Z(mm) 위↑")
ax.set_xlim(-30, 24); ax.set_ylim(-10.8, 5)
ax.legend(loc="upper right", fontsize=9); ax.grid(alpha=.3)
ax.set_title("버튼 단면 — 택트가 캐리어 홀더에 고정(플랜지+링 사이) + 누름캡", fontsize=11)
plt.tight_layout(); plt.savefig(f"{OUT}/button_explain.png", dpi=100); print("saved button_explain.png")

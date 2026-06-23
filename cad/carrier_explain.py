#!/usr/bin/env python3
"""캐리어 구멍/나사구멍 설명 — 무엇이 무엇인지 라벨.
실행: cad/.venv/bin/python cad/carrier_explain.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
c = trimesh.load(f"{OUT}/carrier_v3.stl")
JOY = (0, -9)
BTN = {"A": (-14, 14), "B": (0, 14), "menu": (14, 14)}
POSTS = [(-18, -18), (18, -18), (-18, 16), (18, 16)]

labels = [("돔홀 Ø18\n(조이스틱 스틱)", (*JOY, 0), "green")]
labels += [(f"버튼홀 Ø8\n({k}, 캡)", (x, y, 0), "red") for k, (x, y) in BTN.items()]
labels += [("PCB 스탠드오프\nM2(보드 고정)", (x, y, -5), "blue") for (x, y) in POSTS]

fig = plt.figure(figsize=(15, 7))
for k, (el, az, ttl) in enumerate([(88, -90, "top (윗면 — 버튼·돔 둥근홀)"), (-88, -90, "bottom (밑면 — 스탠드오프 M2)"), (20, -60, "iso")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(c.vertices[c.faces], facecolor=(.4, .6, .95), edgecolor="none", alpha=.6))
    for t, p, col in labels:
        ax.scatter(*p, color=col, s=25)
        if k == 2:
            ax.text(*p, t, color=col, fontsize=7)
    ce = c.vertices.mean(0); r = (c.vertices.max(0) - c.vertices.min(0)).max() / 2
    ax.set_xlim(ce[0]-r, ce[0]+r); ax.set_ylim(ce[1]-r, ce[1]+r); ax.set_zlim(ce[2]-r, ce[2]+r)
    ax.view_init(el, az); ax.set_title(ttl, fontsize=9); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
fig.suptitle("캐리어 구멍 정리 — 둥근홀: 돔(초록)+버튼3(빨강) / 나사구멍: PCB 스탠드오프4 M2(파랑)")
plt.tight_layout(); plt.savefig(f"{OUT}/carrier_explain.png", dpi=95)
print("saved carrier_explain.png")
print("스탠드오프 4개 위치:", POSTS, "→ 만능보드를 여기 M2로 고정")
print("버튼홀 3개(Ø8):", list(BTN.values()), "→ 누름캡 / 돔홀 Ø18:", JOY)

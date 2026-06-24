#!/usr/bin/env python3
"""원본 컨트롤러 솔리드를 3투영으로 색깔별 렌더 — 어느 솔리드가 링인지 눈으로 확인.
실행: cad/.venv/bin/python cad/render.py [솔리드인덱스 ...]
인자를 주면 그 솔리드만(나머지 회색) 강조 렌더.
"""
import sys
import matplotlib
matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
import numpy as np
import cadquery as cq

SRC = "/home/minsung/Downloads/touch-accessory-guidelines-2.0/Touch for Quest and Rift S/Left Controller for Quest and Rift S.stp"
OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out/original_solids.png"

highlight = [int(a) for a in sys.argv[1:] if a.isdigit()]

r = cq.importers.importStep(SRC)
solids = r.solids().vals()

def verts(s):
    vs, _ = s.tessellate(0.6)
    return np.array([(v.x, v.y, v.z) for v in vs]) if vs else np.empty((0, 3))

V = [verts(s) for s in solids]
cmap = plt.cm.tab20(np.linspace(0, 1, len(solids)))

def panel(ax, a, b, xl, yl):
    for i, v in enumerate(V):
        if len(v) == 0:
            continue
        if highlight:
            col = cmap[i] if i in highlight else (0.85, 0.85, 0.85, 0.5)
            z = 3 if i in highlight else 1
        else:
            col, z = cmap[i], 2
        ax.scatter(v[:, a], v[:, b], s=1.5, color=col, zorder=z, label=str(i) if not highlight or i in highlight else None)
    ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_aspect("equal"); ax.grid(alpha=.3)

fig, axs = plt.subplots(1, 3, figsize=(19, 7))
panel(axs[0], 0, 1, "X", "Y (top 뷰)")
panel(axs[1], 0, 2, "X", "Z (front 뷰)")
panel(axs[2], 1, 2, "Y", "Z (side 뷰)")
h, l = axs[2].get_legend_handles_labels()
axs[2].legend(h, l, markerscale=6, ncol=2, fontsize=7, title="solid idx")
ttl = f"원본 컨트롤러 솔리드 (강조: {highlight})" if highlight else "원본 컨트롤러 — 14솔리드 색깔별"
fig.suptitle(ttl)
plt.tight_layout()
plt.savefig(OUT, dpi=85)
print("saved", OUT)

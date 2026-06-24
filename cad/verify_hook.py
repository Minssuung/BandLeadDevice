#!/usr/bin/env python3
"""훅 잠김(retention) 검증 — 훅 평면 단면도 + '그립이 립 위를 막는가' 수치.
실행: cad/.venv/bin/python cad/verify_hook.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import pymeshfix
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
Zc = -4.0; LIP_OUT = 1.6; CATCH_Z = 0.0; WIN_TOP = 0.6


def load(p):
    m = trimesh.load(p)
    if not m.is_volume:
        v, f = pymeshfix.clean_from_arrays(np.asarray(m.vertices), np.asarray(m.faces))
        m = trimesh.Trimesh(v, f); trimesh.repair.fix_normals(m)
    return m


gm = trimesh.load(f"{OUT}/grip_capped.stl")        # solid 그립(창 없음=캐치 재료 확인용)
V = gm.vertices
band = V[np.abs(V[:, 2] - Zc) < 3.0][:, :2]
P = Polygon(band[ConvexHull(band).vertices])
from shapely.geometry import LineString
skirt_out = P.buffer(-4.3)
cx, cy = float(P.centroid.x), float(P.centroid.y)
it = LineString([(cx, cy), (cx + 1e3, cy)]).intersection(skirt_out.boundary)
pt = max(it.geoms, key=lambda p: p.x) if it.geom_type == "MultiPoint" else it
hx = float(pt.x)                   # +X 훅 (실제 외곽)
lip_x = hx + 1.0                   # 림 벽 안쪽(립 점유)

# --- 수치 검증: 립 위(z>창윗변)에 그립 재료가 있나? (있으면 위로 못 빠짐=잠김) ---
pts = np.array([[lip_x, cy, 1.5],    # 캐치 위 (그립 솔리드여야 잠김)
                [lip_x, cy, -1.0],   # 립 위치 (창=구멍이어야 함)
                [hx - 1.0, cy, -1.0]])
solid_capped = gm.contains(pts)
gw = load(f"{OUT}/grip_assembled.stl")             # 창 뚫린 그립
solid_win = gw.contains(pts)
print("점검 위치          | 통짜그립 | 창뚫린그립")
print(f"  립 위 z=+1.5    |  {bool(solid_capped[0])!s:5} |  {bool(solid_win[0])!s:5}   <- True/False 면 캐치 작동(위는 막힘)")
print(f"  립 자리 z=-1.0  |  {bool(solid_capped[1])!s:5} |  {bool(solid_win[1])!s:5}   <- 창뚫린쪽 False 여야(립 들어갈 구멍)")
RETAIN = bool(solid_win[0]) and (not bool(solid_win[1]))
print(f"==> 잠김 성립: {RETAIN}  (립 위는 그립이 막고 + 립 자리엔 창 구멍)")

# --- 단면도: y=cy 평면 (XZ) ---
c = load(f"{OUT}/carrier_v2.stl")
fig, ax = plt.subplots(figsize=(7, 7))
for m, col, lab in [(gw, "green", "그립(창)"), (c, "royalblue", "캐리어")]:
    sec = m.section(plane_origin=[0, cy, 0], plane_normal=[0, 1, 0])
    if sec is None:
        continue
    for e in sec.entities:
        pp = sec.vertices[e.points]
        ax.plot(pp[:, 0], pp[:, 2], col, lw=1.2)
    ax.plot([], [], col, label=lab)
ax.axhline(WIN_TOP, color="red", ls="--", lw=.8); ax.text(hx - 8, WIN_TOP + .2, "창 윗변(캐치 막음)", color="red", fontsize=8)
ax.axhline(CATCH_Z, color="purple", ls=":", lw=.8); ax.text(hx - 8, CATCH_Z - .8, "립 캐치면", color="purple", fontsize=8)
ax.set_xlim(hx - 12, hx + 5); ax.set_ylim(-9, 7)
ax.set_aspect("equal"); ax.set_xlabel("X (반경→)"); ax.set_ylabel("Z (위→)")
ax.set_title(f"+X 훅 단면 (y={cy:.1f}) — 립이 창에 물리고 위는 그립이 막음"); ax.legend(loc="upper left", fontsize=8)
plt.tight_layout(); plt.savefig(f"{OUT}/verify_hook.png", dpi=100)
print("saved verify_hook.png")

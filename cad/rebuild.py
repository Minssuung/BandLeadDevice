#!/usr/bin/env python3
"""옵션 B: 원본 de-ringed 표면을 참조해 파라메트릭 watertight 솔리드 그립 재생성.
점군 → Y슬라이스별 볼록외곽 → **48점 각도 리샘플**(정점수 통일) → loft → 솔리드.
실행: cad/.venv/bin/python cad/rebuild.py
"""
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cadquery as cq

SRC = "/home/minsung/Downloads/touch-accessory-guidelines-2.0/Touch for Quest and Rift S/Left Controller for Quest and Rift S.stp"
OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
RING = {4, 5}
K = 28      # Y 슬라이스
N = 48      # 슬라이스당 리샘플 점수 (통일)

def resample(xz, n=N):
    c = xz.mean(0); d = xz - c
    ang = np.arctan2(d[:, 1], d[:, 0]); rad = np.hypot(d[:, 0], d[:, 1])
    o = np.argsort(ang); ang, rad = ang[o], rad[o]
    tgt = np.linspace(-np.pi, np.pi, n, endpoint=False)
    r = np.interp(tgt, ang, rad, period=2 * np.pi)
    return np.column_stack([c[0] + r * np.cos(tgt), c[1] + r * np.sin(tgt)])

r = cq.importers.importStep(SRC)
solids = r.solids().vals()
comp = cq.Compound.makeCompound([s for i, s in enumerate(solids) if i not in RING])
v, _ = comp.tessellate(0.6)
P = np.array([(p.x, p.y, p.z) for p in v])
print("points:", len(P))

ymin, ymax = P[:, 1].min(), P[:, 1].max()
edges = np.linspace(ymin, ymax, K + 1)
wires = []
for i in range(K):
    lo, hi = edges[i], edges[i + 1]; yc = float((lo + hi) / 2)
    band = P[(P[:, 1] >= lo) & (P[:, 1] <= hi)]
    if len(band) < 12:
        continue
    try:
        hull = band[:, [0, 2]][ConvexHull(band[:, [0, 2]]).vertices]
    except Exception:
        continue
    rs = resample(hull)
    pts = [cq.Vector(float(x), yc, float(z)) for x, z in rs]
    pts.append(pts[0])
    wires.append(cq.Wire.makePolygon(pts))
print("wires:", len(wires))

solid = None
for ruled in (False, True):
    try:
        s = cq.Solid.makeLoft(wires, ruled=ruled)
        if s.isValid() and s.Volume() > 1:
            solid = s; print(f"loft ok ruled={ruled} vol={s.Volume():.0f}"); break
    except Exception as e:
        print(f"loft ruled={ruled}:", e)
if solid is None:
    raise SystemExit("loft 실패")

cq.exporters.export(cq.Workplane(obj=solid), f"{OUT}/grip_rebuilt.step")
cq.exporters.export(cq.Workplane(obj=solid), f"{OUT}/grip_rebuilt.stl")
print("exported grip_rebuilt.step/.stl  bbox=%s" % (solid.BoundingBox().DiagonalLength,))

vv, tt = solid.tessellate(0.4)
PP = np.array([(p.x, p.y, p.z) for p in vv]); TT = np.array(tt)
fig = plt.figure(figsize=(17, 6))
for k, (el, az, ttl) in enumerate([(18, -65, "iso"), (0, -90, "front -Y"), (0, 0, "side +X")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.scatter(P[::8, 0], P[::8, 1], P[::8, 2], s=0.4, color=(.85, .4, .4), alpha=.18)
    ax.add_collection3d(Poly3DCollection(PP[TT], facecolor=(0.5, 0.68, 0.9), edgecolor="none", alpha=.8))
    c = PP.mean(0); rng = (PP.max(0) - PP.min(0)).max() / 2
    ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
    ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
fig.suptitle("재생성 솔리드(파랑) vs 원본 점군(빨강)")
plt.tight_layout(); plt.savefig(f"{OUT}/grip_rebuilt.png", dpi=85)
print("saved grip_rebuilt.png")

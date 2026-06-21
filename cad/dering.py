#!/usr/bin/env python3
"""링(4,5) 제거 → 그립 셸 union 시도 → STEP/STL + 3D 렌더.
셸 후보 = {0,6,7,8,9,10,11} (버튼/스틱 인서트 1,2,3,12,13 및 링 4,5 제외)
union 실패 시 Compound(불린 없음)로 export·렌더하고 어디서 실패했는지 보고.
"""
import numpy as np, traceback
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cadquery as cq

SRC = "/home/minsung/Downloads/touch-accessory-guidelines-2.0/Touch for Quest and Rift S/Left Controller for Quest and Rift S.stp"
OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
SHELL = [0, 6, 7, 8, 9, 10, 11]

r = cq.importers.importStep(SRC)
solids = r.solids().vals()
shell = [solids[i] for i in SHELL]

obj = None
try:
    u = shell[0].fuse(*shell[1:])         # 단일 BOP (체인보다 견고)
    try: u = u.clean()
    except Exception: pass
    print(f"UNION OK: solids={len(u.Solids())} valid={u.isValid()} vol={u.Volume():.0f}")
    obj = u
except Exception as e:
    print("UNION FAILED:", e)
    # 어느 솔리드가 문제인지 개별 점검
    for i in SHELL:
        s = solids[i]
        print(f"  solid[{i}] valid={s.isValid()} closed={s.Closed()} vol={s.Volume():.0f}")
    obj = cq.Compound.makeCompound(shell)
    print("→ Compound(불린 없음)로 진행")

bb = obj.BoundingBox()
print(f"bbox=({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f})")
wp = cq.Workplane(obj=obj)
for ext in ("step", "stl"):
    try:
        cq.exporters.export(wp, f"{OUT}/grip_dering.{ext}"); print("exported", ext)
    except Exception as e:
        print(f"export {ext} fail:", e)

# 3D 렌더
try:
    v, t = obj.tessellate(0.4)
    P = np.array([(p.x, p.y, p.z) for p in v]); T = np.array(t)
    fig = plt.figure(figsize=(17, 6))
    for k, (el, az, ttl) in enumerate([(18, -65, "iso"), (0, -90, "front -Y"), (0, 0, "side +X")]):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(P[T], facecolor=(0.6, 0.7, 0.85), edgecolor="none"))
        c = P.mean(0); rng = (P.max(0) - P.min(0)).max() / 2
        ax.set_xlim(c[0]-rng, c[0]+rng); ax.set_ylim(c[1]-rng, c[1]+rng); ax.set_zlim(c[2]-rng, c[2]+rng)
        ax.view_init(elev=el, azim=az); ax.set_title(ttl); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
    plt.tight_layout(); plt.savefig(f"{OUT}/grip_dering.png", dpi=85)
    print("saved grip_dering.png")
except Exception:
    traceback.print_exc()

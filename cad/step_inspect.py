#!/usr/bin/env python3
"""원본 Quest 좌 컨트롤러 STEP 구조 분석 — 링 위치/연결 파악용.
실행: cad/.venv/bin/python cad/inspect.py
"""
import time, sys
import cadquery as cq

SRC = "/home/minsung/Downloads/touch-accessory-guidelines-2.0/Touch for Quest and Rift S/Left Controller for Quest and Rift S.stp"
WIP = "/home/minsung/Downloads/설계v1/controller_carrier.step"

def report(path, label):
    print(f"\n===== {label} =====\n{path}")
    t = time.time()
    r = cq.importers.importStep(path)
    print(f"import: {time.time()-t:.1f}s")
    solids = r.solids().vals()
    comp = r.val()
    bb = comp.BoundingBox()
    print(f"solids: {len(solids)}")
    print(f"bbox mm: X[{bb.xmin:.1f},{bb.xmax:.1f}] Y[{bb.ymin:.1f},{bb.ymax:.1f}] Z[{bb.zmin:.1f},{bb.zmax:.1f}]"
          f"  dims=({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f})")
    # 솔리드별 (링이 별도 솔리드면 여기서 분리됨)
    rows = []
    for i, s in enumerate(solids):
        b = s.BoundingBox()
        try: v = s.Volume()
        except Exception: v = float('nan')
        rows.append((v, i, b))
    rows.sort(reverse=True)
    for v, i, b in rows[:15]:
        print(f"  solid[{i:>2}] vol={v:>12.0f}  center=({b.center.x:6.1f},{b.center.y:6.1f},{b.center.z:6.1f})  "
              f"dims=({b.xlen:5.1f},{b.ylen:5.1f},{b.zlen:5.1f})")
    # 면 개수(복잡도)
    try:
        print(f"faces: {len(r.faces().vals())}")
    except Exception as e:
        print("faces: ?", e)

if __name__ == "__main__":
    report(SRC, "원본 Quest 좌 컨트롤러")
    if "--wip" in sys.argv:
        report(WIP, "carrier WIP")

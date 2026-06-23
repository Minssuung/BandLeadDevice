#!/usr/bin/env python3
"""V3 종합 감사 — fact-checker급 자가검증.
연결성·watertight·트리거 전구간 스윙간섭·리프트 셸관통·배선 바닥관통·자석포켓·간섭.
실행: cad/.venv/bin/python cad/audit_v3.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import trimesh
import pymeshfix
from trimesh import transformations as TF

OUT = "/home/minsung/dev_ws/BandLeadDevice/cad/out"
C = [0, -25, -9]
issues = []


def load(f):
    return trimesh.load(f"{OUT}/{f}.stl")


def repair(m):
    if not m.is_volume:
        v, f = pymeshfix.clean_from_arrays(np.asarray(m.vertices), np.asarray(m.faces))
        m = trimesh.Trimesh(v, f); trimesh.repair.fix_normals(m)
    return m


print("=== 1) 연결성 + watertight ===")
for f, exp in [("grip_body_v3", 1), ("carrier_v3", 1), ("button_caps_v3", 3), ("trigger_lever_v3", 1)]:
    m = load(f); p = m.split(only_watertight=False)
    wt = m.is_watertight
    print(f"  {f}: {len(p)}조각  watertight={wt}")
    if len(p) != exp:
        issues.append(f"{f} 조각 {len(p)}≠{exp} (떠있는 조각)")
    if not wt:
        issues.append(f"{f} not watertight")

g = repair(load("grip_body_v3")); cc = repair(load("carrier_v3")); lev = repair(load("trigger_lever_v3"))

print("=== 2) 트리거 전구간 스윙 vs 그립 ===")
for deg in [0, 4, 8, 12, 16]:
    lr = lev.copy(); lr.apply_transform(TF.rotation_matrix(np.radians(deg), [1, 0, 0], C))
    it = trimesh.boolean.intersection([g, lr], engine="manifold")
    v = 0 if (it is None or it.is_empty) else it.volume
    print(f"  {deg:2d}°: {round(v)} mm³")
    if v > 60:
        issues.append(f"트리거 {deg}°에서 그립과 간섭 {round(v)}mm³ (걸림)")

print("=== 3) 파트 간 간섭 ===")
for a, b, nm, lim in [(cc, g, "캐리어↔그립", 200), (cc, lev, "캐리어↔레버", 60)]:
    it = trimesh.boolean.intersection([a, b], engine="manifold")
    v = 0 if (it is None or it.is_empty) else it.volume
    print(f"  {nm}: {round(v)} mm³")
    if v > lim:
        issues.append(f"{nm} 간섭 {round(v)}mm³ > {lim}")

print("=== 4) contains 검사 (포켓·관통이 실제로 셸에 있나) ===")
checks = [
    ("자석포켓(레버 빔)", lev, (0, -24.5, -18), False),
    ("레버 본체(자석 뒤)", lev, (0, -26.8, -18), True),
    ("리프트 포켓(빔)", g, (12.75, 17.8, -34), False),
    ("리프트 레버창 +X(빔)", g, (18, 17.8, -34), False),
    ("리프트 브래킷 솔리드", g, (4, 17.8, -34), True),
    ("배선출구 바닥축(빔)", g, (0, 37.2, -93.7), False),
    ("배선출구 옆 +X벽(솔리드)", g, (17, 32.1, -78), True),
    ("홀 마운트바 솔리드", g, (0, -18, -18), True),
    ("AH49E 포켓(빔)", g, (0, -20.5, -18), False),
]
for nm, mesh, pt, want in checks:
    got = bool(mesh.contains([pt])[0])
    ok = "OK" if got == want else "**틀림**"
    print(f"  {nm}: contains={got} (期待 {want}) {ok}")
    if got != want:
        issues.append(f"{nm}: contains={got}, 期待 {want}")

print("\n=== 감사 결과 ===")
if issues:
    print(f"문제 {len(issues)}개:")
    for i in issues:
        print("  -", i)
else:
    print("문제 없음 — 모든 검사 통과 ✅")

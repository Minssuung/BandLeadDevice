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
for f, exp in [("grip_body_v3", 1), ("carrier_v3", 1), ("button_caps_v3", 2), ("trigger_lever_v3", 1)]:
    m = load(f); p = m.split(only_watertight=False)
    wt = m.is_watertight
    print(f"  {f}: {len(p)}조각  watertight={wt}")
    if len(p) != exp:
        issues.append(f"{f} 조각 {len(p)}≠{exp} (떠있는 조각)")
    if not wt:
        issues.append(f"{f} not watertight")

g = repair(load("grip_body_v3")); cc = repair(load("carrier_v3")); lev = repair(load("trigger_lever_v3"))
pb = repair(load("perfboard_v3"))

print("=== 2) 트리거 전구간 스윙 vs 그립 ===")
for deg in [0, 4, 8, 12, 16]:
    lr = lev.copy(); lr.apply_transform(TF.rotation_matrix(np.radians(deg), [1, 0, 0], C))
    it = trimesh.boolean.intersection([g, lr], engine="manifold")
    v = 0 if (it is None or it.is_empty) else it.volume
    print(f"  {deg:2d}°: {round(v)} mm³")
    if v > 60:
        issues.append(f"트리거 {deg}°에서 그립과 간섭 {round(v)}mm³ (걸림)")

print("=== 3) 파트 간 간섭 ===")
pairs = [(cc, g, "캐리어↔그립", 200), (cc, lev, "캐리어↔레버", 60), (pb, g, "허브보드↔그립(트레이안착·홀바회피)", 50)]
for a, b, nm, lim in pairs:
    it = trimesh.boolean.intersection([a, b], engine="manifold")
    v = 0 if (it is None or it.is_empty) else it.volume
    print(f"  {nm}: {round(v)} mm³")
    if v > lim:
        issues.append(f"{nm} 간섭 {round(v)}mm³ > {lim}")

# 조이스틱 모듈(러프 30×26×5, PCB+핀 z-8..-13) vs 그립·트리거 전구간
import trimesh.creation as tc
mod = tc.box(extents=[31, 25, 5], transform=TF.translation_matrix([0, -7, -10.5]))   # 실측 KY-023(가로23.4×세로29.1) 눕힘 + 여유
itg = trimesh.boolean.intersection([g, mod], engine="manifold")
vg = 0 if (itg is None or itg.is_empty) else itg.volume
print(f"  조이스틱모듈↔그립: {round(vg)} mm³")
if vg > 30:
    issues.append(f"조이스틱모듈↔그립 {round(vg)}mm³")
vmax = 0
for deg in [0, 8, 16]:
    lr = lev.copy(); lr.apply_transform(TF.rotation_matrix(np.radians(deg), [1, 0, 0], C))
    it = trimesh.boolean.intersection([mod, lr], engine="manifold")
    vmax = max(vmax, 0 if (it is None or it.is_empty) else it.volume)
print(f"  조이스틱모듈↔트리거(전구간): {round(vmax)} mm³")
if vmax > 5:
    issues.append(f"조이스틱모듈↔트리거 {round(vmax)}mm³")

print("=== 4) contains 검사 (포켓·관통이 실제로 셸에 있나) ===")
checks = [
    ("자석포켓(레버 빔)", lev, (0, -24.5, -18), False),
    ("레버 본체(자석 뒤)", lev, (0, -26.8, -18), True),
    ("리프트 택트 포켓(빔, 앞면 곡률)", g, (0, 6.8, -50), False),
    ("리프트 택트 홀더벽(솔리드)", g, (4, 6, -50), True),
    ("리프트 패드통로(빔, 앞)", g, (0, -1, -48), False),
    ("IMU 파일럿(빔, 홀, 트리거 아래)", g, (-11, 27.2, -90.9), False),
    ("IMU 플레이트(솔리드)", g, (-13, 31, -75), True),
    ("허브보드 위립(솔리드, 흔들림방지)", g, (22.5, 5, -14.9), True),
    ("리프트 레버슬롯(빔, 앞면)", g, (0, -2, -47), False),
    ("리프트 피벗보스(솔리드, 핀밖)", g, (8, 2, -36), True),
    ("배선출구 바닥축(빔)", g, (0, 37.2, -93.7), False),
    ("배선출구 옆 +X벽(솔리드)", g, (17, 32.1, -78), True),
    ("트리거 코일 틈(핀 x6, 빔)", g, (6, -25, -9), False),
    ("트리거 스프링 포스트(솔리드)", g, (7.5, -22, -6), True),
    ("홀 마운트바 솔리드", g, (0, -17, -18), True),
    ("AH49E 포켓(빔)", g, (0, -19, -18), False),
    ("앞훅 캐치(그립 솔리드, 창 위)", g, (13, -27.5, -5), True),
    ("앞훅 창(빔)", g, (13, -27.5, -10), False),
    ("리프트 레버 nub자리(빔, 당김경로)", g, (0, 3, -52), False),
    ("조이스틱 스탠드오프 솔리드(캐리어)", cc, (16, 4.7, -5), True),
    ("조이스틱 스탠드오프 파일럿(빔)", cc, (14.55, 4.7, -5), False),
    ("조이스틱 돔홀(빔)", cc, (0, -7, -1.5), False),
    ("택트 홀더 포켓(빔, 캐리어)", cc, (11, 16, -6), False),
    ("택트 홀더 벽(솔리드)", cc, (11, 20, -6), True),
    ("허브보드 솔리드", pb, (0, 5, -16), True),
    ("버튼 torque홀(빔)", cc, (-11, 16, -1.5), False),
    ("버튼 kbd홀(빔)", cc, (11, 16, -1.5), False),
    ("버튼홀 안막힘(스탠드오프와 분리)", cc, (-13, 16, -1.5), False),
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

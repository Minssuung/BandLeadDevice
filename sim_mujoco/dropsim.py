#!/usr/bin/env python3
"""OpenArm MuJoCo 실패-안전(중력 낙하) 시뮬레이터.

viz(기구학)로는 못 보는 것 = 토크를 끄면 팔이 중력으로 어떻게 떨어지나.
미니암은 "이상 시 토크 OFF"가 안전(멈춤)이지만 OpenArm은 토크 OFF = 중력 낙하 = 오답.
자세별로 낙하 크기·속도·방향을 정량화 → 어느 자세가 위험한지, 중력보상이 막는지 검증.

사용: python3 dropsim.py [--T 3.0] [--render]
"""
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.path.abspath(os.path.join(HERE, "..", "viz", "openarm_description"))


def build_model():
    """package:// 치환 + dae visual 제거한 MJCF용 URDF 로드."""
    import re
    src = open(os.path.join(URDF_DIR, "output.urdf")).read()
    src = src.replace("package://openarm_description/", "")
    src = re.sub(r"<visual>.*?</visual>", "", src, flags=re.S)
    tmp = os.path.join(URDF_DIR, "output_mjc.urdf")
    open(tmp, "w").write(src)
    cwd = os.getcwd()
    os.chdir(URDF_DIR)
    try:
        m = mujoco.MjModel.from_xml_path("output_mjc.urdf")
    finally:
        os.chdir(cwd)
    return m


def jadr(m, i):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_right_joint{i}")
    return m.jnt_qposadr[jid]


# OpenArm right_joint 한계(deg) — 작업공간 펜스와 동일
LIMITS = {1: (-80, 200), 2: (-10, 190), 3: (-90, 90), 4: (0, 140),
          5: (-90, 90), 6: (-45, 45), 7: (-45, 45)}

# 텔레옵에서 나올 대표 자세 (right_joint deg). 빈 dict = 전부 0(중립).
POSES = {
    "neutral(중립 0)": {},
    "shoulder_flex_90(팔 앞 수평)": {2: 90},
    "shoulder_abduct_90(팔 옆 수평)": {1: 90},
    "arm_forward_elbow_90(앞으로+팔꿈치굽힘)": {2: 90, 4: 90},
    "shoulder_high(팔 들어올림)": {2: 140},
    "elbow_90_only(팔꿈치만)": {4: 90},
    "wrist_extended(손목 폄+팔수평)": {2: 90, 6: 40},
}


def tip_body(m):
    """오른팔 말단(손목/그리퍼 base) body id — 손끝 낙하 거리 측정용."""
    for nm in ("openarm_right_link7", "openarm_right_ee_base_link",
               "openarm_right_finger_link1"):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, nm)
        if bid >= 0:
            return bid
    return m.nbody - 1


def drop_test(m, qset, gravcomp=False, T=3.0):
    d = mujoco.MjData(m)
    for i, v in qset.items():
        d.qpos[jadr(m, i)] = math.radians(v)
    mujoco.mj_forward(m, d)
    q0 = {i: d.qpos[jadr(m, i)] for i in range(1, 8)}
    tb = tip_body(m)
    tip0 = d.xpos[tb].copy()
    n = int(T / m.opt.timestep)
    peak_vel = 0.0
    for _ in range(n):
        if gravcomp:
            d.qfrc_applied[:] = d.qfrc_bias   # 중력+코리올리 상쇄 = 중력보상 정책
        mujoco.mj_step(m, d)
        peak_vel = max(peak_vel, float(np.max(np.abs(d.qvel))))
    drop = {i: math.degrees(d.qpos[jadr(m, i)] - q0[i]) for i in range(1, 8)}
    tip_drop = float(np.linalg.norm(d.xpos[tb] - tip0))
    max_joint = max(abs(v) for v in drop.values())
    return {"drop": drop, "max_joint_deg": max_joint, "tip_drop_m": tip_drop,
            "peak_vel_dps": math.degrees(peak_vel)}


def main():
    T = 3.0
    if "--T" in sys.argv:
        T = float(sys.argv[sys.argv.index("--T") + 1])
    m = build_model()
    print(f"OpenArm MuJoCo: {m.njnt} joints, 질량 {sum(m.body_mass):.1f}kg, "
          f"timestep {m.opt.timestep*1000:.1f}ms, 중력 {m.opt.gravity[2]:.1f}m/s²\n")
    print(f"=== 토크 OFF {T:.0f}초 중력 낙하 (실패-안전 검증) ===\n")
    hdr = f"{'자세':<34}{'최대관절낙하':>12}{'손끝낙하':>10}{'최대속도':>10}{'중력보상ON':>12}"
    print(hdr)
    print("-" * len(hdr))
    rows = []
    for name, p in POSES.items():
        off = drop_test(m, p, gravcomp=False, T=T)
        on = drop_test(m, p, gravcomp=True, T=T)
        risk = "⚠위험" if off["max_joint_deg"] > 30 else ("주의" if off["max_joint_deg"] > 10 else "안정")
        print(f"{name:<34}{off['max_joint_deg']:>9.1f}°{off['tip_drop_m']*100:>8.1f}cm"
              f"{off['peak_vel_dps']:>8.0f}°/s{on['max_joint_deg']:>9.1f}° {risk}")
        rows.append((name, off, on, risk))
    print("\n해석:")
    print("  · 최대관절낙하/손끝낙하 = 토크 끊겼을 때 그 자세에서 떨어지는 양")
    print("  · 중력보상ON = qfrc_bias 상쇄 정책 적용 시 낙하(≈0이면 정책 유효)")
    print("  · ⚠위험(>30°) 자세는 OpenArm 실물서 'torque-off=안전' 가정 금지 → 브레이크/중력보상 필수")
    worst = max(rows, key=lambda r: r[1]["max_joint_deg"])
    print(f"\n최악 자세: {worst[0]} — {worst[1]['max_joint_deg']:.0f}° 낙하 "
          f"/ 손끝 {worst[1]['tip_drop_m']*100:.0f}cm / {worst[1]['peak_vel_dps']:.0f}°/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())

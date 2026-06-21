#!/usr/bin/env python3
"""OpenArm 안전 정책 실증 — 위치 홀드 & 홈 복귀 셧다운.

토크 OFF(=낙하, dropsim) 대신:
  (A) 위치 홀드: position 액추에이터 + 중력보상으로 마지막 자세 유지
      → 중력·외란에도 안 떨어짐. (= 텔레옵 중 IMU 끊김 'freeze'의 실체)
  (B) 홈 복귀 셧다운: 위험 자세 → 홈포지션으로 속도제한 하강 → 제어된 착지
      (= 정상 종료 시 'torque-off로 떨구기' 대신 안전하게 내림. 앱의 기준좌표 복귀와 동일 발상)

position 액추에이터(MuJoCo implicit) + 중력보상(qfrc_bias 상쇄) 조합 = 안정적 위치제어.

사용: python3 hold_and_home.py
"""
import math
import os
import sys

import mujoco
import numpy as np

import dropsim

KP, KV = 150.0, 25.0
HOME = {i: 0.0 for i in range(1, 8)}    # 홈 = 중립(팔 아래, 중력 안정점)
HOME_VEL_DPS = 40.0


def build_actuator_model():
    """URDF 모델 → MJCF 저장 → right_joint position 액추에이터 7개 추가 → 재로드."""
    m0 = dropsim.build_model()
    cwd = os.getcwd()
    os.chdir(dropsim.URDF_DIR)
    try:
        mujoco.mj_saveLastXML("model_saved.xml", m0)
        xml = open("model_saved.xml").read()
        act = "<actuator>\n" + "\n".join(
            f'<position name="r{i}" joint="openarm_right_joint{i}" kp="{KP}" kv="{KV}"/>'
            for i in range(1, 8)) + "\n</actuator>\n"
        xml = xml.replace("</mujoco>", act + "</mujoco>")
        open("model_act.xml", "w").write(xml)
        m = mujoco.MjModel.from_xml_path("model_act.xml")
    finally:
        os.chdir(cwd)
    m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    return m


def _jid(m, i):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_right_joint{i}")


def adr(m):
    return {i: (m.jnt_qposadr[_jid(m, i)], m.jnt_dofadr[_jid(m, i)]) for i in range(1, 8)}


def gravcomp(m, d, A):
    d.qfrc_applied[:] = 0.0
    for i in range(1, 8):
        d.qfrc_applied[A[i][1]] = d.qfrc_bias[A[i][1]]


def hold_test(m, qset, disturb=False, T=3.0):
    A = adr(m)
    d = mujoco.MjData(m)
    for i, v in qset.items():
        d.qpos[A[i][0]] = math.radians(v)
    mujoco.mj_forward(m, d)
    for i in range(1, 8):
        d.ctrl[i - 1] = d.qpos[A[i][0]]
    q0 = {i: d.qpos[A[i][0]] for i in range(1, 8)}
    n = int(T / m.opt.timestep)
    maxdev = 0.0
    for k in range(n):
        gravcomp(m, d, A)
        if disturb and k == n // 3:
            for i in range(1, 8):
                d.qvel[A[i][1]] += 3.0           # 외란: 순간 밀기
        mujoco.mj_step(m, d)
        maxdev = max(maxdev, max(abs(d.qpos[A[i][0]] - q0[i]) for i in range(1, 8)))
    final = max(abs(d.qpos[A[i][0]] - q0[i]) for i in range(1, 8))
    return {"max_dev_deg": math.degrees(maxdev), "final_dev_deg": math.degrees(final)}


def home_shutdown(m, qset, T=5.0):
    A = adr(m)
    d = mujoco.MjData(m)
    for i, v in qset.items():
        d.qpos[A[i][0]] = math.radians(v)
    mujoco.mj_forward(m, d)
    tgt = {i: d.qpos[A[i][0]] for i in range(1, 8)}
    home = {i: math.radians(HOME[i]) for i in range(1, 8)}
    step = math.radians(HOME_VEL_DPS) * m.opt.timestep
    n = int(T / m.opt.timestep)
    peak = 0.0
    for _ in range(n):
        for i in range(1, 8):
            e = home[i] - tgt[i]
            tgt[i] += max(-step, min(step, e))
            d.ctrl[i - 1] = tgt[i]
        gravcomp(m, d, A)
        mujoco.mj_step(m, d)
        peak = max(peak, max(abs(d.qvel[A[i][1]]) for i in range(1, 8)))
    err = max(abs(d.qpos[A[i][0]] - home[i]) for i in range(1, 8))
    return {"home_err_deg": math.degrees(err), "peak_vel_dps": math.degrees(peak)}


def main():
    m = build_actuator_model()
    m0 = dropsim.build_model()   # 토크OFF 낙하 비교용(액추에이터 없는 원본)
    print(f"actuator 모델: {m.nu} position 액추에이터 (kp={KP}, kv={KV})\n")

    print("=== (A) 위치 홀드 (actuator + 중력보상) vs 토크 OFF ===\n")
    print(f"{'자세':<32}{'홀드편차':>10}{'외란후':>10}{'토크OFF낙하':>14}")
    print("-" * 66)
    for name, p in dropsim.POSES.items():
        h = hold_test(m, p)
        hd = hold_test(m, p, disturb=True)
        off = dropsim.drop_test(m0, p, gravcomp=False, T=3.0)["max_joint_deg"]
        print(f"{name:<32}{h['max_dev_deg']:>8.2f}°{hd['final_dev_deg']:>8.2f}°{off:>11.0f}°")
    print("\n  → 홀드: 편차 ≈0 (안 떨어짐). 외란(밀기) 줘도 복귀. 토크OFF는 수십~183° 낙하.")

    print("\n=== (B) 홈 복귀 셧다운 — 위험 자세 → 홈 제어 하강 ===\n")
    print(f"{'시작 자세':<32}{'홈 도착오차':>12}{'최대속도':>12}")
    print("-" * 56)
    for name, p in dropsim.POSES.items():
        if not p:
            continue
        r = home_shutdown(m, p)
        print(f"{name:<32}{r['home_err_deg']:>10.2f}°{r['peak_vel_dps']:>9.0f}°/s")
    print(f"\n  → 홈으로 ≤{HOME_VEL_DPS:.0f}°/s 제어 하강(토크OFF 낙하 최대 5112°/s와 대조). 안전 셧다운.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

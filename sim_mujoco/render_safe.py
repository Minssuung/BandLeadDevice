#!/usr/bin/env python3
"""안전 셧다운 비교 영상 — 토크OFF(추락) vs 홈복귀(안전 하강).

같은 위험 자세(팔 들어올림)에서:
  왼쪽  = 토크 OFF → 중력 추락
  오른쪽 = 홈 복귀 셧다운(actuator+중력보상, 40°/s) → 제어된 하강
보고서 핵심 대조 영상.

사용: MUJOCO_GL=glfw python3 render_safe.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "glfw")

import math
import subprocess
import sys

import mujoco
import numpy as np

import dropsim
import hold_and_home as hh
from render_drop import camera, W, H, FPS, VID_DIR

T = 4.0
POSE = {2: 140}   # 팔 들어올림 (최악 자세)


def render_run(m, qset, mode, outname):
    d = mujoco.MjData(m)
    A = hh.adr(m) if mode == "home" else None
    for i, v in qset.items():
        qadr = (A[i][0] if mode == "home"
                else dropsim.jadr(m, i))
        d.qpos[qadr] = math.radians(v)
    mujoco.mj_forward(m, d)
    if mode == "home":
        tgt = {i: d.qpos[A[i][0]] for i in range(1, 8)}
        home = {i: math.radians(hh.HOME[i]) for i in range(1, 8)}
        step = math.radians(hh.HOME_VEL_DPS) * m.opt.timestep
    r = mujoco.Renderer(m, H, W)
    cam = camera()
    out = os.path.join(VID_DIR, outname)
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", out],
        stdin=subprocess.PIPE)
    spf = max(1, int((1.0 / FPS) / m.opt.timestep))
    for _ in range(int(T * FPS)):
        for _ in range(spf):
            if mode == "home":
                for i in range(1, 8):
                    e = home[i] - tgt[i]
                    tgt[i] += max(-step, min(step, e))
                    d.ctrl[i - 1] = tgt[i]
                hh.gravcomp(m, d, A)
            mujoco.mj_step(m, d)
        r.update_scene(d, camera=cam)
        proc.stdin.write(r.render().astype(np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()
    return out


def main():
    os.makedirs(VID_DIR, exist_ok=True)
    m_off = dropsim.build_model()
    m_home = hh.build_actuator_model()
    print(f"안전 셧다운 비교 영상 렌더 ({W}x{H}, {T}s)")
    render_run(m_off, POSE, "off", "shutdown_OFF.mp4")
    print("  shutdown_OFF.mp4 (토크OFF 추락)")
    render_run(m_home, POSE, "home", "shutdown_HOME.mp4")
    print("  shutdown_HOME.mp4 (홈복귀 안전하강)")
    # 나란히 비교
    comp = os.path.join(VID_DIR, "shutdown_COMPARE.mp4")
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error",
         "-i", os.path.join(VID_DIR, "shutdown_OFF.mp4"),
         "-i", os.path.join(VID_DIR, "shutdown_HOME.mp4"),
         "-filter_complex", "[0:v]pad=iw+4:ih:0:0:black[a];[a][1:v]hstack", comp],
        check=True)
    print("  shutdown_COMPARE.mp4 (좌:추락 / 우:안전하강 — 보고서용)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

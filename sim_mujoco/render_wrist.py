#!/usr/bin/env python3
"""손목 작업공간 대책 비교 영상 — 펜스 클램프 vs 스케일다운.

OpenArm 손목 한계 ±45°인데 사람 손목은 ±150°. 두 매핑 정책:
  왼쪽  펜스 클램프: 입력 0~45°는 1:1, 45° 넘으면 멈춤(45° 이상 동작 잘림)
  오른쪽 스케일다운: 입력 0~150°를 0~45°로 선형 압축(전 범위 매핑, 둔함)
같은 손목 입력(0→150° sweep)에 두 정책 적용 → 나란히 비교.

사용: MUJOCO_GL=glfw python3 render_wrist.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "glfw")

import math
import subprocess
import sys

import mujoco
import numpy as np

import hold_and_home as hh
from render_drop import W, H, FPS, VID_DIR

WRIST_LIMIT = 45.0     # OpenArm j7 한계
HUMAN_MAX = 150.0      # 사람 손목 최대 입력
SCALE = WRIST_LIMIT / HUMAN_MAX   # 스케일다운 게인 (0.3)
T = 4.0


def wrist_camera():
    c = mujoco.MjvCamera()
    c.azimuth, c.elevation, c.distance = 150, -10, 1.9
    c.lookat[:] = [0.0, -0.05, 0.70]   # 팔 전체가 프레임에 들어오도록 (아래 잘림 방지)
    return c


def render_policy(m, policy, outname):
    A = hh.adr(m)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = wrist_camera()
    out = os.path.join(VID_DIR, outname)
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", out],
        stdin=subprocess.PIPE)
    spf = max(1, int((1.0 / FPS) / m.opt.timestep))
    n = int(T * FPS)
    for f in range(n):
        # 사람 손목 입력 0→150→0 (삼각파)
        phase = f / n
        human = HUMAN_MAX * (1 - abs(2 * phase - 1))   # 0→150→0
        if policy == "fence":
            j7 = min(WRIST_LIMIT, human)               # 45°에서 클램프
        else:
            j7 = human * SCALE                          # 0.3배 압축
        for _ in range(spf):
            d.ctrl[6] = math.radians(j7)               # right_joint7
            hh.gravcomp(m, d, A)
            mujoco.mj_step(m, d)
        r.update_scene(d, camera=cam)
        proc.stdin.write(r.render().astype(np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()
    return out


def main():
    os.makedirs(VID_DIR, exist_ok=True)
    m = hh.build_actuator_model()
    print(f"손목 대책 비교 영상 (입력 0→{HUMAN_MAX:.0f}°→0, 게인 {SCALE:.2f})")
    render_policy(m, "fence", "wrist_FENCE.mp4")
    print("  wrist_FENCE.mp4 (펜스: 45°에서 멈춤)")
    render_policy(m, "scale", "wrist_SCALE.mp4")
    print("  wrist_SCALE.mp4 (스케일다운: 전 범위 압축)")
    comp = os.path.join(VID_DIR, "wrist_COMPARE.mp4")
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error",
         "-i", os.path.join(VID_DIR, "wrist_FENCE.mp4"),
         "-i", os.path.join(VID_DIR, "wrist_SCALE.mp4"),
         "-filter_complex", "[0:v]pad=iw+4:ih:0:0:black[a];[a][1:v]hstack", comp],
        check=True)
    print("  wrist_COMPARE.mp4 (좌:펜스 / 우:스케일다운)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

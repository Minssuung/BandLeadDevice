#!/usr/bin/env python3
"""자기충돌 시각화 영상 — SV6에서 발견한 충돌을 눈으로.

중립 → 팔을 몸통 쪽으로 모으는 자세로 이동(기구학 보간) → 충돌 지점을 빨간 점으로 표시.
어깨 link1↔link3, 팔↔베이스 충돌이 어디서 나는지 보고서용 시각화.

사용: MUJOCO_GL=glfw python3 render_collision.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "glfw")

import math
import subprocess
import sys

import mujoco
import numpy as np

import dropsim
from render_drop import W, H, FPS, VID_DIR

T = 4.0
# SV6 충돌 대표 자세 (팔을 가슴 안쪽으로 모음 + 팔꿈치 굽힘) — right_joint deg
COLLIDE_POSE = {1: -29, 2: -28, 3: 20, 4: 124, 5: 0, 6: 0, 7: 0}


def jadr(m, i):
    return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_right_joint{i}")]


def camera():
    c = mujoco.MjvCamera()
    c.azimuth, c.elevation, c.distance = 110, -10, 1.8
    c.lookat[:] = [0.0, -0.05, 0.7]
    return c


def main():
    os.makedirs(VID_DIR, exist_ok=True)
    m = dropsim.build_model()
    d = mujoco.MjData(m)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True   # 충돌점 표시
    # force 화살표 끔(어수선)
    r = mujoco.Renderer(m, H, W)
    cam = camera()
    out = os.path.join(VID_DIR, "self_collision.mp4")
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", out],
        stdin=subprocess.PIPE)
    n = int(T * FPS)
    base = mujoco.MjData(m)
    mujoco.mj_forward(m, base)
    base_ncon = base.ncon
    max_ncon = 0
    for f in range(n):
        phase = min(1.0, f / (n * 0.6))            # 0.6T에 도달 후 유지
        for i in range(1, 8):
            d.qpos[jadr(m, i)] = math.radians(COLLIDE_POSE[i] * phase)
        mujoco.mj_forward(m, d)                     # 기구학 + contact 계산
        max_ncon = max(max_ncon, d.ncon)
        r.update_scene(d, camera=cam, scene_option=opt)
        proc.stdin.write(r.render().astype(np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()
    print(f"self_collision.mp4 — 중립→충돌자세, contact {base_ncon}→{max_ncon}개 (빨간점=충돌지점)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

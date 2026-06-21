#!/usr/bin/env python3
"""OpenArm 중력 낙하 영상 렌더 (보고서용).

dropsim 의 수치를 3D 영상으로 — 같은 자세에서 토크OFF(낙하) vs 중력보상ON(유지) 비교.
출력: sim_mujoco/videos/*.mp4 (ffmpeg rawvideo pipe, glfw 오프스크린).

사용: python3 render_drop.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "glfw")  # egl/osmesa 실패 → glfw

import math
import subprocess
import sys

import mujoco
import numpy as np

import dropsim  # build_model, jadr (모델/관절 헬퍼 재사용)

HERE = os.path.dirname(os.path.abspath(__file__))
VID_DIR = os.path.join(HERE, "videos")
W, H, FPS, T = 640, 480, 30, 2.5

# 렌더할 자세 (보고서 대표) — dropsim 결과의 위험 자세
SCENES = [
    ("arm_up_OFF",        {2: 140}, "off",      "팔 들어올림 — 토크 OFF (낙하)"),
    ("arm_up_GRAVCOMP",   {2: 140}, "gravcomp", "팔 들어올림 — 중력보상 ON (유지)"),
    ("arm_forward_OFF",   {2: 90},  "off",      "팔 앞 수평 — 토크 OFF (낙하)"),
]


def camera():
    c = mujoco.MjvCamera()
    c.azimuth, c.elevation, c.distance = 135, -15, 2.2
    c.lookat[:] = [0.0, -0.1, 0.55]
    return c


def render_scene(m, name, qset, mode, label):
    d = mujoco.MjData(m)
    for i, v in qset.items():
        d.qpos[dropsim.jadr(m, i)] = math.radians(v)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = camera()
    out = os.path.join(VID_DIR, name + ".mp4")
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", out],
        stdin=subprocess.PIPE)
    steps_per_frame = max(1, int((1.0 / FPS) / m.opt.timestep))
    q0 = {i: d.qpos[dropsim.jadr(m, i)] for i in range(1, 8)}
    for _ in range(int(T * FPS)):
        for _ in range(steps_per_frame):
            if mode == "gravcomp":
                d.qfrc_applied[:] = d.qfrc_bias
            mujoco.mj_step(m, d)
        r.update_scene(d, camera=cam)
        proc.stdin.write(r.render().astype(np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()
    drop = max(abs(math.degrees(d.qpos[dropsim.jadr(m, i)] - q0[i])) for i in range(1, 8))
    print(f"  {name}.mp4  ({label}) — 최종 낙하 {drop:.0f}°")
    return out


def main():
    if subprocess.run(["which", "ffmpeg"], capture_output=True).returncode != 0:
        print("ffmpeg 없음 — 설치 필요"); return 1
    os.makedirs(VID_DIR, exist_ok=True)
    m = dropsim.build_model()
    print(f"OpenArm 낙하 영상 렌더 ({W}x{H}, {FPS}fps, {T}s) → {VID_DIR}/")
    for name, qset, mode, label in SCENES:
        render_scene(m, name, qset, mode, label)
    print("완료 — videos/*.mp4")
    return 0


if __name__ == "__main__":
    sys.exit(main())

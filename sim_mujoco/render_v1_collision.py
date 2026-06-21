#!/usr/bin/env python3
"""OpenArm v1 충돌 감지 시각화 영상 — 중립→충돌 자세, 충돌점(빨강) 표시.

어댑터의 MuJoCo 충돌 검사가 무엇을 잡는지 눈으로 보여주는 보고서용 영상.
caring v1 scene.xml 로 우완을 자기충돌 자세로 보간 이동 → mj_forward 접촉 검사
→ 충돌점(CONTACTPOINT) 빨강 렌더 → ffmpeg mp4.

실행(caring venv): .venv/bin/python render_v1_collision.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import math
import subprocess
import sys

import mujoco
import numpy as np

SCENE = "/home/minsung/dev_ws/caring_openarm/src/caring_openarm_mujoco/v1/scene.xml"
OUT = "/home/minsung/dev_ws/BandLeadDevice/sim_mujoco/videos/openarm_v1_collision.mp4"
W, H, FPS, T = 640, 480, 30, 5.0
# fact-check 로 확인한 '진짜 자기충돌' 자세(우완 deg) — ncon 크게 증가(3→14)
COLLIDE = {1: 200, 2: 190, 3: 80, 4: 140, 5: 0, 6: 0, 7: 0}


def main():
    m = mujoco.MjModel.from_xml_path(SCENE)
    m.vis.scale.contactwidth = 0.12     # 충돌점 크게 (잘 보이게)
    m.vis.scale.contactheight = 0.06
    m.vis.scale.forcewidth = 0.06       # 충돌 힘 화살표 굵게
    m.vis.map.force = 0.3               # 화살표 길이 스케일
    d = mujoco.MjData(m)
    adr = {i: m.jnt_qposadr[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_right_joint{i}")] for i in range(1, 8)}
    r = mujoco.Renderer(m, H, W)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 230, 8, 1.5
    cam.lookat[:] = [0.0, -0.05, 0.62]   # 받침대까지 전체가 보이도록 (하단 잘림 방지)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT],
        stdin=subprocess.PIPE)
    mujoco.mj_forward(m, d)
    base = d.ncon
    n = int(T * FPS)
    maxcon = base
    for f in range(n):
        phase = min(1.0, f / (n * 0.55))      # 0.55T 에 충돌 자세 도달 후 유지
        for i in range(1, 8):
            d.qpos[adr[i]] = math.radians(COLLIDE[i] * phase)
        mujoco.mj_forward(m, d)               # 자세 + 접촉 검사 (동역학 아님)
        maxcon = max(maxcon, d.ncon)
        r.update_scene(d, camera=cam, scene_option=opt)
        proc.stdin.write(r.render().astype(np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()
    print(f"✅ {OUT}")
    print(f"   base ncon={base} → 충돌 자세 최대 ncon={maxcon} (빨간점=충돌 지점)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

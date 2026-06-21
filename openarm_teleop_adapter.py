#!/usr/bin/env python3
"""BandLead IMU 밴드 → 실물 OpenArm(v1) 텔레옵 어댑터.

흐름: IMU UDP(4210) → bridge.calib_ik 분해 → 부호/매핑(+우완 j1·j2 반전)
      → 관절한계 클램프 → JointTrajectory 발행
        (/{side}_joint_trajectory_controller/joint_trajectory)

안전장치:
  - DRY-RUN 기본: 실제 발행 안 하고 계산된 관절각만 출력(매핑/부호 검증).
    확인 후 --execute 로 실제 구동.
  - stale-freeze: 패킷 STALE_MS 끊기면 명령 중단(JTC 가 마지막 자세 홀드).
  - bridge.smooth_joints: 속도클램프 180°/s + 점프 디바운스(분해 단계에서 이미 적용).
  - 관절한계 클램프: URDF v1_tcp 한계로 초과 방지(자기충돌/프레임 보호).
  - --scale: 관절각 스케일(보수적 테스트, 기본 1.0).
  - --exec-time: JTC time_from_start(클수록 저속).

실행(ROS sourced + caring venv):
  source /opt/ros/jazzy/setup.bash
  source ~/dev_ws/caring_openarm/install/setup.bash
  export ROS_DOMAIN_ID=34
  ~/dev_ws/caring_openarm/.venv/bin/python openarm_teleop_adapter.py --side right
      # → DRY-RUN (실물 안 움직임, 매핑 검증)
  ... --side right --execute --scale 0.5 --exec-time 0.4
      # → 실제 저속 구동 (검증 후)
"""
import argparse
import json
import math
import os
import socket
import sys
import time
import xml.etree.ElementTree as ET

_BL = "/home/minsung/dev_ws/BandLeadDevice"
sys.path.insert(0, os.path.join(_BL, "viz"))                              # bridge
sys.path.insert(0, os.path.join(_BL, "work_process/esp_imu_pc_control"))  # calib_ik

import bridge  # noqa: E402  (calib_ik 분해 재사용 — import 시 서버 미기동: main 가드)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

try:
    import mujoco  # noqa: E402  (실시간 충돌 검사 — caring venv 제공)
except Exception:  # noqa
    mujoco = None

# viz 에서 확정한 기본 부호/매핑 (UI 의 signmap JSON 으로 실시간 오버라이드).
DEF_SIGN = {1: -1, 2: 1, 3: 1, 4: -1, 5: 1, 6: -1, 7: 1}
DEF_MAP = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 7, 7: 6}   # calib_ik j → OpenArm 관절 j (j6↔j7)
DEF_FLIP: list = []   # 우완 OpenArm 관절 추가반전 (실물검증: j1·j2 불필요)
SIGNMAP_PATH = os.environ.get(
    "OPENARM_MONITOR_TELEOP_SIGNMAP",
    os.path.expanduser("~/.local/share/openarm_monitor/teleop_signmap.json"))


def load_signmap():
    """signmap JSON → (sign, jmap, flip). 없거나 깨지면 기본값."""
    sign, jmap, flip = dict(DEF_SIGN), dict(DEF_MAP), list(DEF_FLIP)
    try:
        with open(SIGNMAP_PATH) as f:
            d = json.load(f)
        for k, v in (d.get("sign") or {}).items():
            sign[int(k)] = int(v)
        for k, v in (d.get("map") or {}).items():
            jmap[int(k)] = int(v)
        flip = [int(x) for x in (d.get("right_flip") or [])]
    except Exception:  # noqa
        pass
    return sign, jmap, flip


def _signmap_mtime():
    try:
        return os.path.getmtime(SIGNMAP_PATH)
    except OSError:
        return 0.0

UDP_PORT = 4210
DEFAULT_URDF = "/home/minsung/dev_ws/caring_openarm/glue/v1_tcp.urdf"
COLLISION_SCENE = os.environ.get(
    "OPENARM_MONITOR_COLLISION_SCENE",
    "/home/minsung/dev_ws/caring_openarm/src/caring_openarm_mujoco/v1/scene.xml")
ESTOP_FLAG = os.path.expanduser(
    "~/.local/share/openarm_monitor/teleop_estop.flag")   # 앱 source.estop() 이 생성
# 그리퍼-프레임: 모델 collision 이 실물보다 ~5cm 큼(차렷 실측 5cm 여유인데 모델 0mm).
# 실물거리 ≈ 모델거리 + OFFSET. 실물거리 < MARGIN 이면 충돌(닿기 전 차단).
GRIPPER_FRAME_OFFSET = 0.05   # 모델이 실물보다 큰 양 (m)
GRIPPER_MARGIN = 0.01         # 실물 1cm 여유에서 멈춤 (m)
_REF_RESET = {"v": False}   # SIGUSR1 → 다음 패킷에서 기준 재설정 (앱 [기준 재설정])


def load_limits_deg(urdf_path, side):
    """URDF 에서 openarm_{side}_joint1~7 한계(deg) 로드."""
    lim = {}
    try:
        root = ET.parse(urdf_path).getroot()
        for j in root.iter("joint"):
            name = j.get("name", "")
            if name.startswith(f"openarm_{side}_joint") and name[-1].isdigit():
                jn = int(name[-1])
                el = j.find("limit")
                if 1 <= jn <= 7 and el is not None:
                    lim[jn] = (math.degrees(float(el.get("lower"))),
                               math.degrees(float(el.get("upper"))))
    except Exception as e:  # noqa
        print(f"[adapter] URDF 한계 로드 실패({e}) — 클램프 비활성")
    return lim


class TeleopAdapter(Node):
    def __init__(self, args):
        super().__init__("bandlead_teleop")
        self.side = args.side
        self.scale = args.scale
        self.exec_time = args.exec_time
        self.execute = args.execute
        self.joint_names = [f"openarm_{self.side}_joint{i}" for i in range(1, 8)]
        self.limits = load_limits_deg(args.urdf, self.side)
        self.sign, self.jmap, self.flip = load_signmap()
        self._smtime = _signmap_mtime()
        self._last_safe = {i: 0.0 for i in range(1, 8)}
        self._col_block = 0
        self._col_m = self._col_d = None
        self._col_adr = {}
        self._col_base = 0
        self._col_margin = 0.0   # contact dist 임계(0=관통/접촉)
        self._grip_geoms = []    # 그리퍼(hand/finger) collision geom
        self._frame_geoms = []   # 중앙프레임/world collision geom. 조기차단은 MJCF margin 필요
        self._init_collision()
        if self.execute and self._col_m is None:
            print("[adapter] ⚠ 충돌검사 불가 — 안전상 EXECUTE 거부(DRY-RUN 강제, fail-closed)")
            self.execute = False
        self._cur_q = {}   # /joint_states 실측 관절각(deg) — 충돌 freeze 시 현재자세 홀드
        self.create_subscription(JointState, "/joint_states", self._on_jstate, 10)
        self._want_viewer = getattr(args, "viewer", False)
        self._viewer = None
        self.pub = self.create_publisher(
            JointTrajectory,
            f"/{self.side}_joint_trajectory_controller/joint_trajectory", 10)
        print(f"[adapter] side={self.side} execute={self.execute} "
              f"scale={self.scale} exec_time={self.exec_time}s "
              f"한계로드={sorted(self.limits)}")
        if not self.execute:
            print("[adapter] ⚠ DRY-RUN — 실제 명령 안 보냄(매핑 검증만). 확인 후 --execute")

    def map_sign_clamp(self, raw):
        """calib_ik raw 7각(deg) → OpenArm 관절각(deg): 부호·매핑·우완반전·클램프."""
        oa = {}
        for cj in range(1, 8):
            oaj = self.jmap.get(cj, cj)
            val = raw.get(cj, 0.0) * self.sign.get(cj, 1) * self.scale
            if self.side == "right" and oaj in self.flip:
                val = -val
            lo, hi = self.limits.get(oaj, (-360.0, 360.0))
            oa[oaj] = max(lo, min(hi, val))
        return oa

    def _init_collision(self):
        """v1 MuJoCo scene 로드 — 목표 관절각의 자기충돌/프레임 충돌 실시간 검사용."""
        if mujoco is None or not os.path.isfile(COLLISION_SCENE):
            print("[adapter] ⚠ 충돌검사 비활성 (mujoco/scene 없음) — 한계클램프만")
            return
        try:
            self._col_m = mujoco.MjModel.from_xml_path(COLLISION_SCENE)
            self._col_d = mujoco.MjData(self._col_m)
            self._col_adr = {
                i: self._col_m.jnt_qposadr[mujoco.mj_name2id(
                    self._col_m, mujoco.mjtObj.mjOBJ_JOINT,
                    f"openarm_{self.side}_joint{i}")] for i in range(1, 8)}
            mujoco.mj_forward(self._col_m, self._col_d)
            self._col_base = int(self._col_d.ncon)
            self._grip_geoms = [gi for gi in range(self._col_m.ngeom)
                if any(k in self._bodyname(gi) for k in
                       (f"{self.side}_hand", f"{self.side}_right_finger",
                        f"{self.side}_left_finger"))
                and self._col_m.geom_contype[gi] != 0]
            self._frame_geoms = [gi for gi in range(self._col_m.ngeom)
                if self._bodyname(gi) in ("openarm_body_link0", "world")
                and self._col_m.geom_contype[gi] != 0]
            print(f"[adapter] 충돌검사 ON (v1 scene · base ncon={self._col_base} · "
                  f"그리퍼{len(self._grip_geoms)}↔프레임{len(self._frame_geoms)} 근접검사)")
        except Exception as e:  # noqa
            print(f"[adapter] ⚠ 충돌검사 로드 실패({e}) — 한계클램프만")
            self._col_m = None

    def _bodyname(self, geom_id):
        return mujoco.mj_id2name(
            self._col_m, mujoco.mjtObj.mjOBJ_BODY,
            int(self._col_m.geom_bodyid[geom_id])) or ""

    def _collision(self, oa):
        """oa(OpenArm 관절각 deg)에서 target 팔이 연루된 실제 충돌 판정.
        그리퍼 손가락쌍(home 기준 접촉)은 제외하고, target 팔 링크/hand 가 무언가
        (비인접 자기충돌·중앙프레임 openarm_body_link0·world·반대팔)와 dist<=margin
        으로 닿으면 True. base 카운트 비교(손가락 접촉에 흔들림)를 폐기한 판정."""
        if self._col_m is None:
            return False          # 검사 불가 — EXECUTE 는 __init__ 에서 이미 거부됨
        for i in range(1, 8):
            self._col_d.qpos[self._col_adr[i]] = math.radians(oa.get(i, 0.0))
        mujoco.mj_forward(self._col_m, self._col_d)
        sd = f"openarm_{self.side}_"
        for c in range(int(self._col_d.ncon)):
            con = self._col_d.contact[c]
            if con.dist > self._col_margin:
                continue
            n1 = self._bodyname(con.geom1)
            n2 = self._bodyname(con.geom2)
            if "finger" in n1 and "finger" in n2:
                continue          # 그리퍼 손가락쌍(home 기준 접촉) 무시
            if n1.startswith(sd) or n2.startswith(sd):
                return True       # target 팔이 무언가와 충돌
        # 그리퍼-프레임 근접: 모델이 실물보다 큰 오프셋 보정 — 실물 닿기 전 차단
        for hg in self._grip_geoms:
            for fg in self._frame_geoms:
                dm = mujoco.mj_geomDistance(self._col_m, self._col_d, hg, fg, 1.0, None)
                if dm + GRIPPER_FRAME_OFFSET < GRIPPER_MARGIN:
                    return True
        return False

    def _on_jstate(self, msg):
        """실물 /joint_states → 현재 관절각(deg) 캐시 (충돌 freeze 시 현재자세 홀드용)."""
        pre = f"openarm_{self.side}_joint"
        for nm, p in zip(msg.name, msg.position):
            suf = nm[len(pre):] if nm.startswith(pre) else ""
            if suf.isdigit():
                self._cur_q[int(suf)] = math.degrees(p)

    def _start_viewer(self):
        """시뮬 뷰어 — 목표 자세를 MuJoCo 창에 렌더(충돌점 빨강). DRY-RUN 충돌 미리보기."""
        if not self._want_viewer or self._col_m is None or mujoco is None:
            return
        try:
            from mujoco import viewer as _mjviewer
            self._viewer = _mjviewer.launch_passive(
                self._col_m, self._col_d,
                show_left_ui=False, show_right_ui=False)
            self._viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            print("[adapter] MuJoCo 시뮬 뷰어 ON — 목표 자세 + 충돌점(빨강) 시각화")
        except Exception as e:  # noqa
            print(f"[adapter] ⚠ 뷰어 실패({e}) — 뷰어 없이 진행")
            self._viewer = None

    def publish(self, oa):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(oa[i]) for i in range(1, 8)]
        sec = int(self.exec_time)
        pt.time_from_start = Duration(sec=sec,
                                      nanosec=int((self.exec_time - sec) * 1e9))
        msg.points = [pt]
        if self.execute:
            self.pub.publish(msg)

    def run(self, stale_ms=200):
        bridge.load_calib()
        self._start_viewer()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", UDP_PORT))
        sock.settimeout(stale_ms / 1000.0)
        print(f"[adapter] UDP {UDP_PORT} 수신 — 첫 유효패킷에서 기준 자동설정. Ctrl-C 정지")
        ref_set = False
        n = 0
        while rclpy.ok():
            try:
                buf, _ = sock.recvfrom(8192)
            except socket.timeout:
                continue   # stale → freeze (명령 중단, JTC 가 마지막 자세 홀드)
            except OSError:
                continue
            try:
                pkt = json.loads(buf.decode("utf-8", "replace"))
            except ValueError:
                continue
            if not isinstance(pkt, dict) or pkt.get("type") == "status":
                continue
            if not ref_set or _REF_RESET["v"]:
                bridge.set_reference(pkt)
                ref_set = True
                _REF_RESET["v"] = False
                print("[adapter] 기준 자세 설정됨 — 텔레옵 시작")
                continue
            if os.path.exists(ESTOP_FLAG):
                continue   # E-stop 플래그(앱 source.estop) → 명령 중단(freeze), JTC 홀드
            rclpy.spin_once(self, timeout_sec=0.0)   # /joint_states 콜백 처리(실측 갱신)
            raw = bridge.compute_joints(pkt)   # smooth(속도클램프/디바운스) 적용된 raw 7각
            if not raw:
                continue
            oa = self.map_sign_clamp(raw)
            col = self._collision(oa)   # _col_d 가 목표 oa 자세로 forward 됨
            if self._viewer is not None and self._viewer.is_running():
                self._viewer.sync()     # 목표 자세 + 충돌점(빨강) 렌더 (DRY-RUN 시뮬)
            if col:
                # 충돌 → 현재 실측 자세 홀드(되돌아가기 방지). 실측 없으면 마지막 안전
                oa_pub = dict(self._cur_q) if len(self._cur_q) == 7 else dict(self._last_safe)
                self._col_block += 1
            else:
                oa_pub = oa
                self._last_safe = dict(oa)
            self.publish(oa_pub)
            n += 1
            if n % 10 == 0:                     # signmap 실시간 reload (UI 조정 반영)
                mt = _signmap_mtime()
                if mt != self._smtime:
                    self.sign, self.jmap, self.flip = load_signmap()
                    self._smtime = mt
                    print(f"[adapter] signmap reload — sign={self.sign} "
                          f"map={self.jmap} flip={self.flip}")
            if n % 15 == 0:
                s = " ".join(f"j{i}:{oa[i]:+.0f}" for i in range(1, 8))
                r = " ".join(f"r{i}:{raw.get(i, 0.0):+.0f}" for i in range(1, 8))
                cb = " ⚠충돌" if col else ""
                print(f"[{'EXEC' if self.execute else 'DRY '}] 목표OA {s}{cb} (COL누적{self._col_block})")
                print(f"          RAW {r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--scale", type=float, default=1.0, help="관절각 스케일(보수적 테스트)")
    ap.add_argument("--exec-time", type=float, default=0.3, dest="exec_time",
                    help="JTC time_from_start(s) — 클수록 저속")
    ap.add_argument("--execute", action="store_true", help="실제 발행(미지정=DRY-RUN)")
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--viewer", action="store_true",
                    help="MuJoCo 시뮬 뷰어 창 — 목표 자세+충돌점 시각화(DRY-RUN 미리보기)")
    args = ap.parse_args()
    rclpy.init()
    node = TeleopAdapter(args)
    import signal
    # 앱(teleop_proc.stop())이 terminate() 로 깔끔히 멈추도록 — rclpy.ok() 를 내려
    # run() 루프를 빠져나오게 한다 (SIGTERM 무시되던 문제 해결).
    signal.signal(signal.SIGTERM, lambda *a: rclpy.shutdown())
    signal.signal(signal.SIGUSR1, lambda *a: _REF_RESET.update(v=True))  # 기준 재설정
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n[adapter] 정지 (JTC 가 마지막 자세 홀드)")
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa
            pass


if __name__ == "__main__":
    main()

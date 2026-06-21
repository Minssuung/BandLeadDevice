#!/usr/bin/env python3
"""IMU -> Motor integrated control (3 IMU version).

Left panel:  IMU sensor (WT901C-485, Modbus RTU x3)
Right panel: DYNAMIXEL motor (ID 1~7 control, ID 8 reserved)

IMU1 Pitch -> ID 1 goal position
IMU1 Roll  -> ID 2 goal position
IMU1 Yaw   -> ID 3 goal position
IMU2 Pitch -> ID 4 goal position
IMU3 Yaw   -> ID 5 goal position
IMU3 Roll  -> ID 6 goal position
IMU3 Pitch -> ID 7 goal position
"""

import sys
import os
import math
import json
import socket
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from serial.tools import list_ports
from PyQt5.QtCore import Qt, QSize, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QKeySequence
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QLineEdit,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)
from stylesheet import apply_stylesheet
try:
    from calib_ik import CalibIK
except Exception:
    CalibIK = None
try:
    from leader_arm import FeetechLeader
except Exception:
    FeetechLeader = None
try:
    from inspection_recorder import InspectionRecorder
except Exception:
    InspectionRecorder = None


# ── DYNAMIXEL 제어 테이블 (Protocol 2.0) ─────────────────────────────────
PROTOCOL_VERSION      = 2.0
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCELERATION = 108  # RAM, 4B
ADDR_PROFILE_VELOCITY     = 112  # RAM, 4B (0=무제한=전속력! 반드시 낮게 설정)
ADDR_MAX_POSITION_LIMIT   = 48   # EEPROM, 4B (토크 OFF에서만 쓰기 가능)
ADDR_MIN_POSITION_LIMIT   = 52   # EEPROM, 4B

# 모터 자체 충돌벽(EEPROM 위치한계): 앱이 뭘 보내든, 기준자세가 뭐든 모터 펌웨어가 거부.
#   ID4(팔꿈치): 굽힘 방향 800틱 = 사용자 실측값(여유 이미 포함). 신전 방향은 충돌 없음.
MOTOR_POS_LIMITS = {4: (800, 4095)}  # motor_id: (min_tick, max_tick)

# ── 리더 외골격(Feetech STS, ID1~7=관절, 8=그리퍼) → 팔로워 1:1 관절 복사 ──
LEADER_SIGN = {1: -1.0, 2: 1.0, 3: -1.0, 4: 1.0, 5: -1.0, 6: 1.0, 7: -1.0}  # 실측 반복조정: 1,3,5,7=-1, 4는 +1로 원복
LEADER_SCALE = {i: 1.0 for i in range(1, 8)}  # 기어/링키지 비율 다르면 조정
# 리더-팔로워 영점 차이 보정(deg): 팔로워 모터/IMU모드 설정은 안 건드리고 리더 경로에서만.
#   m5: 리더 중립일 때 팔로워 j5가 90° 돌아간 위치가 정렬 자세 (실측). 방향 반대면 -90으로.
#   ⚠ 전달 시작 시 해당 모터가 오프셋만큼 부드럽게 이동함(70°/s 보간).
LEADER_ZERO_OFFSET_DEG = {5: 90.0}  # 사용자 확정: +90이 정렬 자세 맞음
LEADER_GRIPPER_ID = 8
LEADER_STALE_SEC = 0.3
def _detect_leader_ports():
    """리더 어댑터(CH343, usb-1a86) 자동 탐지 — 시리얼이 케이블/포트에 따라 바뀌어서."""
    import glob as _glob
    found = sorted(_glob.glob("/dev/serial/by-id/usb-1a86_USB_Single_Serial_*"))
    return tuple(found) if found else ("/dev/ttyACM0", "/dev/ttyACM1")

LEADER_PORTS = _detect_leader_ports()
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
OPERATING_MODE_POSITION = 3

# ── 안전: 실물 속도 프로파일 (전속력 슬램 방지) ───────────────────────────
# Profile Velocity 단위 ≈ 0.229 rev/min/unit. 150 ≈ 34 RPM ≈ 206°/s (사람 팔속도 대응, 2026-06-11 상향)
# ※ 느리면 ↑ / 무서우면 ↓ — MAX_VEL_DEG_S 도 같이 조절할 것 (goal이 모터보다 빠르면 안 됨)
SAFE_PROFILE_VELOCITY     = 150
SAFE_PROFILE_ACCELERATION = 80

# IK(보정) 모드 관절 방향 부호. ⚠️ 실물 리그는 시뮬과 달라서 드라이런으로 맞춰야 함.
# 어떤 관절이 반대로 가면 그 번호를 -1.0 으로. (실측: 1,4,7 반대 → -1)
IK_SIGN = {1: -1.0, 2: -1.0, 3: 1.0, 4: 1.0, 5: -1.0, 6: 1.0, 7: -1.0}
# 안전 백스톱: 관절 한계(기준자세 대비, deg). IK 발산/오류 시 모터 폭주를 물리적으로 차단.
# j4=140: 사람 팔꿈치 풀벤드 추종(실충돌은 모터 EEPROM 한계 850틱이 따로 막음)
JOINT_LIMIT_DEG = {1: 120, 2: 120, 3: 90, 4: 140, 5: 90, 6: 90, 7: 90}
# 목표각 변화율 상한(벽시계 기준). 모터 Profile Velocity(150≈206°/s)보다 낮게
#   → 목표가 팔보다 앞서 달리지 않음. 180°/s = 사람 팔속도 대응(2026-06-11 상향).
MAX_VEL_DEG_S = 180.0
# 명령-실위치 괴리 워치독(9차 FC 권고, 오픈루프 보완): 0.5초마다 토크ON 모터 1개씩
# 순회 read. 같은 모터 2연속(=순회 2바퀴, 지속적) 30°+ 괴리 → 충돌/걸림 의심, 전달 freeze.
# 일시 괴리(빠른 스윕 중 프로파일 추종랙 ~10°)는 1회성이라 안 걸림.
PP_WATCH_GAP_DEG = 30.0
PP_WATCH_INTERVAL_S = 0.5
# stale 복귀 직후 램프인: 끊긴 사이 팔이 멀리 갔어도 천천히 따라잡기(런지 방지)
RAMP_VEL_DEG_S = 15.0
RAMP_SEC = 1.0
# IMU 패킷이 이 시간(초) 이상 안 오면 릴레이 정지(stale freeze) — 묵은 자세로 명령 금지.
#   펌웨어 poll_fail로 패킷 간격이 들쑥날쑥(0.4s론 너무 자주 걸림) → 0.7s
IMU_STALE_SEC = 0.7
# D1a 단일 센서 stale 전용 임계 — 전체 끊김(0.7s)보다 짧게. 한 IMU만 죽으면 다른 관절과
# 부정합 자세로 추종하므로 빠른 freeze가 안전(FC 권고). 정상 백오프(500ms+age)보다는 위.
SENSOR_STALE_SEC = 0.4
# 목표-현재 차이가 이보다 크면 '비정상 점프' 경고 (보간 추종 자체는 정상이라 조용히)
CLAMP_WARN_JUMP_DEG = 25.0

MOTOR_IDS = (1, 2, 3, 4, 5, 6, 7)
HOLD_MOTOR_IDS = (8,)
RELAY_IMU_MOTOR_MAP = {
    "imu1": {1: "pitch", 2: "roll", 3: "yaw"},
    "imu2": {4: "pitch"},
    "imu3": {5: "yaw", 6: "roll", 7: "pitch"},
}
MOTOR_DIRECTION = {
    1: -1.0,
    2: -1.0,
    3: 1.0,
    4: 1.0,
    5: 1.0,
    6: -1.0,
    7: 1.0,
}
SAFE_INIT_STEP_TICKS = 8
SAFE_INIT_INTERVAL_MS = 20
RELAY_DEADBAND_DEG = {1: 0.8, 2: 0.8, 3: 1.2, 4: 1.5, 5: 0.8, 6: 0.8, 7: 1.5}
RELAY_MIN_COMMAND_STEP = 3
# Wrist(pitch, motor7)가 elbow(pitch, motor4) 변화에 과민 반응할 때 상쇄용 게인.
# 0.0이면 비활성, 1.0이면 motor4 변화량을 motor7에서 동일 크기로 상쇄.
MOTOR7_ELBOW_COMP_GAIN = 1.0


def angle_to_pos(angle: float) -> int:
    """[-180, +180] 도(degree)를 [0, 4095] 위치로 선형 매핑."""
    return int(max(0, min(4095, (angle + 180.0) / 360.0 * 4095)))


def clamp_angle_deg(angle: float) -> float:
    """각도를 [-180, +180] 범위로 정규화."""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return wrapped


def unwrap_angle_deg(angle: float, prev_angle: float) -> float:
    """이전 각도 기준 언래핑: ±180° 경계 점프를 제거해 연속적인 각도값을 반환한다."""
    diff = angle - prev_angle
    diff = (diff + 180.0) % 360.0 - 180.0
    return prev_angle + diff


def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_euler_deg(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """(w, x, y, z) 쿼터니언을 ZYX 기준 Euler(deg)로 변환."""
    w, x, y, z = quat_normalize(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return (roll, pitch, yaw)


def euler_deg_to_quat(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> Tuple[float, float, float, float]:
    """ZYX Euler(deg)를 (w, x, y, z) 쿼터니언으로 변환."""
    roll = math.radians(roll_deg) / 2.0
    pitch = math.radians(pitch_deg) / 2.0
    yaw = math.radians(yaw_deg) / 2.0

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return quat_normalize(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


class OneEuroFilter:
    """One Euro 필터 (Casiez 2012): 저속에선 min_cutoff로 강하게 평활(지터 억제),
    속도가 붙으면 컷오프를 올려(beta) 필터 지연을 자동으로 풀어준다 — 텔레옵 동기화용.
    dt-aware라 가변 패킷 간격(평소 31ms, 가끔 140ms 공백)도 그대로 처리."""

    # beta는 입력 단위 스케일 의존(°/s 기준): 0.01은 100°/s서 지연 46ms로 구 LPF(21ms)보다
    # 느림(실측) → 0.05로 상향: 100°/s서 ~27ms, 200°/s서 ~14ms, 정지 평활은 min_cutoff가 지배.
    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.05,
                 d_cutoff: float = 1.0) -> None:
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev: Optional[float] = None
        self.dx_prev = 0.0

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self) -> None:
        self.x_prev = None
        self.dx_prev = 0.0

    def filter(self, x: float, dt: float) -> float:
        if not math.isfinite(x):  # NaN/inf 가드: 묵은 출력 유지 (명시적 안전선)
            return self.x_prev if self.x_prev is not None else 0.0
        if self.x_prev is None or dt <= 0.0:
            self.x_prev = x
            self.dx_prev = 0.0
            return x
        a_d = self._alpha(self.d_cutoff, dt)
        dx = (x - self.x_prev) / dt
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


def ensure_quat_continuity(
    q_cur: Tuple[float, float, float, float],
    q_prev: Optional[Tuple[float, float, float, float]],
) -> Tuple[float, float, float, float]:
    """이전 쿼터니언과의 논리적 거리를 최소화하여 연속성을 보장한다."""
    if q_prev is None:
        return q_cur
    dot_prod = sum(a * b for a, b in zip(q_cur, q_prev))
    if dot_prod < 0.0:
        return (-q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3])
    return q_cur


# ── UDP IMU 수신 스레드 ───────────────────────────────────────────────────────
class LeaderReaderThread(QThread):
    """리더 외골격(Feetech) 위치 폴링 스레드 (~50Hz). 토크 OFF = 순수 엔코더."""
    # ⚠ pyqtSignal(dict)는 QVariantMap 변환으로 정수 키가 문자열로 바뀜 → object 필수!
    positions = pyqtSignal(object)   # {id(int): ticks}
    error = pyqtSignal(str)

    def __init__(self, leader, parent=None):
        super().__init__(parent)
        self._leader = leader
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        fails = 0
        while self._running:
            try:
                pos = self._leader.read_positions()
            except Exception as exc:
                self.error.emit(str(exc))
                return
            if pos:
                fails = 0
                self.positions.emit(pos)
            else:
                fails += 1
                if fails > 30:
                    self.error.emit("리더 응답 없음 (전원/케이블)")
                    return
            self.msleep(15)


class UdpImuReaderThread(QThread):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    serial_error = pyqtSignal(str)
    parsed = pyqtSignal(dict)
    status_received = pyqtSignal(str)

    def __init__(self, host: str, port: int, timeout: float = 0.2) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.timeout = timeout
        self._running = True
        self._sock: Optional[socket.socket] = None

    def run(self) -> None:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(max(self.timeout, 0.01))
            self._sock.bind((self.host, self.port))
            self.connected.emit(f"Connected: UDP {self.host}:{self.port}")
        except Exception as exc:
            self.serial_error.emit(f"UDP bind failed: {exc}")
            return

        try:
            while self._running and self._sock:
                try:
                    payload, _addr = self._sock.recvfrom(2048)
                except socket.timeout:
                    continue

                data = self._parse_payload(payload)
                if data is not None:
                    self.parsed.emit(data)
        except Exception as exc:
            self.serial_error.emit(f"UDP read error: {exc}")
        finally:
            if self._sock is not None:
                self._sock.close()
                self._sock = None
            self.disconnected.emit()

    def stop(self) -> None:
        self._running = False

    def _parse_payload(self, payload: bytes) -> Optional[Dict[str, float]]:
        try:
            packet = json.loads(payload.decode("utf-8", errors="replace").strip())
        except json.JSONDecodeError:
            return None

        if not isinstance(packet, dict):
            return None

        if packet.get("type") == "status":
            imu1_id = packet.get("imu1_slave", "?")
            imu2_id = packet.get("imu2_slave", "?")
            imu3_id = packet.get("imu3_slave", "?")
            msg = (
                f"[STATUS] baud={packet.get('imu_baud','?')} "
                f"bytes={packet.get('imu_bytes','?')} "
                f"headers={packet.get('imu_headers','?')} "
                f"csum_fail={packet.get('imu_csum_fail','?')} "
                f"frames={packet.get('imu_frames','?')} "
                f"poll_ok={packet.get('imu_poll_ok','?')} "
                f"poll_fail={packet.get('imu_poll_fail','?')} "
                f"comb={packet.get('comb_ok','?')}/{packet.get('comb_fail','?')} "
                f"qo={packet.get('qo_ok','?')}/{packet.get('qo_fail','?')} "
                f"imu1(id={imu1_id})_avail={packet.get('imu1_available','?')} "
                f"imu2(id={imu2_id})_avail={packet.get('imu2_available','?')} "
                f"imu3(id={imu3_id})_avail={packet.get('imu3_available','?')}"
            )
            self.status_received.emit(msg)
            return None

        try:
            roll = float(packet["roll"])
            pitch = float(packet["pitch"])
            yaw = float(packet["yaw"])
        except (KeyError, TypeError, ValueError):
            return None

        try:
            imu1_roll = float(packet.get("imu1_roll", roll))
            imu1_pitch = float(packet.get("imu1_pitch", pitch))
            imu1_yaw = float(packet.get("imu1_yaw", yaw))
            imu2_roll = float(packet.get("imu2_roll", roll))
            imu2_pitch = float(packet.get("imu2_pitch", pitch))
            imu2_yaw = float(packet.get("imu2_yaw", yaw))
            imu3_roll = float(packet.get("imu3_roll", roll))
            imu3_pitch = float(packet.get("imu3_pitch", pitch))
            imu3_yaw = float(packet.get("imu3_yaw", yaw))
        except (TypeError, ValueError):
            return None

        q0 = packet.get("q0")
        q1 = packet.get("q1")
        q2 = packet.get("q2")
        q3 = packet.get("q3")
        if q0 is None or q1 is None or q2 is None or q3 is None:
            qw, qx, qy, qz = euler_deg_to_quat(roll, pitch, yaw)
        else:
            try:
                qw, qx, qy, qz = quat_normalize((float(q0), float(q1), float(q2), float(q3)))
            except (TypeError, ValueError):
                qw, qx, qy, qz = euler_deg_to_quat(roll, pitch, yaw)

        # 펌웨어가 보내는 IMU별 '센서 직접' 쿼터니언 (짐벌락 해결).
        # 없거나(None) 단위쿼터니언(센서 미응답)이면 None → 오일러 폴백.
        def _read_imu_quat(prefix):
            vals = [packet.get(f"{prefix}_q{k}") for k in range(4)]
            if any(v is None for v in vals):
                return None
            try:
                raw = tuple(float(v) for v in vals)
            except (TypeError, ValueError):
                return None
            # NaN/0/비정상 크기 거부 — NaN은 하류 클램프(max/min)를 뚫고 풀스케일 명령이 됨!
            if not all(math.isfinite(v) for v in raw):
                return None
            _n = math.sqrt(sum(v * v for v in raw))
            if not (0.5 < _n < 2.0):
                return None
            q = quat_normalize(raw)
            if abs(q[0] - 1.0) < 1e-6 and abs(q[1]) < 1e-6 and abs(q[2]) < 1e-6 and abs(q[3]) < 1e-6:
                return None
            return q

        return {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "imu1_roll": imu1_roll,
            "imu1_pitch": imu1_pitch,
            "imu1_yaw": imu1_yaw,
            "imu2_roll": imu2_roll,
            "imu2_pitch": imu2_pitch,
            "imu2_yaw": imu2_yaw,
            "imu3_roll": imu3_roll,
            "imu3_pitch": imu3_pitch,
            "imu3_yaw": imu3_yaw,
            "q0": qw,
            "q1": qx,
            "q2": qy,
            "q3": qz,
            "imu1_quat": _read_imu_quat("imu1"),
            "imu2_quat": _read_imu_quat("imu2"),
            "imu3_quat": _read_imu_quat("imu3"),
            "ax": float(packet.get("ax", 0.0)),
            "ay": float(packet.get("ay", 0.0)),
            "az": float(packet.get("az", 0.0)),
            "wx": float(packet.get("wx", 0.0)),
            "wy": float(packet.get("wy", 0.0)),
            "wz": float(packet.get("wz", 0.0)),
            # D1a: 센서별 신선도(ms). -1=한 번도 성공 못함, 키 없음=구펌웨어
            "imu1_age": packet.get("imu1_age"),
            "imu2_age": packet.get("imu2_age"),
            "imu3_age": packet.get("imu3_age"),
        }


class TorqueToggleButton(QPushButton):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setCheckable(True)
        self.setChecked(False)
        self.setCursor(Qt.PointingHandCursor)
        self.setObjectName("torqueToggle")
        self.setStyleSheet(
            "QPushButton#torqueToggle {"
            "padding: 0px;"
            "min-height: 0px;"
            "max-height: 16777215px;"
            "border: none;"
            "background: transparent;"
            "}"
        )
        self.setFixedSize(36, 20)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setFocusPolicy(Qt.NoFocus)

    def sizeHint(self):  # type: ignore[override]
        return QSize(36, 20)

    def paintEvent(self, _event):  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(1, 1, -1, -1)
        radius = rect.height() / 2

        if self.isChecked():
            track_fill = QColor("#FB0082")
            track_border = QColor("#FB0082")
            knob_x = rect.right() - rect.height() + 1
        else:
            track_fill = QColor("#D9D9D9")
            track_border = QColor("#C8C8C8")
            knob_x = rect.left()

        painter.setPen(QPen(track_border, 1))
        painter.setBrush(track_fill)
        painter.drawRoundedRect(rect, radius, radius)

        knob_rect = rect.adjusted(
            knob_x - rect.left() + 2,
            2,
            -(rect.width() - (knob_x - rect.left()) - rect.height() + 2),
            -2,
        )
        painter.setPen(QPen(QColor("#FFFFFF"), 1))
        painter.setBrush(QColor("#FFFFFF"))
        painter.drawEllipse(knob_rect)

        if not self.isEnabled():
            painter.setBrush(QColor(255, 255, 255, 120))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(rect, radius, radius)

        painter.end()

    def setChecked(self, checked: bool) -> None:  # type: ignore[override]
        super().setChecked(checked)
        self.update()


@dataclass
class MotorWidgets:
    torque_btn: TorqueToggleButton
    slider: QSlider
    spin: QSpinBox
    send_btn: QPushButton
    read_btn: QPushButton
    present_label: QLabel


class ImuWithMotorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IMU with Motor Control (3 IMU / 7 Motor)")
        self.resize(1680, 820)

        self.motor_widgets: Dict[int, MotorWidgets] = {}
        self.value_labels:  Dict[str, QLabel]       = {}

        # IMU 상태
        self.imu_reader:    Optional[UdpImuReaderThread] = None
        self.imu_connected: bool = False

        # Motor 상태
        self.port_handler:    Optional[PortHandler] = None
        self.packet_handler   = PacketHandler(PROTOCOL_VERSION)
        self.motor_connected: bool = False
        self._sync_writer:    Optional[GroupSyncWrite] = None

        # 전달(Relay) 상태
        self._relay_active: bool = False
        self._latest_imu: Dict[str, float] = {}
        self._relay_ref_rpy: Dict[str, Dict[str, float]] = {
            imu_key: {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            for imu_key in RELAY_IMU_MOTOR_MAP
        }
        self._relay_ref_quat: Dict[str, Tuple[float, float, float, float]] = {
            imu_key: (1.0, 0.0, 0.0, 0.0)
            for imu_key in RELAY_IMU_MOTOR_MAP
        }
        # 보정(calibration)+IK: calibration.json 있으면 로드 (viz에서 내보낸 축)
        self._calib_ik = None
        self._relay_qrel: Dict[str, list] = {}
        if CalibIK is not None:
            try:
                cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
                self._calib_ik = CalibIK.load(cal_path)
            except Exception:
                self._calib_ik = None
        self._relay_base_pos: Dict[int, int] = {motor_id: 2048 for motor_id in MOTOR_IDS}
        self._last_relay_goal: Dict[int, int] = {}
        self._relay_prev_angle: Dict[int, float] = {}  # 변화율 클램프용 (마지막 명령각)
        self._last_imu_time: float = 0.0               # stale freeze용
        self._imu_seq: int = 0                         # 새 패킷 게이트(같은 패킷 재처리 방지)
        # 리더 외골격 상태
        self._leader = None
        self._leader_thread = None
        self._leader_pos: Dict[int, int] = {}
        self._leader_time: float = 0.0
        self._leader_ref: Dict[int, int] = {}
        self._leader_status_t: float = 0.0
        # 검사 기록기(블랙박스 KPI 측정 — inspection_report.py 로 오프라인 분석)
        self._inspect: Optional["InspectionRecorder"] = None
        self._insp_timer: Optional[QTimer] = None
        self._insp_sync_reader = None
        self._insp_read_fail_n = 0
        self._relay_proc_seq: int = -1
        self._relay_proc_time: float = 0.0
        self._relay_proc_dt: float = 0.2               # 마지막 처리 간격(벽시계)
        self._ramp_until: float = 0.0                  # stale 복귀 램프인 종료 시각
        self._clamp_log_state: Dict[int, dict] = {}    # 속도클램프 로그 스로틀
        self._hold_motor_ids_available = []
        self._hold_motor_goal: Dict[int, int] = {}
        self._last_hold_write_time = 0.0

        # 쿼터니언 연속성 & 저역통과필터
        self._prev_quat: Dict[str, Optional[Tuple[float, float, float, float]]] = {
            imu_key: None for imu_key in RELAY_IMU_MOTOR_MAP
        }
        # q_rel → Euler 언래핑용: 마지막으로 반환한 상대 Euler 각도 저장
        self._prev_rel_euler: Dict[str, Dict[str, float]] = {
            imu_key: {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            for imu_key in RELAY_IMU_MOTOR_MAP
        }
        # 고정 LPF(α=0.6) → One Euro: 정지 평활은 더 강하게, 빠른 동작 지연은 자동 해제
        self._one_euro: Dict[int, OneEuroFilter] = {
            motor_id: OneEuroFilter() for motor_id in MOTOR_IDS
        }
        self._filtered_motor_angles: Dict[int, float] = {motor_id: 0.0 for motor_id in MOTOR_IDS}
        
        # Kinematic Chain: 상위 관절의 현재 절대 쿼터니언 (하위 관절 로컬 회전 계산용)
        self._imu1_abs_cur: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._imu2_abs_cur: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

        # 릴레이 속도 제한 타이머 (100 Hz — 평균 대기 25ms→5ms.
        # FC 검증: SyncWrite는 tx-only broadcast 49B@1Mbaud=0.49ms라 100Hz 무리 없음)
        self._relay_timer = QTimer(self)
        self._relay_timer.setTimerType(Qt.PreciseTimer)
        self._relay_timer.setInterval(10)
        self._relay_timer.timeout.connect(self._relay_tick)

        self._build_ui()
        self._refresh_ports()
        self._set_motor_controls_enabled(False)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main = QHBoxLayout(root)
        main.setSpacing(10)
        main.setContentsMargins(10, 10, 10, 10)

        left_panel = self._build_imu_panel()
        right_panel = self._build_motor_panel()

        main.addWidget(left_panel, 1)
        main.addWidget(right_panel, 1)

    def _build_imu_panel(self) -> QWidget:
        pane = QWidget()
        layout = QVBoxLayout(pane)
        layout.setSpacing(8)

        imu_conn = QGroupBox("IMU 센서 연결")
        imu_conn_layout = QVBoxLayout(imu_conn)

        row = QHBoxLayout()
        row.addWidget(QLabel("Host"))
        self.imu_host_input = QLineEdit("0.0.0.0")
        self.imu_host_input.setMinimumWidth(150)
        row.addWidget(self.imu_host_input)

        row.addWidget(QLabel("UDP Port"))
        self.imu_udp_port_input = QLineEdit("4210")
        self.imu_udp_port_input.setMaximumWidth(90)
        row.addWidget(self.imu_udp_port_input)

        self.imu_refresh_btn = QPushButton("새로고침")
        self.imu_refresh_btn.setObjectName("outlined")
        self.imu_refresh_btn.clicked.connect(self._refresh_ports)
        row.addWidget(self.imu_refresh_btn)

        self.imu_connect_btn = QPushButton("연결")
        self.imu_connect_btn.setObjectName("important")
        self.imu_connect_btn.clicked.connect(self._imu_connect)
        row.addWidget(self.imu_connect_btn)

        self.imu_disconnect_btn = QPushButton("해제")
        self.imu_disconnect_btn.setObjectName("important")
        self.imu_disconnect_btn.setEnabled(False)
        self.imu_disconnect_btn.clicked.connect(self._imu_disconnect)
        row.addWidget(self.imu_disconnect_btn)

        imu_conn_layout.addLayout(row)

        self.imu_status = QLabel("상태: Disconnected")
        self.imu_status.setObjectName("emphasized")
        imu_conn_layout.addWidget(self.imu_status)

        imu_data = QGroupBox("IMU 실시간 값 (RPY / Quaternion)")
        imu_data_layout = QGridLayout(imu_data)
        items = [
            ("roll", "Roll (deg)"),
            ("pitch", "Pitch (deg)"),
            ("yaw", "Yaw (deg)"),
            ("imu1_roll", "IMU1 Roll (deg)"),
            ("imu1_pitch", "IMU1 Pitch (deg)"),
            ("imu1_yaw", "IMU1 Yaw (deg)"),
            ("imu2_roll", "IMU2 Roll (deg)"),
            ("imu2_pitch", "IMU2 Pitch (deg)"),
            ("imu2_yaw", "IMU2 Yaw (deg)"),
            ("imu3_roll", "IMU3 Roll (deg)"),
            ("imu3_pitch", "IMU3 Pitch (deg)"),
            ("imu3_yaw", "IMU3 Yaw (deg)"),
            ("q0", "Qw"),
            ("q1", "Qx"),
            ("q2", "Qy"),
            ("q3", "Qz"),
            ("ax", "Ax (g)"),
            ("ay", "Ay (g)"),
            ("az", "Az (g)"),
            ("wx", "Wx (deg/s)"),
            ("wy", "Wy (deg/s)"),
            ("wz", "Wz (deg/s)"),
        ]
        for idx, (key, title) in enumerate(items):
            r = idx // 3
            c = (idx % 3) * 2
            value = QLabel("0.000")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setObjectName("emphasized")
            self.value_labels[key] = value
            imu_data_layout.addWidget(QLabel(title), r, c)
            imu_data_layout.addWidget(value, r, c + 1)

        imu_log = QGroupBox("IMU 로그")
        imu_log_layout = QVBoxLayout(imu_log)
        self.imu_log_text = QTextEdit()
        self.imu_log_text.setReadOnly(True)
        self.imu_log_text.setPlainText("IMU 로그는 이후 단계에서 표시됩니다.")
        imu_log_layout.addWidget(self.imu_log_text)

        layout.addWidget(imu_conn)
        layout.addWidget(imu_data)
        layout.addWidget(imu_log)
        return pane

    def _build_motor_panel(self) -> QWidget:
        pane = QWidget()
        layout = QVBoxLayout(pane)
        layout.setSpacing(8)

        motor_conn = QGroupBox("모터 연결")
        motor_conn_layout = QVBoxLayout(motor_conn)

        row = QHBoxLayout()
        row.addWidget(QLabel("포트"))
        self.motor_port_combo = QComboBox()
        row.addWidget(self.motor_port_combo)

        row.addWidget(QLabel("Baud"))
        self.motor_baud_combo = QComboBox()
        self.motor_baud_combo.addItems(["57600", "115200", "1000000"])
        self.motor_baud_combo.setCurrentText("1000000")
        row.addWidget(self.motor_baud_combo)

        self.motor_refresh_btn = QPushButton("새로고침")
        self.motor_refresh_btn.setObjectName("outlined")
        self.motor_refresh_btn.clicked.connect(self._refresh_ports)
        row.addWidget(self.motor_refresh_btn)

        self.motor_connect_btn = QPushButton("연결 확인")
        self.motor_connect_btn.setObjectName("important")
        self.motor_connect_btn.clicked.connect(self._motor_connect_and_check)
        row.addWidget(self.motor_connect_btn)

        self.motor_disconnect_btn = QPushButton("연결 해제")
        self.motor_disconnect_btn.setObjectName("important")
        self.motor_disconnect_btn.setEnabled(False)
        self.motor_disconnect_btn.clicked.connect(self._motor_disconnect)
        row.addWidget(self.motor_disconnect_btn)

        motor_conn_layout.addLayout(row)

        self.motor_status = QLabel("상태: Disconnected")
        self.motor_status.setObjectName("emphasized")
        motor_conn_layout.addWidget(self.motor_status)

        self.motor_id_status = QLabel("ID 1~7 연결 확인 대기")
        self.motor_id_status.setObjectName("secondary")
        motor_conn_layout.addWidget(self.motor_id_status)

        motor_area = QGroupBox("모터 제어 (ID 1 ~ 7)")
        motor_area_layout = QVBoxLayout(motor_area)
        motor_area_layout.setSpacing(8)

        init_row = QHBoxLayout()
        self.init_btn = QPushButton("초기 위치 설정 (모두 중앙)")
        self.init_btn.setObjectName("contained")
        self.init_btn.clicked.connect(self._initialize_motor_positions)
        init_row.addWidget(self.init_btn)
        init_row.addStretch(1)
        motor_area_layout.addLayout(init_row)

        motor_grid = QGridLayout()
        motor_grid.setHorizontalSpacing(8)
        motor_grid.setVerticalSpacing(8)
        for idx, motor_id in enumerate(MOTOR_IDS):
            row_idx = idx // 4
            col_idx = idx % 4
            motor_grid.addWidget(self._create_motor_group(motor_id), row_idx, col_idx)
        motor_area_layout.addLayout(motor_grid)

        map_box = QGroupBox("IMU -> Motor 전달")
        map_layout = QVBoxLayout(map_box)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("전달 모드"))
        self.relay_mode_combo = QComboBox()
        self.relay_mode_combo.addItem("RPY Relative (시작 시점 기준)", "rpy")
        self.relay_mode_combo.addItem("Quaternion Relative (시작 시점 기준)", "quat")
        self.relay_mode_combo.addItem("보정+분해 swing-twist (calibration.json)", "calib_ik")
        self.relay_mode_combo.addItem("리더-팔로워 (외골격 관절 복사)", "leader")
        self.relay_mode_combo.setCurrentIndex(1)
        self.relay_mode_combo.currentIndexChanged.connect(self._update_map_hint)
        mode_row.addWidget(self.relay_mode_combo)
        mode_row.addStretch(1)
        map_layout.addLayout(mode_row)

        self.rezero_btn = QPushButton("기준 자세 재설정")
        self.rezero_btn.setObjectName("outlined")
        self.rezero_btn.clicked.connect(self._capture_relay_reference)
        map_layout.addWidget(self.rezero_btn)

        self.map_status = QLabel(
            "IMU1(P→ID1, R→ID2, Y→ID3) | IMU2(P→ID4) | IMU3(Y→ID5, R→ID6, P→ID7)"
        )
        self.map_status.setObjectName("secondary")
        map_layout.addWidget(self.map_status)

        self.start_relay_btn = QPushButton("전달 시작")
        self.start_relay_btn.setObjectName("contained")
        self.start_relay_btn.clicked.connect(self._toggle_relay)
        map_layout.addWidget(self.start_relay_btn)

        # ── 리더 외골격 연결 (Feetech STS, 관절 복사 텔레옵) ──
        ld_row = QHBoxLayout()
        ld_row.addWidget(QLabel("리더"))
        self.leader_port_combo = QComboBox()
        for _p in LEADER_PORTS:
            self.leader_port_combo.addItem(_p, _p)
        self.leader_connect_btn = QPushButton("리더 연결")
        self.leader_connect_btn.clicked.connect(self._toggle_leader)
        self.leader_status = QLabel("미연결")
        ld_row.addWidget(self.leader_port_combo)
        ld_row.addWidget(self.leader_connect_btn)
        ld_row.addWidget(self.leader_status, stretch=1)
        map_layout.addLayout(ld_row)

        # ── 2-포즈 재정렬 (웨어러블 장착 틀어짐 보정) ──
        ra_row = QHBoxLayout()
        self.ra_p1_btn = QPushButton("재정렬 P1 (차렷)")
        self.ra_p2_btn = QPushButton("P2 (앞으로나란히·손바닥아래)")
        self.ra_apply_btn = QPushButton("적용")
        self.ra_p1_btn.clicked.connect(lambda: self._reanchor_capture("p1"))
        self.ra_p2_btn.clicked.connect(lambda: self._reanchor_capture("p2"))
        self.ra_apply_btn.clicked.connect(self._reanchor_apply)
        for _b in (self.ra_p1_btn, self.ra_p2_btn, self.ra_apply_btn):
            ra_row.addWidget(_b)
        map_layout.addLayout(ra_row)

        # ── 검사 기록 (블랙박스 KPI: 리더각·목표각·실제각 동시 20Hz CSV) ──
        insp_row = QHBoxLayout()
        insp_row.addWidget(QLabel("검사"))
        self.insp_scn_input = QLineEdit("S1_track_j1")
        self.insp_scn_input.setToolTip("시나리오 ID (검사기준서 참조: S1~S8)")
        self.insp_record_btn = QPushButton("검사 기록 시작")
        self.insp_record_btn.setObjectName("outlined")
        self.insp_record_btn.clicked.connect(self._toggle_inspection)
        self.insp_mark_btn = QPushButton("마크")
        self.insp_mark_btn.setObjectName("outlined")
        self.insp_mark_btn.setToolTip(
            "이벤트 타임스탬프 기록. 안전정지(K7) 판정은 라벨에 'cut' 포함 시에만"
            " (예: S6_cut) — 그 외 라벨은 주석으로만 기록. 시나리오 입력칸 텍스트=라벨")
        self.insp_mark_btn.clicked.connect(self._inspection_mark)
        self.insp_still_btn = QPushButton("정지 마크 [Space]")
        self.insp_still_btn.setObjectName("outlined")
        self.insp_still_btn.setToolTip(
            "정지구간 경계 표시('still' 라벨). 사람 착용 HM 시나리오에서 정지 시작·끝에"
            " 한 번씩 → 배회(K4)+드리프트(K5) 자동 산출. 정지 4구간+ & span 60s+면 드리프트도 나옴.\n"
            "★단축키 Space — 마우스 클릭 시 IMU 흔들림 방지용. 키보드로 누르세요.")
        self.insp_still_btn.clicked.connect(self._inspection_still_mark)
        # 정지 마크는 스페이스바로 — 마우스 클릭이 IMU를 흔들어 정지구간을 오염시킴(사용자 요청).
        # 측정 중에만 활성(평소엔 스페이스가 일반 위젯 동작을 하도록).
        self._still_shortcut = QShortcut(QKeySequence(Qt.Key_Space), self)
        self._still_shortcut.setContext(Qt.ApplicationShortcut)
        self._still_shortcut.activated.connect(self._inspection_still_mark)
        self._still_shortcut.setEnabled(False)
        self.insp_status = QLabel("")
        self.insp_status.setObjectName("secondary")
        insp_row.addWidget(self.insp_scn_input, stretch=1)
        insp_row.addWidget(self.insp_record_btn)
        insp_row.addWidget(self.insp_mark_btn)
        insp_row.addWidget(self.insp_still_btn)
        insp_row.addWidget(self.insp_status, stretch=1)
        map_layout.addLayout(insp_row)
        self._set_relay_button_state(False)

        layout.addWidget(motor_conn)
        layout.addWidget(motor_area)
        layout.addWidget(map_box)
        self._update_map_hint()
        return pane

    def _create_motor_group(self, motor_id: int) -> QGroupBox:
        box = QGroupBox(f"Motor ID {motor_id}")
        gl = QGridLayout(box)
        gl.setHorizontalSpacing(8)
        gl.setVerticalSpacing(8)

        torque_btn    = TorqueToggleButton()
        present_label = QLabel("Present Position: -")
        present_label.setObjectName("secondary")

        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 4095)
        slider.setValue(2048)

        spin = QSpinBox()
        spin.setRange(0, 4095)
        spin.setValue(2048)

        send_btn = QPushButton("Goal Position 전송")
        send_btn.clicked.connect(lambda _, mid=motor_id: self._send_goal(mid))

        read_btn = QPushButton("현재 위치 읽기")
        read_btn.clicked.connect(lambda _, mid=motor_id: self._read_present(mid))

        slider.valueChanged.connect(spin.setValue)
        spin.valueChanged.connect(slider.setValue)
        torque_btn.clicked.connect(lambda _, mid=motor_id: self._toggle_torque(mid))

        gl.addWidget(torque_btn, 0, 0)
        gl.addWidget(present_label, 0, 1, 1, 3)

        gl.addWidget(QLabel("Goal Position"), 1, 0)
        gl.addWidget(slider, 1, 1, 1, 2)
        gl.addWidget(spin, 1, 3)

        gl.addWidget(send_btn, 2, 2)
        gl.addWidget(read_btn, 2, 3)

        self.motor_widgets[motor_id] = MotorWidgets(
            torque_btn=torque_btn,
            slider=slider,
            spin=spin,
            send_btn=send_btn,
            read_btn=read_btn,
            present_label=present_label,
        )
        return box

    def _refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        self.motor_port_combo.clear()
        for p in ports:
            text = f"{p.device} ({p.description})"
            self.motor_port_combo.addItem(text, p.device)
        if not ports:
            self.motor_port_combo.addItem("사용 가능한 포트 없음", "")

    # ── IMU 연결 ─────────────────────────────────────────────────────────────

    def _imu_connect(self) -> None:
        if self.imu_connected:
            return
        host = self.imu_host_input.text().strip() or "0.0.0.0"
        try:
            udp_port = int(self.imu_udp_port_input.text().strip())
            if udp_port < 1 or udp_port > 65535:
                raise ValueError("invalid port")
        except ValueError:
            QMessageBox.warning(self, "포트 오류", "유효한 UDP 포트(1~65535)를 입력해주세요.")
            return

        self.imu_reader = UdpImuReaderThread(host, udp_port)
        self.imu_reader.connected.connect(self._on_imu_connected)
        self.imu_reader.disconnected.connect(self._on_imu_disconnected)
        self.imu_reader.serial_error.connect(self._on_imu_error)
        self.imu_reader.parsed.connect(self._on_imu_parsed)
        self.imu_reader.status_received.connect(self._log_imu)
        self.imu_reader.start()

        self.imu_connect_btn.setEnabled(False)
        self.imu_status.setText("상태: Connecting…")

    def _imu_disconnect(self) -> None:
        if self.imu_reader is not None:
            self.imu_reader.stop()
            self.imu_reader.wait(2000)
            self.imu_reader = None
        self.imu_connected = False
        self.imu_connect_btn.setEnabled(True)
        self.imu_disconnect_btn.setEnabled(False)
        self.imu_status.setText("상태: Disconnected")
        self._log_imu("IMU 연결 해제")
        if self._relay_active:
            self._relay_active = False
            self._set_relay_button_state(False)

    def _on_imu_connected(self, info: str) -> None:
        self.imu_connected = True
        self.imu_disconnect_btn.setEnabled(True)
        self.imu_status.setText("상태: Connected")
        self._log_imu(info)

    def _on_imu_disconnected(self) -> None:
        if self.imu_connected:
            self.imu_connected = False
            self.imu_connect_btn.setEnabled(True)
            self.imu_disconnect_btn.setEnabled(False)
            self.imu_status.setText("상태: Disconnected")
            self._log_imu("IMU 연결 끊김")
            if self._relay_active:
                self._relay_active = False
                self._set_relay_button_state(False)

    def _on_imu_error(self, msg: str) -> None:
        self.imu_connected = False
        self.imu_connect_btn.setEnabled(True)
        self.imu_disconnect_btn.setEnabled(False)
        self.imu_status.setText("상태: Error")
        self._log_imu(f"오류: {msg}")
        QMessageBox.critical(self, "IMU 오류", msg)

    def _on_imu_parsed(self, data: dict) -> None:
        # 값 라벨 갱신은 10Hz로 스로틀 (쿼터니언-온리 폴링시 패킷 ~90Hz × 라벨 수십개
        # setText는 이벤트루프 낭비 — 제어 경로와 무관한 표시용)
        _now = time.monotonic()  # wall-clock 역행(NTP)에도 라벨 동결 없게
        if _now - getattr(self, "_label_update_t", 0.0) >= 0.1:
            self._label_update_t = _now
            for key, label in self.value_labels.items():
                label.setText(f"{data.get(key, 0.0):.3f}")
        self._latest_imu = data
        self._last_imu_time = time.time()  # stale freeze 판정용
        self._imu_seq += 1                 # 새 패킷 게이트(릴레이는 새 패킷만 처리)
        # 센서 재부팅/yaw 리셋 감지 (7차 FC#2): 6축 모드선 재부팅 시 yaw=0 리셋되는데
        # 자기치유가 없음 → 한 프레임 60°+ 점프(수동 600°/s+, 사람 불가)면 기준 재설정 필요
        if not hasattr(self, "_prev_abs_quat"):
            self._prev_abs_quat = {}
        for _k in ("imu1", "imu2", "imu3"):
            _q = data.get(f"{_k}_quat")
            if _q is None:
                continue
            _p = self._prev_abs_quat.get(_k)
            if _p is not None:
                _dot = abs(sum(a * b for a, b in zip(_p, _q)))
                _jump = 2.0 * math.degrees(math.acos(min(1.0, _dot)))
                if _jump > 60.0 and time.time() - getattr(self, "_reboot_warn_t", 0) > 3.0:
                    self._reboot_warn_t = time.time()
                    self._log_imu(f"⚠ {_k} 자세 급변 {_jump:.0f}° — 센서 재부팅/yaw리셋 의심! [기준 설정] 다시 누르세요")
            self._prev_abs_quat[_k] = list(_q)

    def _log_imu(self, msg: str) -> None:
        self.imu_log_text.append(msg)

    # ── Motor 연결 ────────────────────────────────────────────────────────────

    def _apply_position_limits(self) -> None:
        """모터 EEPROM 위치한계(Min/Max Position Limit) 기록 = 모터 자체 충돌벽.
        EEPROM이라 토크 OFF에서만 쓰기 가능 + 마모 방지 위해 값이 다를 때만 기록."""
        for motor_id, (mn, mx) in MOTOR_POS_LIMITS.items():
            try:
                cur_mn, r1, e1 = self.packet_handler.read4ByteTxRx(
                    self.port_handler, motor_id, ADDR_MIN_POSITION_LIMIT)
                cur_mx, r2, e2 = self.packet_handler.read4ByteTxRx(
                    self.port_handler, motor_id, ADDR_MAX_POSITION_LIMIT)
                if r1 != COMM_SUCCESS or r2 != COMM_SUCCESS:
                    self._log_imu(f"ID{motor_id} 위치한계 읽기 실패 — 건너뜀")
                    continue
                if cur_mn == mn and cur_mx == mx:
                    continue  # 이미 설정됨 (EEPROM 마모 방지)
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_MIN_POSITION_LIMIT, mn)
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_MAX_POSITION_LIMIT, mx)
                self._log_imu(f"ID{motor_id} 모터 자체 위치한계 기록: {mn}~{mx}틱 (충돌벽, EEPROM)")
            except Exception as exc:
                self._log_imu(f"ID{motor_id} 위치한계 설정 오류: {exc}")

    def _motor_connect_and_check(self) -> None:
        if self.motor_connected:
            QMessageBox.information(self, "안내", "이미 연결되어 있습니다.")
            return

        port = self.motor_port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "포트 오류", "사용 가능한 포트를 선택해주세요.")
            return
        try:
            baud = int(self.motor_baud_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, "Baud 오류", "유효한 Baud 값을 선택해주세요.")
            return

        self.port_handler = PortHandler(port)
        try:
            if not self.port_handler.openPort():
                QMessageBox.critical(self, "연결 실패", f"포트를 열 수 없습니다: {port}")
                self.port_handler = None
                return

            if not self.port_handler.setBaudRate(baud):
                self.port_handler.closePort()
                self.port_handler = None
                QMessageBox.critical(self, "연결 실패", f"Baud 설정 실패: {baud}")
                return
        except Exception as e:
            self.port_handler = None
            QMessageBox.critical(
                self,
                "포트 열기 실패",
                f"포트 '{port}' 를 열 수 없습니다.\n"
                f"디바이스 연결을 확인해주세요.\n\n오류: {str(e)}"
            )
            return

        alive_ids, dead_ids = [], []
        for motor_id in MOTOR_IDS:
            model_no, comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler, motor_id
            )
            if comm_result == COMM_SUCCESS and dxl_error == 0:
                alive_ids.append((motor_id, model_no))
            else:
                dead_ids.append(motor_id)

        if dead_ids:
            self.port_handler.closePort()
            self.port_handler = None
            self.motor_status.setText("상태: Disconnected")
            self.motor_id_status.setText(f"미응답 ID: {dead_ids}")
            QMessageBox.warning(
                self,
                "연결 확인 실패",
                f"일부 모터가 응답하지 않았습니다.\n"
                f"응답: {[mid for mid, _ in alive_ids]}  미응답: {dead_ids}",
            )
            return

        mode_fail_ids = []
        for motor_id, _ in alive_ids:
            ok, err = self._ensure_position_mode(motor_id)
            if not ok:
                mode_fail_ids.append((motor_id, err or "unknown"))

        if mode_fail_ids:
            self.port_handler.closePort()
            self.port_handler = None
            self.motor_status.setText("상태: Disconnected")
            self.motor_id_status.setText(f"모드 설정 실패: {[mid for mid, _ in mode_fail_ids]}")
            detail = "\n".join([f"ID {mid}: {msg}" for mid, msg in mode_fail_ids])
            QMessageBox.critical(
                self,
                "연결 실패",
                "Position 모드(3) 설정에 실패했습니다.\n" + detail,
            )
            return

        # 모터 자체 충돌벽: EEPROM 위치한계 기록 (토크 OFF 상태인 지금만 가능)
        self._apply_position_limits()

        self.motor_connected = True
        self._sync_writer = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, 4
        )
        self._set_motor_controls_enabled(True)
        for motor_id in MOTOR_IDS:
            self.motor_widgets[motor_id].torque_btn.setChecked(False)
            self._set_goal_controls_enabled(motor_id, False)

        self.motor_connect_btn.setEnabled(False)
        self.motor_disconnect_btn.setEnabled(True)
        self.motor_status.setText(f"상태: Connected ({port}, {baud})")
        self.motor_id_status.setText(
            f"확인 완료: {[(mid, mdl) for mid, mdl in alive_ids]}"
        )
        QMessageBox.information(self, "연결 성공", "ID 1~7 모터 연결 확인 완료")

    def _ensure_position_mode(self, motor_id: int) -> Tuple[bool, Optional[str]]:
        if self.port_handler is None:
            return False, "port handler is None"

        self.port_handler.clearPort()
        mode, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE
        )
        if comm_result != COMM_SUCCESS:
            return False, self.packet_handler.getTxRxResult(comm_result)
        if dxl_error != 0:
            return False, self.packet_handler.getRxPacketError(dxl_error)

        if mode == OPERATING_MODE_POSITION:
            return True, None

        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if comm_result != COMM_SUCCESS:
            return False, self.packet_handler.getTxRxResult(comm_result)
        if dxl_error != 0:
            return False, self.packet_handler.getRxPacketError(dxl_error)

        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION
        )
        if comm_result != COMM_SUCCESS:
            return False, self.packet_handler.getTxRxResult(comm_result)
        if dxl_error != 0:
            return False, self.packet_handler.getRxPacketError(dxl_error)

        self.port_handler.clearPort()
        mode_verify, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE
        )
        if comm_result != COMM_SUCCESS:
            return False, self.packet_handler.getTxRxResult(comm_result)
        if dxl_error != 0:
            return False, self.packet_handler.getRxPacketError(dxl_error)
        if mode_verify != OPERATING_MODE_POSITION:
            return False, f"mode verify fail: {mode_verify}"

        return True, None

    def _motor_disconnect(self) -> None:
        if self._relay_active:
            self._relay_active = False
            self._relay_timer.stop()
            self._set_relay_button_state(False)
        # 검사 기록은 모터(출력 측정) 의존 → 같이 정리
        if self._inspect is not None and self._inspect.active:
            self._inspection_stop()
        self._insp_sync_reader = None

        # 안전: 연결해제/종료 시 전 모터 토크를 명시적으로 OFF (UI 신호에 의존하지 않음)
        if self.port_handler is not None:
            for motor_id in tuple(MOTOR_IDS) + tuple(HOLD_MOTOR_IDS):
                try:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                except Exception:
                    pass

        for w in self.motor_widgets.values():
            w.torque_btn.setChecked(False)

        if self._sync_writer is not None:
            self._sync_writer.clearParam()
            self._sync_writer = None
        if self.port_handler is not None:
            self.port_handler.closePort()
        self.port_handler    = None
        self.motor_connected = False
        self._hold_motor_ids_available = []
        self._hold_motor_goal.clear()
        self._set_motor_controls_enabled(False)

        self.motor_connect_btn.setEnabled(True)
        self.motor_disconnect_btn.setEnabled(False)
        self.motor_status.setText("상태: Disconnected")
        self.motor_id_status.setText("ID 1~7 연결 확인 대기")

    def _set_motor_controls_enabled(self, enabled: bool) -> None:
        for w in self.motor_widgets.values():
            w.torque_btn.setEnabled(enabled)
            w.read_btn.setEnabled(enabled)
            if not enabled:
                w.slider.setEnabled(False)
                w.spin.setEnabled(False)
                w.send_btn.setEnabled(False)

    def _set_goal_controls_enabled(self, motor_id: int, enabled: bool) -> None:
        w = self.motor_widgets[motor_id]
        w.slider.setEnabled(enabled)
        w.spin.setEnabled(enabled)
        w.send_btn.setEnabled(enabled)

    def _require_motor_connected(self) -> bool:
        if not self.motor_connected or self.port_handler is None:
            QMessageBox.warning(self, "미연결", "먼저 모터 연결 확인을 진행해주세요.")
            return False
        return True

    def _toggle_torque(self, motor_id: int) -> None:
        w = self.motor_widgets[motor_id]
        enabling = w.torque_btn.isChecked()

        if not self.motor_connected or self.port_handler is None:
            w.torque_btn.blockSignals(True)
            w.torque_btn.setChecked(not enabling)
            w.torque_btn.blockSignals(False)
            QMessageBox.warning(self, "미연결", "먼저 모터 연결 확인을 진행해주세요.")
            return

        self.port_handler.clearPort()

        if enabling:
            # ★ 근본 해결: 토크 켜기 전에 goal=현재위치로 → 켜져도 제자리 유지
            #   (안 하면 레지스터에 남은 옛 목표값으로 혼자 점프함)
            present, _err = self._safe_read_present_position(motor_id)
            if present is not None:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_GOAL_POSITION, present)
                w.slider.blockSignals(True); w.spin.blockSignals(True)
                w.slider.setValue(present); w.spin.setValue(present)
                w.slider.blockSignals(False); w.spin.blockSignals(False)
                self._last_relay_goal[motor_id] = present
                self._relay_base_pos[motor_id] = present
                # 전달 중 토크 재인에이블: 각도 기준도 '현재=0'으로 재시드 + 램프인.
                # (토크 OFF 동안 _relay_prev_angle만 전진해 있으면 켜는 순간 풀속 추격 — FC 지적)
                if getattr(self, "_relay_active", False):
                    self._relay_prev_angle[motor_id] = 0.0
                    self._ramp_until = time.time() + RAMP_SEC
                    self._log_imu(f"ID{motor_id} 토크 재인에이블 — 램프인({RAMP_VEL_DEG_S:.0f}°/s)으로 재추종")

        value = TORQUE_ENABLE if enabling else TORQUE_DISABLE
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, value
        )

        if comm_result != COMM_SUCCESS or dxl_error != 0:
            w.torque_btn.blockSignals(True)
            w.torque_btn.setChecked(not enabling)
            w.torque_btn.blockSignals(False)
            w.torque_btn.update()
            if comm_result != COMM_SUCCESS:
                QMessageBox.critical(
                    self, "통신 오류", self.packet_handler.getTxRxResult(comm_result)
                )
            else:
                QMessageBox.critical(
                    self, "패킷 오류", self.packet_handler.getRxPacketError(dxl_error)
                )
            return

        w.torque_btn.update()
        if enabling:
            self._apply_safe_profile(motor_id)
        self._set_goal_controls_enabled(motor_id, enabling)

    def _apply_safe_profile(self, motor_id: int) -> None:
        """안전 저속 프로파일(Profile Velocity/Acceleration)을 모터에 기록.
        RAM이라 토크 ON 상태에서 기록 가능. 0(무제한=전속력) 방지가 핵심."""
        if not self.motor_connected or self.port_handler is None:
            return
        try:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_PROFILE_ACCELERATION, SAFE_PROFILE_ACCELERATION)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, SAFE_PROFILE_VELOCITY)
            self._log_imu(f"ID{motor_id} 저속 프로파일 적용 (vel={SAFE_PROFILE_VELOCITY})")
        except Exception as exc:
            self._log_imu(f"ID{motor_id} 속도제한 설정 실패: {exc} — 토크 끄세요!")

    def _send_goal(self, motor_id: int) -> None:
        if not self._require_motor_connected():
            return
        goal = self.motor_widgets[motor_id].spin.value()
        comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_GOAL_POSITION, goal
        )
        if comm_result != COMM_SUCCESS:
            QMessageBox.critical(self, "통신 오류", self.packet_handler.getTxRxResult(comm_result))
        elif dxl_error != 0:
            QMessageBox.critical(self, "패킷 오류", self.packet_handler.getRxPacketError(dxl_error))

    def _initialize_motor_positions(self) -> None:
        if not self._require_motor_connected():
            return

        init_pos = 2048
        fail_ids = set()
        current_pos: Dict[int, int] = {}

        for motor_id in MOTOR_IDS:
            present, _ = self._safe_read_present_position(motor_id)
            if present is None:
                present = self.motor_widgets[motor_id].spin.value()
            current_pos[motor_id] = present

        pending = {motor_id for motor_id in MOTOR_IDS if current_pos[motor_id] != init_pos}

        while pending:
            completed = []
            for motor_id in list(pending):
                cur = current_pos[motor_id]
                delta = init_pos - cur
                if delta == 0:
                    completed.append(motor_id)
                    continue

                step = min(SAFE_INIT_STEP_TICKS, abs(delta))
                next_pos = cur + (step if delta > 0 else -step)

                comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_GOAL_POSITION, next_pos
                )
                if comm_result != COMM_SUCCESS or dxl_error != 0:
                    fail_ids.add(motor_id)
                    completed.append(motor_id)
                    continue

                current_pos[motor_id] = next_pos
                w = self.motor_widgets[motor_id]
                w.slider.blockSignals(True)
                w.spin.blockSignals(True)
                w.slider.setValue(next_pos)
                w.spin.setValue(next_pos)
                w.slider.blockSignals(False)
                w.spin.blockSignals(False)

                if next_pos == init_pos:
                    completed.append(motor_id)

            for motor_id in completed:
                pending.discard(motor_id)

            QApplication.processEvents()
            QThread.msleep(SAFE_INIT_INTERVAL_MS)

        if fail_ids:
            QMessageBox.warning(
                self,
                "초기 위치 설정 부분 실패",
                f"설정 실패: ID {sorted(fail_ids)}\n나머지는 설정되었습니다.",
            )
        else:
            QMessageBox.information(self, "초기 위치 설정 완료", "모든 모터를 저속으로 중앙 위치(2048)로 이동했습니다.")

    def _safe_read_present_position(self, motor_id: int) -> Tuple[Optional[int], Optional[str]]:
        if self.port_handler is None:
            return None, "포트가 연결되어 있지 않습니다."

        last_err: Optional[str] = None
        for _ in range(3):
            try:
                self.port_handler.clearPort()
                data, comm_result, dxl_error = self.packet_handler.readTxRx(
                    self.port_handler, motor_id, ADDR_PRESENT_POSITION, 4
                )
            except Exception as exc:
                last_err = f"현재 위치 읽기 예외(ID {motor_id}): {exc}"
                continue

            if comm_result != COMM_SUCCESS:
                last_err = self.packet_handler.getTxRxResult(comm_result)
                continue
            if dxl_error != 0:
                last_err = self.packet_handler.getRxPacketError(dxl_error)
                continue
            if len(data) < 4:
                last_err = f"현재 위치 응답 길이 부족(ID {motor_id}): {len(data)} bytes"
                continue

            present = (
                int(data[0])
                | (int(data[1]) << 8)
                | (int(data[2]) << 16)
                | (int(data[3]) << 24)
            )
            return int(max(0, min(4095, present))), None

        return None, last_err or f"현재 위치 읽기 실패(ID {motor_id})"

    def _read_present(self, motor_id: int) -> None:
        if not self._require_motor_connected():
            return

        present, err = self._safe_read_present_position(motor_id)
        if present is None:
            QMessageBox.critical(self, "현재 위치 읽기 실패", err or "알 수 없는 오류")
            return
        self.motor_widgets[motor_id].present_label.setText(f"Present Position: {present}")

    # ── 검사 기록 (블랙박스 KPI 측정) ────────────────────────────────────────
    def _toggle_inspection(self) -> None:
        if self._inspect is not None and self._inspect.active:
            self._inspection_stop()
            return
        if InspectionRecorder is None:
            QMessageBox.warning(self, "검사 기록", "inspection_recorder.py 를 불러오지 못했습니다.")
            return
        if not self.motor_connected:
            QMessageBox.warning(self, "검사 기록", "모터 연결 후 시작하세요 (출력 측정 = Present Position).")
            return
        scenario = self.insp_scn_input.text().strip() or "scenario"
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "inspection_logs")
        meta = {
            "relay_mode": self.relay_mode_combo.currentData(),
            "relay_active": self._relay_active,
            "leader_connected": self._leader is not None,
            "imu_connected": self.imu_connected,
            "safe_profile_velocity": SAFE_PROFILE_VELOCITY,
            "max_vel_deg_s": MAX_VEL_DEG_S,
            "joint_limit_deg": {str(k): v for k, v in JOINT_LIMIT_DEG.items()},
            "leader_zero_offset_deg": {str(k): v for k, v in LEADER_ZERO_OFFSET_DEG.items()},
        }
        self._inspect = InspectionRecorder(
            out_dir, MOTOR_IDS, LEADER_SIGN, LEADER_SCALE,
            snapshot_fn=self._inspection_snapshot, log_fn=self._log_imu)
        try:
            path = self._inspect.start(scenario, meta)
        except Exception as exc:
            self._inspect = None
            QMessageBox.critical(self, "검사 기록", f"시작 실패: {exc}")
            return
        self._insp_read_fail_n = 0
        self._insp_still_n = 0
        if hasattr(self, "_still_shortcut"):
            self._still_shortcut.setEnabled(True)  # 측정 중 스페이스=정지마크 활성
        if self._insp_timer is None:
            self._insp_timer = QTimer(self)
            self._insp_timer.setInterval(50)  # 20Hz — 분석기 리샘플 기준과 동일
            self._insp_timer.timeout.connect(self._inspection_tick)
        self._insp_timer.start()
        self.insp_record_btn.setText("검사 기록 중지")
        self.insp_status.setText(os.path.basename(path))
        self._log_imu(f"검사 기록 시작 [{scenario}] → {path}")

    def _inspection_stop(self) -> None:
        if self._insp_timer is not None:
            self._insp_timer.stop()
        if hasattr(self, "_still_shortcut"):
            self._still_shortcut.setEnabled(False)  # 측정 종료 → 스페이스 일반 동작 복귀
        if self._inspect is not None:
            path = self._inspect.stop()
            if path:
                self._log_imu(f"검사 기록 종료 ({self._inspect.rows}행) → {path}")
                self.insp_status.setText(f"저장됨: {os.path.basename(path)}")
            self._inspect = None
        self.insp_record_btn.setText("검사 기록 시작")

    def _inspection_mark(self) -> None:
        if self._inspect is None or not self._inspect.active:
            return
        label = self.insp_scn_input.text().strip() or "mark"
        self._inspect.mark(label)
        self._log_imu(f"검사 마크: {label}")

    def _inspection_still_mark(self) -> None:
        """정지 마크 토글: 정지 시작/끝에 한 번씩 → 'still_start'/'still_end' 라벨로 쌍 기록.
        HM 시나리오(사람 착용 배회 K4 + 드리프트 K5)의 정지구간 경계 표시용.
        명시 라벨이라 짧은 정지 폐기 시에도 시작/끝 위상이 안 어긋남(FC-A)."""
        if self._inspect is None or not self._inspect.active:
            self._log_imu("정지 마크: 검사 기록 중에만 동작합니다.")
            return
        self._insp_still_n = getattr(self, "_insp_still_n", 0) + 1
        is_start = (self._insp_still_n % 2 == 1)
        self._inspect.mark("still_start" if is_start else "still_end")
        self._log_imu(f"정지 마크 #{self._insp_still_n} ({'정지 시작' if is_start else '정지 끝'})")

    def _inspection_tick(self) -> None:
        if self._inspect is None:
            return
        try:
            self._inspect.tick()
        except Exception as exc:
            self._log_imu(f"검사 기록 오류 — 중지: {exc}")
            self._inspection_stop()
            return
        if self._inspect is not None and self._inspect.rows % 20 == 0:  # 1Hz 상태
            self.insp_status.setText(
                f"{self._inspect.rows}행 | 읽기실패틱 {self._insp_read_fail_n}")

    def _inspection_snapshot(self) -> dict:
        now = time.time()
        present, fail_n = self._inspection_read_presents()
        return {
            "leader_raw": dict(self._leader_pos) if self._leader_pos else {},
            "leader_age": (now - self._leader_time) if self._leader_time else None,
            "imu_age": (now - self._last_imu_time) if self._last_imu_time else None,
            "target": dict(getattr(self, "_relay_target", {}) or {}),
            "goal": dict(self._last_relay_goal),
            "base": dict(self._relay_base_pos),
            "present": present,
            "read_fail": fail_n,
        }

    def _inspection_read_presents(self) -> Tuple[Dict[int, int], int]:
        """전 모터 Present Position 일괄 읽기 (GroupSyncRead, 실패 관절은 누락)."""
        if self.port_handler is None:
            return {}, len(MOTOR_IDS)
        if self._insp_sync_reader is None:
            self._insp_sync_reader = GroupSyncRead(
                self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, 4)
            for mid in MOTOR_IDS:
                self._insp_sync_reader.addParam(mid)
        out: Dict[int, int] = {}
        fail = 0
        try:
            comm = self._insp_sync_reader.txRxPacket()
        except Exception:
            comm = -1
        if comm != COMM_SUCCESS:
            # 실패 시 잔류 status 패킷이 다음 틱의 '현재 위치'로 둔갑하지 않게 버퍼 비움
            self._insp_read_fail_n += 1
            try:
                self.port_handler.clearPort()
            except Exception:
                pass
            return {}, len(MOTOR_IDS)
        for mid in MOTOR_IDS:
            if self._insp_sync_reader.isAvailable(mid, ADDR_PRESENT_POSITION, 4):
                out[mid] = int(self._insp_sync_reader.getData(mid, ADDR_PRESENT_POSITION, 4))
            else:
                fail += 1
        if fail:
            self._insp_read_fail_n += 1
            try:
                self.port_handler.clearPort()
            except Exception:
                pass
        return out, fail

    # ── 전달(Relay) ───────────────────────────────────────────────────────────

    # ── 리더 외골격 연결/수신 ─────────────────────────────────────────────

    def _toggle_leader(self) -> None:
        if self._leader is not None:  # 해제
            if self._leader_thread is not None:
                self._leader_thread.stop()
                self._leader_thread.wait(1000)
                self._leader_thread = None
            self._leader.close()
            self._leader = None
            self._leader_pos = {}
            self.leader_connect_btn.setText("리더 연결")
            self.leader_status.setText("미연결")
            return
        if FeetechLeader is None:
            QMessageBox.critical(self, "모듈 없음", "leader_arm.py 를 불러올 수 없습니다.")
            return
        port = self.leader_port_combo.currentData()
        try:
            leader = FeetechLeader(port)
            leader.open()
        except Exception as exc:
            QMessageBox.critical(self, "리더 연결 실패", str(exc))
            return
        self._leader = leader
        self._leader_thread = LeaderReaderThread(leader, self)
        self._leader_thread.positions.connect(self._on_leader_pos)
        self._leader_thread.error.connect(self._on_leader_error)
        self._leader_thread.start()
        self.leader_connect_btn.setText("리더 해제")
        self.leader_status.setText(f"연결됨 {port} (ID {leader.alive_ids})")
        self._log_imu(f"리더 연결: {port}, 응답 ID {leader.alive_ids} (토크 OFF=엔코더 모드)")
        # 리더를 연결했다 = 리더 모드를 쓰겠다 → 전달 모드 자동 전환
        _idx = self.relay_mode_combo.findData("leader")
        if _idx >= 0:
            self.relay_mode_combo.setCurrentIndex(_idx)

    def _apply_leader_zero_offsets(self) -> None:
        """리더-팔로워 영점 차이를 팔로워 base 틱에 반영 (리더 경로 전용).
        base를 옮기면 안전창(JOINT_LIMIT ±)도 정렬 자세 중심으로 같이 이동."""
        for _mid, _off in LEADER_ZERO_OFFSET_DEG.items():
            _b = self._relay_base_pos.get(_mid)
            if _b is None:
                continue
            _nb = _b + int(round(_off / 360.0 * 4095))
            _pmn, _pmx = MOTOR_POS_LIMITS.get(_mid, (0, 4095))
            self._relay_base_pos[_mid] = max(_pmn, min(_pmx, _nb))
            self._log_imu(f"리더 영점보정 ID{_mid}: base {_b}→{self._relay_base_pos[_mid]}틱 ({_off:+.0f}°)")

    def _on_leader_pos(self, pos: dict) -> None:
        self._leader_pos = pos
        self._leader_time = time.time()
        if self._leader_time - self._leader_status_t > 0.3:  # 상태표시 ~3Hz
            self._leader_status_t = self._leader_time
            grip = pos.get(LEADER_GRIPPER_ID)
            j = " ".join(f"{i}:{pos.get(i, '?')}" for i in range(1, 8))
            self.leader_status.setText(f"수신 | {j} | 그립 {grip}")

    def _on_leader_error(self, msg: str) -> None:
        self._log_imu(f"리더 오류: {msg}")
        self.leader_status.setText(f"오류: {msg}")
        if self._relay_active and self.relay_mode_combo.currentData() == "leader":
            self._toggle_relay()  # 안전: 리더 죽으면 전달 중지

    def _toggle_relay(self) -> None:
        # 시작/중지 어느 쪽이든 워치독 freeze 상태는 초기화
        self._pp_freeze = False
        self._pp_mismatch = {}
        if self._relay_active:
            self._relay_active = False
            self._relay_timer.stop()
            self._set_relay_button_state(False)
            self._update_map_hint()
            return

        if not self.motor_connected:
            QMessageBox.warning(self, "모터 미연결", "먼저 모터 연결을 확인해주세요.")
            return

        # ── 리더-팔로워 모드: IMU 불필요, 리더 기준만 캡처하고 바로 시작 ──
        if self.relay_mode_combo.currentData() == "leader":
            if self._leader is None or not self._leader_pos:
                QMessageBox.warning(self, "리더 미연결", "[리더 연결] 후 다시 시작하세요.")
                return
            if time.time() - self._leader_time > LEADER_STALE_SEC:
                QMessageBox.warning(self, "리더 신선도", "리더 데이터가 오래됐습니다. 연결 상태를 확인하세요.")
                return
            self._leader_ref = dict(self._leader_pos)  # 현재 리더 자세 = 0점
            if not self._capture_relay_motor_base():
                QMessageBox.warning(self, "베이스 정합성 실패",
                                    "모터 베이스 캡처가 정합성 게이트(W1)를 통과하지 못했습니다.\n"
                                    "로그를 확인하고 모터 통신을 점검 후 다시 시작하세요.")
                return
            self._apply_leader_zero_offsets()
            try:
                self._prepare_hold_motors()
            except Exception as exc:
                self._log_imu(f"고정 모터 준비 건너뜀: {exc}")
            self._relay_prev_angle = {}
            self._relay_target = {}
            self._stale_warned = False
            self._ramp_until = time.time() + RAMP_SEC  # 첫 시작 램프인(FC #4)
            self._relay_active = True
            self._relay_timer.start()
            self._set_relay_button_state(True)
            self._log_imu("리더-팔로워 전달 시작 (현재 리더 자세 = 기준)")
            return

        if not self.imu_connected:
            QMessageBox.warning(self, "IMU 미연결", "먼저 IMU 센서를 연결해주세요.")
            return
        if not self._latest_imu:
            QMessageBox.warning(self, "IMU 데이터 대기", "아직 IMU 데이터가 들어오지 않았습니다. 잠시 후 다시 시도해주세요.")
            return

        # 신선도 게이트: 묵은 패킷으로 기준을 잡으면 이후 모든 상대각이 오염됨
        if time.time() - self._last_imu_time > IMU_STALE_SEC:
            QMessageBox.warning(self, "IMU 신선도", "IMU 데이터가 오래됐습니다. 수신 확인 후 다시 시작하세요.")
            return
        if not self._capture_relay_reference():
            QMessageBox.warning(self, "기준 캡처 실패", "전달 기준 자세를 캡처하지 못했습니다. IMU 수신 상태를 확인해주세요.")
            return
        if not self._capture_relay_motor_base():
            QMessageBox.warning(self, "베이스 정합성 실패",
                                "모터 베이스 캡처가 정합성 게이트(W1)를 통과하지 못했습니다.\n"
                                "로그를 확인하고 모터 통신을 점검 후 다시 시작하세요.")
            return
        try:
            self._prepare_hold_motors()
        except Exception as exc:
            self._log_imu(f"고정 모터 준비 건너뜀: {exc}")

        # Q5: 보정축 품질 경고 (병적 보정이면 분해 부정확)
        if self._calib_ik is not None:
            for _w in self._calib_ik.axis_quality():
                self._log_imu(f"⚠ 보정품질: {_w}")
        self._relay_prev_angle = {}   # 변화율 클램프 상태 초기화(첫 프레임은 기준=0)
        self._stale_warned = False
        # 첫 시작 램프인(15°/s) — base 오류(m5 90°형)가 W1을 뚫어도 슬램 대신 저속,
        # 그 사이 괴리 워치독이 잡게(FC #4 보강).
        self._ramp_until = time.time() + RAMP_SEC
        self._relay_active = True
        self._relay_timer.start()
        self._set_relay_button_state(True)

    def _set_relay_button_state(self, active: bool) -> None:
        if active:
            self.start_relay_btn.setText("전달 중지")
            self.start_relay_btn.setObjectName("relayStop")
        else:
            self.start_relay_btn.setText("전달 시작")
            self.start_relay_btn.setObjectName("contained")

        style = self.start_relay_btn.style()
        style.unpolish(self.start_relay_btn)
        style.polish(self.start_relay_btn)
        self.start_relay_btn.update()

    def _stale_sensor(self, data: dict) -> Optional[str]:
        """D1a: 활성 IMU 중 하나라도 단독 stale이면 그 이름 반환(없으면 None).
        펌웨어 imuN_age(ms): -1=한 번도 성공못함, age>임계=갱신 끊김. 키 없음=구펌웨어→스킵.
        IK 모드에서 실제 쓰는 IMU만 검사(RELAY_IMU_MOTOR_MAP 키 = imu1/imu2/imu3)."""
        thresh_ms = SENSOR_STALE_SEC * 1000.0
        for imu_key in RELAY_IMU_MOTOR_MAP:
            age = data.get(f"{imu_key}_age")
            if age is None:        # 구펌웨어(필드 없음) → 이 게이트 비활성
                continue
            try:
                age = float(age)
            except (TypeError, ValueError):
                return imu_key     # 파싱 불가 = 신뢰 못 함 → stale 취급(보수적)
            if not math.isfinite(age):  # NaN/inf = "신선함"으로 오판 방지(FC 거짓음성)
                return imu_key
            if age < 0 or age > thresh_ms:
                return imu_key
        return None

    def _relay_tick(self) -> None:
        if not self._relay_active:
            return
        if getattr(self, "_pp_freeze", False):
            return  # 괴리 워치독 freeze — [전달 중지]→[시작] 또는 [기준 설정]으로 해제
        # ── 리더-팔로워: 리더 틱→각도 1:1 복사 (보정/분해 불필요) ──
        if self.relay_mode_combo.currentData() == "leader":
            now = time.time()
            if now - self._leader_time > LEADER_STALE_SEC:
                if not getattr(self, "_stale_warned", False):
                    self._log_imu(f"⚠ 리더 끊김(>{LEADER_STALE_SEC:.1f}s) — freeze")
                    self._stale_warned = True
                return
            if getattr(self, "_stale_warned", False):
                self._ramp_until = now + RAMP_SEC
                self._log_imu("리더 복구 — 램프인 후 재개")
                self._stale_warned = False
            if self._leader_ref:
                tgt = {}
                for mid in MOTOR_IDS:
                    p = self._leader_pos.get(mid)
                    r = self._leader_ref.get(mid)
                    if p is None or r is None:
                        continue
                    d = p - r
                    if d > 2048:    # 0/4095 경계 랩어라운드
                        d -= 4096
                    elif d < -2048:
                        d += 4096
                    tgt[mid] = (d * 360.0 / 4096.0) * LEADER_SIGN.get(mid, 1.0) * LEADER_SCALE.get(mid, 1.0)
                self._relay_target = tgt
            self._relay_send_step()
            return

        if not self._latest_imu:
            return
        now = time.time()
        # stale freeze: 패킷이 끊기면 묵은 자세로 명령하지 않고 정지(마지막 목표 유지)
        if now - self._last_imu_time > IMU_STALE_SEC:
            if not getattr(self, "_stale_warned", False):
                self._log_imu(f"⚠ IMU 끊김(>{IMU_STALE_SEC:.1f}s) — 전달 일시정지(freeze)")
                self._stale_warned = True
            return
        # D1a: 센서별 신선도 — 패킷은 오는데 한 IMU만 죽은 모드(전체 stale보다 위험: 틀린
        # 자세로 계속 추종). 펌웨어 imuN_age(ms)로 개별 감지. -1=한 번도 성공못함도 stale.
        _stale_imu = self._stale_sensor(self._latest_imu)
        if _stale_imu:
            if not getattr(self, "_sensor_stale_warned", False):
                self._log_imu(f"🛑 {_stale_imu} 센서 단독 stale — 전달 freeze "
                              f"(패킷은 오지만 해당 IMU 미갱신=틀린 자세 추종 위험)")
                self._sensor_stale_warned = True
            return
        if getattr(self, "_sensor_stale_warned", False):
            self._ramp_until = now + RAMP_SEC
            self._log_imu("센서 신선도 복구 — 램프인 후 재개")
            self._sensor_stale_warned = False
        if getattr(self, "_stale_warned", False):
            # 복귀: 끊긴 사이 팔이 멀리 갔을 수 있음 → 램프인(저속)으로 따라잡기
            self._ramp_until = now + RAMP_SEC
            self._log_imu(f"IMU 복구 — 램프인 {RAMP_SEC:.0f}s({RAMP_VEL_DEG_S:.0f}°/s) 후 전달 재개")
            self._stale_warned = False
        # 목표 갱신은 새 패킷에서만 (같은 패킷 재처리시 필터 무력화).
        # + 50Hz 디시메이트: 90Hz 패킷 × IK분해 2.7ms = 메인스레드 ~27% (FC 권고) —
        #   전송은 _relay_send_step 100Hz 보간이라 체감 손실 없음.
        if (self._imu_seq != self._relay_proc_seq
                and now - self._relay_proc_time >= 0.02):
            self._relay_proc_dt = min(max(now - self._relay_proc_time, 0.01), 0.3)
            self._relay_proc_time = now
            self._relay_proc_seq = self._imu_seq
            self._relay_to_motors(self._latest_imu)   # 목표 계산·저장
        # 전송은 매 틱(20Hz): 목표를 향해 속도제한 보간 → 5-6Hz 계단을 부드럽게
        self._relay_send_step()

    def _update_map_hint(self) -> None:
        mode = self.relay_mode_combo.currentData() if hasattr(self, "relay_mode_combo") else "rpy"
        if mode == "quat":
            self.map_status.setText(
                "IMU1(P→ID1, R→ID2, Y→ID3) | IMU2(P→ID4) | IMU3(Y→ID5, R→ID6, P→ID7) 상대 Quaternion"
            )
        else:
            self.map_status.setText(
                "IMU1 상대 P→ID1, R→ID2, Y→ID3 | IMU2 상대 P→ID4 | IMU3 상대 Y→ID5, R→ID6, P→ID7"
            )

    def _get_relay_imu_rpy(self, data: dict, imu_sel: str) -> Tuple[float, float, float]:
        prefix = imu_sel + "_"
        roll  = data.get(prefix + "roll",  data.get("roll",  0.0))
        pitch = data.get(prefix + "pitch", data.get("pitch", 0.0))
        yaw   = data.get(prefix + "yaw",   data.get("yaw",   0.0))
        return float(roll), float(pitch), float(yaw)

    def _capture_relay_reference(self) -> bool:
        # 리더 모드: 현재 리더 자세를 0점으로 (IMU 불필요)
        if self.relay_mode_combo.currentData() == "leader":
            if not self._leader_pos:
                self._log_imu("기준 캡처 실패: 리더 데이터 없음")
                return False
            self._leader_ref = dict(self._leader_pos)
            self._relay_prev_angle = {}
            self._last_relay_goal.clear()
            if self._relay_active:
                if not self._capture_relay_motor_base():
                    self._pp_freeze = True
                    self._log_imu("🛑 재설정 중 베이스 정합성 실패 — 전달 freeze (W1)")
                self._apply_leader_zero_offsets()
            self._log_imu("리더 기준 자세 재설정 (현재 리더 자세 = 0점)")
            return True
        if not self._latest_imu:
            self._log_imu("기준 자세 캡처 실패: IMU 데이터가 아직 없습니다.")
            return False
        if self._calib_ik is not None:
            self._calib_ik.reset()  # 가지 연속성 메모리 초기화
        self._imu_tgt_prev, self._imu_jump_cnt = {}, {}  # 점프 디바운스 리셋
        self._drift_ema, self._drift_prev, self._drift_warn_t = {}, {}, 0.0  # 드리프트 워치독 리셋
        self._pp_freeze, self._pp_mismatch = False, {}  # 괴리 워치독 리셋 (기준설정=재개 의사)
        self._sensor_stale_warned = False               # D1a 센서별 신선도 경고 리셋
        for imu_key in RELAY_IMU_MOTOR_MAP:
            roll, pitch, yaw = self._get_relay_imu_rpy(self._latest_imu, imu_key)
            self._relay_ref_rpy[imu_key] = {"roll": roll, "pitch": pitch, "yaw": yaw}
            self._relay_ref_quat[imu_key] = self._imu_current_quat(
                self._latest_imu, imu_key, roll, pitch, yaw)
            self._prev_quat[imu_key] = self._relay_ref_quat[imu_key]
        
        # Kinematic chain: 기준 시점의 절대 쿼터니언으로 초기화
        self._imu1_abs_cur = self._relay_ref_quat["imu1"]
        self._imu2_abs_cur = self._relay_ref_quat["imu2"]

        self._filtered_motor_angles = {motor_id: 0.0 for motor_id in MOTOR_IDS}
        for _f in self._one_euro.values():  # 기준 재설정 = 0점 불연속 → 필터 상태도 리셋
            _f.reset()
        self._prev_rel_euler = {
            imu_key: {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            for imu_key in RELAY_IMU_MOTOR_MAP
        }
        self._last_relay_goal.clear()
        # 기준 재설정 = 0점이 불연속 점프 → 변화율클램프 prev도 리셋해야 런지 안 함 (5차 A8-1)
        self._relay_prev_angle = {}
        if self._relay_active:
            # 전달 중 재설정이면 모터 베이스도 현재 위치로 재캡처 (새 0점 = 현재 자세)
            if not self._capture_relay_motor_base():
                self._pp_freeze = True
                self._log_imu("🛑 재설정 중 베이스 정합성 실패 — 전달 freeze (W1)")
            self._log_imu("전달 기준 자세를 현재 값으로 재설정했습니다. (Parent Frame Kinematic Chain 활성화)")
        return True

    # ── 2-포즈 재정렬: 자세를 1.5초 평균으로 캡처 → 장착변화 보정 ──────────

    def _reanchor_capture(self, slot: str) -> None:
        if self._calib_ik is None:
            QMessageBox.warning(self, "보정 없음", "calibration.json 이 없습니다.")
            return
        if self._relay_active:
            QMessageBox.warning(self, "전달 중", "전달을 멈춘 뒤 재정렬하세요.")
            return
        if not self._latest_imu or self._latest_imu.get("imu1_quat") is None:
            QMessageBox.warning(self, "IMU 대기", "IMU(쿼터니언) 데이터가 아직 없습니다.")
            return
        if getattr(self, "_ra_timer", None) and self._ra_timer.isActive():
            return  # 이미 캡처 중
        self._ra_slot = slot
        self._ra_samples = {"imu1": [], "imu2": [], "imu3": []}
        self._ra_ticks = 0
        if not hasattr(self, "_ra_timer") or self._ra_timer is None:
            self._ra_timer = QTimer(self)
            self._ra_timer.timeout.connect(self._reanchor_tick)
        self._ra_timer.setInterval(150)
        self._ra_timer.start()
        self._log_imu(f"재정렬 {slot.upper()} 캡처중… 1.5초간 자세 유지 (움직이지 마세요)")

    def _reanchor_tick(self) -> None:
        from calib_ik import gravity_from_quat, angle_deg
        d = self._latest_imu or {}
        for imu in ("imu1", "imu2", "imu3"):
            q = d.get(f"{imu}_quat")
            if q is not None:
                self._ra_samples[imu].append(gravity_from_quat(q))
        self._ra_ticks += 1
        if self._ra_ticks < 10:
            return
        self._ra_timer.stop()
        # 평균 + 정지 게이트(샘플 산포 ≤ 3°)
        result = {}
        for imu, gs in self._ra_samples.items():
            if len(gs) < 5:
                self._log_imu(f"재정렬 실패: {imu} 샘플 부족({len(gs)})")
                return
            mean = [sum(g[i] for g in gs) / len(gs) for i in range(3)]
            n = max(1e-9, (mean[0]**2 + mean[1]**2 + mean[2]**2) ** 0.5)
            mean = [x / n for x in mean]
            scatter = max(angle_deg(g, mean) for g in gs)
            if scatter > 3.0:
                self._log_imu(f"재정렬 실패: {imu} 움직임 감지(산포 {scatter:.1f}°>3°) — 다시 캡처")
                return
            result[imu] = mean
        if not hasattr(self, "_ra_g"):
            self._ra_g = {}
        self._ra_g[self._ra_slot] = result
        self._log_imu(f"재정렬 {self._ra_slot.upper()} 캡처 완료 ✓"
                      + ("  → 이제 [적용]" if {"p1", "p2"} <= set(self._ra_g) else "  → 다음 자세 캡처"))

    def _reanchor_apply(self) -> None:
        if self._calib_ik is None:
            return
        g = getattr(self, "_ra_g", {})
        if "p1" not in g or "p2" not in g:
            QMessageBox.warning(self, "캡처 부족", "P1, P2 둘 다 캡처한 뒤 적용하세요.")
            return
        gpairs = {imu: {"p1": g["p1"][imu], "p2": g["p2"][imu]} for imu in ("imu1", "imu2", "imu3")}
        if not self._calib_ik.has_anchor():
            # 최초 1회: 풀 보정이 유효한 '직후'에만 앵커로 저장해야 함
            ret = QMessageBox.question(
                self, "앵커 저장",
                "저장된 앵커가 없습니다.\n지금 캡처를 '앵커'(기준)로 저장할까요?\n"
                "※ 반드시 풀 캘리브레이션이 유효한 상태(보정 직후)에서만 저장하세요!")
            if ret != QMessageBox.Yes:
                return
            self._calib_ik.set_anchor(gpairs)
            try:
                cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
                self._calib_ik.save(cal_path)
                self._log_imu("앵커 저장됨 (calibration.json) — 다음 착용부터 P1/P2/적용 만으로 재정렬됩니다.")
            except Exception as exc:
                self._log_imu(f"앵커 저장 실패: {exc}")
            return
        rep = self._calib_ik.apply_reanchor(gpairs)
        for imu, r in rep.items():
            if r["status"] == "applied":
                warn = " ⚠큰변화" if r.get("warn") else ""
                self._log_imu(f"재정렬 {imu}: 적용 {r['rot_deg']:.1f}° (잔차 {r['resid_deg']:.1f}°){warn}")
            elif r["status"] == "noop":
                self._log_imu(f"재정렬 {imu}: 변화 {r.get('rot_deg', 0):.1f}°<3° → 원본 유지")
            else:
                self._log_imu(f"재정렬 {imu}: 거부 — {r.get('reason', '?')}")
        self._log_imu("재정렬 완료. [전달 시작] 시 기준 자세가 자동 재캡처됩니다.")

    def _capture_relay_motor_base(self) -> bool:
        """모터 베이스(현재 위치) 캡처 + W1 정합성 게이트.
        m5 90° 대책: 베이스 오류는 슬루가 속도만 줄이지 방향을 못 고침 → 결정론적 차단.
          ① 재읽기 2회 대조(읽기 응답 어긋남 검출)
          ② 직전 알려진 Goal과 >30°(341틱) 차이면 베이스 점프 의심
        하나라도 걸리면 False 반환(전달 시작 거부). 반환 True일 때만 커밋."""
        if not self.motor_connected or self.port_handler is None:
            return False

        TICK_TOL = 12         # 재읽기 2회 일치 허용(~1°) — 토크상태 무관 안정적이어야
        GOAL_GAP_TICKS = 341  # 30° — 직전 goal 대비 베이스 점프 한계
        bad: List[str] = []
        new_base: Dict[int, int] = {}
        for motor_id in MOTOR_IDS:
            self.port_handler.clearPort()
            p1, _e1 = self._safe_read_present_position(motor_id)
            self.port_handler.clearPort()
            p2, _e2 = self._safe_read_present_position(motor_id)
            if p1 is None or p2 is None:
                bad.append(f"ID{motor_id} 읽기실패")
                continue
            if abs(p1 - p2) > TICK_TOL:
                bad.append(f"ID{motor_id} 재읽기 불일치({p1}≠{p2})")
            base_pos = p2
            # 직전 기준: last_goal 우선, 비었으면(재설정 시 clear됨) 직전 베이스로 대조.
            # → 재설정(베이스 점프 최고위험)에도 GOAL_GAP이 살아있음(FC #2).
            prev_goal = self._last_relay_goal.get(motor_id)
            if prev_goal is None:
                prev_goal = self._relay_base_pos.get(motor_id)
            if prev_goal is not None and abs(base_pos - prev_goal) > GOAL_GAP_TICKS:
                bad.append(f"ID{motor_id} 베이스 점프({base_pos} vs 직전 {prev_goal}, "
                           f"{abs(base_pos - prev_goal) / 4095.0 * 360.0:.0f}°)")
            new_base[motor_id] = base_pos

        if bad:
            self._log_imu("🛑 베이스 정합성 실패 — 전달 시작/재설정 거부 (W1): " + " / ".join(bad))
            self._log_imu("   모터 통신·케이블·ID·토크 상태 확인 후 재시도하세요.")
            return False

        _bases = []
        for motor_id in MOTOR_IDS:
            self._relay_base_pos[motor_id] = new_base[motor_id]
            self._last_relay_goal[motor_id] = new_base[motor_id]
            _bases.append(f"{motor_id}:{new_base[motor_id]}")
        self._log_imu("모터 베이스 캡처(정합 OK): " + " ".join(_bases))
        return True

    def _imu_current_quat(self, data: dict, imu_key: str,
                          roll: float, pitch: float, yaw: float):
        """현재 자세 쿼터니언. 펌웨어가 보낸 '센서 직접' 쿼터니언이 있으면 사용(짐벌락 회피),
        없으면(구펌웨어/센서 미응답) 오일러 역산으로 폴백."""
        q = data.get(f"{imu_key}_quat")
        if q is not None:
            return q
        return euler_deg_to_quat(roll, pitch, yaw)

    def _get_relative_angles_for_imu(self, data: dict, imu_key: str, mode: str) -> Dict[str, float]:
        """
        각 IMU의 로컬 상대 회전을 계산한다 (Parent Frame Kinematic Chain).

        핵심 원리:
          - IMU2의 로컬 변화 = (q1_ref⁻¹·q2_ref)⁻¹ · (q1_cur⁻¹·q2_cur)
            → q1_cur 프레임 기준으로 imu2가 얼마나 움직였는가
          - IMU3의 로컬 변화 = (q2_ref⁻¹·q3_ref)⁻¹ · (q2_cur⁻¹·q3_cur)
            → q2_cur 프레임 기준으로 imu3가 얼마나 움직였는가

        이렇게 하면 상위 관절이 90° 회전해도 하위 관절의 RPY 축이
        상위 관절의 프레임을 기준으로 올바르게 유지된다.
        """
        roll, pitch, yaw = self._get_relay_imu_rpy(data, imu_key)

        if mode == "quat":
            q_cur = self._imu_current_quat(data, imu_key, roll, pitch, yaw)
            q_cur = ensure_quat_continuity(q_cur, self._prev_quat[imu_key])
            self._prev_quat[imu_key] = q_cur

            if imu_key == "imu1":
                # IMU1: 월드 기준 절대 상대 회전
                # q_rel = q1_ref⁻¹ · q1_cur
                q_rel = quat_mul(quat_conjugate(self._relay_ref_quat["imu1"]), q_cur)
                # 이후 imu2 계산을 위해 현재 절대 쿼터니언 저장
                self._imu1_abs_cur = q_cur

            elif imu_key == "imu2":
                # IMU2: imu1 프레임 기준 로컬 상대 회전
                # q2_local_cur = q1_cur⁻¹ · q2_cur
                # q2_local_ref = q1_ref⁻¹ · q2_ref
                # q_rel = q2_local_ref⁻¹ · q2_local_cur
                q2_local_ref = quat_mul(
                    quat_conjugate(self._relay_ref_quat["imu1"]),
                    self._relay_ref_quat["imu2"],
                )
                q2_local_cur = quat_mul(quat_conjugate(self._imu1_abs_cur), q_cur)
                q_rel = quat_mul(quat_conjugate(q2_local_ref), q2_local_cur)
                # 이후 imu3 계산을 위해 현재 절대 쿼터니언 저장
                self._imu2_abs_cur = q_cur

            elif imu_key == "imu3":
                # IMU3: imu2 프레임 기준 로컬 상대 회전
                # q3_local_cur = q2_cur⁻¹ · q3_cur
                # q3_local_ref = q2_ref⁻¹ · q3_ref
                # q_rel = q3_local_ref⁻¹ · q3_local_cur
                q3_local_ref = quat_mul(
                    quat_conjugate(self._relay_ref_quat["imu2"]),
                    self._relay_ref_quat["imu3"],
                )
                q3_local_cur = quat_mul(quat_conjugate(self._imu2_abs_cur), q_cur)
                q_rel = quat_mul(quat_conjugate(q3_local_ref), q3_local_cur)

            else:
                q_rel = quat_mul(quat_conjugate(self._relay_ref_quat[imu_key]), q_cur)

            self._relay_qrel[imu_key] = q_rel  # IK 경로가 이 체인 상대쿼터니언을 사용
            d_roll, d_pitch, d_yaw = quat_to_euler_deg(q_rel)
            # ±180° 경계 점프 제거 (Euler 언래핑)
            prev = self._prev_rel_euler[imu_key]
            d_roll  = unwrap_angle_deg(d_roll,  prev["roll"])
            d_pitch = unwrap_angle_deg(d_pitch, prev["pitch"])
            d_yaw   = unwrap_angle_deg(d_yaw,   prev["yaw"])
            self._prev_rel_euler[imu_key] = {"roll": d_roll, "pitch": d_pitch, "yaw": d_yaw}
            return {"roll": d_roll, "pitch": d_pitch, "yaw": d_yaw}

        # RPY 모드: 단순 차분 (gimbal lock 주의, 참고용)
        prev = self._prev_rel_euler[imu_key]
        d_roll  = unwrap_angle_deg(roll  - self._relay_ref_rpy[imu_key]["roll"],  prev["roll"])
        d_pitch = unwrap_angle_deg(pitch - self._relay_ref_rpy[imu_key]["pitch"], prev["pitch"])
        d_yaw   = unwrap_angle_deg(yaw   - self._relay_ref_rpy[imu_key]["yaw"],   prev["yaw"])
        self._prev_rel_euler[imu_key] = {"roll": d_roll, "pitch": d_pitch, "yaw": d_yaw}
        return {"roll": d_roll, "pitch": d_pitch, "yaw": d_yaw}

    def _relay_to_motors(self, data: dict) -> None:
        mode = self.relay_mode_combo.currentData()
        use_ik = (mode == "calib_ik" and self._calib_ik is not None
                  and self._calib_ik.is_calibrated())

        raw_mapping: Dict[int, float] = {}
        if use_ik:
            # 운동학 체인으로 q_rel 채우기 (반드시 imu1→imu2→imu3 순서)
            for imu_key in ("imu1", "imu2", "imu3"):
                self._get_relative_angles_for_imu(data, imu_key, "quat")
            motor_deg = self._calib_ik.motor_angles_deg(self._relay_qrel)
            # 드리프트 워치독(경고만): yaw 노출 채널의 '느리고 꾸준한 흐름' 감지
            #   (롤채널 드리프트 ~27°/min 실측 → 사람이 봐도 모르게 흘러가는 걸 알려줌)
            if not hasattr(self, "_drift_ema"):
                self._drift_ema, self._drift_prev, self._drift_warn_t = {}, {}, 0.0
            for _mid in (3, 4, 5, 7):
                _cur = motor_deg.get(_mid, 0.0)
                _pv = self._drift_prev.get(_mid)
                self._drift_prev[_mid] = _cur
                if _pv is None:
                    continue
                _v = (_cur - _pv) / max(1e-3, self._relay_proc_dt)
                _ema = 0.996 * self._drift_ema.get(_mid, 0.0) + 0.004 * _v  # τ≈8s
                self._drift_ema[_mid] = _ema
                if 0.08 < abs(_ema) < 2.0 and time.time() - getattr(self, "_drift_warn_t", 0.0) > 30:
                    self._drift_warn_t = time.time()
                    self._log_imu(f"⚠ j{_mid} 드리프트 의심 ({_ema*60:+.0f}°/min 꾸준한 흐름)"
                                  f" — 중립자세에서 [기준 설정] 권장")
            # 점프 디바운스(6차 FC#5): 한 프레임짜리 >45° 점프(가지오류 등)는 보류,
            # 3프레임 연속 확인돼야 추종 (사람 동작은 프레임당 45° 불가능 → 영향 없음)
            if not hasattr(self, "_imu_tgt_prev"):
                self._imu_tgt_prev, self._imu_jump_cnt = {}, {}
            for mid in MOTOR_IDS:
                newv = motor_deg.get(mid, 0.0)
                oldv = self._imu_tgt_prev.get(mid)
                if oldv is not None and abs(newv - oldv) > 45.0:
                    c = self._imu_jump_cnt.get(mid, 0) + 1
                    if c < 3:
                        self._imu_jump_cnt[mid] = c
                        motor_deg[mid] = oldv  # 보류 (이전 목표 유지)
                        if c == 1:
                            self._log_imu(f"⚠ 분해점프 보류 ID{mid}: {oldv:+.0f}°→{newv:+.0f}° (3프레임 확인 대기)")
                    else:
                        self._imu_jump_cnt[mid] = 0
                        self._imu_tgt_prev[mid] = newv
                        self._log_imu(f"분해점프 확정 ID{mid}: {newv:+.0f}° 추종 (점진 이동)")
                else:
                    self._imu_jump_cnt[mid] = 0
                    self._imu_tgt_prev[mid] = newv
            raw_mapping = {mid: motor_deg.get(mid, 0.0) for mid in MOTOR_IDS}
            # 디버그 로그: IK 관절각 + 손목 q_rel (ID7 이상 추적용). /tmp/bandlead_ik.log
            try:
                q3 = self._relay_qrel.get("imu3", [1, 0, 0, 0])
                # 팔꿈치 투영(j4) vs 회전크기 동시기록: 순수 굴곡시 차이 = 보정축 오차 φ 실측
                _emag = 0.0
                _q2r = self._relay_qrel.get("imu2")
                _a2 = self._calib_ik._axis("imu2", "a") if self._calib_ik else None
                if _q2r and _a2:
                    from calib_ik import hinge_angle as _hg, _canon as _cn
                    _emag = math.degrees(_hg(_cn(_q2r), _a2))
                _res = getattr(self._calib_ik, "last_res_deg", {})
                # 파일 기록은 10Hz 스로틀 (50Hz 그대로면 open/s 50회 + ~20MB/h tmpfs 잠식 — FC)
                if time.time() - getattr(self, "_iklog_t", 0.0) >= 0.1:
                    self._iklog_t = time.time()
                    with open("/tmp/bandlead_ik.log", "a", encoding="utf-8") as _f:
                        _f.write(
                            f"{time.time():.3f} | " +
                            " ".join(f"j{i}={motor_deg.get(i, 0.0):+7.1f}" for i in MOTOR_IDS) +
                            f" | el_mag={_emag:+6.1f}" +
                            f" | res={_res.get('imu1', 0):.1f}/{_res.get('imu3', 0):.1f}" +
                            f" | q3=[{q3[0]:+.3f},{q3[1]:+.3f},{q3[2]:+.3f},{q3[3]:+.3f}]\n"
                        )
                # 표현불가(커버리지 구멍)/특이 감지 경고 (스로틀)
                _rw = max(_res.get("imu1", 0), _res.get("imu3", 0))
                if _rw > 5.0 and time.time() - getattr(self, "_res_warn_t", 0) > 2.0:
                    self._res_warn_t = time.time()
                    self._log_imu(f"⚠ 분해잔차 {_rw:.0f}° (자세가 보정축 표현범위 밖 — 극단자세/보정품질)")
            except Exception:
                pass
        else:
            for imu_key, motor_axis_map in RELAY_IMU_MOTOR_MAP.items():
                relative_angles = self._get_relative_angles_for_imu(data, imu_key, mode)
                for motor_id, axis in motor_axis_map.items():
                    raw_mapping[motor_id] = relative_angles[axis]
            # elbow-wrist 연동 보정(구 매핑 전용). IK 모드에선 적용 안 함.
            raw_mapping[7] = raw_mapping.get(7, 0.0) - (
                MOTOR7_ELBOW_COMP_GAIN * raw_mapping.get(4, 0.0)
            )

        # One Euro: 패킷 도메인(새 패킷마다)에서 적용, dt는 패킷 간격 (FC 확인: 위치 동일)
        _dt_pkt = getattr(self, "_relay_proc_dt", 0.031)
        for motor_id, raw_angle in raw_mapping.items():
            self._filtered_motor_angles[motor_id] = (
                self._one_euro[motor_id].filter(raw_angle, _dt_pkt)
            )

        mapping = {
            motor_id: self._filtered_motor_angles[motor_id]
            for motor_id in raw_mapping
        }

        sign = IK_SIGN if use_ik else MOTOR_DIRECTION
        mapping = {
            motor_id: angle * sign[motor_id]
            for motor_id, angle in mapping.items()
        }

        # 목표만 갱신(5-6Hz) — 실제 전송은 _relay_send_step()이 20Hz로 부드럽게 보간.
        #   (패킷 계단을 그대로 보내면 툭툭 끊김 + 클램프가 매 패킷 걸림)
        self._relay_target = mapping

    def _relay_send_step(self) -> None:
        """20Hz 보간 전송: 최신 목표(_relay_target)를 향해 매 틱 속도제한 보폭으로 전진.
        명령이 연속적이라 부드럽고, 목표가 점프해도 40°/s 이상 못 따라감(안전)."""
        target = getattr(self, "_relay_target", None)
        if not target:
            return
        _now = time.time()
        _dt = min(max(_now - getattr(self, "_relay_step_time", 0.0), 0.005), 0.3)
        self._relay_step_time = _now
        _vel = RAMP_VEL_DEG_S if _now < self._ramp_until else MAX_VEL_DEG_S
        _max_step = _vel * _dt
        # GUI(슬라이더/상태줄) 갱신은 5틱당 1회(20Hz) — 100Hz 위젯 갱신은 이벤트루프 낭비
        self._relay_gui_tick = getattr(self, "_relay_gui_tick", 0) + 1
        gui_now = (self._relay_gui_tick % 5 == 0)

        # 1단계: 각 모터의 '원하는 목표' (데드밴드 + 관절한계)
        wants: Dict[int, float] = {}
        for motor_id, angle in target.items():
            a = angle
            if abs(a) < RELAY_DEADBAND_DEG.get(motor_id, 2.0):
                a = 0.0
            _lim = JOINT_LIMIT_DEG.get(motor_id, 100.0)  # ★ 안전 백스톱1
            wants[motor_id] = max(-_lim, min(_lim, a))

        # 2단계: ★동기화 보간 — 모든 관절 스텝을 '같은 비율'로 축소(제일 멀리 갈 관절 기준).
        #   관절별 따로 제한하면 휩쓸기 중 경로 모양이 깨져 '쳐졌다가 도달'(sag).
        #   비율 보존 = 관절들이 같이 도착 = 경로 모양 유지.
        _scale = 1.0
        for motor_id, w in wants.items():
            _prev = self._relay_prev_angle.get(motor_id)
            if _prev is not None:
                need = abs(w - _prev)
                if need > _max_step:
                    _scale = max(_scale, need / _max_step)

        sync_goals: Dict[int, int] = {}  # motor_id -> goal_pos (실제로 전송할 것만)
        parts = []
        for motor_id, _want in wants.items():
            _prev = self._relay_prev_angle.get(motor_id)
            if _prev is None:
                angle_clamped = _want
            else:
                angle_clamped = _prev + (_want - _prev) / _scale  # ★ 안전 백스톱2(속도제한) 겸 보간
                # 보간 전진은 정상 → 조용히. '비정상 점프'(>25°)만 경고(스로틀)
                if abs(_want - _prev) > CLAMP_WARN_JUMP_DEG:
                    st = self._clamp_log_state.setdefault(motor_id, {"on": False, "n": 0, "t": 0.0})
                    st["n"] += 1
                    if not st["on"] or _now - st["t"] > 1.0:
                        self._log_imu(f"⚠ 점프감지 ID{motor_id}: 목표 {_want:+.0f}° (현재 {_prev:+.0f}°)"
                                      f" — {_vel:.0f}°/s로 추종 중")
                        st["on"], st["t"], st["n"] = True, _now, 0
                else:
                    st = self._clamp_log_state.get(motor_id)
                    if st and st["on"]:
                        st["on"] = False
            self._relay_prev_angle[motor_id] = angle_clamped

            delta_pos = int(round(angle_clamped / 360.0 * 4095))
            delta_pos = max(-2048, min(2047, delta_pos))

            base_pos = self._relay_base_pos.get(motor_id, 2048)
            # 모터 EEPROM 한계 밖 Goal은 모터가 거부(범위오류) → 앱에서도 같은 한계로 클램프
            _pmn, _pmx = MOTOR_POS_LIMITS.get(motor_id, (0, 4095))
            pos = max(_pmn, min(_pmx, base_pos + delta_pos))
            last_goal = self._last_relay_goal.get(motor_id)
            # ★ 위치 도메인 슬루: 각도클램프는 '베이스 오류'를 못 잡음 (goal=틀린베이스+0이면
            #   첫 프레임에 모터가 풀속도로 슬램). 마지막 전송 goal 기준 틱 스텝도 제한.
            if last_goal is not None:
                _step_ticks = max(2, int(_max_step / 360.0 * 4095) + 1)
                if abs(pos - last_goal) > _step_ticks:
                    pos = last_goal + _step_ticks if pos > last_goal else last_goal - _step_ticks
            w = self.motor_widgets[motor_id]
            if w.torque_btn.isChecked():
                # ★FC A-2 필수수정: 민스텝을 슬루스텝과 연동. 10ms 틱에서 램프인(15°/s)은
                #   슬루 2틱 < 민스텝 3틱이라 영원히 미전송 → last_goal 동결 → 램프 종료시
                #   누적분을 풀속(184°/s)으로 한 번에 추격(런지). min()으로 동결 불가능하게.
                _min_step = min(RELAY_MIN_COMMAND_STEP, _step_ticks) if last_goal is not None else RELAY_MIN_COMMAND_STEP
                if last_goal is None or abs(pos - last_goal) >= _min_step:
                    sync_goals[motor_id] = pos
                    self._last_relay_goal[motor_id] = pos
                    if gui_now:
                        w.slider.blockSignals(True)
                        w.spin.blockSignals(True)
                        w.slider.setValue(pos)
                        w.spin.setValue(pos)
                        w.slider.blockSignals(False)
                        w.spin.blockSignals(False)
            if gui_now:
                parts.append(f"ID{motor_id}: d{angle_clamped:+.1f}°→{pos}")

        # 변경된 모터가 있으면 GroupSyncWrite 로 일괄 전송
        if sync_goals and self._sync_writer is not None:
            self._sync_writer.clearParam()
            for motor_id, pos in sync_goals.items():
                param = [
                    pos & 0xFF,
                    (pos >> 8) & 0xFF,
                    (pos >> 16) & 0xFF,
                    (pos >> 24) & 0xFF,
                ]
                self._sync_writer.addParam(motor_id, param)
            comm_result = self._sync_writer.txPacket()
            if comm_result != COMM_SUCCESS:
                self._log_imu(
                    f"SyncWrite 실패: {self.packet_handler.getTxRxResult(comm_result)}"
                )

        self._apply_hold_motors()
        if gui_now:
            self.map_status.setText(" | ".join(parts))

        # ── 명령-실위치 괴리 워치독 ──
        # 저빈도(0.5s) 모터 1개씩이라 read 블로킹(~수ms)이 틱에 주는 영향 미미.
        # 램프인(전달 시작/복구 직후) 동안은 워치독을 빨리 — base 오류 슬램을 조기 검출(FC #4)
        _watch_iv = 0.12 if _now < self._ramp_until else PP_WATCH_INTERVAL_S
        if _now - getattr(self, "_pp_watch_t", 0.0) >= _watch_iv:
            self._pp_watch_t = _now
            _wids = [m for m in MOTOR_IDS
                     if self.motor_widgets[m].torque_btn.isChecked()
                     and self._last_relay_goal.get(m) is not None]
            if _wids:
                _mid = _wids[getattr(self, "_pp_watch_idx", 0) % len(_wids)]
                self._pp_watch_idx = getattr(self, "_pp_watch_idx", 0) + 1
                _present, _ = self._safe_read_present_position(_mid)
                if _present is not None:
                    if not hasattr(self, "_pp_mismatch"):
                        self._pp_mismatch = {}
                    _gap = abs(_present - self._last_relay_goal[_mid]) / 4095.0 * 360.0
                    if _gap > PP_WATCH_GAP_DEG:
                        _n = self._pp_mismatch.get(_mid, 0) + 1
                        self._pp_mismatch[_mid] = _n
                        if _n >= 2:
                            self._pp_freeze = True
                            self._log_imu(
                                f"🛑 괴리 워치독: ID{_mid} 명령-실위치 {_gap:.0f}° 지속"
                                f" — 충돌/걸림 의심, 전달 freeze! ([전달 중지]→[시작]으로 재개)")
                        else:
                            self._log_imu(f"⚠ 괴리 감지 ID{_mid}: 명령-실위치 {_gap:.0f}° (1회, 관찰 중)")
                    else:
                        self._pp_mismatch[_mid] = 0

    def _prepare_hold_motors(self) -> None:
        self._hold_motor_ids_available = []
        self._hold_motor_goal.clear()
        self._last_hold_write_time = 0.0

        if not self.motor_connected or self.port_handler is None:
            return

        for motor_id in HOLD_MOTOR_IDS:
            try:
                _model_no, comm_result, dxl_error = self.packet_handler.ping(self.port_handler, motor_id)
            except Exception:
                continue

            if comm_result != COMM_SUCCESS or dxl_error != 0:
                continue

            try:
                comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
                )
                if comm_result != COMM_SUCCESS or dxl_error != 0:
                    self._log_imu(f"고정 토크 ON 실패(ID {motor_id})")
                    continue

                self._apply_safe_profile(motor_id)
                present, _err = self._safe_read_present_position(motor_id)
                if present is None:
                    present = 2048

                comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_GOAL_POSITION, present
                )
                if comm_result != COMM_SUCCESS or dxl_error != 0:
                    self._log_imu(f"고정 목표 설정 실패(ID {motor_id})")
                    continue
            except Exception:
                continue

            self._hold_motor_ids_available.append(motor_id)
            self._hold_motor_goal[motor_id] = present

        if self._hold_motor_ids_available:
            self._log_imu(f"고정 모터 활성화: {self._hold_motor_ids_available}")
        else:
            self._log_imu("고정 대상 모터(ID 8) 미연결: 고정 기능을 건너뜁니다.")

    def _apply_hold_motors(self) -> None:
        if not self._hold_motor_ids_available or self.port_handler is None:
            return

        now = time.time()
        if now - self._last_hold_write_time < 0.2:
            return

        for motor_id in self._hold_motor_ids_available:
            goal = self._hold_motor_goal.get(motor_id)
            if goal is None:
                continue
            try:
                comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_GOAL_POSITION, goal
                )
                if comm_result != COMM_SUCCESS or dxl_error != 0:
                    self._log_imu(f"고정 재전송 실패(ID {motor_id})")
            except Exception:
                continue

        self._last_hold_write_time = now

    # ── 종료 정리 ─────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._imu_disconnect()
        self._motor_disconnect()
        if self._leader is not None:
            self._toggle_leader()  # 스레드 정지 + 포트 닫기
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    apply_stylesheet(app)
    win = ImuWithMotorWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

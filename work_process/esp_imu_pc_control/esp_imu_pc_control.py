#!/usr/bin/env python3
"""IMU -> Motor integrated control.

Left panel:  IMU sensor (WT61C-TTL, WitMotion protocol)
Right panel: DYNAMIXEL motor (ID 1, 2, 3, Protocol 2.0)

Roll  -> ID 2 goal position
Pitch -> ID 1 goal position
Yaw   -> ID 3 goal position
"""

import sys
import math
import json
import socket
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from serial.tools import list_ports
from PyQt5.QtCore import Qt, QSize, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
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

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from stylesheet import apply_stylesheet


# ── DYNAMIXEL 제어 테이블 (Protocol 2.0) ─────────────────────────────────
PROTOCOL_VERSION      = 2.0
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
OPERATING_MODE_POSITION = 3

MOTOR_IDS = (1, 2, 3)
HOLD_MOTOR_IDS = (4, 5, 6, 7, 8)
MOTOR_DIRECTION = {1: 1.0, 2: -1.0, 3: -1.0}
SAFE_INIT_STEP_TICKS = 8
SAFE_INIT_INTERVAL_MS = 20
RELAY_DEADBAND_DEG = {1: 2.0, 2: 2.0, 3: 3.0}
RELAY_MIN_COMMAND_STEP = 10


def angle_to_pos(angle: float) -> int:
    """[-180, +180] 도(degree)를 [0, 4095] 위치로 선형 매핑."""
    return int(max(0, min(4095, (angle + 180.0) / 360.0 * 4095)))


def clamp_angle_deg(angle: float) -> float:
    """각도를 [-180, +180] 범위로 정규화."""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return wrapped


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


def ensure_quat_continuity(
    q_cur: Tuple[float, float, float, float],
    q_prev: Optional[Tuple[float, float, float, float]],
) -> Tuple[float, float, float, float]:
    """이전 쿼터니언과의 논리적 거리를 최소화하여 연속성을 보장한다.
    
    q와 -q는 같은 회전을 나타내므로, 이전 쿼터니언과의 dot product가 음수면
    현재 쿼터니언의 부호를 반전시킨다.
    """
    if q_prev is None:
        return q_cur
    
    # dot product 계산
    dot_prod = sum(a * b for a, b in zip(q_cur, q_prev))
    
    # dot product가 음수면 현재 쿼터니언의 부호 반전
    if dot_prod < 0.0:
        return (-q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3])
    return q_cur


# ── IMU 파서 ───────────────────────────────────────────────────────────────────
class WT61Parser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.has_quaternion_frame = False
        self.latest: Dict[str, float] = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "ax":   0.0, "ay":   0.0,  "az":  0.0,
            "wx":   0.0, "wy":   0.0,  "wz":  0.0,
            "q0":   1.0, "q1":   0.0,  "q2":  0.0, "q3": 0.0,
        }

    @staticmethod
    def _to_i16(lo: int, hi: int) -> int:
        val = (hi << 8) | lo
        return val - 0x10000 if val >= 0x8000 else val

    def feed(self, chunk: bytes):
        frames = []
        self.buffer.extend(chunk)

        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue

            frame = self.buffer[:11]
            if (sum(frame[:10]) & 0xFF) != frame[10]:
                del self.buffer[0]
                continue

            ftype = frame[1]
            d0, d1, d2, d3, d4, d5 = frame[2:8]

            if ftype == 0x51:
                self.latest.update({
                    "ax": self._to_i16(d0, d1) / 32768.0 * 16.0,
                    "ay": self._to_i16(d2, d3) / 32768.0 * 16.0,
                    "az": self._to_i16(d4, d5) / 32768.0 * 16.0,
                })
            elif ftype == 0x52:
                self.latest.update({
                    "wx": self._to_i16(d0, d1) / 32768.0 * 2000.0,
                    "wy": self._to_i16(d2, d3) / 32768.0 * 2000.0,
                    "wz": self._to_i16(d4, d5) / 32768.0 * 2000.0,
                })
            elif ftype == 0x53:
                self.latest.update({
                    "roll":  self._to_i16(d0, d1) / 32768.0 * 180.0,
                    "pitch": self._to_i16(d2, d3) / 32768.0 * 180.0,
                    "yaw":   self._to_i16(d4, d5) / 32768.0 * 180.0,
                })
                if not self.has_quaternion_frame:
                    q0, q1, q2, q3 = euler_deg_to_quat(
                        self.latest["roll"],
                        self.latest["pitch"],
                        self.latest["yaw"],
                    )
                    self.latest.update({
                        "q0": q0,
                        "q1": q1,
                        "q2": q2,
                        "q3": q3,
                    })
            elif ftype == 0x59:
                d6, d7 = frame[8:10]
                self.has_quaternion_frame = True
                self.latest.update({
                    "q0": self._to_i16(d0, d1) / 32768.0,
                    "q1": self._to_i16(d2, d3) / 32768.0,
                    "q2": self._to_i16(d4, d5) / 32768.0,
                    "q3": self._to_i16(d6, d7) / 32768.0,
                })

            frames.append((ftype, bytes(frame), dict(self.latest)))
            del self.buffer[:11]

        return frames


class UdpImuReaderThread(QThread):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    serial_error = pyqtSignal(str)
    parsed = pyqtSignal(dict)

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
            return None

        try:
            roll = float(packet["roll"])
            pitch = float(packet["pitch"])
            yaw = float(packet["yaw"])
        except (KeyError, TypeError, ValueError):
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

        return {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "q0": qw,
            "q1": qx,
            "q2": qy,
            "q3": qz,
            "ax": float(packet.get("ax", 0.0)),
            "ay": float(packet.get("ay", 0.0)),
            "az": float(packet.get("az", 0.0)),
            "wx": float(packet.get("wx", 0.0)),
            "wy": float(packet.get("wy", 0.0)),
            "wz": float(packet.get("wz", 0.0)),
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
        self.setWindowTitle("IMU with Motor Control")
        self.resize(1320, 780)

        self.motor_widgets: Dict[int, MotorWidgets] = {}
        self.value_labels:  Dict[str, QLabel]       = {}

        # IMU 상태
        self.imu_reader:    Optional[UdpImuReaderThread] = None
        self.imu_connected: bool = False

        # Motor 상태
        self.port_handler:    Optional[PortHandler] = None
        self.packet_handler   = PacketHandler(PROTOCOL_VERSION)
        self.motor_connected: bool = False

        # 전달(Relay) 상태
        self._relay_active: bool = False
        self._latest_imu: Dict[str, float] = {}
        self._relay_ref_rpy: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._relay_ref_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._relay_base_pos: Dict[int, int] = {motor_id: 2048 for motor_id in MOTOR_IDS}
        self._last_relay_goal: Dict[int, int] = {}
        self._hold_motor_ids_available = []
        self._hold_motor_goal: Dict[int, int] = {}
        self._last_hold_write_time = 0.0
        
        # 쿼터니언 연속성 & 저역통과필터 (노이즈 제거 및 안정성 개선)
        self._prev_quat: Optional[Tuple[float, float, float, float]] = None
        self._lpf_alpha = 0.15  # 저역통과필터 계수 (0.0~1.0, 낮을수록 부드러움)
        self._filtered_rpy: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        # 릴레이 속도 제한 타이머 (20 Hz)
        self._relay_timer = QTimer(self)
        self._relay_timer.setInterval(50)
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

        self.motor_id_status = QLabel("ID 1/2/3 연결 확인 대기")
        self.motor_id_status.setObjectName("secondary")
        motor_conn_layout.addWidget(self.motor_id_status)

        motor_area = QGroupBox("모터 제어 (ID 1, 2, 3)")
        motor_area_layout = QVBoxLayout(motor_area)
        motor_area_layout.setSpacing(8)

        init_row = QHBoxLayout()
        self.init_btn = QPushButton("초기 위치 설정 (모두 중앙)")
        self.init_btn.setObjectName("contained")
        self.init_btn.clicked.connect(self._initialize_motor_positions)
        init_row.addWidget(self.init_btn)
        init_row.addStretch(1)
        motor_area_layout.addLayout(init_row)

        for motor_id in MOTOR_IDS:
            motor_area_layout.addWidget(self._create_motor_group(motor_id))

        map_box = QGroupBox("IMU -> Motor 전달")
        map_layout = QVBoxLayout(map_box)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("전달 모드"))
        self.relay_mode_combo = QComboBox()
        self.relay_mode_combo.addItem("RPY Relative (시작 시점 기준)", "rpy")
        self.relay_mode_combo.addItem("Quaternion Relative (시작 시점 기준)", "quat")
        self.relay_mode_combo.currentIndexChanged.connect(self._update_map_hint)
        mode_row.addWidget(self.relay_mode_combo)
        mode_row.addStretch(1)
        map_layout.addLayout(mode_row)

        self.rezero_btn = QPushButton("기준 자세 재설정")
        self.rezero_btn.setObjectName("outlined")
        self.rezero_btn.clicked.connect(self._capture_relay_reference)
        map_layout.addWidget(self.rezero_btn)

        self.map_status = QLabel("Roll→ID2, Pitch→ID1, Yaw→ID3")
        self.map_status.setObjectName("secondary")
        map_layout.addWidget(self.map_status)

        self.start_relay_btn = QPushButton("전달 시작")
        self.start_relay_btn.setObjectName("contained")
        self.start_relay_btn.clicked.connect(self._toggle_relay)
        map_layout.addWidget(self.start_relay_btn)
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

    # ── IMU 연결 ──────────────────────────────────────────────────────────────────────

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
        for key, label in self.value_labels.items():
            label.setText(f"{data.get(key, 0.0):.3f}")
        self._latest_imu = data

    def _log_imu(self, msg: str) -> None:
        self.imu_log_text.append(msg)

    # ── Motor 연결 ─────────────────────────────────────────────────────────────────────

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

        self.motor_connected = True
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
        QMessageBox.information(self, "연결 성공", "ID 1, 2, 3 모터 연결 확인 완료")

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

        # Operating Mode 변경은 Torque OFF 상태에서만 허용된다.
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

        for w in self.motor_widgets.values():
            w.torque_btn.setChecked(False)

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
        self.motor_id_status.setText("ID 1/2/3 연결 확인 대기")

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

        # 릴레이 TxOnly 잔류 응답 패킷 제거
        self.port_handler.clearPort()

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
        self._set_goal_controls_enabled(motor_id, enabling)

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
        """모든 모터를 초기 위치(중앙 2048)로 설정한다."""
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
        """SDK의 간헐적 IndexError를 흡수하고 안전하게 현재 위치를 읽는다."""
        if self.port_handler is None:
            return None, "포트가 연결되어 있지 않습니다."

        last_err: Optional[str] = None
        for _ in range(3):
            try:
                # read4ByteTxRx()는 드물게 4바이트 미만 응답에서도 인덱싱해 IndexError를 낼 수 있어
                # raw read 후 길이를 먼저 검증한다.
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

    # ── 전달(Relay) ───────────────────────────────────────────────────────────────────────

    def _toggle_relay(self) -> None:
        if self._relay_active:
            self._relay_active = False
            self._relay_timer.stop()
            self._set_relay_button_state(False)
            self._update_map_hint()
            return

        if not self.imu_connected:
            QMessageBox.warning(self, "IMU 미연결", "먼저 IMU 센서를 연결해주세요.")
            return
        if not self.motor_connected:
            QMessageBox.warning(self, "모터 미연결", "먼저 모터 연결을 확인해주세요.")
            return

        self._capture_relay_reference()
        self._capture_relay_motor_base()
        try:
            self._prepare_hold_motors()
        except Exception as exc:
            self._log_imu(f"고정 모터 준비 건너뜀: {exc}")

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

        # Re-apply style when objectName changes at runtime.
        style = self.start_relay_btn.style()
        style.unpolish(self.start_relay_btn)
        style.polish(self.start_relay_btn)
        self.start_relay_btn.update()

    def _relay_tick(self) -> None:
        """20 Hz 타이머 콜백 – 최신 IMU 값으로 모터 전달."""
        if self._relay_active and self._latest_imu:
            self._relay_to_motors(self._latest_imu)

    def _update_map_hint(self) -> None:
        mode = self.relay_mode_combo.currentData() if hasattr(self, "relay_mode_combo") else "rpy"
        if mode == "quat":
            self.map_status.setText("상대 Quaternion(시작 시점 기준) → dRoll/dPitch/dYaw")
        else:
            self.map_status.setText("상대 Roll→ID2, 상대 Pitch→ID1, 상대 Yaw→ID3")

    def _capture_relay_reference(self) -> None:
        if not self._latest_imu:
            return
        self._relay_ref_rpy = {
            "roll": self._latest_imu.get("roll", 0.0),
            "pitch": self._latest_imu.get("pitch", 0.0),
            "yaw": self._latest_imu.get("yaw", 0.0),
        }
        self._relay_ref_quat = self._get_quaternion_from_data(self._latest_imu)
        self._prev_quat = self._relay_ref_quat  # 쿼터니언 연속성 초기화
        self._filtered_rpy = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}  # 필터 초기화
        self._last_relay_goal.clear()
        if self._relay_active:
            self._log_imu("전달 기준 자세를 현재 값으로 재설정했습니다.")

    def _capture_relay_motor_base(self) -> None:
        """전달 시작 시 현재 모터 위치를 기준점으로 저장한다."""
        if not self.motor_connected or self.port_handler is None:
            return

        for motor_id in MOTOR_IDS:
            base_pos = self.motor_widgets[motor_id].spin.value()
            self.port_handler.clearPort()
            present, err = self._safe_read_present_position(motor_id)
            if present is not None:
                base_pos = present
            elif err:
                self._log_imu(f"기준 위치 읽기 실패(ID {motor_id}): {err}")
            self._relay_base_pos[motor_id] = base_pos

    def _get_quaternion_from_data(self, data: dict) -> Tuple[float, float, float, float]:
        q = (
            data.get("q0", 1.0),
            data.get("q1", 0.0),
            data.get("q2", 0.0),
            data.get("q3", 0.0),
        )
        q_norm = math.sqrt(sum(component * component for component in q))
        if q_norm > 0.5:
            return quat_normalize(q)
        return euler_deg_to_quat(
            data.get("roll", 0.0),
            data.get("pitch", 0.0),
            data.get("yaw", 0.0),
        )

    def _relay_to_motors(self, data: dict) -> None:
        mode = self.relay_mode_combo.currentData()
        if mode == "quat":
            q_cur = self._get_quaternion_from_data(data)
            # 쿼터니언 연속성 보장 (q와 -q의 부호 일치)
            q_cur = ensure_quat_continuity(q_cur, self._prev_quat)
            self._prev_quat = q_cur
            
            q_rel = quat_mul(quat_conjugate(self._relay_ref_quat), q_cur)
            d_roll, d_pitch, d_yaw = quat_to_euler_deg(q_rel)
            raw_mapping = {
                1: clamp_angle_deg(d_pitch),
                2: clamp_angle_deg(d_roll),
                3: clamp_angle_deg(d_yaw),
            }
        else:
            d_roll = clamp_angle_deg(data.get("roll", 0.0) - self._relay_ref_rpy["roll"])
            d_pitch = clamp_angle_deg(data.get("pitch", 0.0) - self._relay_ref_rpy["pitch"])
            d_yaw = clamp_angle_deg(data.get("yaw", 0.0) - self._relay_ref_rpy["yaw"])
            raw_mapping = {1: d_pitch, 2: d_roll, 3: d_yaw}
        
        # 저역통과필터: 노이즈 제거 및 부드러운 모터 움직임
        self._filtered_rpy["roll"] = (
            self._lpf_alpha * raw_mapping[2] + 
            (1.0 - self._lpf_alpha) * self._filtered_rpy["roll"]
        )
        self._filtered_rpy["pitch"] = (
            self._lpf_alpha * raw_mapping[1] + 
            (1.0 - self._lpf_alpha) * self._filtered_rpy["pitch"]
        )
        self._filtered_rpy["yaw"] = (
            self._lpf_alpha * raw_mapping[3] + 
            (1.0 - self._lpf_alpha) * self._filtered_rpy["yaw"]
        )

        mapping = {
            1: self._filtered_rpy["pitch"],
            2: self._filtered_rpy["roll"],
            3: self._filtered_rpy["yaw"],
        }
        
        mapping = {
            motor_id: clamp_angle_deg(angle * MOTOR_DIRECTION[motor_id])
            for motor_id, angle in mapping.items()
        }

        parts   = []
        for motor_id, angle in mapping.items():
            angle_clamped = clamp_angle_deg(angle)
            if abs(angle_clamped) < RELAY_DEADBAND_DEG.get(motor_id, 2.0):
                angle_clamped = 0.0
            
            delta_pos = int(round(angle_clamped / 360.0 * 4095))
            delta_pos = max(-2048, min(2047, delta_pos))

            base_pos = self._relay_base_pos.get(motor_id, 2048)
            pos = max(0, min(4095, base_pos + delta_pos))
            last_goal = self._last_relay_goal.get(motor_id)
            w   = self.motor_widgets[motor_id]
            if w.torque_btn.isChecked():
                if last_goal is None or abs(pos - last_goal) >= RELAY_MIN_COMMAND_STEP:
                    comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, motor_id, ADDR_GOAL_POSITION, pos
                    )
                    if comm_result != COMM_SUCCESS or dxl_error != 0:
                        if comm_result != COMM_SUCCESS:
                            self._log_imu(
                                f"전달 쓰기 실패(ID {motor_id}): {self.packet_handler.getTxRxResult(comm_result)}"
                            )
                        else:
                            self._log_imu(
                                f"전달 쓰기 패킷 오류(ID {motor_id}): {self.packet_handler.getRxPacketError(dxl_error)}"
                            )
                        continue
                    self._last_relay_goal[motor_id] = pos
                    w.slider.blockSignals(True)
                    w.spin.blockSignals(True)
                    w.slider.setValue(pos)
                    w.spin.setValue(pos)
                    w.slider.blockSignals(False)
                    w.spin.blockSignals(False)
            parts.append(f"ID{motor_id}: d{angle_clamped:+.1f}°→{pos}")

        self._apply_hold_motors()

        self.map_status.setText(" | ".join(parts))

    def _prepare_hold_motors(self) -> None:
        """릴레이 시작 시 4~8번 모터를 현재 위치로 고정하도록 준비한다."""
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
            self._log_imu("고정 대상 모터(4~8) 미연결: 고정 기능을 건너뜁니다.")

    def _apply_hold_motors(self) -> None:
        """고정 대상 모터(4~8)에 동일 목표를 주기적으로 재전송해 처짐을 방지한다."""
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
                # 고정 대상 모터가 중간에 분리되어도 릴레이는 계속 진행한다.
                continue

        self._last_hold_write_time = now

    # ── 종료 정리 ──────────────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._imu_disconnect()
        self._motor_disconnect()
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

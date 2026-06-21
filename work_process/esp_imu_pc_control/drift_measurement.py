#!/usr/bin/env python3
"""IMU + 로봇팔 드리프트 측정 도구

IMU와 Dynamixel 모터 Present Position 을 동시에 기록하여
'IMU 센서 드리프트'와 '모터(로봇팔) 실제 위치 드리프트'를 함께 측정합니다.

- IMU: Roll/Pitch/Yaw 변화량
- 모터: Present Position(tick) → 각도(deg) 변화량 (ID 1/2/3)
- 기준 설정 버튼 클릭 → 그 시점을 0기준으로 저장
- 30초 간격 자동 기록 / 5분·10분·30분 체크포인트 강조
- 측정 종료 시 CSV 저장

측정 시나리오
  A) 릴레이 OFF + 모터 토크 ON  → 중력/기계 드리프트 확인
  B) 릴레이 ON  + IMU 고정       → IMU 드리프트가 모터에 얼마나 전달되는지 확인
"""

import csv
import json
import math
import os
import socket
import sys
import time
from datetime import datetime
from typing import Dict, Optional, Tuple

from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtGui import QColor, QFont

try:
    from stylesheet import apply_stylesheet
    _HAS_STYLESHEET = True
except ImportError:
    _HAS_STYLESHEET = False

try:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    _HAS_DYNAMIXEL = True
except ImportError:
    _HAS_DYNAMIXEL = False

from serial.tools import list_ports

# ── Dynamixel 제어 테이블 ──────────────────────────────────────────────────────
PROTOCOL_VERSION      = 2.0
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0
OPERATING_MODE_POSITION = 3
MOTOR_IDS             = (1, 2, 3)
MOTOR_DIRECTION       = {1: 1.0, 2: -1.0, 3: -1.0}
RELAY_DEADBAND_DEG    = {1: 2.0, 2: 2.0, 3: 3.0}
RELAY_MIN_COMMAND_STEP = 10
RELAY_INTERVAL_MS     = 50
HOLD_MOTOR_IDS        = (4, 5, 6, 7, 8)
ALL_MOTOR_IDS         = MOTOR_IDS + HOLD_MOTOR_IDS

# tick → degree 변환 (0~4095 = 0~360°)
def tick_to_deg(tick: int) -> float:
    return tick / 4095.0 * 360.0


def angle_to_pos(angle: float) -> int:
    return int(max(0, min(4095, (angle + 180.0) / 360.0 * 4095)))


# ── 체크포인트 (초 단위) ─────────────────────────────────────────────────────────
CHECKPOINTS_SEC = {300: "5분", 600: "10분", 1800: "30분"}
LOG_INTERVAL_SEC = 30          # 기록 간격 (초)
HIGHLIGHT_COLOR = QColor("#FFF3CD")   # 체크포인트 강조 배경
LARGE_DRIFT_THRESHOLD_DEG = 2.0       # 이 이상이면 빨간색 표시


# ── UDP IMU 수신 스레드 ─────────────────────────────────────────────────────────
class UdpImuThread(QThread):
    parsed  = pyqtSignal(dict)
    status  = pyqtSignal(str)
    error   = pyqtSignal(str)

    def __init__(self, host: str, port: int) -> None:
        super().__init__()
        self._host = host
        self._port = port
        self._running = True
        self._sock: Optional[socket.socket] = None

    def run(self) -> None:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(0.2)
            self._sock.bind((self._host, self._port))
            self.status.emit(f"수신 중: UDP {self._host}:{self._port}")
        except OSError as exc:
            self.error.emit(f"UDP 바인드 실패: {exc}")
            return

        while self._running:
            try:
                raw, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            data = self._parse(raw)
            if data is not None:
                self.parsed.emit(data)

        if self._sock:
            self._sock.close()

    def stop(self) -> None:
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    @staticmethod
    def _parse(raw: bytes) -> Optional[Dict[str, float]]:
        try:
            pkt = json.loads(raw.decode("utf-8", errors="replace").strip())
        except json.JSONDecodeError:
            return None
        if not isinstance(pkt, dict) or pkt.get("type") == "status":
            return None
        try:
            return {
                "roll":  float(pkt["roll"]),
                "pitch": float(pkt["pitch"]),
                "yaw":   float(pkt["yaw"]),
            }
        except (KeyError, TypeError, ValueError):
            return None


# ── 메인 윈도우 ────────────────────────────────────────────────────────────────
class DriftWindow(QMainWindow):
    # 컬럼 정의
    COL_IDX     = 0
    COL_TIME    = 1
    COL_ELAPSED = 2
    COL_ROLL    = 3
    COL_PITCH   = 4
    COL_YAW     = 5
    COL_M1      = 6   # Motor ID1 위치 드리프트(deg)
    COL_M2      = 7
    COL_M3      = 8
    COL_NOTE    = 9
    NCOLS       = 10

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IMU + 로봇팔 드리프트 측정기")
        self.resize(1200, 680)

        # IMU
        self._thread: Optional[UdpImuThread] = None
        self._latest: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._baseline: Optional[Dict[str, float]] = None
        self._start_time: Optional[float] = None

        # Motor
        self._port_handler: Optional["PortHandler"] = None
        self._packet_handler = PacketHandler(PROTOCOL_VERSION) if _HAS_DYNAMIXEL else None
        self._motor_connected: bool = False
        self._motor_baseline: Dict[int, float] = {}   # motor_id → baseline deg
        self._relay_active: bool = False
        self._relay_ref_rpy: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._relay_base_pos: Dict[int, int] = {mid: 2048 for mid in MOTOR_IDS}
        self._hold_base_pos: Dict[int, int] = {mid: 2048 for mid in HOLD_MOTOR_IDS}
        self._last_relay_goal: Dict[int, int] = {}

        self._log_timer = QTimer(self)
        self._log_timer.timeout.connect(self._log_tick)
        self._log_rows: list = []   # 저장용 데이터
        self._relay_timer = QTimer(self)
        self._relay_timer.setInterval(RELAY_INTERVAL_MS)
        self._relay_timer.timeout.connect(self._relay_tick)

        self._build_ui()

    # ── UI 구성 ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setSpacing(8)
        vbox.setContentsMargins(10, 10, 10, 10)

        # IMU 연결 그룹
        conn_group = QGroupBox("IMU UDP 연결")
        conn_lay = QHBoxLayout(conn_group)
        conn_lay.addWidget(QLabel("Host"))
        self._host_edit = QLineEdit("0.0.0.0")
        self._host_edit.setMaximumWidth(140)
        conn_lay.addWidget(self._host_edit)
        conn_lay.addWidget(QLabel("Port"))
        self._port_edit = QLineEdit("4210")
        self._port_edit.setMaximumWidth(80)
        conn_lay.addWidget(self._port_edit)
        self._connect_btn = QPushButton("연결")
        self._connect_btn.clicked.connect(self._on_connect)
        conn_lay.addWidget(self._connect_btn)
        self._disconnect_btn = QPushButton("해제")
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        self._disconnect_btn.setEnabled(False)
        conn_lay.addWidget(self._disconnect_btn)
        self._conn_label = QLabel("상태: 연결 안 됨")
        conn_lay.addWidget(self._conn_label)
        conn_lay.addStretch(1)

        # 모터 연결 그룹
        motor_conn_group = QGroupBox("Dynamixel 모터 연결 (ID 1/2/3 제어 + ID 4/5/6/7/8 고정)")
        motor_conn_lay = QHBoxLayout(motor_conn_group)
        motor_conn_lay.addWidget(QLabel("포트"))
        self._motor_port_combo = __import__('PyQt5.QtWidgets', fromlist=['QComboBox']).QComboBox()
        self._refresh_motor_ports()
        motor_conn_lay.addWidget(self._motor_port_combo)
        motor_conn_lay.addWidget(QLabel("Baud"))
        self._motor_baud_combo = __import__('PyQt5.QtWidgets', fromlist=['QComboBox']).QComboBox()
        self._motor_baud_combo.addItems(["57600", "115200", "1000000"])
        self._motor_baud_combo.setCurrentText("1000000")
        motor_conn_lay.addWidget(self._motor_baud_combo)
        self._motor_refresh_btn = QPushButton("포트 새로고침")
        self._motor_refresh_btn.clicked.connect(self._refresh_motor_ports)
        motor_conn_lay.addWidget(self._motor_refresh_btn)
        self._motor_connect_btn = QPushButton("모터 연결")
        self._motor_connect_btn.clicked.connect(self._on_motor_connect)
        if not _HAS_DYNAMIXEL:
            self._motor_connect_btn.setEnabled(False)
            self._motor_connect_btn.setToolTip("dynamixel_sdk 가 설치되지 않았습니다")
        motor_conn_lay.addWidget(self._motor_connect_btn)
        self._motor_disconnect_btn = QPushButton("모터 해제")
        self._motor_disconnect_btn.clicked.connect(self._on_motor_disconnect)
        self._motor_disconnect_btn.setEnabled(False)
        motor_conn_lay.addWidget(self._motor_disconnect_btn)
        self._motor_status_label = QLabel("모터: 연결 안 됨")
        motor_conn_lay.addWidget(self._motor_status_label)
        motor_conn_lay.addStretch(1)

        # 실시간 값 그룹
        live_group = QGroupBox("실시간 값")
        live_lay = QHBoxLayout(live_group)
        self._live_labels: Dict[str, QLabel] = {}
        live_lay.addWidget(QLabel("[IMU]"))
        for axis in ("roll", "pitch", "yaw"):
            live_lay.addWidget(QLabel(f"{axis.capitalize()}:"))
            lbl = QLabel("0.000°")
            lbl.setFont(QFont("Monospace", 10, QFont.Bold))
            lbl.setMinimumWidth(75)
            self._live_labels[axis] = lbl
            live_lay.addWidget(lbl)
            live_lay.addSpacing(10)
        live_lay.addSpacing(20)
        live_lay.addWidget(QLabel("[모터 위치]"))
        self._motor_live_labels: Dict[int, QLabel] = {}
        for mid in MOTOR_IDS:
            live_lay.addWidget(QLabel(f"ID{mid}:"))
            mlbl = QLabel("-")
            mlbl.setFont(QFont("Monospace", 10, QFont.Bold))
            mlbl.setMinimumWidth(70)
            self._motor_live_labels[mid] = mlbl
            live_lay.addWidget(mlbl)
            live_lay.addSpacing(10)
        live_lay.addStretch(1)

        # 제어 그룹
        ctrl_group = QGroupBox("측정 제어")
        ctrl_lay = QHBoxLayout(ctrl_group)
        self._baseline_btn = QPushButton("기준 설정 (현재값을 기준으로)")
        self._baseline_btn.setEnabled(False)
        self._baseline_btn.clicked.connect(self._set_baseline)
        ctrl_lay.addWidget(self._baseline_btn)
        self._stop_btn = QPushButton("측정 종료 & CSV 저장")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop_and_save)
        ctrl_lay.addWidget(self._stop_btn)
        self._torque_on_btn = QPushButton("토크 ON")
        self._torque_on_btn.setEnabled(False)
        self._torque_on_btn.clicked.connect(lambda: self._set_all_torque(True))
        ctrl_lay.addWidget(self._torque_on_btn)
        self._torque_off_btn = QPushButton("토크 OFF")
        self._torque_off_btn.setEnabled(False)
        self._torque_off_btn.clicked.connect(lambda: self._set_all_torque(False))
        ctrl_lay.addWidget(self._torque_off_btn)
        self._capture_pose_btn = QPushButton("전달 기준 캡처")
        self._capture_pose_btn.setEnabled(False)
        self._capture_pose_btn.clicked.connect(self._capture_relay_reference)
        ctrl_lay.addWidget(self._capture_pose_btn)
        self._relay_btn = QPushButton("전달 시작")
        self._relay_btn.setEnabled(False)
        self._relay_btn.clicked.connect(self._toggle_relay)
        ctrl_lay.addWidget(self._relay_btn)
        ctrl_lay.addStretch(1)
        self._baseline_label = QLabel("기준값: 없음")
        ctrl_lay.addWidget(self._baseline_label)

        # 드리프트 테이블
        table_group = QGroupBox(
            f"드리프트 로그 (매 {LOG_INTERVAL_SEC}초 기록 / "
            f"5분·10분·30분 강조 / ±{LARGE_DRIFT_THRESHOLD_DEG}° 이상 빨간색)"
        )
        table_lay = QVBoxLayout(table_group)
        self._table = QTableWidget(0, self.NCOLS)
        self._table.setHorizontalHeaderLabels(
            ["#", "기록 시각", "경과(초)",
             "IMU Roll(°)", "IMU Pitch(°)", "IMU Yaw(°)",
             "모터ID1(°)", "모터ID2(°)", "모터ID3(°)",
             "비고"]
        )
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setAlternatingRowColors(True)
        col_widths = [30, 85, 75, 100, 100, 100, 90, 90, 90, 120]
        for i, w in enumerate(col_widths):
            self._table.setColumnWidth(i, w)
        table_lay.addWidget(self._table)

        vbox.addWidget(conn_group)
        vbox.addWidget(motor_conn_group)
        vbox.addWidget(live_group)
        vbox.addWidget(ctrl_group)
        vbox.addWidget(table_group)

    # ── 모터 포트 새로고침 ────────────────────────────────────────────────────

    def _refresh_motor_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        self._motor_port_combo.clear()
        for p in ports:
            self._motor_port_combo.addItem(f"{p.device} ({p.description})", p.device)
        if not ports:
            self._motor_port_combo.addItem("포트 없음", "")

    # ── 모터 연결 ─────────────────────────────────────────────────────────────

    def _on_motor_connect(self) -> None:
        if not _HAS_DYNAMIXEL:
            QMessageBox.warning(self, "미설치", "dynamixel_sdk 를 먼저 설치해주세요.")
            return
        port = self._motor_port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "포트 오류", "포트를 선택해주세요.")
            return
        try:
            baud = int(self._motor_baud_combo.currentText())
        except ValueError:
            return

        ph = PortHandler(port)
        if not ph.openPort() or not ph.setBaudRate(baud):
            QMessageBox.critical(self, "연결 실패", f"포트 {port} 열기 실패")
            return

        self._port_handler = ph
        self._motor_connected = True
        self._motor_connect_btn.setEnabled(False)
        self._motor_disconnect_btn.setEnabled(True)
        self._torque_on_btn.setEnabled(True)
        self._torque_off_btn.setEnabled(True)
        self._capture_pose_btn.setEnabled(True)
        self._relay_btn.setEnabled(True)
        self._motor_status_label.setText(f"모터: {port} ({baud}) 연결됨")
        # 기존 기준값 초기화
        self._motor_baseline.clear()
        self._last_relay_goal.clear()

    def _on_motor_disconnect(self) -> None:
        if self._relay_active:
            self._relay_active = False
            self._relay_timer.stop()
            self._relay_btn.setText("전달 시작")
        if self._port_handler:
            self._port_handler.closePort()
        self._port_handler = None
        self._motor_connected = False
        self._motor_connect_btn.setEnabled(True)
        self._motor_disconnect_btn.setEnabled(False)
        self._torque_on_btn.setEnabled(False)
        self._torque_off_btn.setEnabled(False)
        self._capture_pose_btn.setEnabled(False)
        self._relay_btn.setEnabled(False)
        self._motor_status_label.setText("모터: 연결 안 됨")
        self._motor_baseline.clear()
        self._last_relay_goal.clear()
        for lbl in self._motor_live_labels.values():
            lbl.setText("-")

    def _ensure_position_mode(self, motor_id: int) -> Tuple[bool, str]:
        if self._port_handler is None or self._packet_handler is None:
            return False, "모터 미연결"

        mode, comm, err = self._packet_handler.read1ByteTxRx(
            self._port_handler, motor_id, ADDR_OPERATING_MODE
        )
        if comm != COMM_SUCCESS:
            return False, self._packet_handler.getTxRxResult(comm)
        if err != 0:
            return False, self._packet_handler.getRxPacketError(err)
        if mode == OPERATING_MODE_POSITION:
            return True, ""

        comm, err = self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if comm != COMM_SUCCESS:
            return False, self._packet_handler.getTxRxResult(comm)
        if err != 0:
            return False, self._packet_handler.getRxPacketError(err)
        comm, err = self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION
        )
        if comm != COMM_SUCCESS:
            return False, self._packet_handler.getTxRxResult(comm)
        if err != 0:
            return False, self._packet_handler.getRxPacketError(err)
        return True, ""

    def _set_all_torque(self, enabled: bool) -> None:
        if not self._motor_connected or self._port_handler is None or self._packet_handler is None:
            QMessageBox.warning(self, "모터 미연결", "먼저 모터를 연결해주세요.")
            return

        failed = []
        for motor_id in ALL_MOTOR_IDS:
            ok, msg = self._ensure_position_mode(motor_id)
            if not ok:
                failed.append(f"ID {motor_id}: {msg}")
                continue
            comm, err = self._packet_handler.write1ByteTxRx(
                self._port_handler,
                motor_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE if enabled else TORQUE_DISABLE,
            )
            if comm != COMM_SUCCESS:
                failed.append(f"ID {motor_id}: {self._packet_handler.getTxRxResult(comm)}")
            elif err != 0:
                failed.append(f"ID {motor_id}: {self._packet_handler.getRxPacketError(err)}")

        if failed:
            QMessageBox.warning(self, "토크 설정 일부 실패", "\n".join(failed))
        else:
            self._motor_status_label.setText(f"모터: 토크 {'ON' if enabled else 'OFF'}")

    def _read_motor_positions_deg(self) -> Dict[int, Optional[float]]:
        """ID 1/2/3 Present Position 을 읽어 deg 로 반환. 실패 시 None."""
        result: Dict[int, Optional[float]] = {}
        if not self._motor_connected or self._port_handler is None or self._packet_handler is None:
            return {mid: None for mid in MOTOR_IDS}
        for mid in MOTOR_IDS:
            pos = self._read_motor_positions_tick().get(mid)
            result[mid] = tick_to_deg(pos) if pos is not None else None
        return result

    def _read_motor_positions_tick(self) -> Dict[int, Optional[int]]:
        return self._read_motor_positions_tick_for(MOTOR_IDS)

    def _read_motor_positions_tick_for(self, motor_ids) -> Dict[int, Optional[int]]:
        result: Dict[int, Optional[int]] = {}
        if not self._motor_connected or self._port_handler is None or self._packet_handler is None:
            return {mid: None for mid in motor_ids}
        for mid in motor_ids:
            pos, comm, err = self._packet_handler.read4ByteTxRx(
                self._port_handler, mid, ADDR_PRESENT_POSITION
            )
            if comm == COMM_SUCCESS and err == 0:
                result[mid] = pos & 0xFFFF
            else:
                result[mid] = None
        return result

    # ── IMU 연결 ─────────────────────────────────────────────────────────────────

    def _on_connect(self) -> None:
        host = self._host_edit.text().strip() or "0.0.0.0"
        try:
            port = int(self._port_edit.text().strip())
            if not (1 <= port <= 65535):
                raise ValueError
        except ValueError:
            QMessageBox.warning(self, "포트 오류", "1~65535 범위의 포트를 입력해주세요.")
            return

        self._thread = UdpImuThread(host, port)
        self._thread.parsed.connect(self._on_parsed)
        self._thread.status.connect(lambda s: self._conn_label.setText(f"상태: {s}"))
        self._thread.error.connect(self._on_error)
        self._thread.start()

        self._connect_btn.setEnabled(False)
        self._disconnect_btn.setEnabled(True)
        self._baseline_btn.setEnabled(True)
        self._conn_label.setText("상태: 연결 중…")

    def _on_disconnect(self) -> None:
        self._log_timer.stop()
        if self._relay_active:
            self._relay_active = False
            self._relay_timer.stop()
            self._relay_btn.setText("전달 시작")
        if self._thread:
            self._thread.stop()
            self._thread.wait(2000)
            self._thread = None
        self._connect_btn.setEnabled(True)
        self._disconnect_btn.setEnabled(False)
        self._baseline_btn.setEnabled(False)
        self._stop_btn.setEnabled(False)
        self._conn_label.setText("상태: 연결 안 됨")

    def _on_error(self, msg: str) -> None:
        self._conn_label.setText(f"상태: 오류")
        QMessageBox.critical(self, "연결 오류", msg)
        self._on_disconnect()

    # ── 데이터 수신 ───────────────────────────────────────────────────────────

    def _on_parsed(self, data: dict) -> None:
        self._latest = data
        for axis in ("roll", "pitch", "yaw"):
            self._live_labels[axis].setText(f"{data[axis]:.3f}°")
        if self._relay_active:
            self._relay_to_motors(data)
        # 모터 실시간 값도 함께 갱신
        if self._motor_connected:
            pos_map = self._read_motor_positions_deg()
            for mid, deg in pos_map.items():
                self._motor_live_labels[mid].setText(
                    f"{deg:.2f}°" if deg is not None else "err"
                )

    def _capture_relay_reference(self) -> None:
        if not self._latest:
            QMessageBox.warning(self, "IMU 미수신", "먼저 IMU 데이터를 수신해주세요.")
            return
        self._relay_ref_rpy = {
            "roll": self._latest.get("roll", 0.0),
            "pitch": self._latest.get("pitch", 0.0),
            "yaw": self._latest.get("yaw", 0.0),
        }
        current_positions = self._read_motor_positions_tick_for(ALL_MOTOR_IDS)
        for mid in MOTOR_IDS:
            current = current_positions.get(mid)
            if current is not None:
                self._relay_base_pos[mid] = current
        for mid in HOLD_MOTOR_IDS:
            current = current_positions.get(mid)
            if current is not None:
                self._hold_base_pos[mid] = current
        self._last_relay_goal.clear()
        self._motor_status_label.setText("모터: 전달 기준 캡처 완료 (ID4~8 고정 기준 포함)")

    def _toggle_relay(self) -> None:
        if self._relay_active:
            self._relay_active = False
            self._relay_timer.stop()
            self._relay_btn.setText("전달 시작")
            self._motor_status_label.setText("모터: 전달 중지")
            return

        if not self._latest:
            QMessageBox.warning(self, "IMU 미수신", "먼저 IMU를 연결해주세요.")
            return
        if not self._motor_connected:
            QMessageBox.warning(self, "모터 미연결", "먼저 모터를 연결해주세요.")
            return

        self._set_all_torque(True)
        self._capture_relay_reference()
        self._relay_active = True
        self._relay_timer.start()
        self._relay_btn.setText("전달 중지")
        self._motor_status_label.setText("모터: 전달 중")

    def _relay_tick(self) -> None:
        if self._relay_active and self._latest:
            self._relay_to_motors(self._latest)

    def _relay_to_motors(self, data: Dict[str, float]) -> None:
        if self._port_handler is None or self._packet_handler is None:
            return

        relative = {
            1: self._wrap_180(data.get("pitch", 0.0) - self._relay_ref_rpy["pitch"]),
            2: self._wrap_180(data.get("roll", 0.0) - self._relay_ref_rpy["roll"]),
            3: self._wrap_180(data.get("yaw", 0.0) - self._relay_ref_rpy["yaw"]),
        }
        parts = []
        for motor_id, angle in relative.items():
            angle = self._wrap_180(angle * MOTOR_DIRECTION[motor_id])
            if abs(angle) < RELAY_DEADBAND_DEG.get(motor_id, 2.0):
                angle = 0.0

            delta_pos = int(round(angle / 360.0 * 4095))
            delta_pos = max(-2048, min(2047, delta_pos))
            goal = max(0, min(4095, self._relay_base_pos.get(motor_id, 2048) + delta_pos))
            last_goal = self._last_relay_goal.get(motor_id)
            if last_goal is not None and abs(goal - last_goal) < RELAY_MIN_COMMAND_STEP:
                parts.append(f"ID{motor_id}:{angle:+.1f}°")
                continue

            comm, err = self._packet_handler.write4ByteTxRx(
                self._port_handler, motor_id, ADDR_GOAL_POSITION, goal
            )
            if comm == COMM_SUCCESS and err == 0:
                self._last_relay_goal[motor_id] = goal
                parts.append(f"ID{motor_id}:{angle:+.1f}°")

        # 나머지 관절(ID4~8)은 캡처한 위치로 유지해 처짐을 방지
        for motor_id in HOLD_MOTOR_IDS:
            hold_goal = self._hold_base_pos.get(motor_id)
            if hold_goal is None:
                continue
            last_goal = self._last_relay_goal.get(motor_id)
            if last_goal is not None and abs(hold_goal - last_goal) < RELAY_MIN_COMMAND_STEP:
                continue

            comm, err = self._packet_handler.write4ByteTxRx(
                self._port_handler, motor_id, ADDR_GOAL_POSITION, hold_goal
            )
            if comm == COMM_SUCCESS and err == 0:
                self._last_relay_goal[motor_id] = hold_goal

        if parts:
            self._motor_status_label.setText("모터: 전달 중(ID1~3) + 고정(ID4~8) | " + " ".join(parts))

    # ── 기준 설정 ─────────────────────────────────────────────────────────────

    def _set_baseline(self) -> None:
        self._baseline = dict(self._latest)
        self._start_time = time.time()
        self._log_rows.clear()
        self._table.setRowCount(0)

        # 모터 기준값 설정
        self._motor_baseline.clear()
        motor_pos = self._read_motor_positions_deg()
        for mid, deg in motor_pos.items():
            if deg is not None:
                self._motor_baseline[mid] = deg

        r, p, y = self._baseline["roll"], self._baseline["pitch"], self._baseline["yaw"]
        motor_info = "  ".join(
            f"M{mid}={self._motor_baseline[mid]:.2f}°"
            for mid in MOTOR_IDS if mid in self._motor_baseline
        ) or "(모터 미연결)"
        self._baseline_label.setText(
            f"기준: R={r:.3f}° P={p:.3f}° Y={y:.3f}°  |  {motor_info}"
        )

        # 기준 설정 시점을 row 0 으로 기록
        self._add_table_row(
            idx=0,
            elapsed=0.0,
            roll_drift=0.0,
            pitch_drift=0.0,
            yaw_drift=0.0,
            motor_drifts={mid: 0.0 for mid in MOTOR_IDS},
            note="기준 설정",
        )

        self._log_timer.start(LOG_INTERVAL_SEC * 1000)
        self._stop_btn.setEnabled(True)
        self._baseline_btn.setText("기준 재설정")

    # ── 주기 로그 ─────────────────────────────────────────────────────────────

    def _log_tick(self) -> None:
        if self._baseline is None or self._start_time is None:
            return

        elapsed = time.time() - self._start_time
        rd = self._wrap_180(self._latest["roll"]  - self._baseline["roll"])
        pd = self._wrap_180(self._latest["pitch"] - self._baseline["pitch"])
        yd = self._wrap_180(self._latest["yaw"]   - self._baseline["yaw"])

        # 모터 드리프트 계산
        motor_drifts: Dict[int, Optional[float]] = {}
        if self._motor_connected and self._motor_baseline:
            cur_pos = self._read_motor_positions_deg()
            for mid in MOTOR_IDS:
                cur = cur_pos.get(mid)
                base = self._motor_baseline.get(mid)
                if cur is not None and base is not None:
                    motor_drifts[mid] = self._wrap_180(cur - base)
                else:
                    motor_drifts[mid] = None
        else:
            motor_drifts = {mid: None for mid in MOTOR_IDS}

        # 체크포인트 비고 결정
        note = ""
        for cp_sec, cp_label in CHECKPOINTS_SEC.items():
            if abs(elapsed - cp_sec) <= LOG_INTERVAL_SEC / 2:
                note = f"★ {cp_label} 체크포인트"
                break

        idx = self._table.rowCount()
        self._add_table_row(
            idx=idx,
            elapsed=elapsed,
            roll_drift=rd,
            pitch_drift=pd,
            yaw_drift=yd,
            motor_drifts=motor_drifts,
            note=note,
        )

    # ── 테이블 행 추가 ────────────────────────────────────────────────────────

    def _add_table_row(
        self,
        idx: int,
        elapsed: float,
        roll_drift: float,
        pitch_drift: float,
        yaw_drift: float,
        motor_drifts: Optional[Dict[int, Optional[float]]] = None,
        note: str = "",
    ) -> None:
        if motor_drifts is None:
            motor_drifts = {mid: None for mid in MOTOR_IDS}

        is_checkpoint = note.startswith("★")
        now_str = datetime.now().strftime("%H:%M:%S")

        def fmt_motor(v: Optional[float]) -> str:
            return f"{v:+.3f}" if v is not None else "N/A"

        row = self._table.rowCount()
        self._table.insertRow(row)

        cells = [
            str(idx),
            now_str,
            f"{elapsed:.0f}",
            f"{roll_drift:+.3f}",
            f"{pitch_drift:+.3f}",
            f"{yaw_drift:+.3f}",
            fmt_motor(motor_drifts.get(1)),
            fmt_motor(motor_drifts.get(2)),
            fmt_motor(motor_drifts.get(3)),
            note,
        ]

        drift_cols = (self.COL_ROLL, self.COL_PITCH, self.COL_YAW,
                      self.COL_M1, self.COL_M2, self.COL_M3)

        for col, text in enumerate(cells):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)

            if is_checkpoint:
                item.setBackground(HIGHLIGHT_COLOR)
                font = item.font()
                font.setBold(True)
                item.setFont(font)

            if col in drift_cols and text not in ("N/A", "+0.000", "0.000"):
                try:
                    if abs(float(text)) >= LARGE_DRIFT_THRESHOLD_DEG:
                        item.setForeground(QColor("#D32F2F"))
                except ValueError:
                    pass

            self._table.setItem(row, col, item)

        self._table.scrollToBottom()

        # CSV 저장용 데이터 누적
        self._log_rows.append({
            "idx": idx,
            "time": now_str,
            "elapsed_sec": f"{elapsed:.1f}",
            "roll_drift": f"{roll_drift:.4f}",
            "pitch_drift": f"{pitch_drift:.4f}",
            "yaw_drift": f"{yaw_drift:.4f}",
            "motor_id1_drift": fmt_motor(motor_drifts.get(1)),
            "motor_id2_drift": fmt_motor(motor_drifts.get(2)),
            "motor_id3_drift": fmt_motor(motor_drifts.get(3)),
            "note": note,
        })

    # ── 종료 & 저장 ───────────────────────────────────────────────────────────

    def _stop_and_save(self) -> None:
        self._log_timer.stop()
        self._stop_btn.setEnabled(False)

        if not self._log_rows:
            QMessageBox.information(self, "알림", "기록된 데이터가 없습니다.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(os.path.dirname(__file__), f"drift_{ts}.csv")

        try:
            with open(fname, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["idx", "time", "elapsed_sec",
                                "roll_drift", "pitch_drift", "yaw_drift",
                                "motor_id1_drift", "motor_id2_drift", "motor_id3_drift",
                                "note"],
                )
                writer.writeheader()
                writer.writerows(self._log_rows)
        except OSError as exc:
            QMessageBox.critical(self, "저장 실패", str(exc))
            return

        QMessageBox.information(
            self,
            "저장 완료",
            f"드리프트 데이터가 저장되었습니다:\n{fname}",
        )

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    @staticmethod
    def _wrap_180(angle: float) -> float:
        """각도 차이를 [-180, +180] 범위로 정규화."""
        return (angle + 180.0) % 360.0 - 180.0

    def closeEvent(self, event) -> None:
        self._log_timer.stop()
        self._relay_timer.stop()
        if self._thread:
            self._thread.stop()
            self._thread.wait(2000)
        if self._port_handler:
            self._port_handler.closePort()
        event.accept()


# ── 진입점 ────────────────────────────────────────────────────────────────────
def main() -> None:
    app = QApplication(sys.argv)
    if _HAS_STYLESHEET:
        apply_stylesheet(app)
    win = DriftWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

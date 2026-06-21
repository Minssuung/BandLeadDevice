#!/usr/bin/env python3
"""PyQt5 UI for independent control of DYNAMIXEL IDs 1, 2, 3.

- Top area: connection check (port open + baud set + ping IDs)
- Bottom area: per-motor control panel (torque, goal position, present position)
"""

import sys
from dataclasses import dataclass
from typing import Dict, Optional

from serial.tools import list_ports
from PyQt5.QtCore import Qt, QSize
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
    QSpinBox,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
    QSlider,
)

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from stylesheet import apply_stylesheet


# XL330 / X series (Protocol 2.0) control table addresses
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MOTOR_IDS = (1, 2, 3)
DEFAULT_BAUD = 57600


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

        # Track
        painter.setPen(QPen(track_border, 1))
        painter.setBrush(track_fill)
        painter.drawRoundedRect(rect, radius, radius)

        # Knob
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


class MotorControlWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("DYNAMIXEL Motor Control (ID 1,2,3)")
        self.resize(960, 640)

        self.port_handler: Optional[PortHandler] = None
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.connected = False
        self.motor_widgets: Dict[int, MotorWidgets] = {}

        self._build_ui()
        self._refresh_ports()
        self._set_control_enabled(False)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)

        conn_box = QGroupBox("1) 연결 확인")
        conn_layout = QVBoxLayout(conn_box)

        row = QHBoxLayout()
        row.addWidget(QLabel("포트"))

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(260)
        row.addWidget(self.port_combo)

        row.addWidget(QLabel("Baud"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "1000000"])
        self.baud_combo.setCurrentText(str(DEFAULT_BAUD))
        row.addWidget(self.baud_combo)

        self.refresh_btn = QPushButton("포트 새로고침")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        row.addWidget(self.refresh_btn)

        self.connect_check_btn = QPushButton("연결 확인")
        self.connect_check_btn.setObjectName("important")
        self.connect_check_btn.clicked.connect(self._connect_and_check)
        row.addWidget(self.connect_check_btn)

        self.disconnect_btn = QPushButton("연결 해제")
        self.disconnect_btn.setObjectName("important")
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self._disconnect)
        row.addWidget(self.disconnect_btn)

        row.addStretch(1)
        conn_layout.addLayout(row)

        self.connection_status = QLabel("상태: Disconnected")
        self.connection_status.setObjectName("emphasized")
        conn_layout.addWidget(self.connection_status)

        self.id_status = QLabel("ID 1/2/3 연결 확인 대기")
        self.id_status.setObjectName("secondary")
        conn_layout.addWidget(self.id_status)

        main_layout.addWidget(conn_box)

        control_box = QGroupBox("2) 모터 제어 영역 (ID 1, 2, 3)")
        control_layout = QVBoxLayout(control_box)
        control_layout.setSpacing(8)

        for motor_id in MOTOR_IDS:
            group = self._create_motor_group(motor_id)
            control_layout.addWidget(group)

        main_layout.addWidget(control_box)
        main_layout.addStretch(1)

    def _create_motor_group(self, motor_id: int) -> QGroupBox:
        box = QGroupBox(f"Motor ID {motor_id}")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        torque_btn = TorqueToggleButton()
        torque_btn.clicked.connect(lambda _, mid=motor_id: self._toggle_torque(mid))

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

        layout.addWidget(torque_btn, 0, 0)
        layout.addWidget(present_label, 0, 1, 1, 3)

        layout.addWidget(QLabel("Goal Position"), 1, 0)
        layout.addWidget(slider, 1, 1, 1, 2)
        layout.addWidget(spin, 1, 3)

        layout.addWidget(send_btn, 2, 2)
        layout.addWidget(read_btn, 2, 3)

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
        self.port_combo.clear()
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        for p in ports:
            label = f"{p.device} ({p.description})"
            self.port_combo.addItem(label, p.device)

        if self.port_combo.count() == 0:
            self.port_combo.addItem("사용 가능한 시리얼 포트 없음", "")

    def _set_control_enabled(self, enabled: bool) -> None:
        for widgets in self.motor_widgets.values():
            widgets.torque_btn.setEnabled(enabled)
            widgets.slider.setEnabled(enabled)
            widgets.spin.setEnabled(enabled)
            widgets.send_btn.setEnabled(enabled)
            widgets.read_btn.setEnabled(enabled)

    def _set_goal_controls_enabled(self, motor_id: int, enabled: bool) -> None:
        widgets = self.motor_widgets[motor_id]
        widgets.slider.setEnabled(enabled)
        widgets.spin.setEnabled(enabled)
        widgets.send_btn.setEnabled(enabled)

    def _require_connected(self) -> bool:
        if not self.connected or self.port_handler is None:
            QMessageBox.warning(self, "미연결", "먼저 연결 확인을 진행해주세요.")
            return False
        return True

    def _connect_and_check(self) -> None:
        if self.connected:
            QMessageBox.information(self, "안내", "이미 연결되어 있습니다.")
            return

        port_name = self.port_combo.currentData()
        if not port_name:
            QMessageBox.warning(self, "포트 오류", "사용 가능한 포트를 선택해주세요.")
            return

        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, "Baud 오류", "유효한 Baud 값을 선택해주세요.")
            return

        self.port_handler = PortHandler(port_name)
        if not self.port_handler.openPort():
            QMessageBox.critical(self, "연결 실패", f"포트를 열 수 없습니다: {port_name}")
            self.port_handler = None
            return

        if not self.port_handler.setBaudRate(baud):
            self.port_handler.closePort()
            self.port_handler = None
            QMessageBox.critical(self, "연결 실패", f"Baud 설정 실패: {baud}")
            return

        alive_ids = []
        dead_ids = []

        for motor_id in MOTOR_IDS:
            model_no, comm_result, dxl_error = self.packet_handler.ping(self.port_handler, motor_id)
            if comm_result == COMM_SUCCESS and dxl_error == 0:
                alive_ids.append((motor_id, model_no))
            else:
                dead_ids.append(motor_id)

        if dead_ids:
            self.port_handler.closePort()
            self.port_handler = None
            self.connected = False
            self._set_control_enabled(False)
            self.connection_status.setText("상태: Disconnected")
            self.id_status.setText(f"미응답 ID: {dead_ids}")
            QMessageBox.warning(
                self,
                "연결 확인 실패",
                "일부 모터가 응답하지 않았습니다.\n"
                f"응답 ID: {[mid for mid, _ in alive_ids]}\n"
                f"미응답 ID: {dead_ids}",
            )
            return

        self.connected = True
        self._set_control_enabled(True)
        for motor_id in MOTOR_IDS:
            self.motor_widgets[motor_id].torque_btn.setChecked(False)
            self._set_goal_controls_enabled(motor_id, False)
        self.connect_check_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.connection_status.setText(f"상태: Connected ({port_name}, {baud})")
        self.id_status.setText(f"ID 확인 완료: {[(mid, model) for mid, model in alive_ids]}")

        QMessageBox.information(self, "연결 성공", "ID 1, 2, 3 모터 연결 확인 완료")

    def _disconnect(self) -> None:
        if self.port_handler is not None:
            self.port_handler.closePort()
        self.port_handler = None
        self.connected = False
        self._set_control_enabled(False)

        self.connect_check_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.connection_status.setText("상태: Disconnected")
        self.id_status.setText("ID 1/2/3 연결 확인 대기")

    def _toggle_torque(self, motor_id: int) -> None:
        if not self._require_connected():
            return

        widgets = self.motor_widgets[motor_id]
        enabling = widgets.torque_btn.isChecked()
        value = TORQUE_ENABLE if enabling else TORQUE_DISABLE

        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, value
        )
        if comm_result != COMM_SUCCESS:
            widgets.torque_btn.blockSignals(True)
            widgets.torque_btn.setChecked(not enabling)
            widgets.torque_btn.blockSignals(False)
            widgets.torque_btn.update()
            QMessageBox.critical(self, "통신 오류", self.packet_handler.getTxRxResult(comm_result))
            return
        if dxl_error != 0:
            widgets.torque_btn.blockSignals(True)
            widgets.torque_btn.setChecked(not enabling)
            widgets.torque_btn.blockSignals(False)
            widgets.torque_btn.update()
            QMessageBox.critical(self, "패킷 오류", self.packet_handler.getRxPacketError(dxl_error))
            return
        widgets.torque_btn.update()
        self._set_goal_controls_enabled(motor_id, enabling)

    def _send_goal(self, motor_id: int) -> None:
        if not self._require_connected():
            return

        goal = self.motor_widgets[motor_id].spin.value()
        comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_GOAL_POSITION, goal
        )
        if comm_result != COMM_SUCCESS:
            QMessageBox.critical(self, "통신 오류", self.packet_handler.getTxRxResult(comm_result))
            return
        if dxl_error != 0:
            QMessageBox.critical(self, "패킷 오류", self.packet_handler.getRxPacketError(dxl_error))
            return

    def _read_present(self, motor_id: int) -> None:
        if not self._require_connected():
            return

        present, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_POSITION
        )
        if comm_result != COMM_SUCCESS:
            QMessageBox.critical(self, "통신 오류", self.packet_handler.getTxRxResult(comm_result))
            return
        if dxl_error != 0:
            QMessageBox.critical(self, "패킷 오류", self.packet_handler.getRxPacketError(dxl_error))
            return

        self.motor_widgets[motor_id].present_label.setText(f"Present Position: {present}")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._disconnect()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    apply_stylesheet(app)

    win = MotorControlWindow()
    win.show()

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

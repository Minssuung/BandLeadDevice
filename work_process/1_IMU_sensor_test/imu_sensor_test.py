import sys
import time
from dataclasses import dataclass
from typing import Dict, Optional

import serial
from serial.tools import list_ports

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from stylesheet import apply_stylesheet


@dataclass
class PortActivity:
    port: str
    description: str
    hwid: str
    usb_info: str
    last_probe_bytes: int = 0


class WT61Parser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.latest: Dict[str, float] = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "wx": 0.0,
            "wy": 0.0,
            "wz": 0.0,
        }

    @staticmethod
    def _to_i16(lo: int, hi: int) -> int:
        val = (hi << 8) | lo
        if val >= 0x8000:
            val -= 0x10000
        return val

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

            frame_type = frame[1]
            d0, d1, d2, d3, d4, d5, _t0, _t1 = frame[2:10]

            if frame_type == 0x51:
                ax = self._to_i16(d0, d1) / 32768.0 * 16.0
                ay = self._to_i16(d2, d3) / 32768.0 * 16.0
                az = self._to_i16(d4, d5) / 32768.0 * 16.0
                self.latest.update({"ax": ax, "ay": ay, "az": az})
            elif frame_type == 0x52:
                wx = self._to_i16(d0, d1) / 32768.0 * 2000.0
                wy = self._to_i16(d2, d3) / 32768.0 * 2000.0
                wz = self._to_i16(d4, d5) / 32768.0 * 2000.0
                self.latest.update({"wx": wx, "wy": wy, "wz": wz})
            elif frame_type == 0x53:
                roll = self._to_i16(d0, d1) / 32768.0 * 180.0
                pitch = self._to_i16(d2, d3) / 32768.0 * 180.0
                yaw = self._to_i16(d4, d5) / 32768.0 * 180.0
                self.latest.update({"roll": roll, "pitch": pitch, "yaw": yaw})

            frames.append((frame_type, bytes(frame), dict(self.latest)))
            del self.buffer[:11]

        return frames


class SerialReaderThread(QThread):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    serial_error = pyqtSignal(str)
    raw_frame = pyqtSignal(bytes)
    parsed = pyqtSignal(dict)
    traffic = pyqtSignal(int, int)  # rx_total, tx_total

    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self._serial: Optional[serial.Serial] = None
        self._rx_total = 0
        self._tx_total = 0
        self._parser = WT61Parser()

    def run(self) -> None:
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
            info = f"Connected: {self.port} @ {self.baud}"
            self.connected.emit(info)
        except Exception as exc:
            msg = f"Open failed: {exc}"
            if "Permission denied" in str(exc):
                msg += (
                    "\n\nLinux 권한 문제입니다. 아래를 실행 후 다시 로그인하세요:\n"
                    "sudo usermod -aG dialout $USER\n"
                    "(로그아웃/로그인 또는 재부팅 후 재시도)"
                )
            self.serial_error.emit(msg)
            return

        try:
            while self._running and self._serial and self._serial.is_open:
                chunk = self._serial.read(256)
                if chunk:
                    self._rx_total += len(chunk)
                    frames = self._parser.feed(chunk)
                    for _, frame, latest in frames:
                        self.raw_frame.emit(frame)
                        self.parsed.emit(latest)
                    self.traffic.emit(self._rx_total, self._tx_total)
        except Exception as exc:
            self.serial_error.emit(f"Read error: {exc}")
        finally:
            if self._serial and self._serial.is_open:
                self._serial.close()
            self.disconnected.emit()

    def stop(self) -> None:
        self._running = False

    def send_bytes(self, payload: bytes) -> None:
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Serial is not connected")
        sent = self._serial.write(payload)
        self._tx_total += sent
        self.traffic.emit(self._rx_total, self._tx_total)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("WT61C-TTL IMU Tester")
        self.resize(1180, 760)

        self.reader: Optional[SerialReaderThread] = None
        self.last_rx_total = 0
        self.last_tick = time.time()
        self.rx_rate_bps = 0.0

        self._build_ui()
        self._refresh_ports()

        self.rate_timer = QTimer(self)
        self.rate_timer.timeout.connect(self._update_rate)
        self.rate_timer.start(1000)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(10, 12, 10, 10)

        conn_box = QGroupBox("1) USB/Serial 연결 상태")
        conn_layout = QVBoxLayout(conn_box)

        top_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(450)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "57600", "38400"])
        self.baud_combo.setCurrentText("9600")

        self.refresh_btn = QPushButton("포트 새로고침")
        self.refresh_btn.setObjectName("outlined")
        self.probe_btn = QPushButton("선택 포트 1초 프로브")
        self.probe_btn.setObjectName("outlined")
        self.connect_btn = QPushButton("연결")
        self.connect_btn.setObjectName("important")
        self.disconnect_btn = QPushButton("해제")
        self.disconnect_btn.setObjectName("important")
        self.disconnect_btn.setEnabled(False)

        top_row.addWidget(QLabel("포트"))
        top_row.addWidget(self.port_combo)
        top_row.addWidget(QLabel("Baud"))
        top_row.addWidget(self.baud_combo)
        top_row.addWidget(self.refresh_btn)
        top_row.addWidget(self.probe_btn)
        top_row.addWidget(self.connect_btn)
        top_row.addWidget(self.disconnect_btn)
        conn_layout.addLayout(top_row)

        self.status_label = QLabel("상태: Disconnected")
        self.status_label.setObjectName("emphasized")
        conn_layout.addWidget(self.status_label)

        self.port_table = QTableWidget(0, 5)
        self.port_table.setHorizontalHeaderLabels(
            ["Port", "Description", "HWID", "USB Info", "Probe RX bytes(1s)"]
        )
        self.port_table.horizontalHeader().setStretchLastSection(True)
        conn_layout.addWidget(self.port_table)

        traffic_box = QGroupBox("2) 데이터 수신/송신 트래픽")
        traffic_layout = QGridLayout(traffic_box)
        self.rx_total_label = QLabel("0")
        self.tx_total_label = QLabel("0")
        self.rx_rate_label = QLabel("0.0")
        self.last_frame_label = QLabel("-")

        traffic_layout.addWidget(QLabel("RX total (bytes)"), 0, 0)
        traffic_layout.addWidget(self.rx_total_label, 0, 1)
        traffic_layout.addWidget(QLabel("TX total (bytes)"), 0, 2)
        traffic_layout.addWidget(self.tx_total_label, 0, 3)
        traffic_layout.addWidget(QLabel("RX rate (B/s)"), 1, 0)
        traffic_layout.addWidget(self.rx_rate_label, 1, 1)
        traffic_layout.addWidget(QLabel("Last frame time"), 1, 2)
        traffic_layout.addWidget(self.last_frame_label, 1, 3)

        send_row = QHBoxLayout()
        self.tx_input = QLineEdit()
        self.tx_input.setPlaceholderText("송신 HEX 입력 예: FF AA 03 00")
        self.tx_btn = QPushButton("HEX 송신")
        self.tx_btn.setObjectName("contained")
        self.tx_btn.setEnabled(False)
        send_row.addWidget(self.tx_input)
        send_row.addWidget(self.tx_btn)
        traffic_layout.addLayout(send_row, 2, 0, 1, 4)

        data_box = QGroupBox("3) WT61C 파싱 값")
        data_layout = QGridLayout(data_box)
        self.value_labels: Dict[str, QLabel] = {}
        keys = [
            ("roll", "Roll (deg)"),
            ("pitch", "Pitch (deg)"),
            ("yaw", "Yaw (deg)"),
            ("ax", "Ax (g)"),
            ("ay", "Ay (g)"),
            ("az", "Az (g)"),
            ("wx", "Wx (deg/s)"),
            ("wy", "Wy (deg/s)"),
            ("wz", "Wz (deg/s)"),
        ]
        for idx, (key, label) in enumerate(keys):
            r = idx // 3
            c = (idx % 3) * 2
            v = QLabel("0.000")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.value_labels[key] = v
            data_layout.addWidget(QLabel(label), r, c)
            data_layout.addWidget(v, r, c + 1)

        log_box = QGroupBox("4) 원시 프레임 로그 (11-byte)")
        log_layout = QVBoxLayout(log_box)
        self.raw_log = QTextEdit()
        self.raw_log.setReadOnly(True)
        log_layout.addWidget(self.raw_log)

        main_layout.addWidget(conn_box)
        main_layout.addWidget(traffic_box)
        main_layout.addWidget(data_box)
        main_layout.addWidget(log_box)

        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.probe_btn.clicked.connect(self._probe_selected_port)
        self.connect_btn.clicked.connect(self._connect_serial)
        self.disconnect_btn.clicked.connect(self._disconnect_serial)
        self.tx_btn.clicked.connect(self._send_hex)

    @staticmethod
    def _usb_info_from_port(p) -> str:
        parts = []
        if p.vid is not None:
            parts.append(f"VID=0x{p.vid:04X}")
        if p.pid is not None:
            parts.append(f"PID=0x{p.pid:04X}")
        if p.manufacturer:
            parts.append(f"MFG={p.manufacturer}")
        if p.serial_number:
            parts.append(f"SN={p.serial_number}")
        return ", ".join(parts) if parts else "-"

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = sorted(list_ports.comports(), key=lambda x: x.device)
        self.port_table.setRowCount(len(ports))

        for row, p in enumerate(ports):
            label = f"{p.device} | {p.description}"
            self.port_combo.addItem(label, p.device)
            usb_info = self._usb_info_from_port(p)

            self.port_table.setItem(row, 0, QTableWidgetItem(p.device))
            self.port_table.setItem(row, 1, QTableWidgetItem(p.description or "-"))
            self.port_table.setItem(row, 2, QTableWidgetItem(p.hwid or "-"))
            self.port_table.setItem(row, 3, QTableWidgetItem(usb_info))
            self.port_table.setItem(row, 4, QTableWidgetItem("-"))

        if not ports:
            self.status_label.setText("상태: Serial 포트를 찾지 못했습니다.")

    def _selected_port(self) -> Optional[str]:
        if self.port_combo.currentIndex() < 0:
            return None
        return self.port_combo.currentData()

    def _probe_selected_port(self) -> None:
        port = self._selected_port()
        if not port:
            QMessageBox.warning(self, "포트 없음", "먼저 포트를 선택하세요.")
            return

        baud = int(self.baud_combo.currentText())
        read_bytes = 0
        try:
            with serial.Serial(port, baud, timeout=0.2) as s:
                start = time.time()
                while time.time() - start < 1.0:
                    chunk = s.read(256)
                    if chunk:
                        read_bytes += len(chunk)
        except Exception as exc:
            QMessageBox.critical(self, "프로브 실패", f"{port} 프로브 중 오류\n{exc}")
            return

        for row in range(self.port_table.rowCount()):
            item = self.port_table.item(row, 0)
            if item and item.text() == port:
                self.port_table.setItem(row, 4, QTableWidgetItem(str(read_bytes)))
                break

        if read_bytes > 0:
            self.status_label.setText(f"상태: {port}에서 1초간 {read_bytes} bytes 수신")
        else:
            self.status_label.setText(
                f"상태: {port} 프로브 완료 (수신 0 bytes, baud 또는 배선 확인 필요)"
            )

    def _connect_serial(self) -> None:
        if self.reader is not None:
            QMessageBox.information(self, "이미 연결됨", "이미 시리얼 연결이 활성화되어 있습니다.")
            return

        port = self._selected_port()
        if not port:
            QMessageBox.warning(self, "포트 없음", "연결할 포트를 선택하세요.")
            return

        baud = int(self.baud_combo.currentText())
        self.reader = SerialReaderThread(port, baud)
        self.reader.connected.connect(self._on_connected)
        self.reader.disconnected.connect(self._on_disconnected)
        self.reader.serial_error.connect(self._on_serial_error)
        self.reader.raw_frame.connect(self._on_raw_frame)
        self.reader.parsed.connect(self._on_parsed)
        self.reader.traffic.connect(self._on_traffic)
        self.reader.start()

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.tx_btn.setEnabled(True)

    def _disconnect_serial(self) -> None:
        if self.reader is None:
            return
        self.reader.stop()
        self.reader.wait(1200)
        self.reader = None

        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.tx_btn.setEnabled(False)
        self.status_label.setText("상태: Disconnected")

    def _on_connected(self, msg: str) -> None:
        self.status_label.setText(f"상태: {msg}")

    def _on_disconnected(self) -> None:
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.tx_btn.setEnabled(False)

    def _on_serial_error(self, msg: str) -> None:
        self.status_label.setText(f"상태: ERROR - {msg}")
        QMessageBox.critical(self, "Serial Error", msg)
        self._disconnect_serial()

    def _on_raw_frame(self, frame: bytes) -> None:
        now = time.strftime("%H:%M:%S")
        hexs = " ".join(f"{b:02X}" for b in frame)
        self.raw_log.append(f"[{now}] {hexs}")
        self.last_frame_label.setText(now)

        if self.raw_log.document().blockCount() > 500:
            cursor = self.raw_log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _on_parsed(self, data: dict) -> None:
        for k, v in data.items():
            if k in self.value_labels:
                self.value_labels[k].setText(f"{v:.3f}")

    def _on_traffic(self, rx_total: int, tx_total: int) -> None:
        self.rx_total_label.setText(str(rx_total))
        self.tx_total_label.setText(str(tx_total))

    def _update_rate(self) -> None:
        now = time.time()
        dt = max(1e-6, now - self.last_tick)
        current_rx = int(self.rx_total_label.text() or "0")
        self.rx_rate_bps = max(0.0, (current_rx - self.last_rx_total) / dt)
        self.last_rx_total = current_rx
        self.last_tick = now
        self.rx_rate_label.setText(f"{self.rx_rate_bps:.1f}")

    def _send_hex(self) -> None:
        text = self.tx_input.text().strip().replace(" ", "")
        if not text:
            QMessageBox.warning(self, "입력 없음", "송신할 HEX 문자열을 입력하세요.")
            return
        if len(text) % 2 != 0:
            QMessageBox.warning(self, "입력 오류", "HEX 문자열 길이는 짝수여야 합니다.")
            return

        try:
            payload = bytes.fromhex(text)
        except ValueError:
            QMessageBox.warning(self, "입력 오류", "유효한 HEX 문자열이 아닙니다.")
            return

        if not self.reader:
            QMessageBox.warning(self, "연결 없음", "먼저 시리얼 포트에 연결하세요.")
            return

        try:
            self.reader.send_bytes(payload)
        except Exception as exc:
            QMessageBox.critical(self, "송신 실패", str(exc))
            return

        self.raw_log.append(f"[TX] {' '.join(f'{b:02X}' for b in payload)}")

    def closeEvent(self, event):
        self._disconnect_serial()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    apply_stylesheet(app)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

import json
import socket
from datetime import datetime


UDP_IP = "0.0.0.0"
UDP_PORT = 4210
BUFFER_SIZE = 2048


def format_float(value):
	if isinstance(value, (int, float)):
		return f"{value:8.3f}"
	return "   n/a  "


def print_packet(packet, addr):
	ts = datetime.now().strftime("%H:%M:%S")
	roll = format_float(packet.get("roll"))
	pitch = format_float(packet.get("pitch"))
	yaw = format_float(packet.get("yaw"))
	ax = format_float(packet.get("ax"))
	ay = format_float(packet.get("ay"))
	az = format_float(packet.get("az"))

	print(
		f"[{ts}] {addr[0]}:{addr[1]} | "
		f"RPY(deg)=({roll}, {pitch}, {yaw}) | "
		f"ACC(g)=({ax}, {ay}, {az})"
	)


def print_status_packet(packet, addr):
	ts = datetime.now().strftime("%H:%M:%S")
	imu_available = packet.get("imu_available", "?")
	imu_baud = packet.get("imu_baud", "?")
	imu_bytes = packet.get("imu_bytes", "?")
	imu_frames = packet.get("imu_frames", "?")
	rssi = packet.get("rssi", "?")

	print(
		f"[{ts}] {addr[0]}:{addr[1]} | "
		f"STATUS imu_available={imu_available}, imu_baud={imu_baud}, "
		f"imu_bytes={imu_bytes}, imu_frames={imu_frames}, rssi={rssi}"
	)


def main():
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	try:
		sock.bind((UDP_IP, UDP_PORT))
	except OSError as exc:
		if getattr(exc, "errno", None) == 98:
			print(f"[오류] UDP 포트 {UDP_PORT}가 이미 사용 중입니다.")
			print("다른 수신기가 실행 중인지 확인 후 종료하세요.")
			print("예: lsof -nP -iUDP:4210")
			print("예: kill <PID>")
			return
		raise

	print(f"[PC] UDP IMU 수신 대기 중... (포트 {UDP_PORT})")
	print("[PC] 종료: Ctrl+C\n")

	try:
		while True:
			data, addr = sock.recvfrom(BUFFER_SIZE)
			text = data.decode("utf-8", errors="replace").strip()

			try:
				packet = json.loads(text)
				if isinstance(packet, dict):
					if packet.get("type") == "status":
						print_status_packet(packet, addr)
					else:
						print_packet(packet, addr)
				else:
					print(f"[RAW] {addr[0]}:{addr[1]} -> {text}")
			except json.JSONDecodeError:
				print(f"[RAW] {addr[0]}:{addr[1]} -> {text}")
	except KeyboardInterrupt:
		print("\n[PC] 종료")
	finally:
		sock.close()


if __name__ == "__main__":
	main()

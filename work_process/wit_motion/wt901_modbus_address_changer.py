#!/usr/bin/env python3
"""
WitMotion WT901C-485 Modbus RTU 슬레이브 주소 변경 툴

USB-RS485 어댑터로 PC에 IMU를 직접 연결한 후 실행합니다.
여러 레지스터 방법을 순서대로 시도하여 주소를 변경합니다.

의존성: pyserial
  pip install pyserial
"""

import serial
import serial.tools.list_ports
import time
import sys


# ──────────────────────────────────────────────
# Modbus RTU 유틸
# ──────────────────────────────────────────────

def crc16_modbus(data: bytes) -> int:
	crc = 0xFFFF
	for byte in data:
		crc ^= byte
		for _ in range(8):
			if crc & 0x0001:
				crc = (crc >> 1) ^ 0xA001
			else:
				crc >>= 1
	return crc


def build_read_frame(slave_id: int, reg: int, count: int) -> bytes:
	"""FC=0x03 Read Holding Registers 프레임 빌드"""
	data = bytes([
		slave_id, 0x03,
		(reg >> 8) & 0xFF, reg & 0xFF,
		(count >> 8) & 0xFF, count & 0xFF,
	])
	crc = crc16_modbus(data)
	return data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_write_frame(slave_id: int, reg: int, value: int) -> bytes:
	"""FC=0x06 Write Single Register 프레임 빌드"""
	data = bytes([
		slave_id, 0x06,
		(reg >> 8) & 0xFF, reg & 0xFF,
		(value >> 8) & 0xFF, value & 0xFF,
	])
	crc = crc16_modbus(data)
	return data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def send_recv(ser: serial.Serial, frame: bytes, wait: float = 0.3) -> bytes:
	ser.reset_input_buffer()
	ser.write(frame)
	time.sleep(wait)
	return ser.read(ser.in_waiting or 64)


def verify_crc(resp: bytes) -> bool:
	if len(resp) < 4:
		return False
	calc = crc16_modbus(resp[:-2])
	recv = resp[-2] | (resp[-1] << 8)
	return calc == recv


def find_frame(resp: bytes, slave_id: int, fc: int, min_len: int) -> bytes | None:
	"""응답 버퍼에서 유효 Modbus 프레임을 탐색하여 반환"""
	for i in range(len(resp)):
		if i + min_len > len(resp):
			break
		if resp[i] == slave_id and resp[i + 1] == fc:
			chunk = resp[i:i + min_len]
			if verify_crc(chunk):
				return chunk
	return None


# ──────────────────────────────────────────────
# 고수준 Modbus 조작
# ──────────────────────────────────────────────

def ping_slave(ser: serial.Serial, slave_id: int) -> bool:
	"""각도 레지스터(0x003D) 1개 읽기로 슬레이브 존재 확인"""
	frame = build_read_frame(slave_id, 0x003D, 1)
	resp = send_recv(ser, frame, 0.25)
	return find_frame(resp, slave_id, 0x03, 7) is not None


def read_register(ser: serial.Serial, slave_id: int, reg: int, count: int = 1):
	"""레지스터 읽기. 성공 시 int 값 리스트, 실패 시 None"""
	frame = build_read_frame(slave_id, reg, count)
	resp = send_recv(ser, frame, 0.3)
	expected = 3 + count * 2 + 2
	chunk = find_frame(resp, slave_id, 0x03, expected)
	if chunk is None:
		return None
	values = []
	for j in range(count):
		val = (chunk[3 + j * 2] << 8) | chunk[4 + j * 2]
		values.append(val)
	return values


def write_register(ser: serial.Serial, slave_id: int, reg: int, value: int) -> bool:
	"""단일 레지스터 쓰기. FC=0x06 에코 응답 또는 유효 응답 시 True"""
	frame = build_write_frame(slave_id, reg, value)
	resp = send_recv(ser, frame, 0.5)
	# FC=0x06 에코 응답 확인 (8바이트)
	chunk = find_frame(resp, slave_id, 0x06, 8)
	if chunk is not None:
		return True
	# 에코 없이도 오류 응답이 없으면 성공으로 간주
	if len(resp) == 0:
		# 응답 없음 → 일단 True (저장 후 재부팅이면 응답 없는 경우 있음)
		return True
	return False


# ──────────────────────────────────────────────
# 주소 변경 시도 (여러 레지스터 방법)
# ──────────────────────────────────────────────

# WitMotion 모델별 알려진 Device Address 레지스터 목록
ADDR_REGISTERS = [
	(0x001A, "WitMotion 공통 Address 레지스터"),
	(0x00FF, "일부 WitMotion 모델 Address 레지스터"),
	(0x1001, "Modbus 전용 Address 레지스터"),
]
# Save configuration 레지스터 (0x0000 에 0x0000 쓰기)
SAVE_REG = 0x0000
SAVE_VAL = 0x0000


def try_change_address(ser: serial.Serial, old_addr: int, new_addr: int) -> bool:
	"""
	알려진 레지스터를 순서대로 시도하여 주소 변경.
	변경 명령 전송 성공 시 True.
	"""
	for reg, desc in ADDR_REGISTERS:
		print(f"  → [{desc}] 0x{reg:04X} 에 0x{new_addr:02X} 쓰기...", end=" ", flush=True)
		ok = write_register(ser, old_addr, reg, new_addr)
		if ok:
			print("전송 완료")
			time.sleep(0.15)
			# 저장 명령
			print(f"  → 저장 명령 (0x{SAVE_REG:04X} = 0x{SAVE_VAL:04X}) 전송...", end=" ", flush=True)
			write_register(ser, old_addr, SAVE_REG, SAVE_VAL)
			print("전송 완료")
			return True
		else:
			print("실패")
	return False


# ──────────────────────────────────────────────
# 스캔
# ──────────────────────────────────────────────

def scan_bus(ser: serial.Serial, start: int = 0x01, end: int = 0x7F) -> list[int]:
	"""버스에서 응답하는 슬레이브 주소 목록 반환"""
	found = []
	print(f"스캔 범위: 0x{start:02X} ~ 0x{end:02X}", flush=True)
	for addr in range(start, end + 1):
		sys.stdout.write(f"\r  검색 중: 0x{addr:02X} ...    ")
		sys.stdout.flush()
		if ping_slave(ser, addr):
			found.append(addr)
			print(f"\r  ✓ 응답 발견: 0x{addr:02X}                ")
	sys.stdout.write("\r" + " " * 40 + "\r")
	return found


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

def main():
	print("=" * 60)
	print("  WitMotion WT901C-485  Modbus 주소 변경 툴")
	print("=" * 60)

	# 1) 포트 선택
	ports = list(serial.tools.list_ports.comports())
	if not ports:
		print("[오류] 사용 가능한 시리얼 포트가 없습니다.")
		sys.exit(1)

	print("\n[사용 가능한 시리얼 포트]")
	for i, p in enumerate(ports):
		print(f"  {i}: {p.device}  ({p.description})")

	try:
		raw = input("\n포트 번호 선택 [0]: ").strip() or "0"
		port = ports[int(raw)].device
	except (ValueError, IndexError):
		print("[오류] 잘못된 입력")
		sys.exit(1)

	# 2) 파라미터 입력
	baud     = int(input("보레이트 [9600]: ").strip() or "9600")
	old_hex  = input("현재 슬레이브 주소 hex (예: 50) [50]: ").strip() or "50"
	new_hex  = input("변경할 슬레이브 주소 hex (예: 51) [51]: ").strip() or "51"
	old_addr = int(old_hex, 16)
	new_addr = int(new_hex, 16)

	print(f"\n포트={port}  보레이트={baud}  0x{old_addr:02X} → 0x{new_addr:02X}")

	try:
		ser = serial.Serial(port, baud, timeout=0.5)
	except serial.SerialException as e:
		print(f"[오류] 포트 열기 실패: {e}")
		sys.exit(1)

	time.sleep(0.3)  # 포트 안정화

	# 3) 현재 슬레이브 응답 확인
	print(f"\n[STEP 1] 0x{old_addr:02X} 응답 확인...")
	if ping_slave(ser, old_addr):
		print(f"  ✓ 슬레이브 0x{old_addr:02X} 응답 확인!")
	else:
		print(f"  ✗ 응답 없음 (보레이트 또는 주소 불일치 가능)")
		do_scan = input("  버스 전체 스캔을 시도할까요? [Y/n]: ").strip().lower()
		if do_scan != "n":
			found = scan_bus(ser)
			if found:
				old_addr = found[0]
				print(f"  스캔에서 발견된 주소 0x{old_addr:02X} 를 현재 주소로 사용합니다.")
			else:
				print("  응답 없음. 배선·보레이트를 재확인하거나 강제 진행합니다.")
				proceed = input("  강제로 주소 변경을 시도할까요? [y/N]: ").strip().lower()
				if proceed != "y":
					ser.close()
					sys.exit(0)

	# 4) 현재 Address 레지스터 값 읽기 (진단용)
	print(f"\n[STEP 2] 현재 Address 레지스터(0x001A) 읽기 (참고용)...")
	vals = read_register(ser, old_addr, 0x001A, 1)
	if vals is not None:
		print(f"  Register 0x001A = 0x{vals[0]:04X}  ({vals[0]})")
	else:
		print("  읽기 실패 (모델에 따라 읽기를 지원하지 않을 수 있음)")

	# 5) 주소 변경
	print(f"\n[STEP 3] 주소 변경 시도: 0x{old_addr:02X} → 0x{new_addr:02X}")
	changed = try_change_address(ser, old_addr, new_addr)
	if not changed:
		print("  [경고] 모든 방법이 실패했습니다.")

	# 6) 재부팅 대기
	print(f"\n[STEP 4] 장치 재부팅 대기 (3초)...")
	print("  (설정이 저장됐다면 장치가 재시작합니다)")
	time.sleep(3.0)

	# 7) 새 주소로 응답 확인
	print(f"\n[STEP 5] 새 주소 0x{new_addr:02X} 응답 확인...")
	if ping_slave(ser, new_addr):
		print(f"  ✓ 성공! 0x{new_addr:02X} 응답 확인. 주소 변경 완료!")
		print(f"\n  다음 단계: ESP32 코드의 슬레이브 배열을")
		print(f"  {{0x50, 0x51}} 로 변경 후 업로드하세요.")
	else:
		print(f"  ✗ 0x{new_addr:02X} 응답 없음.")
		# 이전 주소 잔존 여부
		if ping_slave(ser, old_addr):
			print(f"  여전히 0x{old_addr:02X} 에 응답 중 → 변경 미적용.")
			print("  장치 전원을 껐다 켜고 다시 시도해보세요.")
		else:
			print(f"  두 주소 모두 응답 없음 → 전원 재연결 후 재확인하세요.")

	ser.close()
	print("\n종료.")


if __name__ == "__main__":
	main()

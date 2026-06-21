#!/usr/bin/env python3
"""WT901C485 쿼터니언 레지스터 주소 확인 프로브 (펌웨어 불필요)

USB-RS485 어댑터로 IMU를 PC에 직접 연결한 뒤 실행하면,
쿼터니언 후보 레지스터(0x51, 0x52)를 읽어 어느 쪽이 진짜 쿼터니언인지
(단위 노름 ‖q‖≈1 인지) 판별한다.

→ 그 결과로 펌웨어의 IMU_QUAT_START_REG 값을 확정한 뒤 한 번만 업로드하면 됨.

배선: IMU(RS485 A/B) → USB-RS485 어댑터 → PC USB
의존성: pyserial
"""

import math
import sys
import time

import serial
import serial.tools.list_ports


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def build_read(slave: int, reg: int, count: int) -> bytes:
    d = bytes([slave, 0x03, (reg >> 8) & 0xFF, reg & 0xFF,
               (count >> 8) & 0xFF, count & 0xFF])
    crc = crc16_modbus(d)
    return d + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def find_frame(resp: bytes, slave: int, expect_len: int):
    for i in range(len(resp)):
        if i + expect_len > len(resp):
            break
        if resp[i] == slave and resp[i + 1] == 0x03:
            chunk = resp[i:i + expect_len]
            calc = crc16_modbus(chunk[:-2])
            recv = chunk[-2] | (chunk[-1] << 8)
            if calc == recv:
                return chunk
    return None


def read_regs(ser, slave, reg, count, wait=0.3):
    """count개 레지스터를 읽어 16bit 부호없는 값 리스트 반환 (실패 None)."""
    ser.reset_input_buffer()
    ser.write(build_read(slave, reg, count))
    time.sleep(wait)
    resp = ser.read(ser.in_waiting or 128)
    chunk = find_frame(resp, slave, 3 + count * 2 + 2)
    if chunk is None:
        return None
    return [(chunk[3 + 2 * j] << 8) | chunk[4 + 2 * j] for j in range(count)]


def s16(v):
    return v - 65536 if v >= 32768 else v


def main():
    # 포트 선택
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("시리얼 포트를 찾을 수 없습니다. USB-RS485 어댑터 연결을 확인하세요.")
        return
    print("=== 시리얼 포트 목록 ===")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    sel = input(f"포트 번호 선택 [0]: ").strip() or "0"
    port = ports[int(sel)].device
    baud = int(input("보레이트 [9600]: ").strip() or "9600")
    slave = int(input("슬레이브 ID(16진수, 예 50) [50]: ").strip() or "50", 16)

    with serial.Serial(port, baud, timeout=0.3) as ser:
        time.sleep(0.2)
        print(f"\n[연결] {port} @ {baud}, slave=0x{slave:02X}\n")

        # 1) 통신 확인: 각도(0x3D) 3개
        ang = read_regs(ser, slave, 0x003D, 3)
        if ang is None:
            print("❌ 각도 레지스터(0x3D) 응답 없음 → 배선/보레이트/슬레이브ID 확인 필요.")
            print("   (보레이트 9600/115200, 슬레이브 50/51/52 바꿔가며 시도)")
            return
        roll, pitch, yaw = [s16(v) / 100.0 for v in ang]
        print(f"✅ 통신 OK | Roll={roll:.2f} Pitch={pitch:.2f} Yaw={yaw:.2f} (deg)\n")

        # 2) 쿼터니언 후보 레지스터 스캔 (‖q‖≈1 인 곳이 진짜 쿼터니언)
        print("=== 쿼터니언 레지스터 스캔 0x48~0x58 (‖q‖≈1 이면 정답) ===")
        best = None
        for start in range(0x0048, 0x0059):
            vals = read_regs(ser, slave, start, 4)
            if vals is None:
                print(f"  0x{start:04X}: 응답 없음")
                continue
            q = [s16(v) / 32768.0 for v in vals]
            norm = math.sqrt(sum(c * c for c in q))
            ok = abs(norm - 1.0) < 0.05
            mark = "✅ 쿼터니언 맞음!" if ok else "❌ 쿼터니언 아님"
            print(f"  0x{start:04X}: q=[{q[0]:+.4f},{q[1]:+.4f},{q[2]:+.4f},{q[3]:+.4f}]  "
                  f"‖q‖={norm:.4f}  {mark}")
            if ok and best is None:
                best = start

        print()
        if best is not None:
            print(f"👉 결론: 펌웨어 IMU_QUAT_START_REG = 0x{best:04X} 로 설정하세요.")
        else:
            print("⚠️ 두 주소 모두 ‖q‖≈1 이 아닙니다. 센서를 가만히 두고 다시 실행하거나,")
            print("   매뉴얼에서 쿼터니언 레지스터 주소를 확인하세요. (이 센서는 쿼터니언 미지원일 수도)")


if __name__ == "__main__":
    main()

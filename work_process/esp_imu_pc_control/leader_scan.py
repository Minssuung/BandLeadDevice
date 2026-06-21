#!/usr/bin/env python3
"""리더 디바이스(외골격) 다이나믹셀 스캔: 포트/baud/ID/모델 자동 탐색.
사용: python3 leader_scan.py [포트]   (포트 생략시 전체 자동)
"""
import glob
import sys

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

BAUDS = [57600, 1000000, 2000000, 115200, 3000000, 4000000, 9600]
ID_RANGE = range(0, 31)

# 모델번호 → 이름 (자주 쓰는 것만)
MODELS = {
    1020: "XM430-W350", 1030: "XM430-W210", 1120: "XM540-W270", 1130: "XM540-W150",
    1060: "XL430-W250", 1190: "XL330-M077", 1200: "XL330-M288",
    1110: "XH430-W210", 1100: "XH430-W350", 35072: "PRO/기타",
}


def scan_port(dev: str) -> bool:
    ph = PortHandler(dev)
    try:
        if not ph.openPort():
            print(f"  {dev}: 열기 실패(사용중이거나 권한)")
            return False
    except Exception as exc:
        print(f"  {dev}: {exc}")
        return False
    found_any = False
    for baud in BAUDS:
        if not ph.setBaudRate(baud):
            continue
        pk = PacketHandler(2.0)
        found = []
        for dxl_id in ID_RANGE:
            model, comm, err = pk.ping(ph, dxl_id)
            if comm == COMM_SUCCESS:
                found.append((dxl_id, model))
        if found:
            found_any = True
            print(f"  {dev} @ {baud} baud:")
            for dxl_id, model in found:
                name = MODELS.get(model, f"model={model}")
                # 현재 위치도 읽어줌 (4B, addr 132)
                pos, comm, err = pk.read4ByteTxRx(ph, dxl_id, 132)
                pos_s = f"{pos}틱({pos / 4095 * 360:.0f}°)" if comm == COMM_SUCCESS else "?"
                print(f"    ID {dxl_id:2d}: {name}  현재위치 {pos_s}")
            break  # 이 포트는 이 baud로 확정
    if not found_any:
        print(f"  {dev}: 모터 없음")
    ph.closePort()
    return found_any


def main():
    ports = sys.argv[1:] or sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        print("USB 시리얼 포트가 없습니다. 리더 디바이스/U2D2 연결을 확인하세요.")
        return
    print(f"스캔 대상: {ports}")
    for dev in ports:
        scan_port(dev)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Phase 2-4: WT61 IMU -> 3x DYNAMIXEL position mapping test.

Usage example:
python3 2_4_python_control_test.py \
  --imu-port /dev/ttyUSB0 --dxl-port /dev/ttyACM0 \
  --dxl-ids 1 2 3 --dxl-baud 57600 --imu-baud 115200
"""

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List

import serial
from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


@dataclass
class Wt61State:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class Wt61Parser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.state = Wt61State()

    @staticmethod
    def _to_i16(lo: int, hi: int) -> int:
        value = (hi << 8) | lo
        if value >= 0x8000:
            value -= 0x10000
        return value

    def feed(self, chunk: bytes) -> List[Wt61State]:
        results: List[Wt61State] = []
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
            if frame_type == 0x53:
                self.state.roll = self._to_i16(d0, d1) / 32768.0 * 180.0
                self.state.pitch = self._to_i16(d2, d3) / 32768.0 * 180.0
                self.state.yaw = self._to_i16(d4, d5) / 32768.0 * 180.0
                results.append(Wt61State(self.state.roll, self.state.pitch, self.state.yaw))

            del self.buffer[:11]

        return results


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def to_le_u32(value: int) -> List[int]:
    return [
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]


def write_torque(packet: PacketHandler, port: PortHandler, motor_ids: Iterable[int], enabled: bool) -> None:
    onoff = TORQUE_ENABLE if enabled else TORQUE_DISABLE
    for mid in motor_ids:
        comm_result, dxl_error = packet.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, onoff)
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(f"[ID:{mid}] comm error: {packet.getTxRxResult(comm_result)}")
        if dxl_error != 0:
            raise RuntimeError(f"[ID:{mid}] packet error: {packet.getRxPacketError(dxl_error)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WT61 -> DYNAMIXEL 3-axis mapping test")
    parser.add_argument("--imu-port", required=True, help="WT61 serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--imu-baud", type=int, default=115200)
    parser.add_argument("--dxl-port", required=True, help="OpenRB/U2D2 serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--dxl-baud", type=int, default=57600)
    parser.add_argument("--dxl-ids", type=int, nargs=3, default=[1, 2, 3], help="roll pitch yaw motor IDs")

    parser.add_argument("--neutral", type=int, default=2048)
    parser.add_argument("--min-pos", type=int, default=1200)
    parser.add_argument("--max-pos", type=int, default=2896)

    parser.add_argument("--gain-roll", type=float, default=8.0, help="ticks per deg")
    parser.add_argument("--gain-pitch", type=float, default=8.0, help="ticks per deg")
    parser.add_argument("--gain-yaw", type=float, default=8.0, help="ticks per deg")

    parser.add_argument("--dir-roll", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--dir-pitch", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--dir-yaw", type=int, choices=[-1, 1], default=1)

    parser.add_argument("--hz", type=float, default=30.0, help="update rate")
    parser.add_argument("--max-step", type=int, default=30, help="max ticks change per cycle")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    running = True

    def handle_stop(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    try:
        imu = serial.Serial(args.imu_port, args.imu_baud, timeout=0.02)
    except Exception as exc:
        print(f"IMU open failed: {exc}")
        print("Tip: Linux permission issue -> sudo usermod -aG dialout $USER")
        return 1

    dxl_port = PortHandler(args.dxl_port)
    packet = PacketHandler(PROTOCOL_VERSION)
    sync_write = GroupSyncWrite(dxl_port, packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    if not dxl_port.openPort():
        print("DYNAMIXEL port open failed")
        imu.close()
        return 1

    if not dxl_port.setBaudRate(args.dxl_baud):
        print("DYNAMIXEL baudrate set failed")
        dxl_port.closePort()
        imu.close()
        return 1

    motor_ids = args.dxl_ids
    current_goal: Dict[int, int] = {mid: args.neutral for mid in motor_ids}

    try:
        write_torque(packet, dxl_port, motor_ids, enabled=True)
        print("Connected. Ctrl+C to stop.")

        parser = Wt61Parser()
        period = 1.0 / max(args.hz, 1.0)
        last_print = 0.0

        while running:
            start = time.time()
            chunk = imu.read(256)
            states = parser.feed(chunk)
            if not states:
                time.sleep(0.002)
                continue

            latest = states[-1]
            roll_target = args.neutral + int(args.dir_roll * latest.roll * args.gain_roll)
            pitch_target = args.neutral + int(args.dir_pitch * latest.pitch * args.gain_pitch)
            yaw_target = args.neutral + int(args.dir_yaw * latest.yaw * args.gain_yaw)

            target_map = {
                motor_ids[0]: clamp(roll_target, args.min_pos, args.max_pos),
                motor_ids[1]: clamp(pitch_target, args.min_pos, args.max_pos),
                motor_ids[2]: clamp(yaw_target, args.min_pos, args.max_pos),
            }

            for mid in motor_ids:
                delta = target_map[mid] - current_goal[mid]
                delta = clamp(delta, -args.max_step, args.max_step)
                current_goal[mid] += delta

            for mid in motor_ids:
                ok = sync_write.addParam(mid, to_le_u32(current_goal[mid]))
                if not ok:
                    raise RuntimeError(f"GroupSyncWrite addParam failed for ID {mid}")

            comm_result = sync_write.txPacket()
            sync_write.clearParam()
            if comm_result != COMM_SUCCESS:
                raise RuntimeError(packet.getTxRxResult(comm_result))

            now = time.time()
            if now - last_print > 0.25:
                last_print = now
                print(
                    f"rpy=({latest.roll:7.2f}, {latest.pitch:7.2f}, {latest.yaw:7.2f}) "
                    f"goal={current_goal}"
                )

            elapsed = time.time() - start
            remain = period - elapsed
            if remain > 0:
                time.sleep(remain)

    except Exception as exc:
        print(f"Runtime error: {exc}")
        return 1
    finally:
        try:
            write_torque(packet, dxl_port, motor_ids, enabled=False)
        except Exception as exc:
            print(f"Warning: failed to disable torque: {exc}")
        dxl_port.closePort()
        imu.close()
        print("Ports closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Feetech STS 계열 리더 외골격 읽기 모듈 (dynamixel_sdk P1.0 프레임 호환).

리더 서보는 모터가 아니라 '각도 엔코더'로만 사용: 토크 OFF로 두고 위치만 폴링.
- 프로토콜: SCS(=DXL 1.0과 동일 프레임) @ 1M baud
- 현재위치: addr 56 (2B), 0~4095 틱 = 360°
- 토크: addr 40 (1B, 0=off)
"""
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

LEADER_BAUD = 1000000
LEADER_PROTOCOL = 1.0
ADDR_STS_TORQUE_ENABLE = 40
ADDR_STS_PRESENT_POSITION = 56
TICKS_PER_REV = 4096.0


class FeetechLeader:
    """리더 한 팔(ID 1..8 = 관절7 + 그리퍼1)의 포지션 폴러."""

    def __init__(self, port: str, ids=(1, 2, 3, 4, 5, 6, 7, 8)):
        self.port_name = port
        self.ids = tuple(ids)
        self.ph = None
        self.pk = PacketHandler(LEADER_PROTOCOL)

    def open(self) -> None:
        ph = PortHandler(self.port_name)
        if not ph.openPort():
            raise RuntimeError(f"리더 포트 열기 실패: {self.port_name}")
        if not ph.setBaudRate(LEADER_BAUD):
            ph.closePort()
            raise RuntimeError(f"리더 baud 설정 실패: {LEADER_BAUD}")
        self.ph = ph
        # 응답 확인 + 토크 OFF (엔코더로만 사용 — 사람이 자유롭게 움직여야 함)
        alive = []
        for i in self.ids:
            _, comm, _ = self.pk.ping(self.ph, i)
            if comm == COMM_SUCCESS:
                alive.append(i)
                self.pk.write1ByteTxRx(self.ph, i, ADDR_STS_TORQUE_ENABLE, 0)
        if not alive:
            self.close()
            raise RuntimeError(f"{self.port_name}: 리더 서보 무응답 (전원/케이블 확인)")
        self.alive_ids = alive

    def close(self) -> None:
        if self.ph is not None:
            try:
                self.ph.closePort()
            except Exception:
                pass
            self.ph = None

    def read_positions(self):
        """{id: ticks} — 실패한 ID는 빠짐. 8개 읽기 ≈ 수 ms @1M."""
        out = {}
        if self.ph is None:
            return out
        for i in self.ids:
            pos, comm, err = self.pk.read2ByteTxRx(self.ph, i, ADDR_STS_PRESENT_POSITION)
            if comm == COMM_SUCCESS and err == 0:
                out[i] = pos
        return out

    @staticmethod
    def ticks_to_deg(delta_ticks: float) -> float:
        return delta_ticks * 360.0 / TICKS_PER_REV


def wiggle_detect(ports=("/dev/ttyACM0", "/dev/ttyACM1"), sec=4.0):
    """어느 포트가 '지금 움직이는 팔'인지 감지 (포트별 위치 변화량 합)."""
    scores = {}
    for p in ports:
        try:
            la = FeetechLeader(p)
            la.open()
        except Exception:
            scores[p] = -1
            continue
        first = la.read_positions()
        t0 = time.time()
        moved = {i: 0 for i in first}
        while time.time() - t0 < sec:
            cur = la.read_positions()
            for i in cur:
                if i in first:
                    moved[i] = max(moved.get(i, 0), abs(cur[i] - first[i]))
            time.sleep(0.03)
        la.close()
        scores[p] = sum(moved.values())
    return scores

#!/usr/bin/env python3
"""사람 착용 드리프트 A/B 측정 (9축 vs 6축) — 리더암 불필요, UDP만 사용.

원리: 같은 중립자세로 돌아올 때마다 "기준 대비 얼마나 틀어져 있나"를 기록.
      참값(중립=기준 그대로)을 알기에 편차 = 순수 드리프트.

프로토콜:
  1. 앱 종료 후 실행 (UDP 4210 사용)
  2. 중립자세로 2초 정지 → 기준 자동 캡처
  3. [30초 자유롭게 휘젓기(텔레옵처럼)] → [중립 복귀, 2초 정지] 반복 5회+
  4. Ctrl+C → 채널별 드리프트(°/분) 요약

같은 절차를 6축/9축 펌웨어로 각각 실행해 비교 (IMU_USE_6AXIS 토글 후 재플래시).
"""
import json
import math
import socket
import time

from calib_ik import qmul, qconj, _rotvec

UDP_PORT = 4210
STILL_DEG_S = 4.0     # 이 미만 회전속도가 (사람 미세떨림 ~1-3°/s 감안)
STILL_SEC = 1.2       # 이 시간 지속되면 '정지'로 판정
MIN_GAP_SEC = 8.0     # 정지 이벤트 간 최소 간격


def ang(qa, qb):
    return math.degrees(math.hypot(*_rotvec(qmul(qconj(qa), qb))))


def read_quats(p):
    out = {}
    for k in ("imu1", "imu2", "imu3"):
        q = [p.get(f"{k}_q{i}") for i in range(4)]
        if None in q:
            return None
        n = math.hypot(*q)
        if not (0.5 < n < 2.0):
            return None
        out[k] = [v / n for v in q]
    return out


def rel(qa, qb):
    return qmul(qconj(qa), qb)


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", UDP_PORT))
    s.settimeout(1.0)
    print("UDP 수신 대기…")

    prev = None
    prev_t = None
    prev_ts = None
    still_since = None
    ref = None
    events = []   # (t, dev1, dev12, dev23)
    t0 = time.time()
    last_event_t = -1e9
    npkt = 0
    last_fb = time.time()
    cur_vel = -1.0
    print("중립자세로 2초 가만히 → 기준 캡처됩니다")
    try:
        while True:
            # 2초마다 진행상황 표시 (패킷 수신/현재 움직임 속도)
            if time.time() - last_fb > 2.0:
                last_fb = time.time()
                if npkt == 0:
                    print("  … 패킷 0개 — ESP 전원/와이파이 확인!")
                elif ref is None:
                    print(f"  … 수신 {npkt/2:.0f}Hz, 현재 움직임 {cur_vel:.1f}°/s "
                          f"({'정지로 인식중' if 0 <= cur_vel < STILL_DEG_S else f'>{STILL_DEG_S}°/s — 더 가만히!'})")
                npkt = 0
            try:
                d, _ = s.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                p = json.loads(d.decode())
                q = read_quats(p)
            except Exception:
                continue
            if q is None:
                continue
            npkt += 1
            now = time.time()
            ts = p.get("ts")  # ESP millis — 네트워크 버스트에 영향 없는 진짜 샘플 간격
            if prev is not None:
                if ts is not None and prev_ts is not None and 0 < (ts - prev_ts) < 2000:
                    dt = (ts - prev_ts) / 1000.0
                else:
                    dt = max(0.02, now - prev_t)  # 버스트 가드: 20ms 미만으로 안 봄
                vel = max(ang(prev[k], q[k]) for k in q) / dt
                cur_vel = vel
                if vel < STILL_DEG_S:
                    if still_since is None:
                        still_since = now
                    if now - still_since >= STILL_SEC:
                        if ref is None:
                            ref = {"q1": q["imu1"],
                                   "r12": rel(q["imu1"], q["imu2"]),
                                   "r23": rel(q["imu2"], q["imu3"])}
                            t0 = now
                            last_event_t = now
                            print(f"기준 캡처 완료! 이제 [30초 휘젓기 → 중립 2초 정지]를 반복하세요 (Ctrl+C로 종료)")
                        elif now - last_event_t >= MIN_GAP_SEC:
                            last_event_t = now
                            dev1 = ang(ref["q1"], q["imu1"])
                            dev12 = ang(ref["r12"], rel(q["imu1"], q["imu2"]))
                            dev23 = ang(ref["r23"], rel(q["imu2"], q["imu3"]))
                            t = now - t0
                            events.append((t, dev1, dev12, dev23))
                            print(f"  [{t:5.0f}s] 편차: 윗팔(절대) {dev1:5.1f}°  윗팔↔아랫팔 {dev12:5.1f}°  아랫팔↔손 {dev23:5.1f}°")
                else:
                    still_since = None
            prev, prev_t, prev_ts = q, now, ts
    except KeyboardInterrupt:
        pass
    if len(events) < 2:
        print("\n이벤트 부족 — 중립 정지를 더 자주(2초씩) 해주세요")
        return
    print(f"\n=== 요약 ({len(events)}회 중립 복귀, {events[-1][0]/60:.1f}분) ===")
    names = ["윗팔(절대)", "윗팔↔아랫팔(팔꿈치 채널)", "아랫팔↔손(손목 채널)"]
    for i, nm in enumerate(names, start=1):
        devs = [e[i] for e in events]
        # 시간-편차 기울기 (°/분)
        ts = [e[0] / 60 for e in events]
        mt = sum(ts) / len(ts); md = sum(devs) / len(devs)
        sxx = sum((t - mt) ** 2 for t in ts) or 1e-9
        slope = sum((t - mt) * (v - md) for t, v in zip(ts, devs)) / sxx
        print(f"  {nm}: 최종 {devs[-1]:.1f}°  최대 {max(devs):.1f}°  추세 {slope:+.1f}°/분")
    print("\n→ 6축/9축 각각 실행해 '추세(°/분)'를 비교하세요. 작은 쪽이 승자.")


if __name__ == "__main__":
    main()

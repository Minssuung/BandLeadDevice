#!/usr/bin/env python3
"""IMU 스트림 '툭툭 끊김' 원인 판별 프로브.

UDP 4210을 직접 수신하며 (PC 앱은 꺼야 함 — 포트 충돌):
  - 도착 간격(arr_gap)  : PC에 패킷이 도착한 시간 간격
  - 소스 간격(esp_gap)  : 패킷 안 ESP ts 필드의 간격 (ESP가 보낸 시점 기준)
  - STATUS의 poll_fail/csum_fail 증가량 (RS485 배선/노이즈 지표)
  - 값 동결(frozen)     : 패킷은 오는데 쿼터니언이 안 변하는 구간 (센서 스테일 지표)

판별 규칙(끊김 이벤트 = arr_gap > 80ms):
  esp_gap도 같이 큼            → ESP 쪽 멈칫 (폴링 타임아웃/전원/펌웨어 블로킹)
  esp_gap은 정상(~31ms)        → WiFi/UDP 버스트 (네트워크)
  poll_fail/csum_fail 증가 동반 → RS485 배선/접촉 (움직일 때만 늘면 거의 확정)
  frozen 동반                  → 센서 내부 스테일

사용: PC 제어 앱 종료 → python3 imu_gap_probe.py → 팔을 60초쯤 움직임(끊기던 동작 포함)
      → Ctrl+C → 요약 출력 복사.
"""

import json
import socket
import time

PORT = 4210
GAP_THRESH_MS = 80.0   # 이 이상 도착 공백이면 '끊김 이벤트'
NOMINAL_MS = 31.0      # 32Hz 기준 정상 간격

def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    sock.bind(("0.0.0.0", PORT))
    print(f"수신 대기: UDP {PORT} (Ctrl+C로 종료·요약)")

    arr_prev = None
    ts_prev = None
    q_prev = None
    arr_gaps = []
    events = []            # (시각, arr_gap, esp_gap, frozen직전여부)
    frozen_run = 0
    frozen_max = 0
    n_data = 0
    poll_fail_0 = csum_fail_0 = None
    poll_fail = csum_fail = None
    t_start = time.monotonic()
    t_report = t_start

    try:
        while True:
            try:
                payload, _ = sock.recvfrom(2048)
            except socket.timeout:
                if time.monotonic() - t_report >= 5.0:
                    t_report = time.monotonic()
                    print(f"[{t_report - t_start:5.0f}s] 패킷 없음 — ESP 전원/IP 확인", flush=True)
                continue
            now = time.monotonic()
            try:
                pkt = json.loads(payload.decode("utf-8", errors="replace"))
            except json.JSONDecodeError:
                continue

            if pkt.get("type") == "status":
                poll_fail = pkt.get("imu_poll_fail")
                csum_fail = pkt.get("imu_csum_fail")
                if poll_fail_0 is None:
                    poll_fail_0, csum_fail_0 = poll_fail, csum_fail
                continue

            ts = pkt.get("ts")
            if ts is None:
                continue
            n_data += 1
            if n_data == 1:
                print("첫 데이터 패킷 수신 OK — 측정 중 (5초마다 요약)", flush=True)

            # 값 동결 감지: 3개 IMU 쿼터니언 전부 직전과 동일하면 frozen
            q = tuple(pkt.get(k, 0.0) for k in (
                "imu1_q0", "imu1_q1", "imu1_q2", "imu1_q3",
                "imu2_q0", "imu2_q1", "imu2_q2", "imu2_q3",
                "imu3_q0", "imu3_q1", "imu3_q2", "imu3_q3"))
            if q_prev is not None and q == q_prev:
                frozen_run += 1
                frozen_max = max(frozen_max, frozen_run)
            else:
                frozen_run = 0
            q_prev = q

            if arr_prev is not None and ts_prev is not None:
                arr_gap = (now - arr_prev) * 1000.0
                esp_gap = float(ts - ts_prev)
                arr_gaps.append(arr_gap)
                if arr_gap > GAP_THRESH_MS:
                    events.append((now - t_start, arr_gap, esp_gap, frozen_run))
            arr_prev, ts_prev = now, ts

            if now - t_report >= 5.0:
                t_report = now
                rate = n_data / (now - t_start)
                recent = arr_gaps[-200:]
                mx = max(recent) if recent else 0.0
                pf = (poll_fail - poll_fail_0) if poll_fail is not None and poll_fail_0 is not None else "?"
                print(f"[{now - t_start:5.0f}s] rate={rate:5.1f}Hz  최근 최대공백={mx:5.0f}ms  "
                      f"끊김이벤트={len(events)}  poll_fail증가={pf}  frozen최장={frozen_max}연속")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    dur = time.monotonic() - t_start
    print("\n===== 요약 =====")
    if not arr_gaps:
        print("데이터 패킷 없음 — ESP 전원/네트워크 확인")
        return
    arr_sorted = sorted(arr_gaps)
    p50 = arr_sorted[len(arr_sorted) // 2]
    p95 = arr_sorted[int(len(arr_sorted) * 0.95)]
    p99 = arr_sorted[int(len(arr_sorted) * 0.99)]
    print(f"수신 {n_data}패킷 / {dur:.0f}초 = {n_data / dur:.1f}Hz")
    print(f"도착간격 p50={p50:.0f}ms p95={p95:.0f}ms p99={p99:.0f}ms 최대={arr_sorted[-1]:.0f}ms")
    if poll_fail is not None and poll_fail_0 is not None:
        print(f"poll_fail 증가={poll_fail - poll_fail_0}  csum_fail 증가={csum_fail - csum_fail_0}")
    print(f"값 동결 최장 {frozen_max}연속 패킷")
    print(f"\n끊김 이벤트(도착공백>{GAP_THRESH_MS:.0f}ms): {len(events)}건 — 최악 10건:")
    print("   시각     도착공백   ESP송신간격   판정")
    for t_ev, ag, eg, fr in sorted(events, key=lambda e: -e[1])[:10]:
        if eg > GAP_THRESH_MS * 0.7:
            verdict = "ESP 멈칫 (소스부터 공백)"
        elif eg <= NOMINAL_MS * 2.5:
            verdict = "WiFi/UDP 버스트 (ESP는 정상 송신)"
        else:
            verdict = "혼합/애매"
        if fr > 0:
            verdict += f" +값동결{fr}"
        print(f"  {t_ev:6.1f}s  {ag:7.0f}ms  {eg:9.0f}ms   {verdict}")

    n_esp = sum(1 for _, ag, eg, _ in events if eg > GAP_THRESH_MS * 0.7)
    n_net = sum(1 for _, ag, eg, _ in events if eg <= NOMINAL_MS * 2.5)
    print(f"\n총평: ESP쪽 {n_esp}건 / 네트워크쪽 {n_net}건 / 기타 {len(events) - n_esp - n_net}건")


if __name__ == "__main__":
    main()

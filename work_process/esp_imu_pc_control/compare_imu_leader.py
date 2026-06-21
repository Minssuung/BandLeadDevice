#!/usr/bin/env python3
"""IMU 파이프라인 vs 리더암(엔코더 기준기) 동시 로깅·정량 비교 도구.

지그 프로토콜: IMU 1/2/3을 리더암의 윗팔/아랫팔/손목 링크에 고정하고
리더암을 손으로 천천히 움직인다. 같은 강체를 두 시스템이 동시에 측정
→ 관절별 오차/게인/지연을 수치로 얻는다.

사용:
  1. 앱 종료 (UDP 4210과 리더 포트를 이 스크립트가 사용)
  2. python3 compare_imu_leader.py [리더포트]
  3. 안내에 따라 정지 → 기준 캡처 → 자유롭게 움직임 → Ctrl+C → 분석 출력
"""
import json
import math
import socket
import sys
import time

import calib_ik as ci
from calib_ik import CalibIK, qmul, qconj
from leader_arm import FeetechLeader

UDP_PORT = 4210
# 앱(esp_imu_pc_control3.py)과 동일한 값 — 변경 시 양쪽 같이 갱신할 것
IK_SIGN = {1: -1.0, 2: -1.0, 3: 1.0, 4: 1.0, 5: -1.0, 6: 1.0, 7: -1.0}
LEADER_SIGN = {1: -1.0, 2: 1.0, 3: -1.0, 4: 1.0, 5: -1.0, 6: 1.0, 7: -1.0}
import glob as _glob
LEADER_PORTS = tuple(sorted(_glob.glob("/dev/serial/by-id/usb-1a86_USB_Single_Serial_*"))) \
    or ("/dev/ttyACM0", "/dev/ttyACM1")
LOG_PATH = "/tmp/bandlead_compare.csv"


def read_quats(payload: dict):
    """UDP JSON → {imu_key: [w,x,y,z]} (정규화·유효성 검사 포함)"""
    out = {}
    for k in ("imu1", "imu2", "imu3"):
        vals = [payload.get(f"{k}_q{i}") for i in range(4)]
        if any(v is None for v in vals):
            return None
        q = [float(v) for v in vals]
        n = math.hypot(*q)
        if not (0.5 < n < 2.0):
            return None
        out[k] = [v / n for v in q]
    return out


def chain_qrel(cur: dict, ref: dict):
    """앱과 동일한 Parent Frame Kinematic Chain 상대 쿼터니언."""
    qrel = {"imu1": qmul(qconj(ref["imu1"]), cur["imu1"])}
    q2_lref = qmul(qconj(ref["imu1"]), ref["imu2"])
    q2_lcur = qmul(qconj(cur["imu1"]), cur["imu2"])
    qrel["imu2"] = qmul(qconj(q2_lref), q2_lcur)
    q3_lref = qmul(qconj(ref["imu2"]), ref["imu3"])
    q3_lcur = qmul(qconj(cur["imu2"]), cur["imu3"])
    qrel["imu3"] = qmul(qconj(q3_lref), q3_lcur)
    return qrel


def leader_deg(pos: dict, ref: dict):
    out = {}
    for mid in range(1, 8):
        p, r = pos.get(mid), ref.get(mid)
        if p is None or r is None:
            continue
        d = p - r
        if d > 2048:
            d -= 4096
        elif d < -2048:
            d += 4096
        out[mid] = d * 360.0 / 4096.0 * LEADER_SIGN.get(mid, 1.0)
    return out


def main():
    # 리더 연결
    ports = sys.argv[1:] or list(LEADER_PORTS)
    leader = None
    for p in ports:
        try:
            leader = FeetechLeader(p)
            leader.open()
            print(f"리더 연결: {p} (ID {leader.alive_ids})")
            break
        except Exception as exc:
            print(f"  {p}: {exc}")
    if leader is None:
        print("리더 연결 실패 — 포트 인자로 지정해보세요")
        return 1

    calib = CalibIK.load("calibration.json")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.5)
    print(f"UDP {UDP_PORT} 수신 대기… (앱이 켜져 있으면 끄세요)")

    def latest_quats(wait=2.0):
        """수신 버퍼 비우고 최신 패킷의 쿼터니언 반환"""
        t0 = time.time()
        q = None
        while time.time() - t0 < wait:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                q2 = read_quats(json.loads(data.decode()))
            except Exception:
                continue
            if q2:
                q = q2
                # 버퍼에 쌓인 것 마저 비우기
                sock.settimeout(0.01)
                try:
                    while True:
                        data, _ = sock.recvfrom(2048)
                        q3 = read_quats(json.loads(data.decode()))
                        if q3:
                            q = q3
                except socket.timeout:
                    pass
                sock.settimeout(0.5)
                return q
        return q

    if latest_quats() is None:
        print("IMU 패킷 없음 — ESP 전원/네트워크 확인")
        return 1
    print("IMU 수신 OK")

    print("\n>>> 기준 캡처: 리더암을 기준 자세로 두고 3초간 만지지 마세요…")
    time.sleep(3.0)
    ref_q = latest_quats()
    ref_l = leader.read_positions()
    if ref_q is None or len(ref_l) < 7:
        print("기준 캡처 실패")
        return 1
    print("기준 캡처 완료. 이제 리더암을 천천히 움직이세요 (여러 관절, 다양한 각도).")
    print(f"기록 중 → {LOG_PATH}   (끝나면 Ctrl+C)\n")

    rows = []
    f = open(LOG_PATH, "w", encoding="utf-8")
    f.write("t," + ",".join(f"imu_j{i}" for i in range(1, 8)) + ","
            + ",".join(f"led_j{i}" for i in range(1, 8)) + ","
            + ",".join(f"{k}_q{c}" for k in ("imu1", "imu2", "imu3") for c in "wxyz")
            + ",led_g8\n")
    t_start = time.time()
    sock.settimeout(0.5)
    try:
        while True:
            # 경량 캡처: 패킷 하나당 한 행 (드레인/분해 없음 — 분해는 오프라인 분석에서)
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                q = read_quats(json.loads(data.decode()))
            except Exception:
                continue
            if q is None:
                continue
            lp = leader.read_positions()   # ~2.5ms
            if len(lp) < 7:
                continue
            qrel = chain_qrel(q, ref_q)
            led_j = leader_deg(lp, ref_l)
            t = time.time() - t_start
            rows.append((t, {m: 0.0 for m in range(1, 8)}, led_j))
            f.write(f"{t:.3f},"
                    + ",".join("0.00" for _ in range(7)) + ","
                    + ",".join(f"{led_j.get(i, 0.0):.2f}" for i in range(1, 8)) + ","
                    + ",".join(f"{qrel[k][c]:.5f}" for k in ("imu1", "imu2", "imu3") for c in range(4))
                    + f",{lp.get(8, 0)}\n")
            if len(rows) % 150 == 0:
                a = rows[-1]
                print(f"  {a[0]:6.1f}s  {len(rows)/a[0]:.0f}Hz  led_j1={a[2].get(1, 0):+7.1f} led_j5={a[2].get(5, 0):+7.1f}")
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        leader.close()

    # ── 분석 ────────────────────────────────────────────────────────────
    n = len(rows)
    print(f"\n=== 분석 ({n}샘플, {rows[-1][0]:.0f}초) ===")
    if n < 50:
        print("샘플 부족 — 더 길게 기록하세요")
        return 0
    print(f"{'관절':4} {'동작범위':>9} {'RMS오차':>8} {'최대오차':>8} {'게인a':>7} {'오프셋b':>8} {'상관':>6}")
    for j in range(1, 8):
        xs = [r[1][j] for r in rows]              # IMU
        ys = [r[2].get(j, 0.0) for r in rows]     # 리더(기준)
        rng = max(ys) - min(ys)
        if rng < 5.0:
            print(f"  j{j}: 동작범위 {rng:.1f}° < 5° → 스킵(움직임 부족)")
            continue
        errs = [x - y for x, y in zip(xs, ys)]
        rms = math.sqrt(sum(e * e for e in errs) / n)
        mx = max(abs(e) for e in errs)
        # 회귀 y = a·x + b  (리더 = a×IMU + b)
        mx_, my_ = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx_) ** 2 for x in xs) or 1e-9
        sxy = sum((x - mx_) * (y - my_) for x, y in zip(xs, ys))
        a = sxy / sxx
        b = my_ - a * mx_
        syy = sum((y - my_) ** 2 for y in ys) or 1e-9
        corr = sxy / math.sqrt(sxx * syy)
        print(f"  j{j}: {rng:8.1f}° {rms:7.2f}° {mx:7.2f}° {a:7.3f} {b:+7.2f}° {corr:6.3f}")
    # 지연 추정: 가장 많이 움직인 관절로 교차상관
    best_j = max(range(1, 8), key=lambda j: max(r[2].get(j, 0.0) for r in rows)
                 - min(r[2].get(j, 0.0) for r in rows))
    xs = [r[1][best_j] for r in rows]
    ys = [r[2].get(best_j, 0.0) for r in rows]
    dt = rows[-1][0] / max(1, n - 1)
    best_lag, best_c = 0, -2.0
    for lag in range(0, min(30, n // 4)):          # IMU가 리더보다 늦는 쪽만
        x2 = xs[lag:]
        y2 = ys[:len(x2)]
        m1, m2 = sum(x2) / len(x2), sum(y2) / len(y2)
        num = sum((p - m1) * (q - m2) for p, q in zip(x2, y2))
        den = math.sqrt(sum((p - m1) ** 2 for p in x2) * sum((q - m2) ** 2 for q in y2)) or 1e-9
        c = num / den
        if c > best_c:
            best_c, best_lag = c, lag
    print(f"\n지연(교차상관, j{best_j} 기준): IMU가 리더보다 약 {best_lag * dt * 1000:.0f}ms 늦음 (상관 {best_c:.3f})")
    print(f"\n해석: 게인a≈1, 오프셋b≈0, 상관≈1 이 이상적. a<1 = 언더리드(게인보정 후보).")
    print(f"CSV: {LOG_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

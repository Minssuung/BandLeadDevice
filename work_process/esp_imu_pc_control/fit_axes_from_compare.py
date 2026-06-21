#!/usr/bin/env python3
"""compare 캡처(쿼터니언 포함)에서 리더 엔코더를 '정답 라벨'로 써서
IMU 보정축을 자동 추출하고, 그 축으로 재분해해 파이프라인 순수 성능을 평가.

입력: /tmp/bandlead_compare.csv (compare_imu_leader.py 확장판이 생성)
출력: 축별 적합 결과 + calibration_jig.json + 재평가표(RMS/게인/상관/지연)
"""
import csv
import json
import math
import sys

import calib_ik as ci
from calib_ik import qmul, qconj, _rotvec, _norm

CSV = sys.argv[1] if len(sys.argv) > 1 else "/tmp/bandlead_compare.csv"

# 축 슬롯 ↔ 리더 관절 매핑.
# ⚠ 분해 순서는 리더의 '물리 체인 순서'를 따라야 함: 어깨 j1→j2→j3, 손목 j5→j6→j7.
#   (순서가 틀리면 작은 각도에선 멀쩡해 보이다가 큰 각도에서 절대각이 완전히 어긋남)
SLOTS = [
    ("imu1", "a", 1), ("imu1", "b", 2), ("imu1", "c", 3),
    ("imu2", "a", 4),
    ("imu3", "a", 5), ("imu3", "b", 6), ("imu3", "c", 7),
]
IMU_JOINTS = {"imu1": (1, 2, 3), "imu2": (4,), "imu3": (5, 6, 7)}


def load(path):
    rows = []
    with open(path) as f:
        r = csv.reader(f)
        head = next(r)
        if not any(c.startswith("imu1_qw") for c in head):
            print("이 CSV엔 쿼터니언이 없습니다 — 확장판 compare로 다시 캡처하세요.")
            sys.exit(1)
        for line in r:
            v = [float(x) for x in line]
            t = v[0]
            led = {j: v[7 + j] for j in range(1, 8)}          # led_j1..7 = v[8..14]
            led = {j: v[7 + j] for j in range(1, 8)}
            q = {}
            base = 15
            for k_i, k in enumerate(("imu1", "imu2", "imu3")):
                q[k] = v[base + 4 * k_i: base + 4 * k_i + 4]
            rows.append((t, led, q))
    return rows


def main():
    rows = load(CSV)
    n = len(rows)
    print(f"{CSV}: {n}샘플, {rows[-1][0]:.0f}초")

    # ── 1) 프레임간 델타에서 축 샘플 수집 (리더 라벨로 분리) ──────────────
    fitted = {}
    for imu, slot, joint in SLOTS:
        others = [j for j in IMU_JOINTS[imu] if j != joint]
        acc = [0.0, 0.0, 0.0]
        ratios = []
        used = 0
        for i in range(1, n):
            dled = rows[i][1][joint] - rows[i - 1][1][joint]
            if abs(dled) < 0.15:
                continue
            if any(abs(rows[i][1][o] - rows[i - 1][1][o]) > 0.35 * abs(dled) for o in others):
                continue  # 다른 관절이 같이 움직임 → 오염 프레임 제외
            dq = qmul(qconj(rows[i - 1][2][imu]), rows[i][2][imu])
            rv = _rotvec(dq)
            ang = math.degrees(math.hypot(*rv))
            if ang < 0.08:
                continue
            s = 1.0 if dled > 0 else -1.0
            ax = _norm(rv)
            acc[0] += s * ax[0]; acc[1] += s * ax[1]; acc[2] += s * ax[2]
            ratios.append(ang / abs(dled))
            used += 1
        if used < 15:
            print(f"  {imu}.{slot}(j{joint}): 샘플 {used}<15 — 동작 부족, 스킵")
            continue
        axis = _norm(acc)
        coh = math.hypot(*acc) / used          # 1=완벽히 일관된 축
        ratio = sorted(ratios)[len(ratios) // 2]
        fitted[(imu, slot)] = axis
        print(f"  {imu}.{slot}(j{joint}): 샘플 {used:4d}  축일관성 {coh:.2f}  회전/관절비 {ratio:.2f} (1=동일관절)")

    need = {("imu1", "a"), ("imu1", "b"), ("imu2", "a"), ("imu3", "a"), ("imu3", "b")}
    if not need <= set(fitted):
        print("\n필수 축이 부족합니다 — 부족한 관절을 따로따로 더 움직여 재캡처하세요.")
        sys.exit(1)

    axes = {}
    for (imu, slot), ax in fitted.items():
        axes.setdefault(imu, {})[slot] = ax
    with open("calibration_jig.json", "w") as f:
        json.dump({"axes": axes}, f, indent=1)
    print("\n→ calibration_jig.json 저장 (지그 전용 — 사람용 calibration.json은 그대로)")

    # ── 2) 적합 축으로 전체 재분해 → 순수 파이프라인 평가 ────────────────
    #   분해 순서 = 리더 물리 체인 순서 (어깨 j1→j2→j3, 손목 j5→j6→j7)
    A1, A2, A3 = axes.get("imu1", {}), axes.get("imu2", {}), axes.get("imu3", {})
    series_imu = {j: [] for j in range(1, 8)}
    series_led = {j: [] for j in range(1, 8)}
    for t, led, q in rows:
        md = {}
        if all(k in A1 for k in "abc"):
            s = ci.decompose(q["imu1"], A1["a"], A1["b"], A1["c"])
            md[1], md[2], md[3] = (math.degrees(x) for x in s)
        if "a" in A2:
            md[4] = math.degrees(ci.twist_angle(ci._canon(q["imu2"]), A2["a"]))
        if all(k in A3 for k in "abc"):
            w = ci.decompose(q["imu3"], A3["a"], A3["b"], A3["c"])
            md[5], md[6], md[7] = (math.degrees(x) for x in w)
        for j in range(1, 8):
            series_imu[j].append(md.get(j, 0.0))
            series_led[j].append(led[j])

    print("\n=== 재평가 (지그 자동보정 축, IK_SIGN 불필요 — 축에 부호 내장) ===")
    print(f"{'관절':4} {'동작범위':>9} {'RMS오차':>8} {'최대':>7} {'게인a':>7} {'오프셋b':>8} {'상관':>6}")
    for j in range(1, 8):
        xs, ys = series_imu[j], series_led[j]
        rng = max(ys) - min(ys)
        if rng < 5:
            print(f"  j{j}: 동작범위 {rng:.1f}°<5° 스킵")
            continue
        errs = [x - y for x, y in zip(xs, ys)]
        rms = math.sqrt(sum(e * e for e in errs) / n)
        mxe = max(abs(e) for e in errs)
        mx_, my_ = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx_) ** 2 for x in xs) or 1e-9
        sxy = sum((x - mx_) * (y - my_) for x, y in zip(xs, ys))
        syy = sum((y - my_) ** 2 for y in ys) or 1e-9
        a = sxy / sxx; b = my_ - a * mx_
        corr = sxy / math.sqrt(sxx * syy)
        print(f"  j{j}: {rng:8.1f}° {rms:7.2f}° {mxe:6.1f}° {a:7.3f} {b:+7.2f}° {corr:6.3f}")

    best_j = max(range(1, 8), key=lambda j: max(series_led[j]) - min(series_led[j]))
    xs, ys = series_imu[best_j], series_led[best_j]
    dt = rows[-1][0] / max(1, n - 1)
    best_lag, best_c, c0 = 0, -2.0, None
    for lag in range(0, min(40, n // 4)):
        x2 = xs[lag:]; y2 = ys[:len(x2)]
        m1, m2 = sum(x2) / len(x2), sum(y2) / len(y2)
        num = sum((p - m1) * (qv - m2) for p, qv in zip(x2, y2))
        den = math.sqrt(sum((p - m1) ** 2 for p in x2) * sum((qv - m2) ** 2 for qv in y2)) or 1e-9
        c = num / den
        if lag == 0:
            c0 = c
        if c > best_c:
            best_c, best_lag = c, lag
    print(f"\n지연(j{best_j}): {best_lag * dt * 1000:.0f}ms (상관 {c0:.3f}→{best_c:.3f})")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""시뮬(viz OpenArm) 운동학 검증 보고서 생성기.

bridge.py 가 기록한 sim_logs/*.csv 를 분석 → 마크다운 보고서 + SVG 그래프.
실물 위험 0으로 "매핑·부호·작업공간·연속성"을 검증 (안전·동역학은 실물/ROS2 영역).

검증 항목:
  - 작업공간: 각 관절 적용각 범위 vs OpenArm URDF 관절한계 (초과 = 사람>로봇)
  - 교차오염: 'still'/동작 마크 구간에서 의도 외 관절 누수 (순수 동작 분리도)
  - 연속성:  프레임간 점프(발산/가지점프 백스톱 확인)
  - FK잔차:  표현불가(보정축 커버리지 구멍)/특이 감지

사용: python3 sim_report.py sim_logs/*.csv [--out 보고서.md]
"""
import csv
import io
import json
import math
import os
import sys

# OpenArm v2.0 openarm_right_joint1~7 한계 (output.urdf 추출, deg)
OPENARM_LIMITS = {1: (-80, 200), 2: (-10, 190), 3: (-90, 90),
                  4: (0, 140), 5: (-90, 90), 6: (-45, 45), 7: (-45, 45)}
JOINT_NAME = {1: "어깨1", 2: "어깨2", 3: "어깨비틀", 4: "팔꿈치",
              5: "손목롤", 6: "손목J6", 7: "손목J7"}
JUMP_DEG = 30.0      # 프레임간 점프 임계(발산/가지점프 의심)
RES_WARN = 5.0       # FK잔차 경고(표현불가/특이)


def load(path):
    meta, raw = {}, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition("=")
                meta[k.strip()] = v.strip()
            elif line.strip():
                raw.append(line)
    data = list(csv.DictReader(io.StringIO("".join(raw))))
    return meta, data


def applied(meta, data):
    """raw 관절각 → OpenArm 실제 적용각 (sign×map). 반환 (t, J{1..7}, res, marks)."""
    sign = json.loads(meta.get("sign", "{}") or "{}")
    jmap = json.loads(meta.get("map", "{}") or "{}")
    t, res1, res3, marks = [], [], [], []
    J = {i: [] for i in range(1, 8)}
    for r in data:
        ev = r.get("ev", "")
        if ev.startswith("MARK:"):
            marks.append((float(r["t"]), ev[5:]))
            continue
        try:
            tt = float(r["t"])
        except (ValueError, KeyError):
            continue
        t.append(tt)
        for i in range(1, 8):
            src = int(jmap.get(str(i), i))           # OpenArm jointi ← raw[src]
            rawv = r.get(f"j{src}_raw", "")
            s = float(sign.get(str(i), 1) or 1)
            J[i].append(float(rawv) * s if rawv not in ("", None) else 0.0)
        res1.append(float(r.get("res_imu1", 0) or 0))
        res3.append(float(r.get("res_imu3", 0) or 0))
    return t, J, res1, res3, marks


def workspace(J):
    out = {}
    for i in range(1, 8):
        v = J[i]
        if not v:
            continue
        lo, hi = min(v), max(v)
        llo, lhi = OPENARM_LIMITS[i]
        over = sum(1 for x in v if x < llo or x > lhi)
        out[i] = {"min": lo, "max": hi, "limit": (llo, lhi),
                  "over_pct": 100.0 * over / len(v), "used_pct": 100.0 * (hi - lo) / (lhi - llo)}
    return out


def continuity(t, J):
    out = {}
    for i in range(1, 8):
        v, jumps, mx = J[i], 0, 0.0
        for k in range(1, len(v)):
            d = abs(v[k] - v[k - 1])
            mx = max(mx, d)
            if d > JUMP_DEG:
                jumps += 1
        out[i] = {"jumps": jumps, "max_step": mx}
    return out


def cross_talk(t, J, marks):
    """동작 마크 쌍 구간에서, 가장 크게 움직인 관절(=의도) 대비 다른 관절 누수 비율."""
    segs = []
    i = 0
    ms = sorted(marks)
    while i + 1 < len(ms):
        (t0, l0), (t1, _) = ms[i], ms[i + 1]
        segs.append((t0, t1, l0)); i += 2
    rows = []
    for t0, t1, lab in segs:
        idx = [k for k, tt in enumerate(t) if t0 <= tt <= t1]
        if len(idx) < 3:
            continue
        amp = {i: max(J[i][k] for k in idx) - min(J[i][k] for k in idx) for i in range(1, 8)}
        intended = max(amp, key=amp.get)
        main = amp[intended]
        leak = {i: (100.0 * amp[i] / main if main > 1e-6 else 0.0)
                for i in range(1, 8) if i != intended}
        worst = max(leak.items(), key=lambda kv: kv[1]) if leak else (None, 0.0)
        rows.append({"label": lab, "intended": intended, "main_amp": main,
                     "worst_leak_j": worst[0], "worst_leak_pct": worst[1]})
    return rows


def svg_timeseries(t, J, path, title=""):
    if not t:
        return None
    W, H, PAD = 920, 360, 44
    x0, y0 = PAD, PAD
    w, h = W - 2 * PAD, H - 2 * PAD
    tmin, tmax = t[0], t[-1] or 1.0
    allv = [x for i in range(1, 8) for x in J[i]] or [0]
    vmin, vmax = min(allv), max(allv)
    if vmax - vmin < 1:
        vmax = vmin + 1
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#17becf"]

    def X(tt):
        return x0 + w * (tt - tmin) / (tmax - tmin or 1)

    def Y(v):
        return y0 + h * (1 - (v - vmin) / (vmax - vmin))

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'style="background:#fff;font-family:sans-serif">']
    parts.append(f'<text x="{x0}" y="20" font-size="14" font-weight="bold">{title}</text>')
    # 0° 기준선
    parts.append(f'<line x1="{x0}" y1="{Y(0):.0f}" x2="{x0+w}" y2="{Y(0):.0f}" '
                 f'stroke="#ccc" stroke-dasharray="3"/>')
    parts.append(f'<text x="{x0-4}" y="{Y(0):.0f}" font-size="10" text-anchor="end">0°</text>')
    for i in range(1, 8):
        pts = " ".join(f"{X(t[k]):.1f},{Y(J[i][k]):.1f}" for k in range(len(t)))
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{colors[i-1]}" stroke-width="1.4"/>')
        parts.append(f'<text x="{x0+w+4}" y="{Y(J[i][-1]):.1f}" font-size="10" '
                     f'fill="{colors[i-1]}">j{i}</text>')
    parts.append(f'<text x="{x0}" y="{H-8}" font-size="10">t={tmin:.0f}~{tmax:.0f}s '
                 f'/ {vmin:.0f}~{vmax:.0f}°</text>')
    parts.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    return path


def analyze(path, svg_dir):
    meta, data = load(path)
    t, J, res1, res3, marks = applied(meta, data)
    if not t:
        return f"## {os.path.basename(path)}\n\n> 데이터 행 없음\n"
    ws = workspace(J)
    cont = continuity(t, J)
    ct = cross_talk(t, J, marks)
    res_max = max(max(res1, default=0), max(res3, default=0))
    res_warn_pct = 100.0 * sum(1 for a, b in zip(res1, res3) if max(a, b) > RES_WARN) / len(t)
    base = os.path.splitext(os.path.basename(path))[0]
    svg = svg_timeseries(t, J, os.path.join(svg_dir, base + ".svg"),
                         title=f"{meta.get('scenario','?')} — OpenArm 적용각(deg)")

    L = [f"## {os.path.basename(path)}", "",
         f"- 시나리오: **{meta.get('scenario','?')}** | 길이: {t[-1]:.0f}s / {len(t)}프레임"
         f" | 손목매핑: {meta.get('map','?')} | 부호: {meta.get('sign','?')}", ""]

    L += ["### 작업공간 (OpenArm 관절한계 대비)", "",
          "| 관절 | 적용범위(°) | URDF 한계(°) | 사용률 | 초과 |", "|---|---|---|---|---|"]
    for i in range(1, 8):
        if i not in ws:
            continue
        w = ws[i]
        flag = f"⚠ {w['over_pct']:.0f}%" if w["over_pct"] > 0.5 else "✅"
        L.append(f"| j{i} {JOINT_NAME[i]} | {w['min']:+.0f}~{w['max']:+.0f} "
                 f"| {w['limit'][0]}~{w['limit'][1]} | {w['used_pct']:.0f}% | {flag} |")
    over_js = [i for i in ws if ws[i]["over_pct"] > 0.5]
    if over_js:
        L += ["", f"> ⚠ 작업공간 초과 관절: {', '.join(f'j{i}' for i in over_js)}"
                  f" — 사람 동작범위 > OpenArm 관절한계 (클램프/동작 제한 필요)"]

    if ct:
        L += ["", "### 교차오염 (동작 마크 구간 — 순수 동작 분리도)", "",
              "| 구간 | 의도 관절 | 진폭(°) | 최대 누수 | 누수율 |", "|---|---|---|---|---|"]
        for r in ct:
            wl = f"j{r['worst_leak_j']} {r['worst_leak_pct']:.0f}%" if r["worst_leak_j"] else "-"
            flag = " ⚠" if r["worst_leak_pct"] > 15 else ""
            L.append(f"| {r['label']} | j{r['intended']} {JOINT_NAME[r['intended']]} "
                     f"| {r['main_amp']:.0f} | {wl}{flag} | {r['worst_leak_pct']:.0f}% |")
        L += ["", "> 누수율 = 의도 외 관절 진폭/의도 관절 진폭. 15%↑면 매핑·보정축 교차오염 의심."]

    L += ["", "### 연속성 · FK잔차", "",
          f"- 프레임간 점프(>{JUMP_DEG:.0f}°): "
          + ", ".join(f"j{i}={cont[i]['jumps']}" for i in range(1, 8) if cont[i]["jumps"] > 0) or "- 점프 0건 (발산/가지점프 없음) ✅",
          f"- 최대 프레임 스텝: " + " ".join(f"j{i}={cont[i]['max_step']:.0f}°" for i in range(1, 8)),
          f"- FK잔차 최대 {res_max:.1f}° / >{RES_WARN:.0f}° 비율 {res_warn_pct:.1f}% "
          + ("⚠ 표현불가(보정축 커버리지 구멍/특이 자세) 존재" if res_warn_pct > 2 else "✅")]
    if svg:
        L += ["", f"![timeseries]({os.path.basename(svg)})"]
    L.append("")
    return "\n".join(L)


def main():
    args, out, skip = [], None, False
    for a in sys.argv[1:]:
        if skip:
            out = a; skip = False; continue
        if a == "--out":
            skip = True; continue
        if a.startswith("--"):
            continue
        args.append(a)
    if not args:
        print("사용: python3 sim_report.py sim_logs/*.csv [--out 보고서.md]")
        return 1
    out = out or os.path.join(os.path.dirname(os.path.abspath(args[0])), "..",
                              "시뮬검증보고서.md")
    svg_dir = os.path.dirname(os.path.abspath(out))
    os.makedirs(svg_dir, exist_ok=True)
    import time as _t
    head = ["# 시뮬(viz OpenArm) 운동학 검증 보고서", "",
            f"- 생성: {_t.strftime('%Y-%m-%d %H:%M')}",
            "- 분해: bridge.py + calib_ik.py (실물 esp 앱과 동일)",
            "- 검증 범위: 매핑·부호·작업공간·연속성 (안전·동역학은 ROS2/실물 영역)", ""]
    body = [analyze(p, svg_dir) for p in args]
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(head + body))
    print(f"보고서: {out}")
    for p in args:
        print(f"  분석: {os.path.basename(p)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

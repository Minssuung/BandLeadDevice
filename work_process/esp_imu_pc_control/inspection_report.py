#!/usr/bin/env python3
"""검사 보고서 생성기 — inspection_recorder CSV → KPI 산출 → 합부 판정 → 마크다운 보고서.

블랙박스 원칙: 입력(L=리더 기준기 관절각)과 출력(P=팔로워 실제 관절각)만으로 판정한다.
내부 신호(C=파이프라인 목표, G=전송 목표)는 원인 분석 참고로만 표에 병기한다.

사용:
  python inspection_report.py inspection_logs/*.csv [--criteria inspection_criteria.json]
                              [--out 검사보고서.md] [--no-svg]
  python inspection_report.py --selftest     # 합성 데이터로 측정기 자체 검증(검교정)

KPI 정의(검사기준서_미니팔로우암 문서와 1:1):
  K1 지연 latency_ms      : 입력→출력 교차상관 최대 지연(속도 신호, 포물선 보간)
  K2 추종 track_rms_deg   : 지연 보상 후 |P−L| (상수 정렬오프셋 제거) RMS / P95 / 최대
  K3 스케일 gain          : P ≈ a·L + b 최소제곱 기울기 (1.0 = 충실)
  K4 정지 wander_deg      : 입력 정지(≥2s) 중 출력 위치 피크-피크
  K5 드리프트 drift_dpm   : 정지 구간별 오차 중앙값의 시간 추세 (°/min)
  K6 반복 repeat_deg      : 기준 포즈 재방문 시 출력 산포 (max−min)
  K7 안전정지 stop_s      : 마크(입력차단) → 출력 속도 <2°/s 0.5s 지속까지
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

FS = 20.0                 # 분석 리샘플 주파수(Hz) = 기록기 틱과 동일
STILL_VEL = 1.5           # 입력 정지 판정 속도(°/s)
STILL_MIN_S = 2.0         # 정지 구간 최소 길이(s)
ACTIVE_RANGE_DEG = 5.0    # '검사 대상 관절' 판정: 입력 가동폭
LAG_MAX_S = 1.5
STOP_VEL = 2.0            # 출력 정지 판정(°/s)
STOP_HOLD_S = 0.5
REPEAT_TOL_DEG = 2.5      # 기준 포즈 재방문 허용 반경

GRADE_ORDER = ["A", "B", "C", "F"]


# ──────────────────────────────────────────────────────────────────────────
# CSV 파싱
# ──────────────────────────────────────────────────────────────────────────
class Capture:
    def __init__(self, path: str):
        self.path = path
        self.meta: Dict[str, object] = {}
        self.marks: List[Tuple[float, str]] = []
        self.joints: List[int] = []
        self.t = np.zeros(0)
        self.L: Dict[int, np.ndarray] = {}
        self.C: Dict[int, np.ndarray] = {}
        self.G: Dict[int, np.ndarray] = {}
        self.P: Dict[int, np.ndarray] = {}
        self.valid: Dict[str, float] = {}  # 채널별 '원시' 유효표본 비율(보간 전)
        self.read_fail_ticks = 0
        self._parse(path)

    def _parse(self, path: str) -> None:
        header: Optional[List[str]] = None
        rows: List[List[str]] = []
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                line = line.rstrip("\n")
                if not line:
                    continue
                if line.startswith("#"):
                    body = line[1:].strip()
                    if "=" in body:
                        k, v = body.split("=", 1)
                        try:
                            self.meta[k.strip()] = json.loads(v)
                        except json.JSONDecodeError:
                            self.meta[k.strip()] = v
                    continue
                if header is None:
                    header = line.split(",")
                    continue
                parts = line.split(",")
                if len(parts) < 2:
                    continue
                if parts[1].startswith("MARK:"):
                    self.marks.append((float(parts[0]), parts[1][5:]))
                    continue
                rows.append(parts)
        if header is None or not rows:
            raise ValueError(f"{path}: 데이터 행 없음")

        idx = {name: i for i, name in enumerate(header)}
        self.joints = sorted(int(c[1:-4]) for c in header if c.startswith("L") and c.endswith("_deg"))

        def col(name: str) -> np.ndarray:
            i = idx[name]
            out = np.full(len(rows), np.nan)
            for r, parts in enumerate(rows):
                if i < len(parts) and parts[i]:
                    try:
                        out[r] = float(parts[i])
                    except ValueError:
                        pass
            return out

        t_raw = col("t")
        ok = np.isfinite(t_raw)
        t_raw = t_raw[ok]
        order = np.argsort(t_raw)
        t_raw = t_raw[order]

        # 균일 20Hz 그리드 리샘플 (유효 표본만으로 보간)
        self.t = np.arange(0.0, float(t_raw[-1]), 1.0 / FS)

        def resample(name: str) -> np.ndarray:
            raw = col(name)[ok][order]
            good = np.isfinite(raw)
            # ⚠ np.interp가 결손을 전부 메우므로 '보간 후 finite 비율'은 항상 1.0이 된다.
            #   판정 게이트는 반드시 이 '원시' 비율(self.valid)을 써야 함 (FC 치명-2).
            self.valid[name] = float(good.mean()) if len(raw) else 0.0
            if good.sum() < max(10, 0.2 * len(raw)):
                return np.full_like(self.t, np.nan)
            return np.interp(self.t, t_raw[good], raw[good])

        for j in self.joints:
            self.L[j] = resample(f"L{j}_deg")
            self.C[j] = resample(f"C{j}_deg")
            self.G[j] = resample(f"G{j}_deg")
            self.P[j] = resample(f"P{j}_deg")
        rf = col("read_fail")[ok][order]
        self.read_fail_ticks = int(np.nansum(rf > 0))

    def vfrac(self, prefix: str, j: int) -> float:
        return self.valid.get(f"{prefix}{j}_deg", 0.0)

    @property
    def has_input(self) -> bool:
        return any(self.vfrac("L", j) > 0.8 for j in self.joints)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0


# ──────────────────────────────────────────────────────────────────────────
# 신호 유틸
# ──────────────────────────────────────────────────────────────────────────
def smooth(x: np.ndarray, n: int = 5) -> np.ndarray:
    if len(x) < n:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def vel(x: np.ndarray) -> np.ndarray:
    return smooth(np.gradient(x, 1.0 / FS))


def xcorr_lag(inp: np.ndarray, out: np.ndarray) -> Tuple[Optional[float], float]:
    """입력→출력 지연(s). 속도 신호 정규화 교차상관 + 포물선 보간. (lag, peak_corr)

    음의 구간(-0.5s)도 탐색한다 — 출력이 입력을 앞서는 건 물리적으로 불가하므로
    음의 피크는 계측 이상 신호이며, 0에 클램프해 거짓 A를 만들면 안 됨 (FC 중요-4).
    """
    vi, vo = vel(inp), vel(out)
    vi = vi - vi.mean()
    vo = vo - vo.mean()
    si, so = vi.std(), vo.std()
    if si < 1e-6 or so < 1e-6:
        return None, 0.0
    neg = int(0.5 * FS)
    max_lag = int(LAG_MAX_S * FS)
    lags = list(range(-neg, max_lag + 1))
    corrs = np.full(len(lags), -1.0)
    n = len(vi)
    for li, lag in enumerate(lags):
        if lag >= 0:
            a, b = vi[: n - lag] if lag else vi, vo[lag:]
        else:
            a, b = vi[-lag:], vo[: n + lag]
        if len(a) > FS:
            corrs[li] = float(np.dot(a, b) / (len(a) * si * so))
    k = int(np.argmax(corrs))
    peak = float(corrs[k])
    if peak < 0.5:
        return None, peak
    # 포물선 보간으로 표본 이하 해상도
    if 0 < k < len(lags) - 1:
        y0, y1, y2 = corrs[k - 1], corrs[k], corrs[k + 1]
        denom = (y0 - 2 * y1 + y2)
        frac = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
        frac = max(-0.5, min(0.5, frac))
    else:
        frac = 0.0
    return (lags[k] + frac) / FS, peak


def shift_back(x: np.ndarray, lag_s: float) -> np.ndarray:
    """출력을 lag만큼 앞으로 당겨 입력과 시간 정렬."""
    lag = lag_s * FS
    idx = np.arange(len(x)) + lag
    return np.interp(np.arange(len(x), dtype=float), np.arange(len(x), dtype=float),
                     x) if lag == 0 else np.interp(idx, np.arange(len(x), dtype=float), x,
                                                   left=np.nan, right=np.nan)


def still_segments(L: Dict[int, np.ndarray], joints: List[int], t: np.ndarray
                   ) -> List[Tuple[int, int]]:
    """모든 검사 관절의 입력이 STILL_VEL 미만으로 STILL_MIN_S 이상 유지되는 구간."""
    if not joints:
        return []
    still = np.ones(len(t), dtype=bool)
    for j in joints:
        still &= np.abs(vel(L[j])) < STILL_VEL
    segs: List[Tuple[int, int]] = []
    start = None
    for i, s in enumerate(still):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if (i - start) / FS >= STILL_MIN_S:
                segs.append((start, i))
            start = None
    if start is not None and (len(t) - start) / FS >= STILL_MIN_S:
        segs.append((start, len(t)))
    return segs


DRIFT_MIN_SPAN_S = 60.0   # 드리프트 회귀 최소 시간 span (leader 경로와 동일 — 군집 외삽 방지)
DRIFT_MIN_PTS = 4         # 드리프트 회귀 최소 정지구간 수 (3점은 잔차 0이라 신뢰 불가)


def _still_pairs(marks: List[Tuple[float, str]]) -> List[Tuple[float, float]]:
    """정지구간 경계 마크를 (시작,끝) 쌍으로. 라벨로 start/end 구분 (FC-A 수정):
    'still_start'/'still_end' 명시 라벨이 있으면 그것으로 페어링(위상 어긋남 불가),
    없으면 'still' 토글로 간주해 짝수 인덱스=시작. 짧은(<STILL_MIN_S) 쌍은 폐기."""
    sm = sorted((t, lab.lower()) for t, lab in marks if "still" in lab.lower())
    has_se = any(("start" in l or "end" in l) for _, l in sm)
    pairs: List[Tuple[float, float]] = []
    if has_se:
        # 명시 라벨: 가장 최근 start 이후 첫 end와 페어링 (스택)
        open_t: Optional[float] = None
        for t, l in sm:
            if "start" in l:
                open_t = t
            elif "end" in l and open_t is not None:
                if t - open_t >= STILL_MIN_S:
                    pairs.append((open_t, t))
                open_t = None
    else:
        # 토글 라벨: 인접 쌍, 짧으면 '쌍 단위'로 건너뛰어 시작/끝 위상 보존 (FC-A)
        i = 0
        while i + 1 < len(sm):
            t0, t1 = sm[i][0], sm[i + 1][0]
            if t1 - t0 >= STILL_MIN_S:
                pairs.append((t0, t1))
            i += 2  # 항상 쌍 단위 전진 — 동작구간을 정지로 오인하지 않음
    return pairs


def still_metrics(cap: "Capture", joints: List[int]
                  ) -> Tuple[Dict[int, float], Dict[int, float], List[dict]]:
    """input_less(사람 착용) 통합 측정: 'still' 마크 쌍 사이의 정지구간들로부터
      · K4 배회(wander) = 구간 내 출력 P 피크-피크의 최대 — 단 구간내 선형추세(드리프트)
        는 제거 후 잔차로(FC-C: 긴 정지의 내부 드리프트가 배회로 이중계상되는 것 차단)
      · K5 드리프트(drift_dpm) = 구간 간 출력 평균각의 시간 추세 회귀 (°/min)
        — unwrap으로 ±180 랩 보정(FC-B1), span≥60s·4점+ 가드(FC-B2)
    한 번의 [동작-정지 반복] 기록으로 둘 다 산출 → drift_ab_test 별도 도구 불필요.
    ⚠ C(목표) 기반 정지 자동검출은 금지 — 드리프트로 C가 흐르면 배회가 마스킹됨(FC).
    반환: (wander_map, drift_map, stills)"""
    pairs = _still_pairs(cap.marks)
    stills: List[dict] = []
    pp: Dict[int, List[float]] = {j: [] for j in joints}          # 구간별 (추세제거) 피크-피크
    seq: Dict[int, Tuple[List[float], List[float]]] = {j: ([], []) for j in joints}  # (중심t, 평균각)
    for t0, t1 in pairs:
        s = int(np.searchsorted(cap.t, t0)); e = int(np.searchsorted(cap.t, t1))
        if e - s < int(STILL_MIN_S * FS):
            continue
        tseg = cap.t[s:e]
        w: Dict[int, float] = {}
        for j in joints:
            seg = cap.P[j][s:e]
            good = np.isfinite(seg)
            if good.sum() < 3:
                continue
            sg, tg = seg[good], tseg[good]
            # 구간내 선형추세 제거 후 잔차 피크-피크 = 순수 배회(드리프트 누설 차단)
            a, b = np.polyfit(tg, sg, 1)
            resid = sg - (a * tg + b)
            w[j] = float(np.nanmax(resid) - np.nanmin(resid))
            pp[j].append(w[j])
            seq[j][0].append((t0 + t1) / 2.0)
            seq[j][1].append(float(np.mean(sg)))
        stills.append({"s": t0, "e": t1, "wander": w})
    wander = {j: float(np.max(v)) for j, v in pp.items() if v}
    drift: Dict[int, float] = {}
    for j, (ts, ms) in seq.items():
        if len(ts) < DRIFT_MIN_PTS:
            continue
        if ts[-1] - ts[0] < DRIFT_MIN_SPAN_S:   # 군집 외삽 방지
            continue
        arr = np.array(ms, dtype=float)
        # ±180 랩 보정: 모터각 절대값이 경계를 넘나드는 j5류 채널 오염 차단(FC-B1)
        unwrapped = np.degrees(np.unwrap(np.radians(arr)))
        slope = float(np.polyfit(np.array(ts), unwrapped, 1)[0])  # °/s
        drift[j] = slope * 60.0                                    # °/min
    return wander, drift, stills


def output_stage_ref(cap: "Capture", joints: List[int]) -> Dict[int, dict]:
    """input_less: C(계산목표)→P(실제) 내부 추종 = '참고' 지표(합부 무관).
    ⚠ C는 IK 하류 내부 신호 — 센서/융합/분해 등 상류 오류는 전혀 못 봄."""
    ref: Dict[int, dict] = {}
    for j in joints:
        C, P = cap.C[j], cap.P[j]
        m = np.isfinite(C) & np.isfinite(P)
        if int(m.sum()) < int(2 * FS) or float(np.nanstd(C[m])) < ACTIVE_RANGE_DEG:
            continue
        lag, peak = xcorr_lag(C, P)
        sc = float(np.nanstd(C[m]))
        ref[j] = {
            "lag_ms": (lag * 1000.0 if lag is not None else None),
            "peak_corr": peak,
            "gain": (float(np.nanstd(P[m]) / sc) if sc > 1e-6 else None),
        }
    return ref


# ──────────────────────────────────────────────────────────────────────────
# KPI 산출
# ──────────────────────────────────────────────────────────────────────────
def analyze(cap: Capture) -> dict:
    res: dict = {"path": cap.path, "meta": cap.meta, "duration": cap.duration,
                 "marks": cap.marks, "read_fail_ticks": cap.read_fail_ticks,
                 "joints": {}, "stills": [], "warnings": []}
    if cap.duration < 5.0:
        res["warnings"].append("기록 5초 미만 — 판정 불가")
        return res

    # 채널 게이트: '원시' 유효표본 80% (보간 후 finite 비율은 항상 1.0이라 무의미)
    out_ok = [j for j in cap.joints if cap.vfrac("P", j) > 0.8]
    low = [f"P{j}({cap.vfrac('P', j) * 100:.0f}%)" for j in cap.joints
           if 0.0 < cap.vfrac("P", j) <= 0.8]
    low += [f"L{j}({cap.vfrac('L', j) * 100:.0f}%)" for j in cap.joints
            if 0.0 < cap.vfrac("L", j) <= 0.8]
    if low:
        res["warnings"].append(f"유효표본 80% 미달 채널 판정 제외: {', '.join(low)}")
    if cap.read_fail_ticks > 0.05 * max(1, len(cap.t)):
        res["warnings"].append(
            f"present 읽기실패 틱 {cap.read_fail_ticks}개(>5%) — 출력 시계열 신뢰도 저하")
    if not cap.has_input:
        res["warnings"].append("입력 기준기(리더) 없음 — 사람 착용 '반증' 모드"
                               "(불합격은 유효, 합격은 추종·게인 인증 아님)")
        active = [j for j in out_ok
                  if np.nanpercentile(cap.P[j], 98) - np.nanpercentile(cap.P[j], 2) > ACTIVE_RANGE_DEG]
        res["active_joints"] = active
        res["input_less"] = True
        # K4 배회 + K5 드리프트: 'still' 마크 쌍 정지구간들에서 통합 산출
        wmap, dmap, stills = still_metrics(cap, out_ok)
        res["stills"] = stills
        res["n_stills"] = len(stills)
        for j, w in wmap.items():
            res["joints"].setdefault(j, {})["wander_deg"] = w
        for j, d in dmap.items():
            res["joints"].setdefault(j, {})["drift_dpm"] = d
        if wmap:
            res["warnings"].append("배회(K4)는 사람 떨림 포함 상한치 — 'still' 마크 쌍 기준")
        elif any("still" in lab.lower() for _, lab in cap.marks):
            res["warnings"].append("'still' 마크가 쌍(시작/끝)을 못 이뤄 배회 산출 불가")
        if dmap:
            res["warnings"].append(f"드리프트(K5)는 정지구간 {len(stills)}개 출력 평균각 추세 회귀(unwrap 적용)")
        elif len(stills) >= 1:
            res["warnings"].append(
                f"드리프트(K5) 산출엔 정지구간 {DRIFT_MIN_PTS}개+ 및 시간span {DRIFT_MIN_SPAN_S:.0f}s+ 필요"
                " (군집 외삽·3점 잔차0 방지) — 동작-정지를 충분히 반복")
        # C→P 출력단 참고 지표 (합부 무관 — IK 상류 오류는 못 봄)
        res["output_stage_ref"] = output_stage_ref(cap, active)
        _safety_stop(cap, res, out_ok)
        return res
    res["input_less"] = False

    # 마크(예: 입력차단) 이후는 입력↔출력 대응이 깨지므로 추종/정지/드리프트/반복
    # KPI는 첫 마크 이전 구간만 사용한다. 안전정지 KPI는 전체 구간 사용.
    if cap.marks:
        n_track = int(np.searchsorted(cap.t, min(m[0] for m in cap.marks)))
        res["warnings"].append(
            f"추종 KPI는 첫 마크({cap.marks[0][1]}, t={cap.marks[0][0]:.0f}s) 이전 구간으로 산출")
    else:
        n_track = len(cap.t)
    if n_track < int(5 * FS):
        res["warnings"].append("마크 이전 구간 5초 미만 — 추종 KPI 산출 불가")
        res["active_joints"] = []
        _safety_stop(cap, res, out_ok)
        return res
    tt = cap.t[:n_track]
    Lt = {j: cap.L[j][:n_track] for j in cap.joints}
    Pt = {j: cap.P[j][:n_track] for j in cap.joints}

    in_ok = [j for j in cap.joints if cap.vfrac("L", j) > 0.8]
    active = [j for j in in_ok if j in out_ok and
              (np.nanpercentile(Lt[j], 98) - np.nanpercentile(Lt[j], 2)) > ACTIVE_RANGE_DEG]
    res["active_joints"] = active
    if not active:
        res["warnings"].append(f"입력 가동폭 {ACTIVE_RANGE_DEG}° 초과 관절 없음 — 추종 KPI 산출 불가")

    for j in active:
        L, P = Lt[j], Pt[j]
        jr: dict = {}
        lag, peak = xcorr_lag(L, P)
        if lag is not None and lag < -0.5 / FS:
            # 출력이 입력을 앞섬 = 물리적 불가 → 계측 이상 (지연 0으로 클램프 금지)
            res["warnings"].append(
                f"j{j}: 음의 지연({lag * 1000:.0f}ms, peak {peak:.2f}) — 계측 이상, K1 판정불가")
            jr["latency_ms"] = None
            lag = 0.0
        else:
            jr["latency_ms"] = lag * 1000.0 if lag is not None else None
        jr["xcorr_peak"] = peak
        Ps = shift_back(P, lag) if lag else P.copy()
        valid = np.isfinite(Ps) & np.isfinite(L)
        e = Ps[valid] - L[valid]
        jr["align_offset_deg"] = float(np.median(e))
        e0 = e - np.median(e)
        jr["track_rms_deg"] = float(np.sqrt(np.mean(e0 ** 2)))
        jr["track_p95_deg"] = float(np.percentile(np.abs(e0), 95))
        jr["track_max_deg"] = float(np.max(np.abs(e0)))
        # 스케일 충실도: 최소제곱 P ≈ a·L + b
        Lv = L[valid]
        if Lv.std() > 1e-6:
            a, b = np.polyfit(Lv, Ps[valid], 1)
            jr["gain"] = float(a)
        else:
            jr["gain"] = None
        jr["input_range_deg"] = float(np.nanpercentile(L, 98) - np.nanpercentile(L, 2))
        res["joints"][j] = jr

    # 정지 구간 KPI (배회/드리프트/반복) — 추종 윈도(첫 마크 이전)만
    segs = still_segments(Lt, active, tt)
    res["n_stills"] = len(segs)
    settle = int(0.5 * FS)  # 구간 첫 0.5s는 정착 시간으로 제외
    ref_pose: Dict[int, float] = {}
    visits: List[Tuple[float, Dict[int, float], Dict[int, float]]] = []
    for (s, eidx) in segs:
        s2 = s + settle
        if eidx - s2 < FS:
            continue
        mid_t = float(cap.t[(s2 + eidx) // 2])
        Lmed = {j: float(np.nanmedian(cap.L[j][s2:eidx])) for j in active}
        Pmed = {j: float(np.nanmedian(cap.P[j][s2:eidx])) for j in active}
        wander = {j: float(np.nanmax(cap.P[j][s2:eidx]) - np.nanmin(cap.P[j][s2:eidx]))
                  for j in active}
        err = {j: Pmed[j] - Lmed[j] for j in active}
        res["stills"].append({"t": mid_t, "dur": (eidx - s) / FS,
                              "wander": wander, "err": err})
        if not ref_pose:
            ref_pose = Lmed
        if all(abs(Lmed[j] - ref_pose[j]) < REPEAT_TOL_DEG for j in active):
            visits.append((mid_t, Lmed, Pmed))

    for j in active:
        jr = res["joints"][j]
        ws = [st["wander"][j] for st in res["stills"]]
        jr["wander_deg"] = float(np.max(ws)) if ws else None
        # 드리프트: 정지 구간 오차 중앙값의 시간 추세(≥3구간, 총 시간폭 ≥60s)
        if len(res["stills"]) >= 3:
            ts = np.array([st["t"] for st in res["stills"]])
            es = np.array([st["err"][j] for st in res["stills"]])
            if ts[-1] - ts[0] >= 60.0:
                slope = np.polyfit(ts, es, 1)[0]
                jr["drift_dpm"] = float(slope * 60.0)
            else:
                jr["drift_dpm"] = None
        else:
            jr["drift_dpm"] = None
        # 반복 정밀도: 기준 포즈 재방문 ≥3회.
        # (P−L) 산포 사용 — Pmed 산포로 하면 재방문 허용반경(±2.5°) 내의 '입력' 산포가
        # 출력 산포로 둔갑해 완벽한 장치도 불합격함 (FC 중요-6).
        if len(visits) >= 3:
            diffs = np.array([v[2][j] - v[1][j] for v in visits])
            jr["repeat_deg"] = float(diffs.max() - diffs.min())
            jr["repeat_n"] = len(visits)
        else:
            jr["repeat_deg"] = None

    _safety_stop(cap, res, active or out_ok)
    return res


def _safety_stop(cap: Capture, res: dict, joints: List[int]) -> None:
    """'cut' 마크 시각 → 출력 전 관절 속도 <STOP_VEL 이 STOP_HOLD_S 지속될 때까지.

    라벨에 'cut'이 없는 마크는 주석(annotation)으로만 기록 — 모든 마크를 K7로
    강제하면 메모용 마크 하나로 종합 F가 됨 (FC 중요-7).
    """
    stops = []
    res["annotations"] = [(t, l) for (t, l) in cap.marks if "cut" not in l.lower()]
    cut_marks = [(t, l) for (t, l) in cap.marks if "cut" in l.lower()]
    if not joints or not cut_marks:
        res["safety_stops"] = stops
        return
    vmax = np.zeros(len(cap.t))
    for j in joints:
        v = np.abs(vel(np.nan_to_num(cap.P[j], nan=0.0)))
        vmax = np.maximum(vmax, v)
    hold = int(STOP_HOLD_S * FS)
    for (mt, label) in cut_marks:
        i0 = int(np.searchsorted(cap.t, mt))
        if len(cap.t) - i0 < hold + int(2.0 * FS):
            stops.append({"t": mt, "label": label, "stop_s": None, "travel_deg": None,
                          "note": "데이터부족(마크 후 <2.5s) — 판정불가"})
            continue
        stop_t = None
        travel = 0.0
        for i in range(i0, len(cap.t) - hold):
            if np.all(vmax[i:i + hold] < STOP_VEL):
                stop_t = float(cap.t[i] - mt)
                break
        iend = int(np.searchsorted(cap.t, mt + 3.0))
        for j in joints:
            seg = cap.P[j][i0:iend]
            if np.isfinite(seg).any():
                travel = max(travel, float(np.nanmax(seg) - np.nanmin(seg)))
        stops.append({"t": mt, "label": label, "stop_s": stop_t, "travel_deg": travel})
    res["safety_stops"] = stops


# ──────────────────────────────────────────────────────────────────────────
# 판정 (criteria JSON)
# ──────────────────────────────────────────────────────────────────────────
def load_criteria(path: Optional[str]) -> dict:
    default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "inspection_criteria.json")
    p = path or default
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


def grade_value(value: Optional[float], thresholds: Dict[str, float],
                lower_is_better: bool = True) -> Optional[str]:
    if value is None:
        return None
    for g in ("A", "B", "C"):
        th = thresholds.get(g)
        if th is None:
            continue
        if (value <= th) if lower_is_better else (value >= th):
            return g
    return "F"


def joint_criteria(crit: dict, kpi: str, mode: str, joint: int) -> Optional[dict]:
    node = crit.get(kpi)
    if node is None:
        return None
    if "A" in node:           # 모드/관절 구분 없는 평면형
        return node
    sub = node.get(mode) or node.get("default")
    if sub is None:
        return None
    return sub.get(str(joint)) or sub.get("default")


def judge(res: dict, crit: dict) -> dict:
    mode = "leader" if res["meta"].get("relay_mode") == "leader" else "imu"
    out = {"mode": mode, "joints": {}, "overall": None}
    worst = "A"

    def upd(g: Optional[str]):
        nonlocal worst
        if g and GRADE_ORDER.index(g) > GRADE_ORDER.index(worst):
            worst = g

    for j, jr in res.get("joints", {}).items():
        jg = {}
        for kpi, val, lower in (
                ("latency_ms", jr.get("latency_ms"), True),
                ("track_rms_deg", jr.get("track_rms_deg"), True),
                ("track_p95_deg", jr.get("track_p95_deg"), True),
                ("wander_deg", jr.get("wander_deg"), True),
                ("drift_dpm", abs(jr["drift_dpm"]) if jr.get("drift_dpm") is not None else None, True),
                ("repeat_deg", jr.get("repeat_deg"), True)):
            th = joint_criteria(crit, kpi, mode, j)
            g = grade_value(val, th) if th else None
            jg[kpi] = g
            upd(g)
        gain = jr.get("gain")
        th = joint_criteria(crit, "gain_err", mode, j)
        if gain is not None and th:
            g = grade_value(abs(gain - 1.0), th)
            jg["gain_err"] = g
            upd(g)
        out["joints"][j] = jg
    th = crit.get("safety_stop_s")
    for st in res.get("safety_stops", []):
        if st.get("note"):           # 데이터부족 → 판정불가(F 아님 — 미측정으로 잡힘)
            st["grade"] = None
            continue
        g = grade_value(st["stop_s"], th) if (th and st["stop_s"] is not None) else ("F" if th else None)
        st["grade"] = g
        upd(g)

    # 시나리오 완전성: 요구 KPI가 미측정이면 '판정불가'로 강등 — 미측정 KPI가
    # 종합판정에서 조용히 빠져 거짓 합격이 되는 걸 차단 (FC 치명-1)
    missing: List[str] = []
    req = None
    scn = str(res["meta"].get("scenario", "")).upper()
    for key, kpis in (crit.get("scenario_kpis") or {}).items():
        if scn.startswith(key.upper()):
            req = kpis
            break
    if req:
        measured_key = {"gain_err": "gain"}
        for kpi in req:
            if kpi == "safety_stop_s":
                done = any(st.get("grade") for st in res.get("safety_stops", []))
            else:
                k = measured_key.get(kpi, kpi)
                done = any(jr.get(k) is not None for jr in res.get("joints", {}).values())
            if not done:
                missing.append(kpi)
    out["missing"] = missing
    out["overall"] = worst if res.get("joints") or res.get("safety_stops") else None
    return out


# ──────────────────────────────────────────────────────────────────────────
# SVG 플롯 (의존성 없는 간단 폴리라인)
# ──────────────────────────────────────────────────────────────────────────
def svg_plot(cap: Capture, res: dict, out_path: str, max_joints: int = 3) -> Optional[str]:
    act = res.get("active_joints") or []
    if not act:
        return None
    act = act[:max_joints]
    W, H, PAD = 900, 160 * len(act), 40
    rows = []
    colors = {"L": "#1f77b4", "P": "#d62728", "G": "#bbbbbb"}

    def poly(t, y, x0, y0, w, h, ymin, ymax, color, dash=""):
        if ymax - ymin < 1e-9:
            ymax = ymin + 1.0
        pts = []
        step = max(1, len(t) // 1200)
        for i in range(0, len(t), step):
            if not math.isfinite(y[i]):
                continue
            x = x0 + (t[i] / t[-1]) * w
            yy = y0 + h - (y[i] - ymin) / (ymax - ymin) * h
            pts.append(f"{x:.1f},{yy:.1f}")
        d = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polyline fill="none" stroke="{color}" stroke-width="1.2"{d} points="{" ".join(pts)}"/>'

    for k, j in enumerate(act):
        y0 = PAD + k * 160
        h = 110
        L, P = cap.L[j], cap.P[j]
        data = np.concatenate([L[np.isfinite(L)], P[np.isfinite(P)]])
        ymin, ymax = float(data.min()), float(data.max())
        rows.append(f'<text x="{PAD}" y="{y0 - 8}" font-size="13" fill="#333">'
                    f'j{j} — 입력 L(파랑) vs 출력 P(빨강), [{ymin:.0f}°, {ymax:.0f}°], '
                    f'{cap.t[-1]:.0f}s</text>')
        rows.append(f'<rect x="{PAD}" y="{y0}" width="{W - 2 * PAD}" height="{h}" '
                    f'fill="none" stroke="#ddd"/>')
        rows.append(poly(cap.t, L, PAD, y0, W - 2 * PAD, h, ymin, ymax, colors["L"]))
        rows.append(poly(cap.t, P, PAD, y0, W - 2 * PAD, h, ymin, ymax, colors["P"]))
        for (mt, label) in cap.marks:
            x = PAD + (mt / cap.t[-1]) * (W - 2 * PAD)
            rows.append(f'<line x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y0 + h}" '
                        f'stroke="#f0a000" stroke-dasharray="4 3"/>')
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H + PAD}" '
           f'font-family="sans-serif">' + "".join(rows) + "</svg>")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(svg)
    return out_path


# ──────────────────────────────────────────────────────────────────────────
# 보고서
# ──────────────────────────────────────────────────────────────────────────
def fmt(v, nd=1, unit=""):
    if v is None:
        return "—"
    return f"{v:.{nd}f}{unit}"


def render_md(results: List[Tuple[dict, dict, Optional[str]]], crit: dict,
              selftest_line: str = "미실행") -> str:
    lines = ["# 미니 팔로우암 검사 결과 보고서",
             "",
             f"- 생성: {time.strftime('%Y-%m-%d %H:%M')}",
             f"- 판정 기준: inspection_criteria.json (등급 A/B/C, 미달 F)",
             f"- 측정기 자기검증(검교정): {selftest_line}",
             ""]
    for res, jg, svg in results:
        meta = res["meta"]
        overall = jg.get("overall") or "판정불가"
        if jg.get("missing"):
            overall = (f"판정불가 — 미측정 KPI: {', '.join(jg['missing'])}"
                       f" (측정분 최악 {jg.get('overall') or '—'})")
        lines += [f"## {os.path.basename(res['path'])}",
                  "",
                  f"- 시나리오: **{meta.get('scenario', '?')}** | 모드: {meta.get('relay_mode', '?')}"
                  f" | 길이: {res['duration']:.0f}s | 읽기실패틱: {res['read_fail_ticks']}",
                  f"- 검사 관절: {res.get('active_joints', [])} | 정지구간: {res.get('n_stills', 0)}개"
                  f" | 종합판정: **{overall}**",
                  ""]
        for w in res.get("warnings", []):
            lines.append(f"> ⚠ {w}")
        if res.get("joints"):
            lines += ["", "| 관절 | 지연(ms) | 추종RMS(°) | P95(°) | 최대(°) | 게인 | 정렬오프셋(°) "
                            "| 배회(°) | 드리프트(°/min) | 반복(°) | 등급(최악) |",
                      "|---|---|---|---|---|---|---|---|---|---|---|"]
            for j, jr in sorted(res["joints"].items()):
                g = jg["joints"].get(j, {})
                worst = "—"
                gs = [v for v in g.values() if v]
                if gs:
                    worst = max(gs, key=GRADE_ORDER.index)
                lines.append(
                    f"| j{j} | {fmt(jr.get('latency_ms'), 0)} | {fmt(jr.get('track_rms_deg'), 2)} "
                    f"| {fmt(jr.get('track_p95_deg'), 2)} | {fmt(jr.get('track_max_deg'), 1)} "
                    f"| {fmt(jr.get('gain'), 3)} | {fmt(jr.get('align_offset_deg'), 1)} "
                    f"| {fmt(jr.get('wander_deg'), 2)} | {fmt(jr.get('drift_dpm'), 2)} "
                    f"| {fmt(jr.get('repeat_deg'), 2)} | {worst} |")
        if res.get("safety_stops"):
            lines += ["", "| cut마크 t(s) | 라벨 | 정지시간(s) | 잔여이동(°) | 등급 |", "|---|---|---|---|---|"]
            for st in res["safety_stops"]:
                grade = st.get("grade") or st.get("note") or "—"
                lines.append(f"| {st['t']:.1f} | {st['label']} | {fmt(st['stop_s'], 2)} "
                             f"| {fmt(st['travel_deg'], 1)} | {grade} |")
        if res.get("output_stage_ref"):
            lines += ["", "**참고: C(목표)→P(실제) 출력단 — 합부 무관, 내부 신호 기준"
                          " (센서·융합·분해 등 IK 상류 오류는 못 봄). ⚠ 큰 동작에서는 속도제한"
                          " 포화로 지연 과대·게인 과소 — 절대 정확도가 아님**", "",
                      "| 관절 | C→P 지연(ms) | 게인 | 피크상관 |", "|---|---|---|---|"]
            for j, r in sorted(res["output_stage_ref"].items()):
                lines.append(f"| j{j} | {fmt(r.get('lag_ms'), 0)} | {fmt(r.get('gain'), 3)} "
                             f"| {fmt(r.get('peak_corr'), 2)} |")
        if res.get("annotations"):
            ann = ", ".join(f"{t:.1f}s:{l}" for t, l in res["annotations"])
            lines += ["", f"- 주석 마크(판정 미사용): {ann}"]
        if svg:
            lines += ["", f"![plot]({os.path.basename(svg)})"]
        lines.append("")
    return "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────────
# 자기검증 (합성 데이터 — 측정기 검교정)
# ──────────────────────────────────────────────────────────────────────────
def _mk_cap(t: np.ndarray, L: Dict[int, np.ndarray], P: Dict[int, np.ndarray],
            marks: List[Tuple[float, str]], mode: str, scenario: str) -> Capture:
    cap = Capture.__new__(Capture)
    cap.path = f"<selftest:{scenario}>"
    cap.meta = {"scenario": scenario, "relay_mode": mode}
    cap.marks = marks
    cap.joints = sorted(L.keys())
    cap.t = t
    cap.L = L
    cap.C = {j: np.full_like(t, np.nan) for j in L}
    cap.G = {j: np.full_like(t, np.nan) for j in L}
    cap.P = P
    cap.valid = {}
    for j in L:
        cap.valid[f"L{j}_deg"] = 1.0
        cap.valid[f"P{j}_deg"] = 1.0
    cap.read_fail_ticks = 0
    return cap


def selftest(verbose: bool = True) -> Tuple[int, int]:
    rng = np.random.default_rng(42)
    fs, dur = FS, 240.0
    t = np.arange(0.0, dur, 1.0 / fs)
    inj = {"lag_s": 0.12, "gain": 0.97, "offset": 3.0, "noise": 0.25,
           "drift_dpm_j2": 0.5, "stop_s": 0.45}

    def motion(seed_phase: float) -> np.ndarray:
        """움직임-정지 교차: 20s 동작 + 8s 정지 반복, 기준 포즈(0°) 복귀."""
        x = np.zeros_like(t)
        period = 28.0
        for i, ti in enumerate(t):
            ph = ti % period
            if ph < 20.0:
                x[i] = 30.0 * math.sin(2 * math.pi * ph / 10.0 + seed_phase) * math.sin(math.pi * ph / 20.0)
            else:
                x[i] = 0.0
        return x

    L = {1: motion(0.0), 2: motion(1.0), 3: motion(2.0)}
    cut_t = 210.0  # 동작 위상(ph=14s) 중 차단 — 정지 위상에 걸면 stop_s≈0이 되어 검증 무의미
    P = {}
    for j in (1, 2, 3):
        lagged = np.interp(t - inj["lag_s"], t, L[j], left=0.0)
        p = inj["gain"] * lagged + inj["offset"] + rng.normal(0, inj["noise"], len(t))
        if j == 2:
            p += inj["drift_dpm_j2"] * (t / 60.0)
        # 마크(cut) 후: 0.45s 동안 감속 후 정지 유지
        icut = int(cut_t * fs)
        istop = int((cut_t + inj["stop_s"]) * fs)
        hold = p[istop] if istop < len(p) else p[-1]
        for i in range(icut, len(p)):
            if i < istop:
                p[i] = p[icut] + (hold - p[icut]) * (i - icut) / max(1, istop - icut)
            else:
                p[i] = hold + rng.normal(0, 0.05)
        P[j] = p

    Lc = {j: L[j].copy() for j in L}
    for j in (1, 2, 3):
        Lc[j][int(cut_t * fs):] = Lc[j][int(cut_t * fs)]  # 입력도 차단 후 정지
    cap = _mk_cap(t, Lc, P, [(cut_t, "input_cut")], "leader", "S2_selftest")

    res = analyze(cap)
    checks = []

    def chk(name, got, want, tol):
        ok = got is not None and abs(got - want) <= tol
        checks.append((name, got, want, tol, ok))

    def chkb(name, ok):
        checks.append((name, None, None, None, bool(ok)))

    for j in (1, 2, 3):
        jr = res["joints"][j]
        chk(f"j{j} latency_ms", jr["latency_ms"], inj["lag_s"] * 1000, 15.0)
        chk(f"j{j} gain", jr["gain"], inj["gain"], 0.02)
        chk(f"j{j} align_offset", jr["align_offset_deg"], inj["offset"],
            0.6 if j != 2 else 1.5)
        want_rms = inj["noise"]
        chk(f"j{j} track_rms", jr["track_rms_deg"], want_rms, 0.35 if j != 2 else 0.9)
        chk(f"j{j} wander", jr["wander_deg"], 4 * inj["noise"], 1.2)
    chk("j2 drift_dpm", res["joints"][2]["drift_dpm"], inj["drift_dpm_j2"], 0.15)
    chk("j1 drift_dpm", res["joints"][1]["drift_dpm"], 0.0, 0.15)
    chk("j1 repeat_deg", res["joints"][1]["repeat_deg"], 0.0, 0.8)
    st = res["safety_stops"][0]
    chk("safety stop_s", st["stop_s"], inj["stop_s"], 0.25)

    # ── judge() 경로 (FC: selftest가 판정층을 0% 커버했음) ──
    try:
        crit = load_criteria(None)
    except Exception:
        crit = {}
    if crit:
        jg = judge(res, crit)
        chkb("judge: cut마크 → K7 판정 산출", any(s.get("grade") for s in res["safety_stops"]))
        chkb("judge: S2 요구 KPI 전부 측정됨(미측정 없음)", not jg["missing"])
        res_nostop = dict(res)
        res_nostop["meta"] = {"scenario": "S6_cut_test", "relay_mode": "leader"}
        res_nostop["safety_stops"] = []
        jg2 = judge(res_nostop, crit)
        chkb("judge: S6에 cut 결과 없으면 '미측정' 검출(거짓합격 차단)",
             "safety_stop_s" in jg2["missing"])

        # 게인 0.5 불량 장치 → F
        t2 = np.arange(0.0, 90.0, 1.0 / fs)
        Lg = {1: 20.0 * np.sin(2 * np.pi * t2 / 8.0)}
        Pg = {1: 0.5 * Lg[1] + rng.normal(0, 0.1, len(t2))}
        res_g = analyze(_mk_cap(t2, Lg, Pg, [], "leader", "S1_gain"))
        jg_g = judge(res_g, crit)
        chk("불량(gain0.5): gain 복원", res_g["joints"][1]["gain"], 0.5, 0.02)
        chkb("불량(gain0.5): 종합 F", jg_g["overall"] == "F")

        # ── 사람 착용(input_less, H3): 마크 쌍 기반 배회 + 거짓합격 차단 ──
        th = np.arange(0.0, 60.0, 1.0 / fs)
        w_inj = 1.0
        Ph = {1: 5.0 * np.sin(2 * np.pi * th / 8.0)}
        seg = (th >= 10.0) & (th <= 16.0)          # 정지 구간: 피크-피크 = w_inj
        Ph[1][seg] = (w_inj / 2.0) * np.sin(2 * np.pi * th[seg] / 0.5)
        cap_h = _mk_cap(th, {1: np.zeros_like(th)}, Ph,
                        [(10.0, "still"), (16.0, "still")], "calib_ik", "H3_human")
        cap_h.valid["L1_deg"] = 0.0                 # 리더 없음 → input_less
        res_h = analyze(cap_h)
        chk("H3 배회 복원(마크쌍)", res_h["joints"].get(1, {}).get("wander_deg"), w_inj, 0.3)
        chkb("H3: wander 측정됨(판정가능)", not judge(res_h, crit)["missing"])
        cap_h2 = _mk_cap(th, {1: np.zeros_like(th)}, Ph, [], "calib_ik", "H3_nomarks")
        cap_h2.valid["L1_deg"] = 0.0
        chkb("H3: still 마크 없으면 미측정 검출(거짓합격 차단)",
             "wander_deg" in judge(analyze(cap_h2), crit)["missing"])

        # ── 사람 착용(HM): 정지구간 4개 + 떨림으로 K5 드리프트 추세 복원 ──
        td = np.arange(0.0, 120.0, 1.0 / fs)
        d_inj = 0.5                                 # °/min 선형 드리프트 주입
        base = d_inj * (td / 60.0)
        moving = np.ones_like(td, dtype=bool)
        marks_d: List[Tuple[float, str]] = []
        for c in (10.0, 40.0, 70.0, 100.0):         # 정지구간 4개, span 90s (≥60s 가드 통과)
            sgm = (td >= c - 2.0) & (td <= c + 2.0)
            moving &= ~sgm
            marks_d += [(c - 2.0, "still_start"), (c + 2.0, "still_end")]
        base = base.copy()
        base += rng.normal(0, 0.4, len(td))         # FC-F: 정지구간에도 사람 떨림 σ=0.4°
        base[moving] += 10.0 * np.sin(2 * np.pi * td[moving] / 5.0)
        cap_d = _mk_cap(td, {1: np.zeros_like(td)}, {1: base}, marks_d, "calib_ik", "HM_drift")
        cap_d.valid["L1_deg"] = 0.0
        res_d = analyze(cap_d)
        chk("HM 드리프트 복원(떨림+span90s)", res_d["joints"].get(1, {}).get("drift_dpm"), d_inj, 0.2)
        chkb("HM: wander+drift 측정됨(판정가능)", not judge(res_d, crit)["missing"])

        # ── FC-B1: ±180 랩어라운드 채널 드리프트 (j5류) ──
        tw = np.arange(0.0, 120.0, 1.0 / fs)
        dw_inj = 2.0                                 # +2°/min, 178°에서 시작 → 랩 발생
        basew = 178.0 + dw_inj * (tw / 60.0)
        basew = ((basew + 180.0) % 360.0) - 180.0    # ±180 랩
        marks_w: List[Tuple[float, str]] = []
        movw = np.ones_like(tw, dtype=bool)
        for c in (10.0, 40.0, 70.0, 100.0):
            sgm = (tw >= c - 2.0) & (tw <= c + 2.0)
            movw &= ~sgm
            marks_w += [(c - 2.0, "still_start"), (c + 2.0, "still_end")]
        capw = _mk_cap(tw, {1: np.zeros_like(tw)}, {1: basew}, marks_w, "calib_ik", "HM_wrap")
        capw.valid["L1_deg"] = 0.0
        resw = analyze(capw)
        chk("FC-B1 랩어라운드 드리프트 복원", resw["joints"].get(1, {}).get("drift_dpm"), dw_inj, 0.3)

        # ── FC-B2: span 가드 — 정지구간 4개지만 앞쪽 15s 군집이면 미산출 ──
        tc = np.arange(0.0, 120.0, 1.0 / fs)
        marks_c: List[Tuple[float, str]] = []
        for c in (4.0, 8.0, 12.0, 16.0):             # span 12s < 60s
            marks_c += [(c - 1.5, "still_start"), (c + 1.5, "still_end")]
        capc = _mk_cap(tc, {1: np.zeros_like(tc)}, {1: 0.5 * (tc / 60.0)}, marks_c, "calib_ik", "HM_clust")
        capc.valid["L1_deg"] = 0.0
        chkb("FC-B2 군집 span<60s → 드리프트 미산출",
             analyze(capc)["joints"].get(1, {}).get("drift_dpm") is None)

        # ── FC-A: 짧은 정지 폐기 시 동작구간을 정지로 오인하지 않음 ──
        ta = np.arange(0.0, 30.0, 1.0 / fs)
        Pa = {1: np.zeros_like(ta)}
        seg_move = (ta >= 6.0) & (ta <= 9.0)         # 6~9s는 동작(큰 변동)
        Pa[1][seg_move] = 20.0 * np.sin(2 * np.pi * ta[seg_move] / 1.5)
        # 마크: 5,6 짧은정지(폐기), 9,13 동작 끝~진짜정지 — i+=2 위상보존이면 (9,13)만 정지
        marks_a = [(5.0, "still_start"), (6.0, "still_end"),
                   (9.0, "still_start"), (13.0, "still_end")]
        capa = _mk_cap(ta, {1: np.zeros_like(ta)}, Pa, marks_a, "calib_ik", "H3_pairing")
        capa.valid["L1_deg"] = 0.0
        resa = analyze(capa)
        # (9,13)은 평평(0) → 배회≈0. 동작구간(6,9)을 정지로 오인했으면 배회가 40°로 튐
        chkb("FC-A 동작구간 오인 안 함(배회<2°)",
             (resa["joints"].get(1, {}).get("wander_deg") or 0.0) < 2.0)

    # 음의 지연(출력이 입력을 앞섬) = 계측 이상 → K1 판정불가 + 경고
    t3 = np.arange(0.0, 90.0, 1.0 / fs)
    Ln = {1: 20.0 * np.sin(2 * np.pi * t3 / 8.0)}
    Pn = {1: np.interp(t3 + 0.2, t3, Ln[1])}
    res_n = analyze(_mk_cap(t3, Ln, Pn, [], "leader", "S1_neg"))
    chkb("계측이상(음의 지연): K1 판정불가 처리", res_n["joints"][1]["latency_ms"] is None)
    chkb("계측이상(음의 지연): 경고 발생", any("음의 지연" in w for w in res_n["warnings"]))

    if verbose:
        print("=== inspection_report 자기검증 (합성 데이터, 주입값 복원 + 불량/엣지 케이스) ===")
        width = max(len(c[0]) for c in checks)
        for name, got, want, tol, ok in checks:
            if want is None:
                print(f"{'PASS' if ok else 'FAIL'}  {name}")
            else:
                print(f"{'PASS' if ok else 'FAIL'}  {name:<{width}}  "
                      f"측정={got if got is None else f'{got:.3f}'}  주입={want:.3f} ±{tol}")
    n_ok = sum(1 for c in checks if c[4])
    if verbose:
        print(f"--- {n_ok}/{len(checks)} 통과 ---")
    return n_ok, len(checks)


# ──────────────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", help="inspection_recorder CSV 파일들")
    ap.add_argument("--criteria", default=None)
    ap.add_argument("--out", default=None, help="보고서 출력 경로(.md)")
    ap.add_argument("--no-svg", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        n_ok, n_total = selftest(verbose=True)
        return 0 if n_ok == n_total else 1
    if not args.csv:
        ap.print_help()
        return 2

    crit = load_criteria(args.criteria)
    # 보고서마다 측정기 자기검증을 실제로 돌려 머리말에 기록 (문구만 인쇄 금지 — FC 사소-12)
    st_ok, st_total = selftest(verbose=False)
    selftest_line = (f"{st_ok}/{st_total} 통과" if st_ok == st_total
                     else f"**{st_ok}/{st_total} — 실패: 본 보고서 수치 신뢰 불가**")
    results = []
    for path in args.csv:
        try:
            cap = Capture(path)
        except Exception as exc:
            print(f"건너뜀 {path}: {exc}", file=sys.stderr)
            continue
        res = analyze(cap)
        jg = judge(res, crit)
        svg = None
        if not args.no_svg:
            svg_path = os.path.splitext(path)[0] + ".svg"
            try:
                svg = svg_plot(cap, res, svg_path)
            except Exception as exc:
                print(f"SVG 실패 {path}: {exc}", file=sys.stderr)
        results.append((res, jg, svg))

    if not results:
        return 1
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.csv[0])),
                                   f"검사보고서_{time.strftime('%Y%m%d_%H%M')}.md")
    md = render_md(results, crit, selftest_line)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(md)
    print(f"보고서: {out}  (자기검증 {selftest_line})")
    for res, jg, _ in results:
        ov = jg.get("overall") or "판정불가"
        if jg.get("missing"):
            ov = f"판정불가(미측정: {', '.join(jg['missing'])})"
        print(f"  {os.path.basename(res['path'])}: 종합 {ov}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""검사 기록기(Inspection Recorder) — 블랙박스 검사용 동시 로깅.

장치 외부 관점의 입력/출력을 한 CSV에 시간 동기로 기록한다:
  입력  L{j}_deg : 리더암 관절각(기록 시작 시점 기준 상대, 팔로워 관절 좌표계로 변환)
  내부  C{j}_deg : 파이프라인 목표각(참고용 — 블랙박스 KPI에는 미사용)
  출력  G{j}_deg : 전송 목표(틱→deg, 베이스 기준 상대)
  출력  P{j}_deg : 모터 실제 위치(Present Position, 베이스 기준 상대)

분석/판정은 inspection_report.py 가 오프라인으로 수행한다. 기록기는 계산하지 않고
원자료만 남긴다(검사 추적성: raw 틱도 함께 기록).
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Callable, Dict, Optional

CSV_VERSION = 1


def wrap_ticks(d: int) -> int:
    """리더(Feetech) 0/4095 경계 랩어라운드 보정."""
    if d > 2048:
        d -= 4096
    elif d < -2048:
        d += 4096
    return d


class InspectionRecorder:
    """앱이 50ms마다 tick(snapshot)을 먹여 주면 CSV로 적재한다.

    snapshot_fn() -> dict:
      leader_raw: Dict[int, int]   리더 raw 틱 (없으면 {})
      leader_age: float            마지막 리더 수신 후 경과(s)
      imu_age:    float            마지막 IMU 패킷 후 경과(s)
      target:     Dict[int, float] 파이프라인 목표각(deg) (없으면 {})
      goal:       Dict[int, int]   마지막 전송 goal 틱 (없으면 {})
      base:       Dict[int, int]   릴레이 베이스 틱 (없으면 {})
      present:    Dict[int, int]   Present Position 틱 (읽기 실패 관절은 누락)
      read_fail:  int              이번 틱 present 읽기 실패 관절 수
    """

    def __init__(self, out_dir: str, motor_ids, leader_sign: Dict[int, float],
                 leader_scale: Dict[int, float],
                 snapshot_fn: Callable[[], dict],
                 log_fn: Callable[[str], None] = print) -> None:
        self.out_dir = out_dir
        self.motor_ids = tuple(motor_ids)
        self.leader_sign = leader_sign
        self.leader_scale = leader_scale
        self._snapshot = snapshot_fn
        self._log = log_fn
        self._fh = None
        self._path: Optional[str] = None
        self._leader_ref: Dict[int, int] = {}
        self._base_fallback: Dict[int, int] = {}
        self._live_base: Dict[int, Optional[int]] = {}
        self._rows = 0
        self._fail_ticks = 0
        self._t0 = 0.0

    # ── 수명 주기 ──────────────────────────────────────────────────────────
    @property
    def active(self) -> bool:
        return self._fh is not None

    @property
    def path(self) -> Optional[str]:
        return self._path

    @property
    def rows(self) -> int:
        return self._rows

    def start(self, scenario: str, meta: Optional[dict] = None) -> str:
        if self._fh is not None:
            raise RuntimeError("이미 기록 중")
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        safe = "".join(c if (c.isalnum() or c in "-_") else "_" for c in scenario) or "scenario"
        self._path = os.path.join(self.out_dir, f"{stamp}_{safe}.csv")
        n = 1
        while os.path.exists(self._path):  # 같은 초 내 재시작 덮어쓰기 방지
            self._path = os.path.join(self.out_dir, f"{stamp}_{safe}_{n}.csv")
            n += 1
        self._fh = open(self._path, "w", encoding="utf-8")
        self._leader_ref = {}
        self._base_fallback = {}
        self._live_base: Dict[int, Optional[int]] = {}
        self._rows = 0
        self._fail_ticks = 0
        self._t0 = time.time()

        head_meta = {
            "csv_version": CSV_VERSION,
            "scenario": scenario,
            "t0_unix": self._t0,
            "t0_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "motor_ids": list(self.motor_ids),
            "leader_sign": {str(k): v for k, v in self.leader_sign.items()},
            "leader_scale": {str(k): v for k, v in self.leader_scale.items()},
        }
        if meta:
            head_meta.update(meta)
        for k, v in head_meta.items():
            self._fh.write(f"# {k}={json.dumps(v, ensure_ascii=False)}\n")

        cols = ["t", "ev", "leader_age", "imu_age", "read_fail"]
        for j in self.motor_ids:
            cols += [f"L{j}_deg", f"C{j}_deg", f"G{j}_deg", f"P{j}_deg",
                     f"Lraw{j}", f"Praw{j}"]
        self._fh.write(",".join(cols) + "\n")
        return self._path

    def stop(self) -> Optional[str]:
        if self._fh is None:
            return None
        path = self._path
        self._fh.write(f"# rows={self._rows} fail_ticks={self._fail_ticks}\n")
        self._fh.close()
        self._fh = None
        return path

    def mark(self, label: str) -> None:
        """이벤트 마크 행 — 분석기가 안전정지 등 타임스탬프 기준으로 사용."""
        if self._fh is None:
            return
        t = time.time() - self._t0
        safe = label.replace(",", ";").replace("\n", " ")
        n_cols = 5 + 6 * len(self.motor_ids)
        row = [f"{t:.3f}", f"MARK:{safe}"] + [""] * (n_cols - 2)
        self._fh.write(",".join(row) + "\n")
        self._fh.flush()

    # ── 기록 ──────────────────────────────────────────────────────────────
    def tick(self) -> None:
        if self._fh is None:
            return
        snap = self._snapshot()
        t = time.time() - self._t0

        leader_raw: Dict[int, int] = snap.get("leader_raw") or {}
        present: Dict[int, int] = snap.get("present") or {}
        base: Dict[int, int] = snap.get("base") or {}
        goal: Dict[int, int] = snap.get("goal") or {}
        target: Dict[int, float] = snap.get("target") or {}
        if snap.get("read_fail"):
            self._fail_ticks += 1

        # 기록 시작 시점의 리더/팔로워 자세 = 입력/출력 0점 (이후 상대각).
        # 입력·출력의 절대 정렬 차이는 분석기가 '정렬 오프셋'으로 별도 보고한다.
        if not self._leader_ref and leader_raw:
            self._leader_ref = dict(leader_raw)
        # 베이스는 '첫 틱에 고정(latch)' — 기록 중 [전달 시작]/[기준 설정]이 live 베이스를
        # 옮기면 P_deg에 계단이 생겨 추종 KPI가 오염됨. 변경은 자동 마크로만 남긴다.
        for j in self.motor_ids:
            live_b = base.get(j)
            if j not in self._base_fallback:
                b = live_b if live_b is not None else present.get(j)
                if b is not None:
                    self._base_fallback[j] = int(b)
                    self._live_base[j] = live_b
            elif live_b is not None and self._live_base.get(j) != live_b:
                self._live_base[j] = live_b
                self.mark(f"base_changed_j{j}_{live_b}")

        vals = [f"{t:.3f}", "",
                self._fmt(snap.get("leader_age")),
                self._fmt(snap.get("imu_age")),
                str(int(snap.get("read_fail") or 0))]
        for j in self.motor_ids:
            lraw = leader_raw.get(j)
            lref = self._leader_ref.get(j)
            if lraw is not None and lref is not None:
                d = wrap_ticks(int(lraw) - int(lref))
                ldeg = d * 360.0 / 4096.0 * self.leader_sign.get(j, 1.0) * self.leader_scale.get(j, 1.0)
            else:
                ldeg = math.nan
            b = self._base_fallback.get(j)   # latch된 베이스만 사용 (live 사용 금지)
            praw = present.get(j)
            pdeg = ((int(praw) - int(b)) * 360.0 / 4095.0) if (praw is not None and b is not None) else math.nan
            g = goal.get(j)
            gdeg = ((int(g) - int(b)) * 360.0 / 4095.0) if (g is not None and b is not None) else math.nan
            c = target.get(j, math.nan)
            vals += [self._fmt(ldeg), self._fmt(c), self._fmt(gdeg), self._fmt(pdeg),
                     str(lraw) if lraw is not None else "",
                     str(praw) if praw is not None else ""]
        self._fh.write(",".join(vals) + "\n")
        self._rows += 1
        if self._rows % 200 == 0:  # ~10초마다 디스크 보존
            self._fh.flush()

    @staticmethod
    def _fmt(v) -> str:
        if v is None:
            return ""
        try:
            f = float(v)
        except (TypeError, ValueError):
            return ""
        if math.isnan(f):
            return ""
        return f"{f:.3f}"

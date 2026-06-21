#!/usr/bin/env python3
"""보정(calibration) 축 + IK 로 IMU 상대회전 → 로봇 관절각.

viz(시뮬)에서 검증된 로직을 그대로 포팅:
- 각 IMU의 상대 쿼터니언(q_rel, 운동학 체인으로 계산된 것)을 받아,
- 보정된 3축(a=1동작, b=2동작, c=비틀)에 대해 Gauss-Newton IK 로
  R(a,θ1)·R(b,θ2)·R(c,θ3) = q_rel 을 만족하는 관절각을 구한다.
- 오일러 분해의 특이점/교차오염이 없음(비직각 축도 정확 분리).

calibration.json 의 axes 를 로드해서 사용.
"""

import json
import math
import os

# ── 쿼터니언/벡터 ──
def qmul(a, b):
    return [a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]]

def qconj(q):
    return [q[0], -q[1], -q[2], -q[3]]

def _aqr(ax, r):  # axis-angle(rad) → quat
    h = r / 2.0; s = math.sin(h)
    return [math.cos(h), ax[0]*s, ax[1]*s, ax[2]*s]

def _qrot(q, v):  # rotate vector by quaternion
    t = qmul(qmul(q, [0, v[0], v[1], v[2]]), qconj(q))
    return [t[1], t[2], t[3]]

def _rotvec(q):  # quaternion → rotation vector (axis*angle, rad)
    qq = [-x for x in q] if q[0] < 0 else q
    s = math.hypot(qq[1], qq[2], qq[3])
    if s < 1e-9:
        return [0.0, 0.0, 0.0]
    a = 2.0 * math.atan2(s, qq[0])
    return [qq[1]/s*a, qq[2]/s*a, qq[3]/s*a]

def _det3(m):
    return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
            + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))

def _solve3(cols, e):  # [col0|col1|col2]·x = e  (Cramer)
    m = [[cols[0][0], cols[1][0], cols[2][0]],
         [cols[0][1], cols[1][1], cols[2][1]],
         [cols[0][2], cols[1][2], cols[2][2]]]
    D = _det3(m)
    if abs(D) < 1e-9:
        return [0.0, 0.0, 0.0]
    def cd(i):
        mm = [row[:] for row in m]
        for r in range(3):
            mm[r][i] = e[r]
        return _det3(mm)
    return [cd(0)/D, cd(1)/D, cd(2)/D]

def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def _norm(a):
    n = math.hypot(a[0], a[1], a[2]) or 1.0
    return [a[0]/n, a[1]/n, a[2]/n]

def twist_angle(q, axis):
    """q 의 axis 기준 비틀기 각도(rad). atan2라 항상 (-π,π] 안 → 감김/발산 불가."""
    p = q[1]*axis[0] + q[2]*axis[1] + q[3]*axis[2]
    return 2.0 * math.atan2(p, q[0])

def hinge_angle(q, axis):
    """1-DOF 경첩(팔꿈치): 회전 '크기' = 굽힘각 → 보정축이 어긋나도 언더벤드 없음.
    부호만 축 투영으로 결정. (twist_angle 은 cos(축오차)만큼 작게 나오는 문제 있음)"""
    qv = (q[1], q[2], q[3])
    mag = 2.0 * math.atan2(math.hypot(qv[0], qv[1], qv[2]), q[0])  # 총 회전각(축 무관)
    s = qv[0]*axis[0] + qv[1]*axis[1] + qv[2]*axis[2]
    return mag if s >= 0 else -mag

def ortho_triad(a, b):
    """축 a,b 로 직교 삼각대 (e1=a, e2⊥a, e3=e1×e2)."""
    e1 = _norm(a)
    bb = [b[0]-_dot(b, e1)*e1[0], b[1]-_dot(b, e1)*e1[1], b[2]-_dot(b, e1)*e1[2]]
    e2 = _norm(bb)
    e3 = _cross(e1, e2)
    return e1, e2, e3

def seq_swing_twist(q, e1, e2, e3):
    """메모리 없는 순차 swing-twist (직교 삼각대). 매 프레임 새로 계산 → 워밍스타트 없음.
    각 성분 atan2 라 (-π,π] 바운드 → 감김/가지점프 구조적으로 불가. (정밀 분해의 '시드'로 사용)"""
    out = []
    r = q
    for ax in (e1, e2, e3):
        t = twist_angle(r, ax)
        out.append(t)
        r = qmul(qconj(_aqr(ax, t)), r)  # 그 축 회전 제거
    return out  # rad

def _canon(q):
    """쿼터니언 더블커버 정규화: w≥0 반구로 → 프레임간 부호 점프(±풀스케일) 방지.
    w==0(정확히 180° 회전)은 첫 비영 성분 부호로 타이브레이크(인접프레임 ±π 점프 방지)."""
    if q[0] < 0:
        return [-q[0], -q[1], -q[2], -q[3]]
    if q[0] == 0.0:
        for v in (q[1], q[2], q[3]):
            if v != 0.0:
                return list(q) if v > 0 else [-q[0], -q[1], -q[2], -q[3]]
    return list(q)

# decompose 안전 파라미터 (fact-checker 3차 NO-GO→GO-WITH-CHANGES)
# ※ λ는 행렬 비특이성 보장용일 뿐(λ=1e-3 워스트스텝 ~|err|/2√λ≈2800° 가능) —
#   실제 바운드 메커니즘은 DECOMP_STEP_MAX + DECOMP_OUT_BAND. λ 믿고 클램프 줄이지 말 것!
DECOMP_LAMBDA   = 1e-3                 # LM 댐핑 → 특이점서도 행렬 풀림(비특이성)
DECOMP_STEP_MAX = math.radians(30.0)   # iter당 스텝 상한
# 시작점별 밴드: 멀티스타트 그리드 간격(180°)의 절반+여유 = 110°. 안전은 이제 밴드가 아니라
#   ①FK잔차 게이트(가짜 해 탈락) ②출력 ±180 랩 ③앱단 클램프가 담당 (6차 FC 구조).
DECOMP_OUT_BAND = math.radians(110.0)
DECOMP_ITERS    = 12
DECOMP_TOL      = 1e-4                  # 잔차 수렴 기준(rad)

def _solve_lm(cols, err, lam):
    """(JᵀJ+λI) d = Jᵀerr. J=[col0|col1|col2]. λ>0이라 항상 풀림(특이점서도 유한)."""
    M = [[_dot(cols[i], cols[j]) for j in range(3)] for i in range(3)]
    for i in range(3):
        M[i][i] += lam
    rhs = [_dot(cols[i], err) for i in range(3)]
    D = _det3(M)
    if abs(D) < 1e-12:
        return [0.0, 0.0, 0.0]
    def cd(k):
        mm = [row[:] for row in M]
        for r in range(3):
            mm[r][k] = rhs[r]
        return _det3(mm)
    return [cd(0)/D, cd(1)/D, cd(2)/D]

# ── 멀티스타트 분해 (6차 fact-checker GO-WITH-CHANGES 반영, 2026-06-11) ──────
# 지그 실측서 발견: 시드±60° 단일 밴드는 큰 손목 복합회전에서 진짜 해를 밴드 밖에
# 가둠(8.8% 프레임 FK잔차>5°, 최대 41°) + 특이자세 통과 시 가지(branch) 점프.
# 해결: 시드 + 듀얼 시프트(롤축 ±180°) 여러 시작점에서 LM → FK잔차 게이트(<2°)로
# '진짜 해'만 후보로 → (stateless) 정준해 선택 / (stateful) 이전 프레임 연속 가지 선택.
# 메모리는 '이산 가지 선택'에만 사용 — 반복 워밍스타트가 아니므로 감김(2326°) 구조적 불가.
# 모든 출력은 (-180,180]로 랩 → 종전보다 더 강한 바운드.

DECOMP_FK_TOL      = math.radians(2.0)   # 이 잔차 미만이어야 '진짜 해' 후보로 인정
DECOMP_LOST_RAD    = math.radians(60.0)  # 이전 출력과 전 후보가 이보다 멀면 '추적 상실'→정준해
DECOMP_MEM_GAP_SEC = 0.5                 # 데이터 공백 이 이상이면 연속성 메모리 무효(FC#4)
# 듀얼 시프트: 1·3축(롤 성격)에 ±180° 조합 → 비직각축에서도 LM이 근처 진짜 해로 수렴
_DUAL_SHIFTS = [(0.0, 0.0, 0.0)] + [
    (s1 * math.pi, 0.0, s3 * math.pi)
    for s1 in (-1, 0, 1) for s3 in (-1, 0, 1) if not (s1 == 0 and s3 == 0)
]

def _wrap_rad(a):
    return math.atan2(math.sin(a), math.cos(a))

def _fk_res(q, e1, e2, e3, th):
    qfk = qmul(qmul(_aqr(e1, th[0]), _aqr(e2, th[1])), _aqr(e3, th[2]))
    return math.hypot(*_rotvec(qmul(q, qconj(qfk))))

def _lm_refine(q, e1, e2, e3, start):
    """단일 시작점 LM 다듬기. 밴드는 '해당 시작점'±60°(후보별 밴드, FC#2).
    반환 (th, res) — NaN이면 None."""
    th = list(start)
    best, best_res = list(th), float("inf")
    for _ in range(DECOMP_ITERS):
        q0 = _aqr(e1, th[0]); q1 = _aqr(e2, th[1]); q2 = _aqr(e3, th[2])
        qfk = qmul(qmul(q0, q1), q2)
        err = _rotvec(qmul(q, qconj(qfk)))
        res = math.hypot(err[0], err[1], err[2])
        if res < best_res:
            best_res, best = res, list(th)
        if res < DECOMP_TOL:
            break
        d = _solve_lm([e1, _qrot(q0, e2), _qrot(qmul(q0, q1), e3)], err, DECOMP_LAMBDA)
        dn = math.hypot(d[0], d[1], d[2])
        if dn > DECOMP_STEP_MAX:
            sc = DECOMP_STEP_MAX / dn
            d = [d[0]*sc, d[1]*sc, d[2]*sc]
        th = [th[i] + d[i] for i in range(3)]
        for i in range(3):
            lo, hi = start[i] - DECOMP_OUT_BAND, start[i] + DECOMP_OUT_BAND
            th[i] = lo if th[i] < lo else (hi if th[i] > hi else th[i])
    res_th = _fk_res(q, e1, e2, e3, th)
    out, res = (th, res_th) if res_th <= best_res else (best, best_res)
    if not all(math.isfinite(v) for v in out):
        return None
    return [_wrap_rad(v) for v in out], res

def decompose_candidates(q, e1, e2, e3):
    """모든 시작점에서 LM → 중복 제거 → FK잔차<2° 통과 후보 리스트.
    통과 후보가 없으면(예: 진짜 특이/데이터 이상) 최소잔차 1개를 반환(플래그용 res 포함)."""
    q = _canon(q)
    seed = list(seq_swing_twist(q, e1, e2, e3))
    starts = []
    for sh in _DUAL_SHIFTS:
        base = [seed[0] + sh[0], seed[1], seed[2] + sh[2]]
        starts.append(base)
        # 듀얼 가지의 θ2는 거울상(-θ2): 롤-피치-롤 듀얼 (θ1+π, -θ2, θ3+π) 일반화
        starts.append([base[0], -seed[1], base[2]])
    cands = []
    for st in starts:
        r = _lm_refine(q, e1, e2, e3, st)
        if r is None:
            continue
        th, res = r
        dup = False
        for k, (cth, cres) in enumerate(cands):
            if max(abs(_wrap_rad(th[i] - cth[i])) for i in range(3)) < math.radians(1.0):
                if res < cres:
                    cands[k] = (th, res)
                dup = True
                break
        if not dup:
            cands.append((th, res))
    if not cands:
        return [(([0.0, 0.0, 0.0]), float("inf"))]
    good = [c for c in cands if c[1] < DECOMP_FK_TOL]
    return good if good else [min(cands, key=lambda c: c[1])]

def decompose(q, e1, e2, e3):
    """무상태(stateless) 분해: 멀티스타트 후보 중 '정준해'(최소 노름) 선택.
    호환 API — 연속성(가지 추적)은 CalibIK 인스턴스 쪽에서 수행."""
    cands = decompose_candidates(q, e1, e2, e3)
    return min(cands, key=lambda c: (round(max(abs(v) for v in c[0]), 6), c[1]))[0]

def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


# ── 2-포즈 재정렬 (장착 틀어짐 보정, fact-checker GO-WITH-CHANGES 반영) ──────
# 원리: 보정축은 "센서 프레임" 기준이라 밴드를 다르게 차면 어긋난다.
# 같은 신체 자세(P1=차렷, P2=앞으로나란히)에서 중력 방향(센서 프레임)을
# 앵커(보정 직후) vs 지금 비교하면, 장착 변화 회전 C 를 구할 수 있고
# (중력 비교는 바라보는 방향/지자기 yaw에 불변), 모든 보정축에 C 를 적용하면 됨.

REANCHOR_MIN_SEP_DEG = 30.0   # P1-P2 중력방향 분리 최소각 (퇴화 방지)
REANCHOR_PRESERVE_TOL = 8.0   # |∠(u1,u2)-∠(v1,v2)| 허용 (자세 재현 실패 검출)
REANCHOR_NOOP_DEG = 3.0       # 이보다 작은 보정은 노이즈 → 적용 안 함
REANCHOR_WARN_DEG = 30.0      # 경고
REANCHOR_REJECT_DEG = 45.0    # 거부 (뭔가 잘못됨)
REANCHOR_RESID_MAX = 10.0     # C·u2 vs v2 잔차 허용


def gravity_from_quat(q):
    """센서 절대 쿼터니언 → 센서 프레임에서 본 월드 '위(ẑ)' 방향.
    앵커/재정렬 양쪽 모두 반드시 이 함수만 사용(규약 일관성)."""
    n = math.hypot(q[0], q[1], q[2], q[3]) or 1.0
    qn = [q[0]/n, q[1]/n, q[2]/n, q[3]/n]
    return _qrot(qconj(qn), [0.0, 0.0, 1.0])


def angle_deg(u, v):
    d = max(-1.0, min(1.0, _dot(_norm(u), _norm(v))))
    return math.degrees(math.acos(d))


def _matvec(C, v):
    return [_dot(C[0], v), _dot(C[1], v), _dot(C[2], v)]


def _rot_angle_deg(C):
    tr = C[0][0] + C[1][1] + C[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def mount_rotation(u1, u2, v1, v2):
    """TRIAD (P1 기준 고정): v_i = C·u_i 인 회전 C 를 구한다.
    반환 (C, report) — 게이트 실패 시 (None, report{reason})."""
    u1, u2, v1, v2 = _norm(u1), _norm(u2), _norm(v1), _norm(v2)
    sep_u, sep_v = angle_deg(u1, u2), angle_deg(v1, v2)
    rep = {"sep_u": sep_u, "sep_v": sep_v}
    if sep_u < REANCHOR_MIN_SEP_DEG or sep_v < REANCHOR_MIN_SEP_DEG:
        rep["reason"] = f"자세 분리각 부족(u={sep_u:.0f}° v={sep_v:.0f}°<{REANCHOR_MIN_SEP_DEG:.0f}°)"
        return None, rep
    if abs(sep_u - sep_v) > REANCHOR_PRESERVE_TOL:
        rep["reason"] = f"자세 재현 실패(분리각 차 {abs(sep_u-sep_v):.0f}°>{REANCHOR_PRESERVE_TOL:.0f}°)"
        return None, rep

    def triad(w1, w2):
        t1 = _norm(w1)
        t2 = _norm(_cross(w1, w2))
        t3 = _cross(t1, t2)
        return t1, t2, t3
    tu, su = triad(u1, u2), triad(v1, v2)
    # C = S · Tᵀ  (열벡터 기준) → 행렬 원소로 직접 구성
    C = [[sum(su[k][r] * tu[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    rep["rot_deg"] = _rot_angle_deg(C)
    rep["resid_deg"] = angle_deg(_matvec(C, u2), v2)
    if rep["resid_deg"] > REANCHOR_RESID_MAX:
        rep["reason"] = f"잔차 과대({rep['resid_deg']:.0f}°>{REANCHOR_RESID_MAX:.0f}°)"
        return None, rep
    return C, rep


class CalibIK:
    """보정값 보관 + q_rel → 모터각 계산."""

    def __init__(self, axes, anchor=None):
        # 원본 축(불변, 풀 보정값). 재정렬은 항상 원본에 적용(세션 누적 금지).
        self._axes0 = {imu: {k: list(v) for k, v in A.items()} for imu, A in axes.items()}
        self.axes = {imu: {k: list(v) for k, v in A.items()} for imu, A in axes.items()}
        self.anchor = anchor  # {"imu1":{"p1":[gx,gy,gz],"p2":[...]}, ...} 보정 직후 중력방향
        self._rebuild_triads()

    def _rebuild_triads(self):
        # 진짜 측정축(a,b,c) 그대로 사용 — 직교화 안 함!
        #   직교화는 옛 순수 swing-twist용. decompose(IK)는 비직각축도 정확히 풀어서
        #   직교화하면 오히려 진짜 축을 비틀어 교차오염 유발(순수 벌림→j1 11° 누수).
        #   c 없으면(이론상) a×b 로 대체.
        self._tri = {}
        for imu in ("imu1", "imu3"):
            A = self.axes.get(imu, {})
            if "a" in A and "b" in A:
                e1 = _norm(A["a"])
                e2 = _norm(A["b"])
                e3 = _norm(A["c"]) if "c" in A else _norm(_cross(e1, e2))
                self._tri[imu] = (e1, e2, e3)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(data["axes"], data.get("anchor"))

    def save(self, path):
        """원본 축 + 앵커만 저장 (재정렬된 세션 축은 저장하지 않음)."""
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"axes": self._axes0, "anchor": self.anchor}, f, indent=1)

    def has_anchor(self):
        return bool(self.anchor) and all(
            imu in self.anchor and "p1" in self.anchor[imu] and "p2" in self.anchor[imu]
            for imu in ("imu1", "imu2", "imu3"))

    def set_anchor(self, gpairs):
        """gpairs = {"imu1":{"p1":[...],"p2":[...]}, ...} (보정이 유효한 직후에만 호출!)"""
        self.anchor = gpairs

    def apply_reanchor(self, gpairs):
        """현 세션 중력쌍으로 장착변화 C(IMU별)를 구해 원본 축에 적용.
        반환: {imu: report}. report["status"] = applied/noop/rejected."""
        out = {}
        new_axes = {imu: {k: list(v) for k, v in A.items()} for imu, A in self._axes0.items()}
        for imu in ("imu1", "imu2", "imu3"):
            anc = (self.anchor or {}).get(imu)
            cur = gpairs.get(imu)
            if not anc or not cur:
                out[imu] = {"status": "rejected", "reason": "앵커/캡처 없음"}
                continue
            C, rep = mount_rotation(anc["p1"], anc["p2"], cur["p1"], cur["p2"])
            if C is None:
                rep["status"] = "rejected"
                out[imu] = rep
                continue
            if rep["rot_deg"] > REANCHOR_REJECT_DEG:
                rep["status"] = "rejected"
                rep["reason"] = f"장착변화 {rep['rot_deg']:.0f}°>{REANCHOR_REJECT_DEG:.0f}° (재보정 필요)"
                out[imu] = rep
                continue
            if rep["rot_deg"] < REANCHOR_NOOP_DEG:
                rep["status"] = "noop"  # 노이즈 수준 → 원본 그대로
                out[imu] = rep
                continue
            for k, v in self._axes0[imu].items():
                new_axes[imu][k] = _norm(_matvec(C, v))
            rep["status"] = "applied"
            rep["warn"] = rep["rot_deg"] > REANCHOR_WARN_DEG
            out[imu] = rep
        self.axes = new_axes
        self._rebuild_triads()
        return out

    def is_calibrated(self):
        a = self.axes
        return all(k in a for k in ("imu1", "imu2", "imu3")) and \
            "a" in a["imu1"] and "b" in a["imu1"] and "a" in a["imu3"] and "b" in a["imu3"]

    def _axis(self, imu, key):
        A = self.axes.get(imu, {})
        if key in ("a", "b"):
            return A.get(key)
        # c: 측정값 있으면 사용, 없으면 a×b
        if "c" in A:
            return A["c"]
        if "a" in A and "b" in A:
            return _norm(_cross(A["a"], A["b"]))
        return None

    def reset(self):
        """기준 재설정 시 가지 연속성 메모리 초기화 (다음 프레임은 정준해부터)."""
        self._cont = {}
        self._cont_time = {}

    def _decompose_cont(self, key, q, tri):
        """가지 연속성 분해: 후보(전부 '진짜 해') 중 이전 출력에 가장 가까운 가지 선택.
        - 메모리는 이산 선택에만 사용(워밍스타트 아님) → 감김 불가, 출력 ±180 바운드
        - 첫 프레임/리셋 후/공백(>0.5s) 후엔 정준해(최소 노름) (FC#4·#6)
        - 전 후보가 이전 출력에서 60° 이상 멀면 추적상실 → 정준해 + 메모리 재시작"""
        import time as _time
        now = _time.time()
        cands = decompose_candidates(q, *tri)
        if not hasattr(self, "_cont"):
            self._cont, self._cont_time = {}, {}
        prev = self._cont.get(key)
        fresh = prev is not None and (now - self._cont_time.get(key, 0.0)) <= DECOMP_MEM_GAP_SEC

        def canonical():
            return min(cands, key=lambda c: (round(max(abs(v) for v in c[0]), 6), c[1]))[0]

        if not fresh:
            th = canonical()
        else:
            def dist(c):
                return max(abs(_wrap_rad(c[0][i] - prev[i])) for i in range(3))
            best = min(cands, key=dist)
            th = best[0] if dist(best) <= DECOMP_LOST_RAD else canonical()
        # 선택된 해의 FK잔차 기록 (표현불가/특이 감지용 — 앱이 로그로 노출)
        sel_res = min(c[1] for c in cands if c[0] == th) if cands else float("inf")
        if not hasattr(self, "last_res_deg"):
            self.last_res_deg = {}
        self.last_res_deg[key] = math.degrees(sel_res)
        self._cont[key] = th
        self._cont_time[key] = now
        return th

    def axis_quality(self, min_deg=30.0):
        """보정축 조건수 점검(Q5). 축 사이각이 너무 작으면 분해 부정확+조건수 악화.
        반환: 경고 문자열 리스트(없으면 []). 전달 시작 시 사용자에게 표시."""
        warns = []
        for imu, nm in (("imu1", "어깨"), ("imu3", "손목")):
            A = self.axes.get(imu, {})
            if not all(k in A for k in "abc"):
                continue
            a, b, c = A["a"], A["b"], A["c"]
            angs = [angle_deg(a, b), angle_deg(a, c), angle_deg(b, c)]
            mn = min(angs)
            if mn < min_deg:
                warns.append(f"{nm} 축 사이각 최소 {mn:.0f}°<{min_deg:.0f}° → 분해 부정확 가능, 재보정 권장")
        return warns

    def motor_angles_deg(self, qrel):
        """qrel = {"imu1":[w,x,y,z], "imu2":..., "imu3":...} → {motor_id: deg}.
        메모리 없는 직교 swing-twist (감김/발산 불가). 매핑: j1-3=어깨, j4=팔꿈치,
        j5=손목3축, j6=손목1축(위아래), j7=손목2축(좌우)."""
        R2D = 180.0 / math.pi
        a2 = self._axis("imu2", "a")

        # 어깨/손목: 멀티스타트 분해 + 가지 연속성(이산 선택만 → 감김 불가)
        sh = [0.0, 0.0, 0.0]
        if "imu1" in self._tri and qrel.get("imu1"):
            sh = self._decompose_cont("imu1", qrel["imu1"], self._tri["imu1"])
        wr = [0.0, 0.0, 0.0]
        if "imu3" in self._tri and qrel.get("imu3"):
            wr = self._decompose_cont("imu3", qrel["imu3"], self._tri["imu3"])
        # 팔꿈치: 축 투영(twist_angle) — 부호있고 ±180 바운드, 연속(0 근처 떨림 없음),
        #   전완 회내(pronation)를 굴곡으로 오인하지 않고 거름(손목 체인으로 감). (fact-checker 3차)
        #   ※ 보정축이 잘 맞아야 정확. 덜 굽으면 팔꿈치축 재보정(윗팔 고정, 회내 없이 굴곡만).
        el = twist_angle(_canon(qrel["imu2"]), a2) if (a2 and qrel.get("imu2")) else 0.0

        return {
            1: sh[0]*R2D, 2: sh[1]*R2D, 3: sh[2]*R2D,
            4: el*R2D,
            5: wr[2]*R2D, 6: wr[0]*R2D, 7: wr[1]*R2D,
        }

#!/usr/bin/env python3
"""IMU 텔레옵 디버그 시각화 브리지 (three.js 프런트엔드와 짝).

ESP32가 보내는 UDP(4210) 패킷을 받아 브라우저로 SSE(Server-Sent Events)로 전달한다.
브라우저(index.html)에서 IMU 3개의 자세를 3D 축(triad)으로 표시 →
짐벌락 / 축 매핑 / 수신율(지연) 을 눈으로 디버깅.

실행:
    python3 viz/bridge.py
    → 브라우저에서 http://localhost:8000 열기

주의: 메인 앱(esp_imu_pc_control3.py)과 UDP 4210 을 동시에 점유할 수 없으니,
      이 디버그 툴을 쓸 땐 메인 앱은 끄고 이 브리지만 실행.

의존성: 파이썬 표준 라이브러리만 (pip 설치 불필요)
"""

import json
import math
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UDP_PORT = 4210
HTTP_PORT = 8000
HERE = os.path.dirname(os.path.abspath(__file__))

# 실물 앱과 동일한 최신 분해기(calib_ik) 사용 — viz의 구버전 JS IK(워밍스타트, j6/j7 스왑)
# 대신 esp_imu_pc_control3.py와 같은 파이프라인으로 OpenArm URDF를 구동해 매핑/부호 검증.
CALIB_DIR = os.path.abspath(os.path.join(HERE, "..", "work_process", "esp_imu_pc_control"))
sys.path.insert(0, CALIB_DIR)
try:
    from calib_ik import CalibIK, qmul, qconj
    _HAS_CALIB = True
except Exception as _e:  # noqa
    print(f"[bridge] calib_ik import 실패({_e}) — 분해 비활성, raw만 전송")
    _HAS_CALIB = False

_state = {"data": {}, "t": 0.0, "rate": 0.0, "joints": None}
_lock = threading.Lock()

# 실물 앱과 동일한 출력 안전장치 — 분해 가지점프(어깨 벌림 임계 등)를 가림.
# bridge엔 원래 없어서 viz에 '팍팍' 점프가 raw로 보였음(viz 검증으로 발견).
MAX_VEL_DEG_S = 180.0   # 속도클램프 (esp MAX_VEL_DEG_S)
JUMP_DEG = 45.0         # 점프 디바운스 임계
JUMP_CONFIRM = 3        # N프레임 연속 점프면 확정(진짜 동작) → 추종
_smooth = {"prev": None, "t": None, "jcnt": {i: 0 for i in range(1, 8)}}


def smooth_joints(j):
    """속도클램프 + 점프 디바운스 (esp _relay_send_step 등가). 가지점프를 천천히 추종/보류."""
    now = time.time()
    if _smooth["prev"] is None:
        _smooth["prev"] = dict(j)
        _smooth["t"] = now
        return j
    dt = max(0.001, min(0.3, now - _smooth["t"]))
    _smooth["t"] = now
    max_step = MAX_VEL_DEG_S * dt
    out = {}
    for i in range(1, 8):
        tgt = j.get(i, 0.0)
        prev = _smooth["prev"][i]
        if abs(tgt - prev) > JUMP_DEG:           # 점프 디바운스: 확정 전엔 보류
            _smooth["jcnt"][i] += 1
            if _smooth["jcnt"][i] < JUMP_CONFIRM:
                out[i] = prev
                continue
        else:
            _smooth["jcnt"][i] = 0
        d = tgt - prev                            # 속도클램프
        out[i] = prev + max(-max_step, min(max_step, d))
    _smooth["prev"] = dict(out)
    return out


# ── 분해 파이프라인 (esp_imu_pc_control3.py와 동일) ────────────────────────────
_ik = {
    "calib": None,                 # CalibIK
    "ref": None,                   # {imu1:[w,x,y,z], imu2:.., imu3:..}
    "imu1_abs": None, "imu2_abs": None,
    "prev_quat": {"imu1": None, "imu2": None, "imu3": None},
}


def _euler_deg_to_quat(roll, pitch, yaw):
    r, p, y = (math.radians(roll) / 2.0, math.radians(pitch) / 2.0, math.radians(yaw) / 2.0)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def _ensure_continuity(q_cur, q_prev):
    if q_prev is None:
        return q_cur
    if sum(a * b for a, b in zip(q_cur, q_prev)) < 0.0:
        return [-v for v in q_cur]
    return q_cur


def _read_imu_quat(pkt, prefix):
    """센서 직접 쿼터니언(짐벌락 회피). NaN/노름 가드 — 없으면 None."""
    vals = [pkt.get(f"{prefix}_q{k}") for k in range(4)]
    if any(v is None for v in vals):
        return None
    try:
        raw = [float(v) for v in vals]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in raw):
        return None
    n = math.sqrt(sum(v * v for v in raw))
    if not (0.5 < n < 2.0):
        return None
    q = [v / n for v in raw]
    if abs(q[0] - 1.0) < 1e-6 and abs(q[1]) < 1e-6 and abs(q[2]) < 1e-6 and abs(q[3]) < 1e-6:
        return None
    return q


def _imu_current_quat(pkt, prefix):
    q = _read_imu_quat(pkt, prefix)
    if q is not None:
        return q
    r = float(pkt.get(f"{prefix}_roll", pkt.get("roll", 0.0)) or 0.0)
    p = float(pkt.get(f"{prefix}_pitch", pkt.get("pitch", 0.0)) or 0.0)
    y = float(pkt.get(f"{prefix}_yaw", pkt.get("yaw", 0.0)) or 0.0)
    return _euler_deg_to_quat(r, p, y)


def load_calib():
    if not _HAS_CALIB:
        return
    path = os.path.join(CALIB_DIR, "calibration.json")
    try:
        _ik["calib"] = CalibIK.load(path)
        ok = _ik["calib"].is_calibrated()
        print(f"[bridge] calibration.json 로드 (보정됨={ok}) — [기준 설정] 후 분해 시작")
    except Exception as e:  # noqa
        print(f"[bridge] 보정 로드 실패({e}) — 분해 비활성, raw만 전송")


def set_reference(pkt):
    """기준 자세 캡처 (esp _capture_relay_reference 동등)."""
    ref = {k: _imu_current_quat(pkt, k) for k in ("imu1", "imu2", "imu3")}
    _ik["ref"] = ref
    _ik["imu1_abs"] = ref["imu1"]
    _ik["imu2_abs"] = ref["imu2"]
    _ik["prev_quat"] = {"imu1": None, "imu2": None, "imu3": None}
    _smooth["prev"] = None                       # 출력 평활 상태 리셋
    _smooth["jcnt"] = {i: 0 for i in range(1, 8)}
    if _ik["calib"]:
        _ik["calib"].reset()
    print("[bridge] 기준 자세 설정됨")


def compute_joints(pkt):
    """체인 상대 q_rel → calib_ik.motor_angles_deg. 부호 없는 raw 관절각(1~7) 반환.
    OpenArm 부호는 viz의 userSign으로 맞춤(=시뮬 부호 검증). 미보정/미기준이면 None."""
    calib = _ik["calib"]
    if calib is None or _ik["ref"] is None or not calib.is_calibrated():
        return None
    ref = _ik["ref"]
    qrel = {}
    for k in ("imu1", "imu2", "imu3"):   # 순서 필수 (imu1_abs/imu2_abs 의존)
        q_cur = _ensure_continuity(_imu_current_quat(pkt, k), _ik["prev_quat"][k])
        _ik["prev_quat"][k] = q_cur
        if k == "imu1":
            qr = qmul(qconj(ref["imu1"]), q_cur)
            _ik["imu1_abs"] = q_cur
        elif k == "imu2":
            q2lr = qmul(qconj(ref["imu1"]), ref["imu2"])
            q2lc = qmul(qconj(_ik["imu1_abs"]), q_cur)
            qr = qmul(qconj(q2lr), q2lc)
            _ik["imu2_abs"] = q_cur
        else:
            q3lr = qmul(qconj(ref["imu2"]), ref["imu3"])
            q3lc = qmul(qconj(_ik["imu2_abs"]), q_cur)
            qr = qmul(qconj(q3lr), q3lc)
        qrel[k] = qr
    try:
        return smooth_joints(calib.motor_angles_deg(qrel))   # 속도클램프+디바운스(실물 등가)
    except Exception as e:  # noqa
        print(f"[bridge] 분해 오류: {e}")
        return None


# ── 시뮬 검증 데이터 로깅 (보고서용 — sim_report.py 가 분석) ────────────────────
_log = {"active": False, "fh": None, "t0": 0.0, "rows": 0, "path": None,
        "sign": {}, "map": {}}
_LOG_DIR = os.path.join(HERE, "sim_logs")


def log_start(scenario, sign, jmap, t_now):
    os.makedirs(_LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(t_now))
    safe = "".join(c if c.isalnum() or c in "_-" else "_" for c in (scenario or "sim"))
    path = os.path.join(_LOG_DIR, f"{stamp}_{safe}.csv")
    fh = open(path, "w", encoding="utf-8")
    fh.write(f"# scenario={scenario}\n")
    fh.write(f"# sign={json.dumps(sign)}\n")
    fh.write(f"# map={json.dumps(jmap)}\n")    # OpenArm jointN ← calib_ik 출력 매핑
    cols = (["t", "ev"]
            + [f"imu{i}_q{k}" for i in (1, 2, 3) for k in range(4)]
            + [f"j{i}_raw" for i in range(1, 8)]   # calib_ik 원시 출력(부호/스왑 전)
            + ["res_imu1", "res_imu3"])            # FK잔차(표현불가/특이 감지)
    fh.write(",".join(cols) + "\n")
    _log.update(active=True, fh=fh, t0=t_now, rows=0, path=path,
                sign=sign or {}, map=jmap or {})
    print(f"[bridge] 시뮬 로깅 시작 [{scenario}] → {path}")
    return path


def log_mark(label):
    if _log["active"] and _log["fh"]:
        t = time.time() - _log["t0"]
        _log["fh"].write(f"{t:.3f},MARK:{label}," + "," * 24 + "\n")
        _log["fh"].flush()


def log_stop():
    p, n = _log.get("path"), _log["rows"]
    if _log["fh"]:
        _log["fh"].close()
    _log.update(active=False, fh=None)
    print(f"[bridge] 시뮬 로깅 종료 ({n}행) → {p}")
    return p, n


def log_row(pkt, joints):
    if not _log["active"] or not _log["fh"]:
        return
    t = time.time() - _log["t0"]
    row = [f"{t:.3f}", ""]
    for i in (1, 2, 3):
        q = _imu_current_quat(pkt, f"imu{i}")
        row += [f"{v:.5f}" for v in q]
    for i in range(1, 8):
        row.append(f"{joints.get(i, 0.0):.2f}" if joints else "")
    res = getattr(_ik["calib"], "last_res_deg", {}) if _ik["calib"] else {}
    row += [f"{res.get('imu1', 0.0):.2f}", f"{res.get('imu3', 0.0):.2f}"]
    _log["fh"].write(",".join(row) + "\n")
    _log["rows"] += 1


def udp_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    print(f"[bridge] UDP {UDP_PORT} 수신 대기")
    count = 0
    win_t = time.time()
    while True:
        try:
            buf, _ = sock.recvfrom(4096)
            pkt = json.loads(buf.decode("utf-8", "replace"))
        except (OSError, ValueError):
            continue
        if not isinstance(pkt, dict) or pkt.get("type") == "status":
            continue
        count += 1
        now = time.time()
        rate = 0.0
        if now - win_t >= 0.5:
            rate = count / (now - win_t)
            count = 0
            win_t = now
        joints = compute_joints(pkt)   # 패킷마다 분해(체인 상태/가지연속성 의존 → 단일 스레드)
        log_row(pkt, joints)           # 로깅 활성 시 기록
        with _lock:
            _state["data"] = pkt
            _state["t"] = now
            _state["joints"] = joints
            if rate:
                _state["rate"] = rate


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    CTYPES = {
        ".urdf": "application/xml", ".xacro": "application/xml",
        ".dae": "model/vnd.collada+xml", ".stl": "application/octet-stream",
        ".js": "text/javascript", ".html": "text/html; charset=utf-8",
        ".png": "image/png", ".jpg": "image/jpeg",
    }

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_file("index.html", "text/html; charset=utf-8")
        elif self.path == "/events":
            self._sse()
        else:
            self._serve_static(self.path)

    def _json_body(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
        except (ValueError, OSError):
            return {}

    def _reply(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/ref":
            with _lock:
                pkt = dict(_state["data"])
            ok = bool(pkt)
            if ok:
                set_reference(pkt)
            self._reply({"ok": ok}, 200 if ok else 409)
        elif self.path == "/log":
            b = self._json_body()
            act = b.get("action")
            if act == "start":
                p = log_start(b.get("scenario", "sim"), b.get("sign", {}),
                              b.get("map", {}), time.time())
                self._reply({"ok": True, "path": os.path.basename(p)})
            elif act == "mark":
                log_mark(b.get("label", "mark"))
                self._reply({"ok": True})
            elif act == "stop":
                p, n = log_stop()
                self._reply({"ok": True, "rows": n,
                             "path": os.path.basename(p) if p else None})
            else:
                self._reply({"ok": False, "err": "unknown action"}, 400)
        else:
            self.send_error(404)

    def _serve_static(self, urlpath):
        from urllib.parse import unquote
        rel = unquote(urlpath.split("?", 1)[0]).lstrip("/")
        full = os.path.normpath(os.path.join(HERE, rel))
        if not full.startswith(HERE) or not os.path.isfile(full):
            self.send_error(404)
            return
        ext = os.path.splitext(full)[1].lower()
        ctype = self.CTYPES.get(ext, "application/octet-stream")
        try:
            with open(full, "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_file(self, name, ctype):
        try:
            with open(os.path.join(HERE, name), "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                with _lock:
                    payload = json.dumps({
                        "data": _state["data"],
                        "joints": _state.get("joints"),   # bridge 분해(calib_ik) 결과 — 있으면 viz가 우선 사용
                        "ref_set": _ik["ref"] is not None,
                        "rate": round(_state["rate"], 1),
                        "age_ms": round((time.time() - _state["t"]) * 1000) if _state["t"] else -1,
                    })
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(1.0 / 30.0)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def main():
    load_calib()
    threading.Thread(target=udp_loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[bridge] 브라우저에서 http://localhost:{HTTP_PORT} 열기  (Ctrl+C 종료)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[bridge] 종료")


if __name__ == "__main__":
    main()

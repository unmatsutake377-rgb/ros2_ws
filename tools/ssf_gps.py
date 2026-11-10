"""GPS·RTK 상태 — 파라미터 앱 위쪽 'RTK' 불과 'ssf b gps' (ROS 불필요).

  감시 노드(tools/ssf_gps_watch.py, 구독 전용)를 tmux(ssf_gps_<배>)로 필요할 때 띄우고,
  그 노드가 남기는 ~/ssf_logs/gps_status_<배>.json 을 읽어 단계로 바꾼다.
  앱이 볼 때마다 ~/ssf_logs/.gps_watch_<배>.hb 를 건드린다 — 2 분 안 보면 감시 노드가 스스로 끝난다.

단계(level):
  fixed   🟢 RTK FIXED          — 웨이포인트 찍어도 됨
  float   🟡 RTK FLOAT          — 보정은 들어옴, FIXED 기다림
  single  🟠 GPS 단독(보정 없음) — NTRIP·인터넷(핫스팟) 확인
  nofix   🔴 위치 없음
  none    ⚫ GPS 데이터 없음      — GPS 드라이버(launch)가 안 돎·3 s 넘게 끊김·감시 노드 시작 중
"""
import json
import os
import time

STALE_SEC = 3.0
START_RETRY_SEC = 10.0
_last_start = {}


def session_name(boat):
    return os.environ.get("SSF_GPS_SESSION_PREFIX", "ssf_gps_") + boat


def _tmux(*args, timeout=5):
    import subprocess
    sock = os.environ.get("SSF_WP_TMUX_SOCKET")          # 테스트용 별도 tmux 서버(-L)
    base = ["tmux", "-L", sock] if sock else ["tmux"]
    try:
        return subprocess.run([*base, *args], capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        return None


def classify(data, now=None):
    """감시 노드 JSON → (level, 화면 글자)."""
    now = time.time() if now is None else now
    if not data:
        return "none", "⚫ GPS 데이터 없음"
    if data.get("error"):
        return "none", "⚫ GPS 감시 실패: " + str(data["error"])
    age = now - float(data.get("t", 0))
    if age > STALE_SEC:
        return "none", f"⚫ GPS 데이터 없음({int(age)} s 전이 마지막 — launch·GPS 드라이버 확인)"
    acc = data.get("h_acc_m")
    sv = data.get("num_sv")
    tail = (f" ±{acc * 100:.1f} cm" if acc is not None and acc < 1 else f" ±{acc:.1f} m" if acc is not None else "") + \
           (f" · 위성 {sv}" if sv is not None else "")
    if not data.get("fix_ok") or int(data.get("fix_type", 0)) < 2:
        return "nofix", "🔴 위치 없음" + (f" · 위성 {sv}" if sv is not None else "")
    carr = int(data.get("carr", 0))
    if carr == 2:
        return "fixed", "🟢 RTK FIXED" + tail
    if carr == 1:
        return "float", "🟡 RTK FLOAT" + tail
    return "single", "🟠 GPS 단독(보정 없음)" + tail


def _ensure_watch(repo, boat, home):
    """감시 노드가 없으면 띄운다(10 s 에 한 번까지만 시도)."""
    s = session_name(boat)
    r = _tmux("has-session", "-t", s)
    if r is None:
        return "tmux 없음"
    if r.returncode == 0:
        return None
    now = time.time()
    if now - _last_start.get(boat, 0) < START_RETRY_SEC:
        return None
    _last_start[boat] = now
    env = ["-e", f"HOME={home}"]
    for k in ("SSF_GPS_TOPIC", "ROS_DOMAIN_ID", "SSF_GPS_HB_MAX"):   # 시험용(격리 도메인·다른 토픽) — 평소엔 없음
        if os.environ.get(k):
            env += ["-e", f"{k}={os.environ[k]}"]
    cmd = ["bash", os.path.join(repo, "tools", "ssf_gps_watch.sh"), boat]
    r = _tmux("new-session", "-d", "-s", s, "-x", "100", "-y", "20", "-c", repo, *env, *cmd)
    if r is not None and r.returncode != 0 and "duplicate session" not in (r.stderr or ""):
        return "감시 노드 시작 실패: " + (r.stderr or "").strip()
    return None


def status(repo, boat, home):
    d = os.path.join(home, "ssf_logs")
    os.makedirs(d, exist_ok=True)
    hb = os.path.join(d, f".gps_watch_{boat}.hb")
    with open(hb, "a"):
        os.utime(hb, None)
    err = _ensure_watch(repo, boat, home)
    data = None
    try:
        with open(os.path.join(d, f"gps_status_{boat}.json"), encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        pass
    level, text = classify(data)
    if err and level == "none":
        text = "⚫ " + err
    out = {"level": level, "text": text, "fixed": level == "fixed"}
    if data and not data.get("error"):
        out.update({k: data.get(k) for k in ("h_acc_m", "num_sv", "lat", "lon", "carr", "fix_type")})
        out["age"] = round(time.time() - float(data.get("t", 0)), 1)
    return out


def stop(boat):
    s = session_name(boat)
    r = _tmux("has-session", "-t", s)
    if r is not None and r.returncode == 0:
        _tmux("kill-session", "-t", s)

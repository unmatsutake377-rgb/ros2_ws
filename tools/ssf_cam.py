"""SSF D455 영상 송신 켜기·끄기·상태 — 파라미터 앱 '영상' 탭과 'ssf b cam' 이 쓴다 (ROS 불필요).

  송신 자체는 기존 tools/cam_stream.sh 가 한다. 여기는 tmux 세션(ssf_cam_<배>) 안에서
  tools/ssf_cam_send.sh 를 켜고 끄고, 로그 끝과 D455 USB 연결 상태를 보여 준다.
  영상은 조종과 완전히 따로다 — 끄고 켜도 RC 조종·자율 노드에 영향 없음. 설정 파일은 건드리지 않는다.
"""
import ipaddress
import os
import subprocess
import time

PORTS = {"A": 5600, "B": 5601}


def session_name(boat):
    return os.environ.get("SSF_CAM_SESSION_PREFIX", "ssf_cam_") + boat


def _tmux(*args, timeout=5):
    sock = os.environ.get("SSF_WP_TMUX_SOCKET")          # 테스트용 별도 tmux 서버(-L) — 웨이포인트와 같은 값
    base = ["tmux", "-L", sock] if sock else ["tmux"]
    try:
        return subprocess.run([*base, *args], capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        raise RuntimeError("이 노트북에 tmux 가 없다")


def _run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return ""


def check_ip(ip):
    try:
        a = ipaddress.IPv4Address(str(ip).strip())
    except ValueError:
        raise RuntimeError(f"IP 형식이 아니다: {ip!r} — MacBook 의 Tailscale 주소(100.x.x.x)")
    if a.is_loopback or a.is_unspecified:
        raise RuntimeError("MacBook 주소를 넣어야 한다(배 자기 주소·127.0.0.1 은 안 됨)")
    return str(a)


def ssh_client_ip():
    """앱 서버를 띄운 SSH 접속의 상대(= MacBook) 주소. 없으면 None."""
    v = os.environ.get("SSH_CLIENT") or os.environ.get("SSH_CONNECTION") or ""
    ip = v.split()[0] if v.split() else ""
    try:
        return check_ip(ip)
    except RuntimeError:
        return None


def status(boat, home, lines=25):
    s = session_name(boat)
    try:
        running = _tmux("has-session", "-t", s).returncode == 0
    except RuntimeError:          # tmux 없는 컴퓨터(연습용) — 상태는 '꺼짐'으로 보여 준다
        running = False
    log =os.path.join(home, "ssf_logs", f"cam_stream_{boat}.log")
    tail = []
    try:
        with open(log, encoding="utf-8", errors="replace") as f:
            tail = f.read().splitlines()[-lines:]
    except OSError:
        pass
    usb = [ln.strip() for ln in _run(["lsusb"]).splitlines() if "realsense" in ln.lower()]
    node = bool(_run(["pgrep", "-f", "realsense2_camera"]).strip())
    return {"running": running, "session": s, "port": PORTS[boat], "log": tail, "log_path": log,
            "usb": usb, "camera_node": node, "mode": "ros" if node else "v4l2", "mac_ip": ssh_client_ip()}


def start(repo, boat, home, host_ip):
    ip = check_ip(host_ip)
    s = session_name(boat)
    if _tmux("has-session", "-t", s).returncode == 0:
        raise RuntimeError(f"이미 송신 중(tmux {s}) — 보기만: ssf {boat.lower()} cam view · 끄기: ssf {boat.lower()} cam stop")
    cmd = ["bash", os.path.join(repo, "tools", "ssf_cam_send.sh"), boat, ip]
    r = _tmux("new-session", "-d", "-s", s, "-x", "120", "-y", "30", "-c", repo, "-e", f"HOME={home}", *cmd)
    if r.returncode != 0 and "duplicate session" in (r.stderr or ""):
        raise RuntimeError(f"이미 송신 중(tmux {s}) — 보기만: ssf {boat.lower()} cam view · 끄기: ssf {boat.lower()} cam stop")
    if r.returncode != 0:
        raise RuntimeError("tmux 시작 실패: " + (r.stderr or r.stdout).strip())
    time.sleep(0.5)
    return status(boat, home)


def stop(boat, home):
    """Ctrl-C(cam_stream.sh 가 스스로 정리) → 2 s 안에 안 끝나면 세션째 끈다."""
    s = session_name(boat)
    if _tmux("has-session", "-t", s).returncode != 0:
        raise RuntimeError("송신 중이 아니다")
    _tmux("send-keys", "-t", s, "C-c")
    for _ in range(20):
        time.sleep(0.1)
        if _tmux("has-session", "-t", s).returncode != 0:
            break
    else:
        _tmux("kill-session", "-t", s)
    return status(boat, home)

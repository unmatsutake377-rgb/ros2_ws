"""SSF 자율 launch 켜기·끄기·상태 — 파라미터 앱 'launch' 탭이 쓴다 (2026-11-12).

  launch 는 tmux 세션 ssf_launch_<배> 안에서 tools/ssf_launch_run.sh 로 돈다(손으로 칠 때와 같은 명령).
  앱이 SSH 와 끊겨도 launch 는 배에서 계속 돈다.

지키는 것
  · 배 글자(boat:=)는 이 노트북에 등록된 배(~/.ssf_boat)로만 — 고르지 않는다(Repo.boat() 가 확인).
  · launch 가 이미 떠 있으면(앱이든 터미널이든) 켜기 거부 — 두 개가 뜨면 같은 토픽에 두 번 발행한다.
  · 켜기 전 boat_boot 확인: CPU 거버너 performance · USB autosuspend -1. 둘 다 sudo boat_boot.sh 가 걸고
    재부팅하면 풀린다 → '오늘 부팅 뒤 boat_boot 를 돌렸나'의 증거로 쓴다. 안 됐으면 켜기 거부 + 칠 명령 안내.
    (sudo 비밀번호는 앱이 다루지 않는다 — 사람이 하루 한 번 직접.)
  · 끄기는 Ctrl+C(SIGINT) 한 번만. 강제 종료(kill-session) 하지 않는다 — 라이다 정지 명령(boat_launch_checklist §4).
    앱이 띄운 것만 끈다. 터미널(tmux ssf 등)에서 띄운 launch 는 그 창에서 끄라고 안내.
  · 웨이포인트 파일은 ~/ssf_logs 의 *.yaml 이름만 고른다(경로를 받지 않는다).
  · 모드 전환(자율 출발)은 하지 않는다 — 조종기로만.
"""
import glob
import os
import re
import subprocess
import time

COURSES = ("main", "tournament")
WP_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,80}\.ya?ml")


def session_name(boat):
    return os.environ.get("SSF_LAUNCH_SESSION_PREFIX", "ssf_launch_") + boat


def _tmux(*args, timeout=5):
    sock = os.environ.get("SSF_WP_TMUX_SOCKET")          # 테스트용 별도 tmux 서버(-L) — 웨이포인트·영상과 같은 값
    base = ["tmux", "-L", sock] if sock else ["tmux"]
    try:
        return subprocess.run([*base, *args], capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        raise RuntimeError("이 노트북에 tmux 가 없다")


def running():
    """launch 가 떠 있나(앱·터미널 구분 없이). 패턴은 테스트에서만 바꾼다."""
    pat = os.environ.get("SSF_LAUNCH_PGREP", "launch_files.launch.py")
    try:
        return subprocess.run(["pgrep", "-f", pat], capture_output=True).returncode == 0
    except FileNotFoundError:
        return False


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def boot_check(sys_root=None):
    """boat_boot(sudo) 가 이번 부팅에 돌았나. ok 가 None 이면 이 컴퓨터에 그 항목이 없다(건너뜀)."""
    root = sys_root or os.environ.get("SSF_SYS_ROOT", "/sys")
    govs = [_read(p) for p in sorted(glob.glob(os.path.join(root, "devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor")))]
    govs = [g for g in govs if g is not None]
    bad = sorted({g for g in govs if g != "performance"})
    items = [{"name": "CPU 거버너", "ok": (not bad) if govs else None,
              "detail": (f"{len(govs)}코어 performance" if not bad else f"{', '.join(bad)} 인 코어 있음") if govs else "없음(건너뜀)"}]
    au = _read(os.path.join(root, "module/usbcore/parameters/autosuspend"))
    items.append({"name": "USB autosuspend", "ok": (au == "-1") if au is not None else None,
                  "detail": f"{au} (-1 이면 꺼짐)" if au is not None else "없음(건너뜀)"})
    return {"ok": all(i["ok"] is not False for i in items), "items": items}


def wp_files(home):
    """~/ssf_logs 의 녹화본(*.yaml, 숨김 파일 제외) — 최근 것 먼저."""
    d = os.path.join(home, "ssf_logs")
    out = []
    for p in glob.glob(os.path.join(d, "*.y*ml")):
        n = os.path.basename(p)
        if WP_NAME.fullmatch(n) and os.path.isfile(p):
            out.append((os.path.getmtime(p), n))
    return [{"name": n, "mtime": time.strftime("%m-%d %H:%M", time.localtime(t))} for t, n in sorted(out, reverse=True)]


def command(boat, course, wp_path=None):
    """화면에 보여 줄 명령(손으로 칠 때와 같은 꼴)."""
    c = f"ros2 launch launch_files launch_files.launch.py boat:={boat} course:={course}"
    return c + (f" waypoints_file:={wp_path}" if wp_path else "")


def _pane(boat):
    """앱 세션 상태: None(없음) / 'run'(launch 중) / 'ended'(끝나고 창만 남음)."""
    s = session_name(boat)
    if _tmux("has-session", "-t", s).returncode != 0:
        return None
    cmd = _tmux("display-message", "-p", "-t", s, "#{pane_current_command}").stdout.strip()
    return "ended" if cmd == "sleep" else "run"


def status(boat, home, lines=150):
    try:
        pane = _pane(boat)
    except RuntimeError:          # tmux 없는 컴퓨터(연습용)
        pane = None
    run = running()
    tail = []
    if pane:
        cap = _tmux("capture-pane", "-p", "-J", "-t", session_name(boat), "-S", f"-{lines}").stdout
        tail = cap.rstrip("\n").split("\n")[-lines:]
    if run and pane == "run":
        owner = "app"
    elif run:
        owner = "terminal"        # 앱이 띄운 게 아니다(손으로 tmux ssf 등)
    else:
        owner = None
    return {"running": run, "owner": owner, "session": session_name(boat), "pane": pane, "log": tail,
            "boot": boot_check(), "files": wp_files(home), "boat": boat}


def _wp_path(home, course, wp):
    wp = (wp or "").strip()
    if not wp:
        if course == "tournament":
            raise RuntimeError("토너먼트는 웨이포인트 파일(레인 녹화본)을 골라야 한다 — 비우면 자리표시 템플릿으로 뜬다")
        return None
    if not WP_NAME.fullmatch(wp):
        raise RuntimeError(f"웨이포인트 파일 이름이 이상하다: {wp!r} — ~/ssf_logs 목록에서 고른다")
    p = os.path.join(home, "ssf_logs", wp)
    if not os.path.isfile(p):
        raise RuntimeError(f"~/ssf_logs/{wp} 가 없다")
    return p


def start(repo, boat, home, course, wp=None):
    if course not in COURSES:
        raise RuntimeError(f"코스는 main·tournament 중 하나 (받은 값 {course!r})")
    path = _wp_path(home, course, wp)
    if running():
        st = status(boat, home)
        where = "앱 launch 탭" if st["owner"] == "app" else "터미널(tmux attach -t ssf 등)"
        raise RuntimeError(f"launch 가 이미 떠 있다({where}) — 두 개 띄우지 않는다. 끄고 다시 켤 것")
    bc = boot_check()
    if not bc["ok"]:
        bad = ", ".join(f"{i['name']} {i['detail']}" for i in bc["items"] if i["ok"] is False)
        raise RuntimeError(f"boat_boot 가 이번 부팅에 안 돌았다({bad}) — 배 노트북 터미널에서 "
                           f"sudo {os.path.join(repo, 'tools', 'boat_boot.sh')} 를 한 번 실행한 뒤 다시")
    s = session_name(boat)
    if _pane(boat) == "ended":
        _tmux("kill-session", "-t", s)            # 지난 launch 의 '끝' 화면만 남은 창(sleep) — 정리
    cmd = ["bash", os.path.join(repo, "tools", "ssf_launch_run.sh"), boat, course] + ([path] if path else [])
    r = _tmux("new-session", "-d", "-s", s, "-x", "200", "-y", "50", "-c", repo, "-e", f"HOME={home}", *cmd)
    if r.returncode != 0:
        raise RuntimeError("tmux 시작 실패: " + (r.stderr or r.stdout).strip())
    _tmux("set-option", "-t", s, "history-limit", "5000")
    time.sleep(0.5)
    st = status(boat, home)
    st["command"] = command(boat, course, path)
    return st


def stop(boat, home):
    """Ctrl+C 한 번. 끝날 때까지 기다리지 않는다(수십 초) — 상태를 다시 읽어 '끝' 줄을 본다."""
    pane = _pane(boat)
    if pane != "run":
        if running():
            raise RuntimeError("앱이 띄운 launch 가 아니다 — 띄운 터미널에서 끈다: "
                               "ssh 로 배에 붙어 tmux attach -t ssf → 그 창에서 Ctrl+C 한 번")
        raise RuntimeError("launch 가 꺼져 있다")
    _tmux("send-keys", "-t", session_name(boat), "C-c")
    time.sleep(0.3)
    st = status(boat, home)
    st["stopping"] = True
    return st

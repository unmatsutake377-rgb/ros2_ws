"""SSF 웨이포인트 — 파라미터 앱·ssf 명령에서 웨이포인트 파일을 '보고 검사'하고 녹화를 준비한다 (ROS 불필요).

  찍기 자체는 기존 녹화 도구(north_goal_angle waypoint_recorder)가 한다. 여기는
    · list_files : 저장소 템플릿 + 노트북 녹화본(~/ssf_logs/waypoints_*.yaml) 목록과 검사 결과
    · prepare    : 녹화 전 준비 — 배 확인, 출력·템플릿 경로 결정, 기존 녹화본 백업
  검사 기준은 docs/절차/웨이포인트_녹화절차.md · north_goal_angle/waypoint_loader.py 그대로다.
  🚨 웨이포인트 파일은 고치지 않는다(보기·백업만). 좌표는 녹화 도구로 다시 찍는다.
"""
import datetime
import glob
import os
import re
import shutil
import sys
import time

import yaml

TOURNAMENT_TEMPLATE_REL = "src/north_goal_angle/config/waypoints_tournament.yaml"
MAIN_TEMPLATE_REL = "src/north_goal_angle/config/waypoints.yaml"
INSTALL_SHARE = "install/north_goal_angle/share/north_goal_angle/config"
LANE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,15}$")   # 첫 글자 영문·숫자('-x' 는 옵션으로 오인)


def _loader(repo):
    """north 노드가 쓰는 검증 모듈 그대로(같은 기준). 저장소에 없으면 이 도구가 든 저장소의 것."""
    here_repo = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    for r in (repo, here_repo):
        d = os.path.join(r, "src", "north_goal_angle")
        if os.path.isfile(os.path.join(d, "north_goal_angle", "waypoint_loader.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            break
    from north_goal_angle import waypoint_loader  # noqa: E402
    return waypoint_loader


def course_of(path):
    """파일 이름으로 코스 추정: 이름에 tournament 가 있으면 토너먼트."""
    return "tournament" if "tournament" in os.path.basename(path).lower() else "main"


def point_in_polygon(lat, lon, poly):
    """poly = [위도, 경도, 위도, 경도, …] (geofence_polygon 형식). 경계 위는 안으로 본다."""
    pts = [(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]
    inside = False
    n = len(pts)
    for i in range(n):
        (y1, x1), (y2, x2) = pts[i], pts[(i + 1) % n]
        if (y1 > lat) != (y2 > lat):
            x = x1 + (lat - y1) * (x2 - x1) / (y2 - y1)
            if lon <= x:
                inside = not inside
    return inside


def check_file(repo, path, course=None, template_modes=None, geofence=None):
    """웨이포인트 파일 하나 검사 → dict(course, points, modes, errors, warnings, notes)."""
    wl = _loader(repo)
    course = course or course_of(path)
    out = {"path": path, "name": os.path.basename(path), "course": course, "points": [], "modes": [],
           "errors": [], "warnings": [], "notes": []}
    try:
        st = os.stat(path)
        out["mtime"] = datetime.datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M")
    except OSError as e:
        out["errors"].append(f"파일을 못 읽음: {e}")
        return out
    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except (OSError, UnicodeDecodeError) as e:      # 권한 없음·글자 깨진 파일 하나 때문에 목록 전체가 죽지 않게
        out["errors"].append(f"파일을 못 읽음: {e.__class__.__name__}: {getattr(e, 'strerror', None) or e}")
        return out
    except yaml.YAMLError as e:
        mark = getattr(e, "problem_mark", None)
        out["errors"].append(f"YAML 문법 오류({mark.line + 1}번째 줄)" if mark else f"YAML 문법 오류: {e}")
        return out
    try:
        wps = wl.parse_waypoints(raw)
        wl.validate_course(course, wps)
    except wl.WaypointError as e:
        out["errors"].append(str(e))
        return out
    labels = wl.waypoint_labels(raw)
    out["points"] = [{"i": i, "lat": w[0], "lon": w[1], "mode": w[2], "dwell": w[3],
                      "label": labels[i] if i < len(labels) else ""} for i, w in enumerate(wps)]
    out["modes"] = [w[2] for w in wps]
    # 아래 경고는 전부 녹화절차 문서·param_check 에 있는 것만
    if wps[-1][2] not in (5, 8):
        out["warnings"].append(f"마지막 WP mode {wps[-1][2]} — 코스 끝 즉시 정지(50000)는 마지막 mode 5·8 일 때만")
    elif wps[-1][2] == 5:
        out["warnings"].append("마지막 WP mode 5 — ship_avoid 가 코스 끝 뒤 최대 1 s 20000 을 더 내고(정지 지연) /wp_mode stale ERROR 가 남음. 마지막은 mode 8 권장")
    old = [i for i, w in enumerate(wps) if w[2] in (2, 3) and w[3] > 1.0]
    if old:
        out["warnings"].append(f"mode 2·3 dwell 이 1 s 넘음(WP {', '.join(map(str, old))}) — 옛 녹화 파일일 수 있음, 0.5 권장")
    if course == "tournament":
        if wps[-1][3] != 0.5:
            out["notes"].append(f"마지막 점 dwell {wps[-1][3]} — 절차서는 결승선 너머 약 3 m · dwell 0.5")
        if geofence:
            outside = [i for i, w in enumerate(wps) if not point_in_polygon(w[0], w[1], geofence)]
            if outside:
                out["warnings"].append(f"경기장 경계(geofence_polygon) 밖 점: WP {', '.join(map(str, outside))} "
                                       f"— 마지막 정지 지점은 경계 안이어야 한다")
            else:
                out["notes"].append(f"모든 점이 경기장 경계({len(geofence) // 2}점) 안")
        else:
            out["notes"].append("geofence_polygon 이 비어 있어 경계 안인지 확인 못 함")
    elif template_modes is not None and out["modes"] != template_modes:
        out["notes"].append(f"저장소 템플릿과 mode 순서 다름 — 템플릿 {template_modes}")
    return out


def launch_arg(path, home):
    """launch 인자 글자 — 집 폴더 안이면 $HOME/… 로(노트북 계정이 달라도 그대로 쓰게).
    🚨 ~/ 는 안 된다: ':=' 뒤라 bash 가 안 풀고 노드가 '~' 를 글자 그대로 연다(north_goal_angle_launch.py 주석)."""
    home = os.path.abspath(home)
    p = os.path.abspath(path)
    if p.startswith(home + os.sep):
        return "waypoints_file:=$HOME/" + os.path.relpath(p, home)
    return "waypoints_file:=" + p


def list_files(repo, home, geofence=None):
    """저장소 템플릿 2개 + 노트북 녹화본(~/ssf_logs/waypoints*.yaml). 녹화본 먼저, 최신 순."""
    tmpl_modes = None
    try:
        tmpl = check_file(repo, os.path.join(repo, MAIN_TEMPLATE_REL), "main")
        tmpl_modes = tmpl["modes"] or None
    except Exception:  # noqa: BLE001 — 템플릿을 못 읽어도 녹화본은 보여 준다
        pass
    rec = sorted(glob.glob(os.path.join(home, "ssf_logs", "waypoints*.yaml")),
                 key=lambda p: os.path.getmtime(p), reverse=True)
    rows = []
    for p in rec:
        if "ntrip" in p.lower():
            continue
        r = check_file(repo, p, None, tmpl_modes, geofence)
        r["kind"] = "녹화본(노트북)"
        r["launch_arg"] = launch_arg(p, home)
        rows.append(r)
    prac = sorted(glob.glob(os.path.join(home, "ssf_logs", "practice_*.yaml")),
                  key=lambda p: os.path.getmtime(p), reverse=True)
    for p in prac:
        r = check_file(repo, p, "main", None, None)
        r["kind"] = "연습(노트북)"
        r["launch_arg"] = launch_arg(p, home)
        m = os.path.basename(p)[len("practice_"):-len(".yaml")]
        r["notes"].insert(0, f"연습용 — {PRACTICE[m][0] if m in PRACTICE else m} 만. launch 는 course:=main · "
                             f"끝나면 조종기로 받는다. 대회 녹화본과 따로 쓴다")
        if m in APPROACH_FIRST and r["modes"] and r["modes"][0] != 5:
            r["warnings"].append("첫 점이 mode 5(접근)가 아니다 — 출발 자리에서 바로 부표를 찾아 게이트 부표 같은 "
                                 "다른 부표를 목표로 잡을 수 있다(11-11 시뮬). 미션 구역 앞에 mode 5 점을 먼저 찍는다")
        rows.append(r)
    for rel in (MAIN_TEMPLATE_REL, TOURNAMENT_TEMPLATE_REL):
        p = os.path.join(repo, rel)
        if os.path.isfile(p):
            r = check_file(repo, p, None, None, geofence)
            r["kind"] = "저장소 템플릿"
            rows.append(r)
    return rows


# ---------------------------------------------------------------- 미션별 연습 (11-11)
# 그 미션 웨이포인트만 담은 짧은 파일을 따로 녹화한다 → launch course:=main waypoints_file:=$HOME/ssf_logs/practice_<미션>.yaml.
# north 는 main 코스에서 mode 0·1·2·3·5·7·8 이면 몇 개든 받는다(waypoint_loader) — 배 코드는 그대로.
# 기본값(구역명·mode·dwell)은 종합임무 템플릿에서 그 미션 줄만 뽑아 쓴다(녹화 도구 --template). 대회 녹화본과 이름이 다르다.
# 🚨 위치유지·탐색은 맨 앞에 '연습 접근'(mode 5) 한 줄을 붙인다(11-11 시뮬): 첫 점부터 mode 2·3 이면 출발 자리에서
#    바로 부표 검출·선회가 켜져 **게이트의 빨강·초록 부표**를 목표로 잡았다(탐색 목표 부표 0/20, 게이트 접촉 9).
#    접근점을 넣으니 게이트 접촉 0·목표 부표 선회 13/20(나머지 7 은 344~348°로 판정선 350° 미달). 기본 좌표 = 코스에서
#    그 미션 바로 앞 WP 자리 — 실제로는 미션 구역 앞, 다른 색 부표가 화면에 덜 들어오는 곳을 찍는다.
APPROACH_FIRST = ("hold", "search")
PRACTICE = {   # 이름: (화면 이름, 템플릿 줄 고르기(mode, 구역명) → bool)
    "gate":   ("게이트",   lambda m, lab: m in (0, 1)),
    "hold":   ("위치유지", lambda m, lab: m == 2),
    "search": ("탐색",     lambda m, lab: m == 3),
    "avoid":  ("회피",     lambda m, lab: m == 5 and "회피" in lab),   # 탐색 접근점(mode 5) 제외
    "dock":   ("도킹",     lambda m, lab: "도킹" in lab),
}


def practice_template(repo, home, mission):
    """종합임무 템플릿에서 mission 줄만 뽑아 ~/ssf_logs/.practice_template_<미션>.yaml 로 → 경로. 없으면 ValueError."""
    if mission not in PRACTICE:
        raise ValueError(f"연습 미션 {mission!r} — {', '.join(PRACTICE)} 중 하나")
    share = os.path.join(repo, INSTALL_SHARE, "waypoints.yaml")
    src = share if os.path.exists(share) else os.path.join(repo, MAIN_TEMPLATE_REL)
    wl = _loader(repo)
    try:
        with open(src, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        wps = wl.parse_waypoints(raw)
    except (OSError, yaml.YAMLError, wl.WaypointError) as e:
        raise ValueError(f"종합임무 템플릿을 못 읽음({src}): {e}")
    labels = wl.waypoint_labels(raw)
    pick = PRACTICE[mission][1]
    idx = [i for i, w in enumerate(wps) if pick(w[2], labels[i] if i < len(labels) else "")]
    rows = [{"구역": labels[i] if i < len(labels) else "", "lat": wps[i][0], "lon": wps[i][1], "mode": wps[i][2],
             "dwell": wps[i][3]} for i in idx]
    if not rows:
        raise ValueError(f"종합임무 템플릿에 '{PRACTICE[mission][0]}' 줄이 없다 — 템플릿이 바뀌었나({src})")
    if mission in APPROACH_FIRST and idx[0] > 0:
        prev = wps[idx[0] - 1]
        rows.insert(0, {"구역": f"연습 접근 — {PRACTICE[mission][0]} 구역 앞", "lat": prev[0], "lon": prev[1],
                        "mode": 5, "dwell": 1.0})
    out = os.path.join(home, "ssf_logs", f".practice_template_{mission}.yaml")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(f"# 연습 '{PRACTICE[mission][0]}' 기본값 — {src} 에서 자동으로 뽑음(녹화 도구 --template 전용, 고치지 말 것)\n")
        yaml.safe_dump({"waypoints": rows}, f, allow_unicode=True, sort_keys=False)
    return out


def plan(repo, home, boat, course="main", lane=None):
    """녹화할 출력·템플릿 경로. 토너먼트는 레인마다 따로(~/ssf_logs/waypoints_tournament_<배>_<레인>.yaml)."""
    logs = os.path.join(home, "ssf_logs")
    if course == "tournament":
        if not lane or not LANE_RE.match(lane):
            raise ValueError("토너먼트는 레인 이름이 필요하다(영문·숫자, 예: 1) — ssf b wp tournament 1")
        out = os.path.join(logs, f"waypoints_tournament_{boat}_{lane}.yaml")
        share = os.path.join(repo, INSTALL_SHARE, "waypoints_tournament.yaml")
        template = share if os.path.exists(share) else os.path.join(repo, TOURNAMENT_TEMPLATE_REL)
    elif course == "main":
        if lane:
            raise ValueError("종합임무는 레인 이름을 받지 않는다")
        out = os.path.join(logs, "waypoints_recorded.yaml")
        template = None          # 녹화 도구 기본(share 의 config/waypoints.yaml)
    elif course == "practice":
        if lane not in PRACTICE:
            raise ValueError(f"연습 미션 이름이 필요하다({', '.join(PRACTICE)}) — ssf b wp practice dock")
        out = os.path.join(logs, f"practice_{lane}.yaml")
        template = practice_template(repo, home, lane)
    else:
        raise ValueError(f"코스 {course!r} — main · tournament · practice")
    return out, template


# ---------------------------------------------------------------- 앱에서 녹화 (tmux 안의 녹화 도구를 대신 눌러 준다)
# 터미널 'ssf b wp …' 와 같은 tmux 세션(ssf_wp_<배>)을 쓴다 — 어느 쪽에서 시작해도 다른 쪽에서 이어서 할 수 있다.
# 보낼 수 있는 키는 녹화 도구 명령(Enter·l·d N·r N·q)과 mode·dwell·이름 답뿐이다.

PROMPTS = (   # (화면 마지막 줄에 있으면, 지금 묻는 것) — waypoint_recorder / recorder_prompts / ssf_wp_record.sh 문구
    ("[Enter=추가", "main"),
    ("mode(미션번호)?", "mode"),
    ("dwell(머무는 초)?", "dwell"),
    ("구역 이름(엔터=", "name"),
    ("엔터를 누르면", "closing"),
)
ANSWER_RE = re.compile(r"^[0-9A-Za-z가-힣 ._()+:/#-]{0,40}$")   # mode·dwell·이름 답
IDX_RE = re.compile(r"^[0-9]{1,3}$")


def session_name(boat):
    return os.environ.get("SSF_WP_SESSION_PREFIX", "ssf_wp_") + boat


def _tmux(*args, timeout=5):
    import subprocess
    sock = os.environ.get("SSF_WP_TMUX_SOCKET")          # 테스트용 별도 tmux 서버(-L)
    base = ["tmux", "-L", sock] if sock else ["tmux"]
    try:
        return subprocess.run([*base, *args], capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        raise RuntimeError("이 노트북에 tmux 가 없다")


ROS_LOG_RE = re.compile(r"^\s*\[(INFO|WARN|ERROR|DEBUG|FATAL)\] \[\d")


def detect_prompt(screen):
    """화면 마지막 글자 줄로 지금 무엇을 묻는지. 프롬프트 뒤에 찍힌 ROS 로그 줄([WARN] [123.4] …)은 건너뛴다
    (rclpy·DDS 경고가 비동기로 찍혀도 '일하는 중'으로 굳지 않게)."""
    lines = [ln for ln in screen.rstrip().split("\n") if ln.strip() and not ROS_LOG_RE.match(ln)]
    last = lines[-1] if lines else ""
    for key, kind in PROMPTS:
        if key in last:
            return kind
    return "busy" if lines else "empty"


def rec_status(boat, lines=60):
    s = session_name(boat)
    if _tmux("has-session", "-t", s).returncode != 0:
        return {"running": False, "session": s, "screen": "", "prompt": None}
    cap = _tmux("capture-pane", "-p", "-J", "-t", s, "-S", f"-{lines}")
    screen = cap.stdout.rstrip("\n") if cap.returncode == 0 else ""
    tag = _tmux("show-options", "-qv", "-t", s, "@ssf_wp").stdout.strip()     # "tournament:1" / "main:"
    return {"running": True, "session": s, "screen": screen, "prompt": detect_prompt(screen), "what": tag}


def rec_start(repo, boat, course, lane=None, home=None):
    """tmux 세션에서 tools/ssf_wp_record.sh 시작(배 확인·백업은 그 스크립트가 한다)."""
    plan(repo, home or os.path.expanduser("~"), boat, course, lane)   # 코스·레인 검사만(경로는 스크립트가 다시 정함)
    s = session_name(boat)
    if _tmux("has-session", "-t", s).returncode == 0:
        raise RuntimeError(f"이미 녹화 중(tmux {s}) — 화면에서 이어서 하거나 저장·종료 뒤 다시")
    cmd = ["bash", os.path.join(repo, "tools", "ssf_wp_record.sh"), boat, course] + ([lane] if lane else [])
    # HOME 을 넘긴다 — 이미 떠 있는 tmux 서버의 환경을 물려받지 않게(~/.ssf_boat·~/ssf_logs 기준)
    home = home or os.path.expanduser("~")
    r = _tmux("new-session", "-d", "-s", s, "-x", "110", "-y", "45", "-c", repo, "-e", f"HOME={home}", *cmd)
    if r.returncode != 0:
        raise RuntimeError("tmux 시작 실패: " + (r.stderr or r.stdout).strip())
    _tmux("set-option", "-t", s, "mouse", "on")
    return rec_status(boat)


def rec_send(boat, kind, arg=None):
    """kind: capture|accept(Enter) · list(l) · delete/recapture(d N / r N) · save(q) · answer(mode·dwell·이름) · close."""
    st = rec_status(boat)
    if not st["running"]:
        raise RuntimeError("녹화 중이 아니다")
    p = st["prompt"]
    if kind in ("capture", "list", "delete", "recapture", "save") and p != "main":
        raise RuntimeError("녹화 도구가 다른 걸 묻고 있다 — 화면 마지막 줄의 질문에 먼저 답하세요")
    if kind in ("answer", "accept") and p not in ("mode", "dwell", "name"):
        raise RuntimeError("지금은 mode·dwell·이름을 묻는 중이 아니다")       # accept(Enter)가 명령 줄에 가면 '찍기'가 된다
    if kind == "close" and p != "closing":
        raise RuntimeError("아직 끝나지 않았다 — 저장(q) 먼저")
    if kind in ("capture", "accept", "close"):
        text = ""
    elif kind == "list":
        text = "l"
    elif kind == "save":
        text = "q"
    elif kind in ("delete", "recapture"):
        if not IDX_RE.match(str(arg or "")):
            raise RuntimeError("번호(0, 1, 2 …)를 넣으세요")
        text = ("d " if kind == "delete" else "r ") + str(arg)
    elif kind == "answer":
        text = str(arg or "").strip()
        if not ANSWER_RE.match(text):
            raise RuntimeError("답에 쓸 수 없는 글자가 있다(40자 이내 한글·영문·숫자·공백·. _ ( ) + : / # -)")
    else:
        raise RuntimeError(f"모르는 동작 {kind!r}")
    s = st["session"]
    before = st["screen"]
    # 화면 올려 보기(tmux copy-mode, 마우스 스크롤로 들어감) 중이면 키가 녹화 도구 대신 copy-mode 로 간다
    #   — 'q' 는 copy-mode 만 끄고 뒤의 Enter 가 '찍기'가 된다. 먼저 빠져나온다.
    if _tmux("display-message", "-p", "-t", s, "#{pane_in_mode}").stdout.strip() == "1":
        _tmux("send-keys", "-t", s, "-X", "cancel")
    _tmux("send-keys", "-t", s, "C-u")      # 입력 줄에 남은 글자(스크롤로 들어간 화살표 등) 지우기 — tty 줄 지우기
    if text:
        _tmux("send-keys", "-t", s, "-l", text)
    _tmux("send-keys", "-t", s, "Enter")
    # 녹화 도구가 받아서 화면이 바뀔 때까지 기다린다 — 그 사이 다른 곳에서 누른 키가 옛 프롬프트를 보고
    # 통과해 다음 질문(mode 등)을 기본값으로 넘기지 않게(잠금은 core.wp_rec 가 잡고 있다)
    end = time.time() + 3.0
    changed = False
    while time.time() < end:
        time.sleep(0.05)
        now = rec_status(boat)
        if not now["running"] or now["screen"] != before:
            changed = True
            break
    return {"sent": text or "⏎", "applied": changed}


def rec_kill(boat):
    """녹화 도구를 저장 없이 끈다(찍은 점은 버려진다 — 화면이 확인을 받는다)."""
    s = session_name(boat)
    if _tmux("has-session", "-t", s).returncode != 0:
        raise RuntimeError("녹화 중이 아니다")
    _tmux("kill-session", "-t", s)
    return {"killed": s}


def backup(out, home):
    """기존 녹화본이 있으면 ~/ssf_logs/waypoint_backup/ 에 복사(녹화 도구는 q 때 그냥 덮어쓴다)."""
    if not os.path.isfile(out):
        return None
    d = os.path.join(home, "ssf_logs", "waypoint_backup")
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, datetime.datetime.now().strftime("%Y%m%d-%H%M%S_") + os.path.basename(out))
    shutil.copy2(out, dst)
    return dst

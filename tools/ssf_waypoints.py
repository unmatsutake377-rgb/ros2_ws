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

import yaml

TOURNAMENT_TEMPLATE_REL = "src/north_goal_angle/config/waypoints_tournament.yaml"
MAIN_TEMPLATE_REL = "src/north_goal_angle/config/waypoints.yaml"
INSTALL_SHARE = "install/north_goal_angle/share/north_goal_angle/config"
LANE_RE = re.compile(r"^[A-Za-z0-9_-]{1,16}$")


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
        rows.append(r)
    for rel in (MAIN_TEMPLATE_REL, TOURNAMENT_TEMPLATE_REL):
        p = os.path.join(repo, rel)
        if os.path.isfile(p):
            r = check_file(repo, p, None, None, geofence)
            r["kind"] = "저장소 템플릿"
            rows.append(r)
    return rows


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
    else:
        raise ValueError(f"코스 {course!r} — main 또는 tournament")
    return out, template


def backup(out, home):
    """기존 녹화본이 있으면 ~/ssf_logs/waypoint_backup/ 에 복사(녹화 도구는 q 때 그냥 덮어쓴다)."""
    if not os.path.isfile(out):
        return None
    d = os.path.join(home, "ssf_logs", "waypoint_backup")
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, datetime.datetime.now().strftime("%Y%m%d-%H%M%S_") + os.path.basename(out))
    shutil.copy2(out, dst)
    return dst

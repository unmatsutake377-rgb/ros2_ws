#!/usr/bin/env python3
"""설정 yaml·웨이포인트 파일 검사 — 고친 뒤 launch 전에 돌린다 (배 노트북, MacBook SSH 안에서).

노드는 타입이 틀리면 부팅 때 죽고(37 ↔ 37.0), 빈 리스트는 타입을 못 정하고, 오타 키는 조용히 버린다.
이 도구는 그런 실수를 launch 전에 잡는다. ROS 없이 돈다(파이썬 + PyYAML).

사용 (~/ros2_ws 에서)
  python3 tools/param_check.py src/ship_direction/config/ship_direction_tournament.yaml
  python3 tools/param_check.py src/*/config/*.yaml                       # 한꺼번에
  python3 tools/param_check.py --waypoints ~/ssf_logs/waypoints_tournament_A_1.yaml --course tournament
  python3 tools/param_check.py --waypoints src/north_goal_angle/config/waypoints.yaml   # course 기본 main

파라미터 yaml 검사
  · YAML 문법(틀리면 몇 번째 줄인지)
  · 기준 파일(기본 = git HEAD 의 같은 파일, 또는 --ref)과 비교:
      - 타입이 바뀜(정수 ↔ 소수, 참/거짓, 문자열, 리스트) → 오류. 노드가 부팅 때 죽거나 값을 거부한다
      - 기준에 없는 키 → 경고(오타면 노드가 조용히 무시한다)
      - 기준에 있는데 사라진 키 → 경고(코드 기본값으로 돌아간다)
      - 바뀐 값 목록 출력(무엇을 고쳤는지 한눈에)
  · 빈 리스트 [] → 오류, 리스트 안에 정수·소수가 섞임 → 오류
웨이포인트 검사: 노드와 같은 검증(north_goal_angle.waypoint_loader) + 코스 mode 검사.

종료 코드: 오류 있으면 1, 없으면 0(경고만 있어도 0).
🚨 NTRIP 접속 정보 파일(~/.ssf/, *ntrip*)은 읽지 않는다 — 값을 화면에 찍지 않기 위해서.
"""
import argparse
import os
import subprocess
import sys

import yaml

# [] 가 정상인 키(노드가 빈 리스트를 '꺼짐'으로 받는다, Ubuntu 확인). 나머지 [] 는 오류.
EMPTY_OK = {"geofence_polygon", "geofence_avoid_polygon"}

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def _refuse_secret(path):
    p = os.path.abspath(os.path.expanduser(path))
    return p.startswith(os.path.expanduser("~/.ssf")) or "ntrip" in os.path.basename(p).lower()


def _load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _ref_from_git(path):
    """git HEAD 의 같은 파일 내용(dict) 또는 None."""
    p = os.path.abspath(path)
    if not p.startswith(REPO + os.sep):
        return None
    rel = os.path.relpath(p, REPO)
    try:
        txt = subprocess.run(["git", "-C", REPO, "show", f"HEAD:{rel}"], capture_output=True,
                             text=True, check=True).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None
    return yaml.safe_load(txt)


def _params(doc):
    """{노드: {키: 값}} — ros__parameters 절만 모은다."""
    out = {}
    if not isinstance(doc, dict):
        return out
    for node, body in doc.items():
        if isinstance(body, dict) and isinstance(body.get("ros__parameters"), dict):
            out[str(node)] = body["ros__parameters"]
    return out


def _tname(v):
    if isinstance(v, bool):
        return "참/거짓"
    if isinstance(v, int):
        return "정수"
    if isinstance(v, float):
        return "소수"
    if isinstance(v, str):
        return "문자열"
    if isinstance(v, list):
        kinds = sorted({_tname(x) for x in v})
        return "리스트[" + ",".join(kinds) + "]"
    return type(v).__name__


def _hint(old, new):
    if isinstance(old, float) and isinstance(new, int) and not isinstance(new, bool):
        return f" → 소수점을 붙이세요: {new}.0"
    if isinstance(old, int) and not isinstance(old, bool) and isinstance(new, float):
        return f" → 정수로 적으세요: {int(new)}"
    if isinstance(old, list) and isinstance(new, list) and old and all(isinstance(x, float) for x in old):
        return " → 리스트 원소마다 소수점(예 [120.0, 90.0])"
    if isinstance(old, str) and not isinstance(new, str):
        return " → 따옴표로 감싸세요(예 \"red\")"
    return ""


def check_param_file(path, ref_path=None):
    errs, warns, changes = [], [], []
    try:
        doc = _load(path)
    except yaml.YAMLError as e:
        mark = getattr(e, "problem_mark", None)
        where = f"{mark.line + 1}번째 줄 {mark.column + 1}번째 칸" if mark else "위치 모름"
        return [f"YAML 문법 오류({where}): {getattr(e, 'problem', e)}"], [], []
    cur = _params(doc)
    if not cur:
        warns.append("ros__parameters 절이 없다(파라미터 파일이 아님?) — 웨이포인트면 --waypoints")
    for node, kv in cur.items():
        for k, v in kv.items():
            if isinstance(v, list):
                if not v and k in EMPTY_OK:
                    pass
                elif not v:
                    errs.append(f"{node}.{k}: 빈 리스트 [] — 노드가 타입을 못 정한다. 끄려면 해당 enabled 를 false 로")
                elif {type(x) for x in v} == {int, float}:
                    errs.append(f"{node}.{k}: 리스트에 정수·소수가 섞임 {v} — 원소마다 소수점")
    ref = _load(ref_path) if ref_path else _ref_from_git(path)
    if ref is None:
        warns.append("기준 파일 없음(git 밖이거나 새 파일) — 타입 비교 생략. --ref 로 원본을 주면 비교한다")
        return errs, warns, changes
    base = _params(ref)
    for node, kv in cur.items():
        bkv = base.get(node)
        if bkv is None:
            warns.append(f"노드 이름 '{node}' 가 기준에 없다 — 오타면 이 절 전체가 무시된다")
            continue
        for k, v in kv.items():
            if k not in bkv:
                warns.append(f"{node}.{k}: 기준에 없는 키 — 오타면 노드가 조용히 무시한다")
                continue
            old = bkv[k]
            if _tname(old) != _tname(v):
                ok_empty = isinstance(old, list) and not old and isinstance(v, list)   # [] → 값 넣기(geofence)
                if not ok_empty:
                    errs.append(f"{node}.{k}: 타입이 바뀜 {_tname(old)} → {_tname(v)} ({old!r} → {v!r}){_hint(old, v)}")
            if old != v:
                changes.append(f"{node}.{k}: {old!r} → {v!r}")
        for k in bkv:
            if k not in kv:
                warns.append(f"{node}.{k}: 기준에 있던 키가 사라짐 — 코드 기본값으로 돌아간다")
    return errs, warns, changes


def check_waypoints(path, course):
    sys.path.insert(0, os.path.join(REPO, "src", "north_goal_angle"))
    from north_goal_angle.waypoint_loader import (WaypointError, parse_waypoints,  # noqa: E402
                                                  validate_course, waypoint_labels)
    try:
        raw = _load(path)
    except yaml.YAMLError as e:
        mark = getattr(e, "problem_mark", None)
        where = f"{mark.line + 1}번째 줄" if mark else "위치 모름"
        return [f"YAML 문법 오류({where}): {getattr(e, 'problem', e)}"], [], []
    try:
        wps = parse_waypoints(raw)
        validate_course(course, wps)
    except WaypointError as e:
        return [str(e)], [], []
    labels = waypoint_labels(raw)
    info = [f"웨이포인트 {len(wps)}개, course={course}, 미션 순서(mode): {[w[2] for w in wps]}"]
    for i, w in enumerate(wps):
        name = labels[i] if i < len(labels) and labels[i] else ""
        info.append(f"  {i:2d} mode {w[2]} dwell {w[3]:<4} {w[0]:.7f}, {w[1]:.7f}  {name}")
    warns = []
    if wps[-1][2] not in (5, 8):
        warns.append(f"마지막 WP mode {wps[-1][2]} — 코스 끝 즉시 정지(50000)는 마지막 mode 5·8 일 때만")
    return [], warns, info


def main():
    ap = argparse.ArgumentParser(description="설정 yaml·웨이포인트 검사 (launch 전에)")
    ap.add_argument("files", nargs="*", help="파라미터 yaml")
    ap.add_argument("--ref", help="비교 기준 파일(기본 git HEAD 의 같은 파일). 파일 하나일 때만")
    ap.add_argument("--waypoints", help="웨이포인트 yaml")
    ap.add_argument("--course", default="main", help="main(기본) 또는 tournament")
    a = ap.parse_args()
    if not a.files and not a.waypoints:
        ap.error("검사할 파일을 주세요")
    bad = False
    targets = [(f, "param") for f in a.files] + ([(a.waypoints, "wp")] if a.waypoints else [])
    for f, kind in targets:
        print(f"\n== {f}")
        if _refuse_secret(f):
            print("  ⛔ NTRIP 접속 정보 파일은 검사하지 않는다(값 노출 방지)")
            continue
        if not os.path.exists(os.path.expanduser(f)):
            print("  ❌ 파일이 없다"); bad = True; continue
        f = os.path.expanduser(f)
        if kind == "wp":
            errs, warns, info = check_waypoints(f, a.course)
            for ln in info:
                print("  " + ln)
        else:
            errs, warns, changes = check_param_file(f, a.ref if len(a.files) == 1 else None)
            if changes:
                print("  바뀐 값:")
                for c in changes:
                    print("    · " + c)
            elif not errs:
                print("  바뀐 값 없음")
        for w in warns:
            print("  ⚠️ " + w)
        for e in errs:
            print("  ❌ " + e)
        if errs:
            bad = True
        else:
            print("  ✅ 통과" + (" (경고 확인)" if warns else ""))
    print("\n결과: " + ("❌ 고친 뒤 다시 검사하세요 — 이대로 launch 하면 노드가 죽거나 값이 무시됩니다" if bad
                       else "✅ launch 해도 됩니다 (고친 값은 launch 재시작 후 반영)"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()

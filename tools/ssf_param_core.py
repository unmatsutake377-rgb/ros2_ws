#!/usr/bin/env python3
"""SSF 파라미터 엔진 — 앱(ssf_param_app.py)과 터미널(ssf)이 같이 쓰는 핵심.

무엇: 배 노트북의 설정 yaml(src/*/config/*.yaml)을 읽어 '이 배·이 코스에서 실제로 쓰는 값'을 보여 주고,
     값 하나를 타입을 지켜 고친다(주석·줄 배치 그대로, 그 줄만 바뀜).
왜:   손으로 고치면 3 ↔ 3.0, 빈 리스트, 엉뚱한 배 파일 같은 실수로 노드가 부팅 때 죽는다.

고칠 때마다
  1. 배 확인: ~/.ssf_boat (A/B) 와 요청한 배가 다르면 거부(1번배에 붙어 2번배 값을 고치는 사고 방지)
  2. 어느 파일인가: 공통 → 코스(_main/_tournament) → 배(_A/_B) 순서로 덮어쓴다(launch 와 같음).
     선택한 코스에서 실제로 이기는 파일을 고친다. 그 파일을 읽는 코스를 '영향'으로 알려 준다.
  3. 타입 유지: 소수 자리에 3 → 3.0, 정수 자리에 5.0 → 5, 5.5 는 거부, 참/거짓·문자열·리스트 원소 타입
  4. 그 줄만 바꾼 뒤 다시 읽어 '그 키만 바뀌었는지' 확인 → param_check 로 검사 → 실패면 원래대로
  5. 백업 ~/ssf_logs/param_backup/, 기록 ~/ssf_logs/param_changes.tsv
  6. launch 는 손대지 않는다 — 노드는 시작할 때 한 번 읽으므로 '재시작 필요' 만 알린다.

🚨 NTRIP 파일(~/.ssf/, *ntrip*)은 목록에도 안 올린다. 웨이포인트 파일은 다루지 않는다(waypoint_recorder).
ROS 없이 돈다(파이썬 3.8+ · PyYAML). 연습용 가짜 배에서도 그대로 돈다(tools/ssf_practice.sh).

터미널 사용 (배 노트북 ~/ros2_ws 에서. MacBook 에선 tools/ssf 가 SSH 로 이걸 부른다)
  python3 tools/ssf_param_core.py init A                     # 이 노트북 = 1번배 (한 번)
  python3 tools/ssf_param_core.py get hold_dist              # 이름 일부로 찾기
  python3 tools/ssf_param_core.py set hold_dist_m 3.5 [--course main|tournament] [--node ship_back]
  python3 tools/ssf_param_core.py undo hold_dist_m           # 저장소(git HEAD) 값으로
  python3 tools/ssf_param_core.py diff                       # 저장소와 다른 값 전부
"""
import argparse
import datetime
import glob
import json
import math
import os
import re
import socket
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_REPO = os.path.abspath(os.path.join(HERE, ".."))
COURSES = ("main", "tournament")
COURSE_KO = {"main": "종합임무", "tournament": "토너먼트"}
BOATS = ("A", "B")
BOAT_KO = {"A": "1번배", "B": "2번배"}

# [] 가 정상인 키(빈 값 = 꺼짐). 비어 있을 때 원소 타입.
EMPTY_LIST_OK = {"geofence_polygon": float}

# yaml 을 고쳐도 소용없는 키 — launch 가 뒤에서 덮어쓴다. (패키지, 키) → 이유
LAUNCH_OVERRIDES = {
    ("ship_direction", "dwa_hull"): "launch 가 boat 인자로 넘긴다(boat:=A → 'A'). yaml 값은 무시된다",
    ("north_goal_angle", "course"): "launch 의 course 인자가 정한다",
    ("north_goal_angle", "waypoints_file"): "launch 의 waypoints_file 인자가 정한다",
    ("ship_back", "active_wp_modes"): "launch 의 search_behavior 인자(hold/orbit)가 덮어쓴다",
    ("ship_turn", "enabled"): "launch 의 search_behavior 인자(hold/orbit)가 덮어쓴다",
}

SKIP_PARTS = ("ublox", "uros", "realsense", "ntrip", "micro-ROS")


class ParamError(Exception):
    """사용자에게 그대로 보여 줄 거부 이유."""


# ---------------------------------------------------------------- 파일 찾기·분류

def scan_files(repo):
    """설정 yaml 상대경로 목록(정렬). 웨이포인트·드라이버·NTRIP 제외."""
    rels = []
    try:
        out = subprocess.run(["git", "-C", repo, "ls-files", "src/*/config/*.yaml"],
                             capture_output=True, text=True, check=True).stdout.split()
        rels = [r for r in out if r]
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    if not rels:
        rels = [os.path.relpath(p, repo) for p in glob.glob(os.path.join(repo, "src", "*", "config", "*.yaml"))]
    keep = []
    for r in sorted(set(rels)):
        low = r.lower()
        base = os.path.basename(low)
        if any(s.lower() in low for s in SKIP_PARTS) or base.startswith("waypoints"):
            continue
        if os.path.isfile(os.path.join(repo, r)):
            keep.append(r)
    return keep


def classify(rel):
    """→ (패키지, 층, 태그). 층 = common | course | boat."""
    pkg = rel.split("/")[1]
    stem = os.path.splitext(os.path.basename(rel))[0]
    if stem.endswith("_tournament"):
        return pkg, "course", "tournament"
    if stem.endswith("_main"):
        return pkg, "course", "main"
    m = re.search(r"_([AB])$", stem)
    if m:
        return pkg, "boat", m.group(1)
    return pkg, "common", None


# ---------------------------------------------------------------- 줄 단위 해석

_SECTION_RE = re.compile(r"^(?P<name>[^\s#][^#]*?):\s*(#.*)?$")
_ROSPARAM_RE = re.compile(r"^(?P<ind>\s+)ros__parameters:\s*(#.*)?$")
_KEY_RE = re.compile(r"^(?P<ind>\s+)(?P<key>[A-Za-z_][\w.]*):(?P<rest>.*)$")


def split_comment(rest):
    """'  3.5   # 설명' → ('3.5', '   ', '# 설명'). 따옴표 안 # 는 값."""
    q = None
    for i, ch in enumerate(rest):
        if q:
            if ch == q:
                q = None
        elif ch in "\"'":
            q = ch
        elif ch == "#" and (i == 0 or rest[i - 1] in " \t"):
            val = rest[:i].rstrip()
            gap = rest[len(val):i]
            return val.strip(), gap, rest[i:].rstrip("\n")
    return rest.strip(), "", ""


def _is_multiline(val):
    if val == "" or val[0] in "|>":
        return True
    if val[0] == "[" and not val.endswith("]"):
        return True
    if val[0] == "{" and not val.endswith("}"):
        return True
    if val[0] in "\"'" and (len(val) < 2 or val[-1] != val[0]):
        return True
    return False


def scan_lines(text):
    """{(절, 키): {line, indent, comment, above, multiline}} — 위치·설명만(값은 yaml 이 기준)."""
    lines = text.split("\n")
    out = {}
    section = None
    in_params = False
    key_indent = None
    last = None
    for i, line in enumerate(lines):
        if not line.strip():
            continue
        m = _SECTION_RE.match(line)
        if m:
            section = m.group("name").strip().strip("'\"")
            in_params, key_indent, last = False, None, None
            continue
        if section is None:
            continue
        if _ROSPARAM_RE.match(line):
            in_params, key_indent, last = True, None, None
            continue
        if not in_params or line.lstrip().startswith("#"):
            continue
        ind = len(line) - len(line.lstrip())
        if key_indent is not None and ind > key_indent:
            if last is not None:
                out[last]["multiline"] = True      # 앞 키의 이어지는 줄(블록 리스트 등)
            continue
        km = _KEY_RE.match(line)
        if not km:
            continue
        if key_indent is None:
            key_indent = ind
        val, gap, comment = split_comment(km.group("rest"))
        above = []
        j = i - 1
        while j >= 0 and lines[j].strip().startswith("#") and len(above) < 8:
            above.insert(0, lines[j].strip().lstrip("#").strip())
            j -= 1
        last = (section, km.group("key"))
        out[last] = {"line": i, "indent": km.group("ind"), "comment": comment.lstrip("#").strip(),
                     "above": [a for a in above if a and not set(a) <= set("-=~ ")],
                     "multiline": _is_multiline(val)}
    return out


def parse_params(text):
    """{절: {키: 값}} — ros__parameters 만."""
    doc = yaml.safe_load(text) or {}
    out = {}
    if isinstance(doc, dict):
        for sec, body in doc.items():
            if isinstance(body, dict) and isinstance(body.get("ros__parameters"), dict):
                out[str(sec)] = body["ros__parameters"]
    return out


# ---------------------------------------------------------------- 타입·표기

def tname(v):
    if isinstance(v, bool):
        return "bool"
    if isinstance(v, int):
        return "int"
    if isinstance(v, float):
        return "float"
    if isinstance(v, str):
        return "str"
    if isinstance(v, list):
        return "list"
    return type(v).__name__


TYPE_KO = {"bool": "참/거짓", "int": "정수", "float": "소수", "str": "문자열", "list": "리스트"}


def _ffmt(x):
    r = repr(float(x))
    if "e" in r or "E" in r:
        r = ("%.12f" % x).rstrip("0")
        if r.endswith("."):
            r += "0"
    if "." not in r:
        r += ".0"
    return r


def fmt(v):
    """yaml 에 쓸 표기(타입이 다시 읽어도 같게)."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return _ffmt(v)
    if isinstance(v, str):
        return json.dumps(v, ensure_ascii=False)
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x) for x in v) + "]"
    raise ParamError(f"이 타입은 앱에서 못 고친다: {type(v).__name__}")


def show(v):
    """화면 표기 — 문자열은 따옴표 없이. 앱이 못 고치는 타입(중첩 등)도 글자로는 보여 준다."""
    if isinstance(v, str):
        return v
    try:
        return fmt(v)
    except ParamError:
        return json.dumps(v, ensure_ascii=False, default=str)


_TRUE = {"true", "on", "yes", "1", "참", "켬", "켜기"}
_FALSE = {"false", "off", "no", "0", "거짓", "끔", "끄기"}


def _num(s, what):
    if isinstance(s, bool):
        raise ParamError(f"{what}: 숫자를 넣으세요")
    if isinstance(s, (int, float)):
        x = float(s)
    else:
        t = str(s).strip()
        try:
            x = float(t)
        except ValueError:
            raise ParamError(f"{what}: 숫자가 아니다 ({t!r})")
    if not math.isfinite(x):
        raise ParamError(f"{what}: 유한한 숫자만 ({s!r})")
    return x


def _as_int(x, what):
    if x != int(x):
        raise ParamError(f"{what}: 정수 자리다 — 소수 {x} 는 못 넣는다(노드가 부팅 때 죽는다). 예: {int(x)}")
    return int(x)


def _elem(old_kind, x, what):
    if old_kind is float:
        return _num(x, what)
    if old_kind is int:
        return _as_int(_num(x, what), what)
    if old_kind is bool:
        if isinstance(x, bool):
            return x
        raise ParamError(f"{what}: true/false 만")
    if old_kind is str:
        return str(x)
    raise ParamError(f"{what}: 원소 타입을 모른다")


def convert(old, raw, key):
    """입력(문자열) → 기존 값과 같은 타입의 새 값. 못 바꾸면 ParamError."""
    if isinstance(old, bool):
        t = str(raw).strip().lower()
        if t in _TRUE:
            return True
        if t in _FALSE:
            return False
        raise ParamError(f"{key}: 참/거짓 자리 — true 또는 false ({raw!r})")
    if isinstance(old, int):
        return _as_int(_num(raw, key), key)
    if isinstance(old, float):
        return _num(raw, key)
    if isinstance(old, str):
        t = str(raw).strip()
        if len(t) >= 2 and t[0] == t[-1] and t[0] in "\"'":
            t = t[1:-1]
        if "\n" in t:
            raise ParamError(f"{key}: 한 줄로")
        return t
    if isinstance(old, list):
        if isinstance(raw, list):
            v = raw
        else:
            try:
                v = yaml.safe_load(str(raw).strip() or "[]")
            except yaml.YAMLError:
                raise ParamError(f"{key}: 리스트 형식 — [값, 값, …] ({raw!r})")
        if not isinstance(v, list):
            raise ParamError(f"{key}: 리스트 자리 — 대괄호로 [값, 값, …]")
        if not v:
            if key in EMPTY_LIST_OK:
                return []
            raise ParamError(f"{key}: 빈 리스트 [] 금지 — 노드가 타입을 못 정한다")
        if old:
            kinds = {type(x) for x in old}
            if len(kinds) != 1:
                raise ParamError(f"{key}: 원래 값의 원소 타입이 섞여 있다 — 앱에서 못 고친다(파일에서 직접)")
            kind = kinds.pop()
        elif key in EMPTY_LIST_OK:
            kind = EMPTY_LIST_OK[key]
        else:
            raise ParamError(f"{key}: 원래 값이 빈 리스트라 원소 타입을 모른다")
        return [_elem(kind, x, f"{key}[{i}]") for i, x in enumerate(v)]
    raise ParamError(f"{key}: 이 타입({type(old).__name__})은 앱에서 못 고친다")


def same(a, b):
    """값·타입 둘 다 같은가(3 과 3.0 은 다르다)."""
    if tname(a) != tname(b):
        return False
    if isinstance(a, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return a == b


# ---------------------------------------------------------------- 특수 검사

def _geofence_check(v):
    sys.path.insert(0, os.path.join(DEFAULT_REPO, "src", "north_goal_angle"))
    try:
        from north_goal_angle.geofence_rays import sanitize_geofence_polygon
    except Exception:          # noqa: BLE001 — 검사기 못 불러도 기본 검사는 했다
        return None
    _, ok, reason = sanitize_geofence_polygon(v)
    return None if ok else reason


def load_catalog(path=None):
    path = path or os.path.join(HERE, "ssf_param_catalog.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def catalog_index(cat):
    """{(패키지, 키, 절 또는 None): 항목}"""
    idx = {}
    n = 0
    for g in cat.get("groups", []):
        for it in g.get("items", []):
            idx[(it["pkg"], it["key"], it.get("section"))] = dict(it, group=g["name"], order=n)
            n += 1
    return idx


def catalog_item(idx, pkg, key, section):
    return idx.get((pkg, key, section)) or idx.get((pkg, key, None))


def check_limits(item, v, key):
    if not item:
        return
    lim = item.get("limits")
    if lim and isinstance(v, (int, float)) and not isinstance(v, bool):
        lo, hi = lim
        if not (lo <= v <= hi):
            raise ParamError(f"{key}: 허용 범위 {lo}~{hi} 밖 ({show(v)}) — {item.get('advice', '')}".rstrip(" —"))
    allowed = item.get("choices")
    if allowed is not None and not any(same(v, c) or v == c for c in allowed):
        raise ParamError(f"{key}: 넣을 수 있는 값 {', '.join(show(c) for c in allowed)} 중 하나")


# ---------------------------------------------------------------- 모델

class Repo:
    """한 배 노트북의 저장소 + 배 이름."""

    def __init__(self, repo=None, boat=None, home=None):
        self.repo = os.path.abspath(repo or os.environ.get("SSF_REPO") or DEFAULT_REPO)
        self.home = home or os.path.expanduser("~")
        self.requested_boat = boat.upper() if boat else None
        self.catalog = load_catalog()
        self.cidx = catalog_index(self.catalog)

    # ---- 배 확인
    @property
    def marker_path(self):
        return os.path.join(self.home, ".ssf_boat")

    def marker(self):
        try:
            with open(self.marker_path, encoding="utf-8") as f:
                b = f.read().strip().upper()
            return b if b in BOATS else None
        except FileNotFoundError:
            return None

    def boat(self):
        """편집해도 되는 배 이름. 안 되면 ParamError."""
        mk = self.marker()
        if mk is None:
            raise ParamError(f"이 노트북이 몇 번 배인지 등록 안 됨 — 'ssf a init'(1번배) 또는 'ssf b init'(2번배). "
                             f"hostname={socket.gethostname()}")
        if self.requested_boat and self.requested_boat != mk:
            raise ParamError(f"🚨 배가 다르다: 요청 {BOAT_KO.get(self.requested_boat, self.requested_boat)}, "
                             f"이 노트북은 {BOAT_KO[mk]}(~/.ssf_boat) — SSH 별명(boat-a/boat-b)을 확인하세요")
        return mk

    def view_boat(self):
        """보기용 배 이름 — 등록 안 됐으면 요청한 배로 보여 준다."""
        try:
            return self.boat()
        except ParamError:
            return self.marker() or self.requested_boat or "A"

    def init(self, boat):
        b = boat.upper()
        if b not in BOATS:
            raise ParamError("A 또는 B")
        old = self.marker()
        with open(self.marker_path, "w", encoding="utf-8") as f:
            f.write(b + "\n")
        return old, b

    # ---- 읽기
    def _read(self, rel):
        with open(os.path.join(self.repo, rel), encoding="utf-8") as f:
            return f.read()

    def head_text(self, rel):
        try:
            return subprocess.run(["git", "-C", self.repo, "show", f"HEAD:{rel}"],
                                  capture_output=True, text=True, check=True).stdout
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

    def load(self, boat=None):
        """→ (entries, groups). entries[i] = 파일·절·키 하나. groups[(pkg, 절, 키)] = entries 목록."""
        boat = boat or self.view_boat()
        entries = []
        for rel in scan_files(self.repo):
            pkg, layer, tag = classify(rel)
            if layer == "boat" and tag != boat:
                continue
            text = self._read(rel)
            try:
                params = parse_params(text)
            except yaml.YAMLError as e:
                entries.append({"rel": rel, "pkg": pkg, "layer": layer, "tag": tag, "error": f"YAML 문법 오류: {e}"})
                continue
            lines = scan_lines(text)
            htext = self.head_text(rel)
            try:
                hparams = parse_params(htext) if htext is not None else None
            except yaml.YAMLError:
                hparams = None
            for sec, kv in params.items():
                for k, v in kv.items():
                    meta = lines.get((sec, k), {})
                    hv = hparams.get(sec, {}).get(k, None) if hparams is not None else None
                    in_head = hparams is not None and k in hparams.get(sec, {})
                    entries.append({"rel": rel, "pkg": pkg, "layer": layer, "tag": tag, "section": sec,
                                    "key": k, "value": v, "head": hv, "in_head": in_head,
                                    "changed": (not in_head) or not same(v, hv),
                                    "line": meta.get("line"), "comment": meta.get("comment", ""),
                                    "above": meta.get("above", []),
                                    "multiline": meta.get("multiline", meta == {})})
        groups = {}
        for e in entries:
            if "error" in e:
                continue
            groups.setdefault((e["pkg"], e["section"], e["key"]), []).append(e)
        return entries, groups

    @staticmethod
    def source(group, course):
        """코스에서 이기는 항목(공통 → 코스 → 배, 뒤가 이김)."""
        win = None
        for layer in ("common", "course", "boat"):
            for e in group:
                if e["layer"] != layer:
                    continue
                if layer == "course" and e["tag"] != course:
                    continue
                win = e
        return win

    def describe(self, gid, group):
        pkg, sec, key = gid
        item = catalog_item(self.cidx, pkg, key, sec)
        eff = {}
        for c in COURSES:
            s = self.source(group, c)
            eff[c] = None if s is None else {"value": s["value"], "show": show(s["value"]), "rel": s["rel"],
                                             "layer": s["layer"], "tag": s["tag"], "changed": s["changed"],
                                             "head": None if not s["in_head"] else show(s["head"])}
        any_e = group[0]
        desc = []
        for e in group:
            for a in e["above"]:
                if a not in desc:
                    desc.append(a)
            if e["comment"] and e["comment"] not in desc:
                desc.append(e["comment"])
        ro = LAUNCH_OVERRIDES.get((pkg, key))
        if any(e["multiline"] for e in group):
            ro = ro or "여러 줄로 적힌 값 — 앱에서 못 고친다(파일에서 직접)"
        if tname(any_e["value"]) not in ("bool", "int", "float", "str", "list"):
            ro = ro or "앱에서 못 고치는 타입"
        return {"id": f"{pkg}|{sec}|{key}", "pkg": pkg, "section": sec, "key": key,
                "type": tname(any_e["value"]), "type_ko": TYPE_KO.get(tname(any_e["value"]), "?"),
                "effective": eff, "files": [{"rel": e["rel"], "layer": e["layer"], "tag": e["tag"],
                                             "show": show(e["value"]), "changed": e["changed"]} for e in group],
                "desc": desc, "readonly": ro,
                "catalog": item, "elem": _list_elem(any_e["value"], key)}

    def params(self):
        boat = self.view_boat()
        entries, groups = self.load(boat)
        out = [self.describe(gid, g) for gid, g in sorted(groups.items())]
        # 비전 /** 와 노드 절이 같은 키 → 노드 절이 이긴다는 표시
        by_pk = {}
        for p in out:
            by_pk.setdefault((p["pkg"], p["key"]), []).append(p)
        for (_pkg, _k), ps in by_pk.items():
            star = [p for p in ps if p["section"] == "/**"]
            if star and len(ps) > 1:
                names = [p["section"] for p in ps if p["section"] != "/**"]
                star[0]["note"] = "이 노드들은 자기 절 값을 쓴다: " + ", ".join(names)
                for p in ps:
                    if p["section"] != "/**":
                        p["note"] = "이 절 값이 비전 공통(/**) 값을 덮어쓴다"
        errors = [{"rel": e["rel"], "error": e["error"]} for e in entries if "error" in e]
        return {"boat": boat, "params": out, "errors": errors}

    # ---- 찾기
    def find(self, key, node=None, groups=None):
        """키(정확히) → [(gid, group)]. node 는 절 이름 일부."""
        if groups is None:
            _, groups = self.load(self.boat())
        hits = [(gid, g) for gid, g in groups.items() if gid[2] == key]
        if node:
            n = node.strip("/")
            hits = [(gid, g) for gid, g in hits if n in gid[1].strip("/") or n == gid[0]]
        return hits

    def resolve(self, key, course=None, node=None, rel=None):
        """→ (gid, 고칠 항목, 영향 코스 목록). rel 을 주면 그 파일 항목을 고친다(바뀐 값 탭의 되돌리기)."""
        boat = self.boat()
        _, groups = self.load(boat)
        if "|" in key:
            pkg, sec, k = key.split("|", 2)
            hits = [((pkg, sec, k), groups[(pkg, sec, k)])] if (pkg, sec, k) in groups else []
            key = k
        else:
            hits = self.find(key, node, groups)
        if not hits:
            near = sorted({gid[2] for gid in groups if key.lower() in gid[2].lower()})[:12]
            raise ParamError(f"'{key}' 없음" + (f" — 비슷한 이름: {', '.join(near)}" if near else ""))
        if len(hits) > 1:
            raise ParamError(f"'{key}' 가 여러 노드에 있다 — --node 로 고르세요: "
                             + ", ".join(f"{gid[1]}({gid[0]})" for gid, _ in hits))
        gid, group = hits[0]
        ro = LAUNCH_OVERRIDES.get((gid[0], gid[2]))
        if ro:
            raise ParamError(f"{key}: yaml 로 못 바꾼다 — {ro}")
        if rel:
            target = next((e for e in group if e["rel"] == rel), None)
            if target is None:
                raise ParamError(f"{key}: {rel} 에 없다")
        elif course:
            if course not in COURSES:
                raise ParamError("코스는 main 또는 tournament")
            target = self.source(group, course)
        else:
            srcs = {c: self.source(group, c) for c in COURSES}
            if srcs["main"] is not srcs["tournament"]:
                raise ParamError(f"{key}: 코스마다 다른 파일이 이긴다 — --course main 또는 --course tournament 를 붙이세요 ("
                                 + ", ".join(f"{COURSE_KO[c]} = {show(s['value'])} @ {s['rel']}"
                                             for c, s in srcs.items() if s) + ")")
            target = srcs["main"]
        if target is None:
            raise ParamError(f"{key}: 이 코스에선 이 값을 읽지 않는다")
        if target["multiline"]:
            raise ParamError(f"{key}: 여러 줄로 적힌 값 — 파일에서 직접({target['rel']})")
        impacts = [c for c in COURSES if self.source(group, c) is target]
        return gid, target, impacts

    # ---- 고치기
    def plan(self, key, raw, course=None, node=None, rel=None):
        gid, t, impacts = self.resolve(key, course, node, rel)
        item = catalog_item(self.cidx, gid[0], gid[2], gid[1])
        new = convert(t["value"], raw, gid[2])
        check_limits(item, new, gid[2])
        warn = []
        if gid[2] == "geofence_polygon" and new:
            reason = _geofence_check(new)
            if reason:
                raise ParamError(f"geofence_polygon: {reason}")
            warn.append(f"폴리곤 {len(new) // 2}점 — 시작 로그 'geofence ON({len(new) // 2}점)' 확인")
        if len(impacts) == 2 and t["layer"] == "common":
            warn.append("공통 파일 — 종합임무·토너먼트 둘 다 바뀐다")
        return {"gid": gid, "target": t, "impacts": impacts, "old": t["value"], "new": new,
                "noop": same(t["value"], new), "warn": warn}

    def edit_text(self, text, section, key, new):
        lines = text.split("\n")
        meta = scan_lines(text).get((section, key))
        if not meta or meta["multiline"]:
            raise ParamError(f"{key}: 줄을 못 찾았다")
        i = meta["line"]
        km = _KEY_RE.match(lines[i])
        val, gap, comment = split_comment(km.group("rest"))
        lines[i] = f"{km.group('ind')}{key}: {fmt(new)}" + ((gap or "  ") + comment if comment else "")
        out = "\n".join(lines)
        before, after = parse_params(text), parse_params(out)
        expect = json.loads(json.dumps(before))
        if section not in after or key not in after[section] or not same(after[section][key], new):
            raise ParamError(f"{key}: 고친 줄을 다시 읽었더니 값이 다르다 — 쓰지 않았다")
        after_cmp = json.loads(json.dumps(after))
        expect[section][key] = after_cmp[section][key]
        if expect != after_cmp:
            raise ParamError(f"{key}: 다른 값까지 바뀌는 편집이라 쓰지 않았다")
        return out

    def apply(self, key, raw, course=None, node=None, source="cli", dry_run=False, rel=None):
        p = self.plan(key, raw, course, node, rel)
        t = p["target"]
        res = {"ok": True, "key": p["gid"][2], "section": p["gid"][1], "pkg": p["gid"][0], "rel": t["rel"],
               "old": show(p["old"]), "new": show(p["new"]), "impacts": p["impacts"], "warn": p["warn"],
               "noop": p["noop"], "dry_run": dry_run}
        if p["noop"] or dry_run:
            return res
        path = os.path.join(self.repo, t["rel"])
        text = self._read(t["rel"])
        new_text = self.edit_text(text, t["section"], p["gid"][2], p["new"])
        bdir = os.path.join(self.home, "ssf_logs", "param_backup")
        os.makedirs(bdir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S.%f")   # 마이크로초 — 이름순 = 시간순
        backup = os.path.join(bdir, f"{stamp}_{t['rel'].replace('/', '__')}")
        k = 1
        while os.path.exists(backup):        # 같은 초에 같은 파일을 두 번 고치면 첫 백업(원본)을 덮지 않게
            backup = os.path.join(bdir, f"{stamp}-{k}_{t['rel'].replace('/', '__')}")
            k += 1
        with open(backup, "w", encoding="utf-8") as f:
            f.write(text)
        tmp = path + ".ssf_tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(new_text)
        os.chmod(tmp, os.stat(path).st_mode & 0o777)
        os.replace(tmp, path)
        errs = self.post_check(t["rel"])
        if errs:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
            raise ParamError("param_check 가 거부해 원래대로 돌렸다: " + " / ".join(errs))
        self.log(t["rel"], t["section"], p["gid"][2], p["old"], p["new"], source)
        res.update(backup=backup, install=self.install_state(t["pkg"], t["rel"]),
                   launch_running=launch_running())
        return res

    def undo(self, key, course=None, node=None, source="cli", rel=None):
        gid, t, _ = self.resolve(key, course, node, rel)
        if not t["in_head"]:
            raise ParamError(f"{gid[2]}: 저장소(HEAD)에 없는 키 — 되돌릴 값이 없다")
        raw = t["head"] if isinstance(t["head"], (list, str)) else fmt(t["head"])
        return self.apply("|".join(gid), raw, course=course, source=source + ":undo", rel=t["rel"])

    def post_check(self, rel):
        sys.path.insert(0, HERE)
        try:
            import param_check
        except Exception:      # noqa: BLE001
            return []
        old_repo = param_check.REPO
        param_check.REPO = self.repo
        try:
            errs, _w, _c = param_check.check_param_file(os.path.join(self.repo, rel))
        finally:
            param_check.REPO = old_repo
        return errs

    def log(self, rel, section, key, old, new, source):
        d = os.path.join(self.home, "ssf_logs")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "param_changes.tsv")
        new_file = not os.path.exists(path)
        with open(path, "a", encoding="utf-8") as f:
            if new_file:
                f.write("시각\t배\thostname\t파일\t절\t키\t전\t후\t어디서\n")
            f.write("\t".join([datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), self.marker() or "?",
                               socket.gethostname(), rel, section, key, show(old), show(new), source]) + "\n")

    def read_log(self, n=50):
        path = os.path.join(self.home, "ssf_logs", "param_changes.tsv")
        try:
            with open(path, encoding="utf-8") as f:
                rows = [ln.rstrip("\n").split("\t") for ln in f if ln.strip()]
        except FileNotFoundError:
            return []
        return [dict(zip(["time", "boat", "host", "rel", "section", "key", "old", "new", "source"], r))
                for r in rows[1:]][-n:][::-1]

    def install_state(self, pkg, rel):
        """설치 폴더가 src 를 가리키나(symlink) → 빌드 불필요."""
        inst = os.path.join(self.repo, "install", pkg, "share", pkg, "config", os.path.basename(rel))
        src = os.path.join(self.repo, rel)
        if os.path.islink(inst) and os.path.realpath(inst) == os.path.realpath(src):
            return "symlink"
        if os.path.exists(inst):
            return "copy"
        return "missing"

    def diff(self):
        boat = self.view_boat()
        entries, groups = self.load(boat)
        rows = []
        for gid, g in sorted(groups.items()):
            for e in g:
                if e["changed"]:
                    rows.append({"id": "|".join(gid), "pkg": gid[0], "section": gid[1], "key": gid[2],
                                 "rel": e["rel"], "layer": e["layer"], "tag": e["tag"],
                                 "head": show(e["head"]) if e["in_head"] else None, "now": show(e["value"]),
                                 "install": self.install_state(gid[0], e["rel"])})
        return rows

    def build(self, pkgs):
        pkgs = [p for p in pkgs if re.fullmatch(r"[A-Za-z0-9_]+", p)]
        if not pkgs:
            raise ParamError("빌드할 패키지가 없다")
        setups = sorted(glob.glob("/opt/ros/*/setup.bash"))
        if not setups:
            raise ParamError("ROS 가 없는 컴퓨터다(연습용?) — 빌드 생략")
        cmd = (f"source {setups[-1]} && cd {self.repo} && "
               f"colcon build --symlink-install --packages-select {' '.join(pkgs)}")
        r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=900)
        tail = (r.stdout + r.stderr).strip().split("\n")[-25:]
        return {"ok": r.returncode == 0, "code": r.returncode, "tail": tail, "pkgs": pkgs}

    # ---- 웨이포인트(보기·검사·녹화 준비만 — 파일은 고치지 않는다. tools/ssf_waypoints.py)
    def geofence(self):
        """토너먼트에서 실제로 읽는 geofence_polygon(6개 이상일 때만), 없으면 None."""
        for p in self.params()["params"]:
            if p["key"] == "geofence_polygon":
                e = p["effective"].get("tournament")
                if e and isinstance(e["value"], list) and len(e["value"]) >= 6:
                    return e["value"]
        return None

    def waypoints(self):
        import ssf_waypoints as wpm
        return wpm.list_files(self.repo, self.home, self.geofence())

    def wp_prepare(self, course="main", lane=None):
        """녹화 전: 배 확인 → 출력·템플릿 경로 → 기존 녹화본 백업."""
        import ssf_waypoints as wpm
        b = self.boat()
        try:
            out, template = wpm.plan(self.repo, self.home, b, course, lane)
        except ValueError as e:
            raise ParamError(str(e))
        return {"boat": b, "out": out, "template": template, "backup": wpm.backup(out, self.home)}

    def wp_rec(self, action, **kw):
        """앱에서 녹화: status / start(course, lane) / send(kind, arg) / kill. 배 확인 뒤에만."""
        import ssf_waypoints as wpm
        b = self.boat() if action != "status" else self.view_boat()
        try:
            if action == "status":
                return wpm.rec_status(b)
            if action == "start":
                return wpm.rec_start(self.repo, b, kw.get("course") or "main", kw.get("lane") or None, self.home)
            if action == "send":
                return wpm.rec_send(b, kw.get("kind"), kw.get("arg"))
            if action == "kill":
                return wpm.rec_kill(b)
        except (RuntimeError, ValueError) as e:
            raise ParamError(str(e))
        raise ParamError(f"모르는 동작 {action!r}")

    def state(self):
        mk = self.marker()
        err = None
        try:
            self.boat()
        except ParamError as e:
            err = str(e)
        head = subprocess.run(["git", "-C", self.repo, "log", "-1", "--format=%h %s"],
                              capture_output=True, text=True).stdout.strip()
        return {"marker": mk, "requested": self.requested_boat, "boat_error": err,
                "boat": self.view_boat(), "boat_ko": BOAT_KO.get(self.view_boat()),
                "host": socket.gethostname(), "repo": self.repo, "head": head,
                "launch_running": launch_running()}


def _list_elem(v, key):
    if not isinstance(v, list):
        return None
    if v:
        return TYPE_KO.get(tname(v[0]))
    t = EMPTY_LIST_OK.get(key)
    return TYPE_KO.get(t.__name__) if t else None


def launch_running():
    try:
        return subprocess.run(["pgrep", "-f", "launch_files.launch.py"], capture_output=True).returncode == 0
    except FileNotFoundError:
        return False


# ---------------------------------------------------------------- 터미널

def _print_result(r):
    if r.get("noop"):
        print(f"  = 이미 {r['new']} — 바꿀 것 없음 ({r['rel']})")
        return
    print(f"  ✅ {r['section']} {r['key']}: {r['old']} → {r['new']}")
    print(f"     파일 {r['rel']} · 영향 코스: {', '.join(COURSE_KO[c] for c in r['impacts'])}")
    for w in r.get("warn", []):
        print("  ⚠️ " + w)
    inst = r.get("install")
    if inst == "symlink":
        print("     설치 폴더 = symlink → 빌드 불필요")
    elif inst in ("copy", "missing"):
        print(f"     🔨 빌드 필요: colcon build --symlink-install --packages-select {r['pkg']}"
              + (" (설치 폴더 없음)" if inst == "missing" else ""))
    print("     🔁 " + ("launch 가 떠 있다 — Ctrl+C 후 다시 띄워야 반영" if r.get("launch_running")
                       else "다음 launch 때 반영"))


def main(argv=None):
    ap = argparse.ArgumentParser(description="SSF 파라미터 보기·고치기 (배 노트북)")
    ap.add_argument("--boat", help="A 또는 B — ~/.ssf_boat 와 다르면 거부")
    ap.add_argument("--repo", help="저장소 경로(기본 이 파일의 상위)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("init"); s.add_argument("which", metavar="A|B")
    s = sub.add_parser("get"); s.add_argument("key"); s.add_argument("--node")
    s = sub.add_parser("set"); s.add_argument("key"); s.add_argument("value", nargs="+")
    s.add_argument("--course", choices=COURSES); s.add_argument("--node")
    s.add_argument("--dry-run", action="store_true")
    s = sub.add_parser("undo"); s.add_argument("key"); s.add_argument("--course", choices=COURSES); s.add_argument("--node")
    sub.add_parser("diff")
    sub.add_parser("log")
    sub.add_parser("wp-list")
    s = sub.add_parser("wp-prep"); s.add_argument("course", choices=COURSES); s.add_argument("lane", nargs="?")
    a = ap.parse_args(argv)
    r = Repo(a.repo, a.boat)
    try:
        if a.cmd == "init":
            if a.boat and a.boat.upper() != a.which.upper():
                raise ParamError("--boat 와 init 값이 다르다")
            old, b = r.init(a.which)
            print(f"✅ 이 노트북({socket.gethostname()}) = {BOAT_KO[b]} ({b})" + (f" — 전에는 {old}" if old and old != b else ""))
            return 0
        st = r.state()
        print(f"[{st['boat_ko']} · {st['host']}]")
        if a.cmd == "get":
            data = r.params()
            q = a.key.lower()
            hits = [p for p in data["params"] if q in p["key"].lower()
                    and (not a.node or a.node.strip('/') in p["section"].strip('/') or a.node == p["pkg"])]
            if not hits:
                print(f"  '{a.key}' 없음"); return 1
            for p in hits:
                tag = f"  {p['key']}  [{p['section']} · {p['pkg']} · {p['type_ko']}]"
                print(tag)
                for c in COURSES:
                    e = p["effective"][c]
                    if e is None:
                        continue
                    mark = f"  (저장소 {e['head']})" if e["changed"] else ""
                    print(f"     {COURSE_KO[c]:5s} = {e['show']}{mark}   ← {e['rel']}")
                if p["readonly"]:
                    print(f"     🔒 {p['readonly']}")
            return 0
        if a.cmd == "set":
            res = r.apply(a.key, " ".join(a.value), a.course, a.node, dry_run=a.dry_run)
            if a.dry_run:
                print(f"  (시험) {res['key']}: {res['old']} → {res['new']} @ {res['rel']}")
            else:
                _print_result(res)
            return 0
        if a.cmd == "undo":
            _print_result(r.undo(a.key, a.course, a.node))
            return 0
        if a.cmd == "diff":
            rows = r.diff()
            if not rows:
                print("  저장소와 같다 — 바꾼 값 없음"); return 0
            for d in rows:
                print(f"  · {d['key']} [{d['section']}] {d['head']} → {d['now']}   ({d['rel']})")
            return 0
        if a.cmd == "log":
            for x in r.read_log(30):
                print(f"  {x['time']} {x['key']} {x['old']} → {x['new']} ({x['rel']}, {x['source']})")
            return 0
        if a.cmd == "wp-list":
            for f in r.waypoints():
                bad = "❌" if f["errors"] else ("⚠️" if f["warnings"] else "✅")
                print(f"  {bad} {f['kind']}  {f['path']}  [{COURSE_KO[f['course']]}]  {f.get('mtime', '')}")
                if f["modes"]:
                    print(f"       {len(f['modes'])}점, mode 순서 {f['modes']}")
                for m in f["errors"]:
                    print("       ❌ " + m)
                for m in f["warnings"]:
                    print("       ⚠️ " + m)
                for m in f["notes"]:
                    print("       ℹ️ " + m)
            return 0
        if a.cmd == "wp-prep":
            p = r.wp_prepare(a.course, a.lane)
            if p["backup"]:
                print(f"  💾 기존 녹화본 백업: {p['backup']}")
            print(f"WP_OUT={p['out']}")
            print(f"WP_TEMPLATE={p['template'] or ''}")
            return 0
    except ParamError as e:
        print("  ❌ " + str(e))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

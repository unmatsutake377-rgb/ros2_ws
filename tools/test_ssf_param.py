"""ssf_param_core / ssf_param_app 테스트 — 진짜 config yaml 사본을 임시 git 저장소에 넣고 돈다(ROS 불필요).

  python3 -m pytest tools/test_ssf_param.py -q
"""
import json
import os
import shutil
import subprocess
import sys
import threading
import urllib.error
import urllib.request

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import ssf_param_core as core  # noqa: E402


def _git(d, *a):
    subprocess.run(["git", "-C", d, *a], check=True, capture_output=True)


@pytest.fixture
def boat(tmp_path):
    """임시 저장소(진짜 config yaml 사본 + git 커밋) + 집 폴더(A번배 등록)."""
    repo = tmp_path / "ros2_ws"
    for rel in subprocess.run(["git", "-C", REPO, "ls-files", "src/*/config/*.yaml"],
                              capture_output=True, text=True, check=True).stdout.split():
        if "ntrip" in rel:
            continue
        dst = repo / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(os.path.join(REPO, rel), dst)
    (repo / "src" / "ntrip_client" / "config").mkdir(parents=True)
    (repo / "src" / "ntrip_client" / "config" / "ntrip_credentials.yaml").write_text(
        "/ntrip:\n  ros__parameters:\n    password: \"x\"\n")
    _git(str(repo), "init", "-q")
    _git(str(repo), "-c", "user.email=t@t", "-c", "user.name=t", "add", "-A")
    _git(str(repo), "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "base")
    home = tmp_path / "home"
    home.mkdir()
    (home / ".ssf_boat").write_text("A\n")
    return core.Repo(str(repo), "A", home=str(home))


def _text(r, rel):
    with open(os.path.join(r.repo, rel), encoding="utf-8") as f:
        return f.read()


def _head(r, rel):
    return subprocess.run(["git", "-C", r.repo, "show", f"HEAD:{rel}"], capture_output=True, text=True).stdout


# ---------------------------------------------------------------- 파일 범위

def test_scan_skips_secret_waypoint_driver_files(boat):
    files = core.scan_files(boat.repo)
    assert files
    assert not any("ntrip" in f or "waypoints" in os.path.basename(f) or "ublox" in f for f in files)


def test_other_boat_file_hidden(boat):
    _, groups = boat.load("A")
    rels = {e["rel"] for g in groups.values() for e in g}
    assert any(r.endswith("ssf_heading_A.yaml") for r in rels)
    assert not any(r.endswith("ssf_heading_B.yaml") for r in rels)


# ---------------------------------------------------------------- 타입

@pytest.mark.parametrize("old,raw,want", [
    (2.5, "3", 3.0), (2.5, "3.25", 3.25), (5, "5.0", 5), (5, "7", 7), (True, "false", False),
    (False, "켬", True), ("red", "green", "green"), ("red", '"blue"', "blue"),
    ([8], "[5, 8]", [5, 8]), ([1.0, 2.0], "[3, 4.5]", [3.0, 4.5]),
])
def test_convert_keeps_type(old, raw, want):
    got = core.convert(old, raw, "k")
    assert core.same(got, want)


@pytest.mark.parametrize("old,raw", [
    (5, "5.5"), (2.5, "abc"), (2.5, "nan"), (2.5, "inf"), (True, "maybe"), ([8], "8"), ([8], "[]"),
    ([1, 2], "[1, 2.5]"),
])
def test_convert_rejects(old, raw):
    with pytest.raises(core.ParamError):
        core.convert(old, raw, "k")


def test_geofence_empty_list_ok_and_float():
    assert core.convert([], "[]", "geofence_polygon") == []
    v = core.convert([], "[35, 128, 35.1, 128, 35.1, 128.1]", "geofence_polygon")
    assert all(isinstance(x, float) for x in v)


def test_every_existing_value_round_trips(boat):
    """지금 모든 yaml 값을 같은 값으로 다시 쓰면 파싱 결과가 그대로 — 표기기(fmt)가 모든 실제 값에 안전."""
    for rel in core.scan_files(boat.repo):
        text = _text(boat, rel)
        params = core.parse_params(text)
        lines = core.scan_lines(text)
        for sec, kv in params.items():
            for k, v in kv.items():
                if lines.get((sec, k), {}).get("multiline", True) or isinstance(v, dict):
                    continue
                out = boat.edit_text(text, sec, k, v)
                assert core.same(core.parse_params(out)[sec][k], v), (rel, k)


# ---------------------------------------------------------------- 편집

def test_set_changes_only_that_line_and_keeps_comment(boat):
    rel = "src/ship_back/config/ship_back.yaml"
    before = _text(boat, rel).split("\n")
    r = boat.apply("hold_dist_m", "3")
    assert r["new"] == "3.0" and r["rel"] == rel and set(r["impacts"]) == {"main", "tournament"}
    after = _text(boat, rel).split("\n")
    diff = [(a, b) for a, b in zip(before, after) if a != b]
    assert len(before) == len(after) and len(diff) == 1
    assert diff[0][1].lstrip().startswith("hold_dist_m: 3.0")
    assert "실측 필요" in diff[0][1]


def test_course_ambiguous_needs_course(boat):
    with pytest.raises(core.ParamError, match="코스마다"):
        boat.apply("side_lock_enabled", "true")
    r = boat.apply("side_lock_enabled", "true", course="tournament")
    assert r["rel"].endswith("ship_direction_tournament.yaml") and r["impacts"] == ["tournament"]
    r = boat.apply("side_lock_enabled", "true", course="main")
    assert r["rel"].endswith("ship_direction/config/ship_direction.yaml") and r["impacts"] == ["main"]


def test_boat_file_wins(boat):
    r = boat.apply("mount_offset_deg", "37")
    assert r["rel"].endswith("ssf_heading_A.yaml") and r["new"] == "37.0"


def test_wrong_boat_refused(boat):
    other = core.Repo(boat.repo, "B", home=boat.home)
    with pytest.raises(core.ParamError, match="배가 다르다"):
        other.apply("hold_dist_m", "3.0")


def test_unregistered_refused(boat):
    os.remove(boat.marker_path)
    with pytest.raises(core.ParamError, match="등록 안 됨"):
        boat.apply("hold_dist_m", "3.0")


def test_launch_override_refused(boat):
    with pytest.raises(core.ParamError, match="launch"):
        boat.apply("dwa_hull", "B")
    with pytest.raises(core.ParamError, match="launch"):
        boat.apply("active_wp_modes", "[2]", node="ship_back")


def test_multi_node_needs_node(boat):
    with pytest.raises(core.ParamError, match="여러 노드"):
        boat.apply("confirm_window", "5")
    r = boat.apply("confirm_window", "5", node="dock")
    assert r["section"] == "image_subscriber_dock"


def test_limits_and_choices(boat):
    with pytest.raises(core.ParamError, match="범위"):
        boat.apply("front_median_kernel", "12")
    with pytest.raises(core.ParamError, match="정수"):
        boat.apply("front_median_kernel", "5.5")
    with pytest.raises(core.ParamError):
        boat.apply("dwa_modes", "[3]", course="tournament")


def test_geofence_swapped_refused_and_valid_applied(boat):
    with pytest.raises(core.ParamError, match="뒤바뀌"):
        boat.apply("geofence_polygon", "[128.5, 35.1, 128.6, 35.1, 128.6, 35.2]", course="tournament")
    r = boat.apply("geofence_polygon", "[35.1862, 128.5655, 35.1866, 128.5655, 35.1866, 128.566]",
                   course="tournament")
    assert r["impacts"] == ["tournament"]
    doc = yaml.safe_load(_text(boat, r["rel"]))
    assert all(isinstance(x, float) for x in doc["/north_goal_angle"]["ros__parameters"]["geofence_polygon"])


def test_avoid_polygon_main_only(boat):
    """장애물 구간 경계(규정 종합임무 (4)) — 종합임무 파일에만, 토너먼트로는 거부."""
    poly = "[35.1862, 128.5655, 35.1866, 128.5655, 35.1866, 128.566]"
    with pytest.raises(core.ParamError, match="종합임무 전용"):
        boat.apply("geofence_avoid_polygon", poly, course="tournament")
    with pytest.raises(core.ParamError, match="뒤바뀌"):
        boat.apply("geofence_avoid_polygon", "[128.5, 35.1, 128.6, 35.1, 128.6, 35.2]", course="main")
    r = boat.apply("geofence_avoid_polygon", poly, course="main")
    assert r["impacts"] == ["main"]
    assert r["rel"].endswith("north_goal_angle_main.yaml")
    assert any("장애물 구간 경계 ON(3점)" in w for w in r.get("warn", []))
    doc = yaml.safe_load(_text(boat, r["rel"]))
    v = doc["/north_goal_angle"]["ros__parameters"]["geofence_avoid_polygon"]
    assert len(v) == 6 and all(isinstance(x, float) for x in v)
    r = boat.apply("geofence_avoid_polygon", "[]", course="main")
    doc = yaml.safe_load(_text(boat, r["rel"]))
    assert doc["/north_goal_angle"]["ros__parameters"]["geofence_avoid_polygon"] == []


def test_dry_run_writes_nothing(boat):
    rel = "src/ship_back/config/ship_back.yaml"
    before = _text(boat, rel)
    r = boat.apply("hold_dist_m", "3.5", dry_run=True)
    assert r["new"] == "3.5" and _text(boat, rel) == before


def test_undo_restores_head_text(boat):
    rel = "src/ship_back/config/ship_back.yaml"
    boat.apply("hold_dist_m", "3.5")
    assert _text(boat, rel) != _head(boat, rel)
    boat.undo("hold_dist_m")
    assert _text(boat, rel) == _head(boat, rel)
    assert boat.diff() == []


def test_undo_restores_exact_text_hsv_red_double_space(boat):
    """hsv.red 원본 '255,  165'(두 칸) — 되돌리기가 칸까지 저장소와 같아야 git 이 '안 바뀜'으로 본다(11-10 시뮬레이션)."""
    rel = "src/color_shape_detector/config/vision.yaml"
    red = [x for x in boat.params()["params"] if x["key"] == "hsv.red"][0]
    v = red["effective"]["main"]["value"]
    nv = [v[0] + 1] + v[1:]
    boat.apply(red["id"], "[" + ", ".join(map(str, nv)) + "]", course="main")
    assert _text(boat, rel) != _head(boat, rel)
    boat.undo(red["id"], course="main")
    assert _text(boat, rel) == _head(boat, rel)


def test_noop_with_different_spacing_restores_head_line(boat):
    """값은 같고 줄 모양만 다르면(옛 버전이 칸을 바꿔 씀) 같은 값 적용·되돌리기로 저장소 줄을 되찾는다."""
    rel = "src/color_shape_detector/config/vision.yaml"
    t = _text(boat, rel)
    assert "255,  165" in t
    with open(os.path.join(boat.repo, rel), "w", encoding="utf-8") as f:
        f.write(t.replace("255,  165", "255, 165"))
    red = [x for x in boat.params()["params"] if x["key"] == "hsv.red"][0]
    r = boat.undo(red["id"], course="main")
    assert r["noop"] and r.get("note")
    assert _text(boat, rel) == _head(boat, rel)


@pytest.mark.parametrize("raw", ["[1, 2, 3, 4, 5]", "[0, 140, 80, 5, 255]", "[0, 300, 80, 5, 255, 255]",
                                 "[0, 200, 80, 5, 100, 255]", "[0, 140, 80, 190, 255, 255]"])
def test_hsv_rule_rejected_like_node(boat, raw):
    """노드(hsv_ranges.parse_flat)가 기본값으로 돌아갈 HSV 는 도구가 미리 거부한다."""
    src = os.path.join(REPO, core.HSV_RANGES_REL)
    if os.path.exists(src):
        dst = os.path.join(boat.repo, core.HSV_RANGES_REL)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy(src, dst)
    elif raw != "[1, 2, 3, 4, 5]":
        pytest.skip("hsv_ranges.py 없음(최소 규칙만)")
    rel = "src/color_shape_detector/config/vision.yaml"
    before = _text(boat, rel)
    blue = [x for x in boat.params()["params"] if x["key"] == "hsv.blue"][0]
    with pytest.raises(core.ParamError, match="hsv.blue"):
        boat.apply(blue["id"], raw, course="main")
    assert _text(boat, rel) == before


def _copy_node_modules(boat):
    """노드 순수 모듈(turn_color_logic·dock_logic)을 임시 저장소로 복사하고 choices 를 다시 읽는다."""
    rels = sorted({rel for rel, _ in core.NODE_CHOICES.values()})
    for rel in rels:
        src = os.path.join(REPO, rel)
        if not os.path.exists(src):
            pytest.skip(f"{rel} 없음(sparse 사본)")
        dst = os.path.join(boat.repo, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy(src, dst)
    return core.apply_node_choices(boat.cidx, boat.repo)


def _pid(boat, key):
    return [x for x in boat.params()["params"] if x["key"] == key][0]


def test_turn_color_choices_follow_node(boat):
    """탐색 부표 색: 노드(turn_color_logic.TURN_COLORS)가 모르는 색은 적용 거부 — 노드는 부팅 ERROR 후 그 모드 검출을 끈다.
    11-11: 앱 선택지에 yellow 가 있었는데 노드는 red/green/white 만 받았다.
    11-11 오후: 노드가 yellow(탐색 좌현선회)·black(위치유지) 을 받게 됐다(계약서_탐색선회_부표색_20261111 C1)."""
    done = _copy_node_modules(boat)
    assert "turn_color_mode3" in done and "target_shape" in done
    rel = "src/color_shape_detector/config/vision.yaml"
    before = _text(boat, rel)
    p = _pid(boat, "turn_color_mode3")
    assert p["catalog"]["choices"] == ["red", "green", "white", "yellow", "black"]
    for bad in ("orange", "blue", "Red"):
        with pytest.raises(core.ParamError, match="turn_color_mode3"):
            boat.apply(p["id"], bad, course="main")
    assert _text(boat, rel) == before
    boat.apply(p["id"], "green", course="main")
    assert 'turn_color_mode3: "green"' in _text(boat, rel) or "turn_color_mode3: green" in _text(boat, rel)


def test_dock_shape_and_color_choices_follow_node(boat):
    """도킹 표식: 모양은 대문자 시작(Square)만, 색은 dock_logic.VALID_COLORS(yellow 포함)."""
    _copy_node_modules(boat)
    sh = _pid(boat, "target_shape")
    assert sh["catalog"]["choices"] == ["Triangle", "Square", "Circle"]
    with pytest.raises(core.ParamError, match="target_shape"):
        boat.apply(sh["id"], "square", course="main")
    col = _pid(boat, "target_color")
    # 11-12: 현장 도크 색은 빨강·초록·파랑뿐 — 카탈로그 field_choices 로 노드 VALID_COLORS 를 좁힌다.
    assert col["catalog"]["choices"] == ["red", "green", "blue"]
    for bad in ("yellow", "orange", "white"):
        with pytest.raises(core.ParamError, match="target_color"):
            boat.apply(col["id"], bad, course="main")
    boat.apply(col["id"], "blue", course="main")


def test_field_choices_never_adds_value_node_rejects(boat):
    """field_choices 는 좁히기만 한다 — 노드가 모르는 값은 빠지고, 교집합이 비면 노드 목록 그대로."""
    _copy_node_modules(boat)
    it = boat.cidx[("color_shape_detector", "target_color", "image_subscriber_dock")]
    it["field_choices"] = ["red", "navy"]
    core.apply_node_choices(boat.cidx, boat.repo)
    assert it["choices"] == ["red"]
    it["field_choices"] = ["navy"]
    core.apply_node_choices(boat.cidx, boat.repo)
    assert "yellow" in it["choices"] and "navy" not in it["choices"]


def test_node_choices_missing_module_keeps_catalog(boat):
    """노드 모듈이 없으면(못 읽으면) 카탈로그 choices 그대로 — 카탈로그 = 노드 TURN_COLORS 와 같은 목록."""
    assert core.apply_node_choices(boat.cidx, boat.repo) == {}
    assert _pid(boat, "turn_color_mode3")["catalog"]["choices"] == ["red", "green", "white", "yellow", "black"]
    hb = [x for x in boat.params()["params"] if x["key"] == "hsv.black"]
    assert hb and hb[0]["catalog"]["label"] == "검정 HSV 범위"


def test_backup_log_and_diff(boat):
    boat.apply("hold_dist_m", "3.5")
    boat.apply("hold_dist_m", "3.0")
    bdir = os.path.join(boat.home, "ssf_logs", "param_backup")
    backups = sorted(os.listdir(bdir))
    assert len(backups) == 2
    with open(os.path.join(bdir, backups[0]), encoding="utf-8") as f:
        assert "hold_dist_m: 2.5" in f.read()
    log = boat.read_log()
    assert [x["new"] for x in log] == ["3.0", "3.5"]
    d = boat.diff()
    assert len(d) == 1 and d[0]["head"] == "2.5" and d[0]["now"] == "3.0"


def test_noop(boat):
    r = boat.apply("hold_dist_m", "2.5")
    assert r["noop"]
    assert not os.path.exists(os.path.join(boat.home, "ssf_logs", "param_changes.tsv"))


def test_catalog_items_exist(boat):
    """현장 조정 목록의 이름이 진짜 yaml 에 있다(오타면 탭에서 조용히 빠진다)."""
    _, groups = boat.load("A")
    have = {(g[0], g[2], g[1]) for g in groups}
    have_any = {(g[0], g[2]) for g in groups}
    for (pkg, key, sec), _it in core.catalog_index(core.load_catalog()).items():
        if sec:
            assert (pkg, key, sec) in have, (pkg, key, sec)
        else:
            assert (pkg, key) in have_any, (pkg, key)


def test_catalog_guide_fields():
    """현장 조정 항목마다 '언제 바꾸나'(change·when) 가 있고, 되돌릴 값(숫자)은 허용 범위 안이다."""
    kinds = {"daily", "measure", "water", "rarely", "fixed"}
    for (pkg, key, _sec), it in core.catalog_index(core.load_catalog()).items():
        assert it.get("change") in kinds, (pkg, key, it.get("change"))
        assert isinstance(it.get("when"), str) and it["when"].strip(), (pkg, key)
        rv, lim = it.get("revert"), it.get("limits")
        if rv is not None and lim:
            try:
                x = float(str(rv))
            except ValueError:
                continue
            assert lim[0] <= x <= lim[1], (key, rv, lim)


def test_catalog_limits_contain_current_values(boat):
    data = boat.params()
    for p in data["params"]:
        c = p["catalog"]
        if not c or not c.get("limits"):
            continue
        for e in p["effective"].values():
            if e:
                lo, hi = c["limits"]
                assert lo <= e["value"] <= hi, (p["key"], e["value"])


# ---------------------------------------------------------------- 서버

@pytest.fixture
def server(boat):
    import ssf_param_app as app
    from http.server import ThreadingHTTPServer
    srv = ThreadingHTTPServer(("127.0.0.1", 0), None)
    port = srv.server_address[1]
    srv.RequestHandlerClass = app.make_handler(boat, port)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    yield port, boat
    srv.shutdown()
    srv.server_close()


def _req(port, path, body=None, headers=None, host=None):
    h = {"Content-Type": "application/json"}
    h.update(headers or {})
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}",
                                 data=None if body is None else json.dumps(body).encode(), headers=h,
                                 method="GET" if body is None else "POST")
    if host:
        req.add_header("Host", host)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read() or b"{}") if "json" in r.headers.get("Content-Type", "") else r.read()
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def test_server_read_and_page(server):
    port, _ = server
    st, page = _req(port, "/")
    assert st == 200 and b"SSF" in page
    st, d = _req(port, "/api/params")
    assert st == 200 and d["boat"] == "A" and len(d["params"]) > 100


def test_server_post_needs_header(server):
    port, boat = server
    st, _ = _req(port, "/api/set", {"id": "ship_back|/ship_back|hold_dist_m", "value": "3.0"})
    assert st == 403
    assert "2.5" in _text(boat, "src/ship_back/config/ship_back.yaml").split("hold_dist_m:")[1][:6]


def test_server_bad_host_refused(server):
    port, _ = server
    st, _ = _req(port, "/api/state", host="evil.example:80")
    assert st == 403


def test_server_set_and_undo(server):
    port, boat = server
    hdr = {"X-SSF": "1"}
    st, r = _req(port, "/api/set", {"id": "ship_back|/ship_back|hold_dist_m", "value": "3"}, hdr)
    assert st == 200 and r["ok"] and r["new"] == "3.0"
    st, r = _req(port, "/api/set", {"id": "ship_dock|/ship_dock|front_median_kernel", "value": "5.5"}, hdr)
    assert r["ok"] is False and "정수" in r["error"]
    st, d = _req(port, "/api/diff")
    assert len(d["rows"]) == 1
    row = d["rows"][0]
    st, r = _req(port, "/api/undo", {"id": row["id"], "rel": row["rel"]}, hdr)
    assert r["ok"]
    st, d = _req(port, "/api/diff")
    assert d["rows"] == []


# ---------------------------------------------------------------- 웨이포인트 (보기·검사·녹화 준비)

import ssf_waypoints as wpm  # noqa: E402

GOOD_T = ("waypoints:\n"
          "  - {구역: \"T0\", lat: 35.18620, lon: 128.56550, mode: 8, dwell: 0.5}\n"
          "  - {구역: \"T1\", lat: 35.18640, lon: 128.56550, mode: 8, dwell: 0.5}\n")


def _wp(boat, name, text):
    d = os.path.join(boat.home, "ssf_logs")
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, name)
    with open(p, "w", encoding="utf-8") as f:
        f.write(text)
    return p


def test_wp_list_templates_and_recorded(boat):
    _wp(boat, "waypoints_tournament_A_1.yaml", GOOD_T)
    files = boat.waypoints()
    kinds = [(f["kind"], f["name"]) for f in files]
    assert kinds[0] == ("녹화본(노트북)", "waypoints_tournament_A_1.yaml")
    # launch 인자는 $HOME 형식(~/ 는 노드가 못 연다)
    assert files[0]["launch_arg"] == "waypoints_file:=$HOME/ssf_logs/waypoints_tournament_A_1.yaml"
    assert ("저장소 템플릿", "waypoints.yaml") in kinds and ("저장소 템플릿", "waypoints_tournament.yaml") in kinds
    rec = files[0]
    assert rec["course"] == "tournament" and rec["modes"] == [8, 8] and not rec["errors"]
    main_t = [f for f in files if f["name"] == "waypoints.yaml"][0]
    assert not main_t["errors"] and main_t["modes"][-1] == 8


def test_wp_errors_and_warnings(boat):
    _wp(boat, "waypoints_recorded.yaml",
        "waypoints:\n  - {lat: 35.1862, lon: 128.5655, mode: 9, dwell: 1.0}\n")
    _wp(boat, "waypoints_tournament_A_2.yaml",
        "waypoints:\n  - {lat: 35.1862, lon: 128.5655, mode: 5, dwell: 1.0}\n")
    _wp(boat, "waypoints_old.yaml",
        "waypoints:\n  - {lat: 35.1862, lon: 128.5655, mode: 3, dwell: 3.0}\n"
        "  - {lat: 35.1863, lon: 128.5656, mode: 8, dwell: 3.0}\n")
    by = {f["name"]: f for f in boat.waypoints()}
    assert "미션번호 9" in by["waypoints_recorded.yaml"]["errors"][0]
    assert "mode 5" in by["waypoints_tournament_A_2.yaml"]["errors"][0]
    assert any("dwell" in w for w in by["waypoints_old.yaml"]["warnings"])


def test_wp_geofence_check(boat):
    poly = [35.1860, 128.5650, 35.1860, 128.5660, 35.1870, 128.5660, 35.1870, 128.5650]
    boat.apply("geofence_polygon", "[" + ", ".join(map(str, poly)) + "]", course="tournament")
    assert boat.geofence() == poly
    _wp(boat, "waypoints_tournament_A_1.yaml", GOOD_T)
    _wp(boat, "waypoints_tournament_A_3.yaml", GOOD_T.replace("35.18640", "35.18800"))
    by = {f["name"]: f for f in boat.waypoints()}
    assert not by["waypoints_tournament_A_1.yaml"]["warnings"]
    assert any("경계" in w and "WP 1" in w for w in by["waypoints_tournament_A_3.yaml"]["warnings"])


def test_wp_prepare_paths_backup_and_boat_check(boat):
    p = boat.wp_prepare("tournament", "1")
    assert p["out"].endswith("ssf_logs/waypoints_tournament_A_1.yaml") and p["backup"] is None
    assert p["template"].endswith("waypoints_tournament.yaml")
    _wp(boat, "waypoints_tournament_A_1.yaml", GOOD_T)
    p = boat.wp_prepare("tournament", "1")
    assert p["backup"] and open(p["backup"], encoding="utf-8").read() == GOOD_T
    m = boat.wp_prepare("main")
    assert m["out"].endswith("ssf_logs/waypoints_recorded.yaml") and m["template"] is None
    with pytest.raises(core.ParamError, match="레인"):
        boat.wp_prepare("tournament")
    with pytest.raises(core.ParamError, match="레인"):
        boat.wp_prepare("tournament", "../x")
    other = core.Repo(boat.repo, "B", home=boat.home)
    with pytest.raises(core.ParamError, match="배가 다르다"):
        other.wp_prepare("main")


def test_practice_templates_pick_one_mission(boat):
    """연습: 종합임무 템플릿에서 그 미션 줄만 뽑아 녹화 기본값으로 — mode 순서가 미션 그대로."""
    want = {"gate": [0, 1], "hold": [5, 2], "search": [5, 3, 3, 3], "avoid": [5, 5], "dock": [5, 7, 7]}  # OT 순서(11-12): 위치유지 1회·도킹 정렬 6m + 두 줄
    tm = [f for f in boat.waypoints() if f["name"] == "waypoints.yaml"][0]["modes"]
    for m, modes in want.items():
        p = boat.wp_prepare("practice", m)
        assert p["out"].endswith(f"ssf_logs/practice_{m}.yaml") and p["backup"] is None
        t = wpm.check_file(boat.repo, p["template"], "main")
        assert not t["errors"], (m, t["errors"])
        if m != "search" or tm.count(3) == 3:   # 위치유지·탐색은 맨 앞 '연습 접근' mode 5
            assert t["modes"] == modes, (m, t["modes"])
        if m == "dock":
            assert all("도킹" in x["label"] for x in t["points"])
    with pytest.raises(core.ParamError, match="연습 미션"):
        boat.wp_prepare("practice")
    with pytest.raises(core.ParamError, match="연습 미션"):
        boat.wp_prepare("practice", "../x")


def test_practice_file_listed_separately(boat):
    _wp(boat, "practice_hold.yaml", "waypoints:\n  - {구역: \"WP2 위치유지\", lat: 35.18688, lon: 128.56604, mode: 2, dwell: 0.5}\n")
    _wp(boat, "waypoints_recorded.yaml", "waypoints:\n  - {lat: 35.1862, lon: 128.5655, mode: 0, dwell: 3.0}\n")
    by = {f["name"]: f for f in boat.waypoints()}
    pr = by["practice_hold.yaml"]
    assert pr["kind"] == "연습(노트북)" and pr["course"] == "main" and pr["modes"] == [2] and not pr["errors"]
    assert pr["launch_arg"] == "waypoints_file:=$HOME/ssf_logs/practice_hold.yaml"
    assert "위치유지" in pr["notes"][0]
    assert any("mode 5(접근)" in w for w in pr["warnings"])          # 접근점 없이 바로 mode 2
    assert by["waypoints_recorded.yaml"]["kind"] == "녹화본(노트북)"       # 대회 녹화본은 그대로
    assert not any(f["name"].startswith(".practice_template") for f in boat.waypoints())


def test_wp_unreadable_file_does_not_break_list(boat):
    p = _wp(boat, "waypoints_noperm.yaml", GOOD_T)
    os.chmod(p, 0)
    try:
        if os.access(p, os.R_OK):
            pytest.skip("root 라 권한 시험 불가")
        by = {f["name"]: f for f in boat.waypoints()}
        assert "못 읽음" in by["waypoints_noperm.yaml"]["errors"][0]
        assert "waypoints.yaml" in by          # 나머지 목록은 그대로
    finally:
        os.chmod(p, 0o644)


def test_wp_lane_names():
    for ok in ("1", "A-1", "lane_2"):
        wpm.plan(REPO, "/tmp", "B", "tournament", ok)
    for bad in ("-x", "_1", "../x", "a b", "", "x" * 17, "레인1", "1;rm"):
        with pytest.raises(ValueError):
            wpm.plan(REPO, "/tmp", "B", "tournament", bad)


def test_point_in_polygon():
    sq = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0]
    assert wpm.point_in_polygon(0.5, 0.5, sq)
    assert not wpm.point_in_polygon(1.5, 0.5, sq)


def test_server_waypoints_readonly(server):
    port, boat = server
    _wp(boat, "waypoints_tournament_A_1.yaml", GOOD_T)
    st, d = _req(port, "/api/waypoints")
    assert st == 200 and d["ok"] and d["files"][0]["name"] == "waypoints_tournament_A_1.yaml"
    st, _ = _req(port, "/api/waypoints", {"x": 1}, {"X-SSF": "1"})
    assert st == 404


# ---------------------------------------------------------------- 앱에서 녹화 (tmux) — tmux 있는 컴퓨터에서만

import shutil as _sh  # noqa: E402
import time as _time  # noqa: E402

FAKE_RECORDER = r'''#!/usr/bin/env python3
# 가짜 waypoint_recorder — 진짜와 같은 프롬프트 문구로 묻고, q 면 out 에 저장
import sys
out = sys.argv[sys.argv.index("waypoint_recorder") + 1]
pts = []
while True:
    cmd = input("[Enter=추가 / l=목록 / d N=삭제 / r N=재캡처 / q=종료] > ").strip()
    if cmd == "q":
        break
    if cmd == "l":
        for i, p in enumerate(pts):
            print(f"  [{i}] {p}")
        continue
    if cmd.startswith("d "):
        i = int(cmd[2:])
        if 0 <= i < len(pts):
            pts.pop(i); print("  삭제")
        else:
            print(f"  범위 밖 — 0~{len(pts) - 1} 사이")   # 진짜 녹화 도구(parse_idx)와 같게
        continue
    if cmd == "":
        print("  표본 20개 평균 → lat 35.1862000, lon 128.5655000 (최대 공분산 0.01)")
        m = input("  mode(미션번호)? [엔터=8] ").strip() or "8"
        d = input("  dwell(머무는 초)? [엔터=0.5] ").strip() or "0.5"
        n = input("  구역 이름(엔터=T0)? ").strip() or "T0"
        pts.append((35.1862 + 0.0001 * len(pts), 128.5655, int(m), float(d), n))
        print(f"  ✔ WP{len(pts) - 1} 추가됨 (총 {len(pts)}개)")
        continue
    print("  알 수 없는 명령")
if pts:
    import os
    os.makedirs(os.path.dirname(out), exist_ok=True)   # 진짜 녹화 도구도 ~/ssf_logs 를 만든다
    with open(out, "w", encoding="utf-8") as f:
        f.write("waypoints:\n")
        for la, lo, m, d, n in pts:
            f.write(f'  - {{구역: "{n}", lat: {la:.7f}, lon: {lo:.7f}, mode: {m}, dwell: {d}}}\n')
    print(f"💾 {len(pts)}개 저장 완료")
'''


@pytest.fixture
def recboat(boat, tmp_path, monkeypatch):
    if not _sh.which("tmux"):
        pytest.skip("tmux 없음")
    root = tmp_path / "fakeros"
    (root / "opt" / "ros" / "x").mkdir(parents=True)
    (root / "bin").mkdir()
    # set -u 로 부르면 죽는 진짜 ROS setup 흉내(정의 안 된 변수 사용)
    (root / "opt" / "ros" / "x" / "setup.bash").write_text(
        f': "$AMENT_TRACE_SETUP_FILES"\nexport PATH={root}/bin:$PATH\n')
    (root / "bin" / "ros2").write_text("#!/usr/bin/env bash\nexec python3 %s/rec.py \"$@\"\n" % root)
    (root / "bin" / "ros2").chmod(0o755)
    (root / "rec.py").write_text(FAKE_RECORDER)
    # 기록 스크립트·도구를 임시 저장소로(그 안의 /opt/ros 경로를 가짜로)
    tools = os.path.join(boat.repo, "tools")
    os.makedirs(tools, exist_ok=True)
    for f in ("ssf_param_core.py", "ssf_waypoints.py", "ssf_param_catalog.yaml", "param_check.py", "ssf_ros_net.sh",
              "ssf_gps.py", "ssf_gps_watch.sh", "ssf_gps_watch.py", "ssf_lock.py"):
        if os.path.exists(os.path.join(HERE, f)):
            shutil.copy(os.path.join(HERE, f), tools)
    src = open(os.path.join(HERE, "ssf_wp_record.sh"), encoding="utf-8").read()
    open(os.path.join(tools, "ssf_wp_record.sh"), "w", encoding="utf-8").write(
        src.replace("/opt/ros/*/setup.bash", f"{root}/opt/ros/*/setup.bash"))
    os.makedirs(os.path.join(boat.repo, "install"), exist_ok=True)
    open(os.path.join(boat.repo, "install", "setup.bash"), "w").write(': "$UNSET_TOO"\n')
    ngdir = os.path.join(boat.repo, "src", "north_goal_angle", "north_goal_angle")
    if not os.path.isdir(ngdir):
        shutil.copytree(os.path.join(REPO, "src", "north_goal_angle", "north_goal_angle"), ngdir)
    sock = f"ssf_wp_test{os.getpid()}"
    monkeypatch.setenv("SSF_WP_TMUX_SOCKET", sock)        # 사용자 tmux 서버와 분리
    yield boat
    subprocess.run(["tmux", "-L", sock, "kill-server"], capture_output=True)


def _wait(boat, prompt, sec=15):
    end = _time.time() + sec
    st = None
    while _time.time() < end:
        st = boat.wp_rec("status")
        if st["running"] and st["prompt"] == prompt:
            return st
        _time.sleep(0.2)
    raise AssertionError(f"{prompt} 를 못 기다림: {st}")


def test_rec_full_flow(recboat):
    b = recboat
    with pytest.raises(core.ParamError, match="녹화 중이 아니다"):
        b.wp_rec("send", kind="capture")
    b.wp_rec("start", course="tournament", lane="9")
    _wait(b, "main")
    with pytest.raises(core.ParamError, match="이미 녹화 중"):
        b.wp_rec("start", course="tournament", lane="9")
    with pytest.raises(core.ParamError, match="mode·dwell"):
        b.wp_rec("send", kind="answer", arg="8")
    b.wp_rec("send", kind="capture")
    _wait(b, "mode")
    with pytest.raises(core.ParamError, match="다른 걸 묻고"):
        b.wp_rec("send", kind="save")
    b.wp_rec("send", kind="accept")
    _wait(b, "dwell")
    b.wp_rec("send", kind="answer", arg="0.5")
    _wait(b, "name")
    b.wp_rec("send", kind="answer", arg="T0 출발")
    st = _wait(b, "main")
    assert "WP0 추가됨" in st["screen"]
    b.wp_rec("send", kind="capture")
    _wait(b, "mode")
    with pytest.raises(core.ParamError, match="쓸 수 없는 글자"):
        b.wp_rec("send", kind="answer", arg="8; rm -rf ~")
    b.wp_rec("send", kind="accept"); _wait(b, "dwell")
    b.wp_rec("send", kind="accept"); _wait(b, "name")
    b.wp_rec("send", kind="answer", arg="T1 결승선 너머")
    _wait(b, "main")
    b.wp_rec("send", kind="save")
    st = _wait(b, "closing")
    assert "검사" in st["screen"], st["screen"]
    out = os.path.join(b.home, "ssf_logs", "waypoints_tournament_A_9.yaml")
    text = open(out, encoding="utf-8").read()
    assert "T0 출발" in text and "T1 결승선 너머" in text and text.count("mode: 8") == 2
    b.wp_rec("send", kind="close")
    end = _time.time() + 5
    while b.wp_rec("status")["running"] and _time.time() < end:
        _time.sleep(0.2)
    assert not b.wp_rec("status")["running"]
    f = [x for x in b.waypoints() if x["name"] == "waypoints_tournament_A_9.yaml"][0]
    assert f["modes"] == [8, 8] and not f["errors"]


def test_rec_practice_dock_defaults(recboat):
    """연습 도킹 녹화: 녹화 도구에 도킹 줄만 뽑은 템플릿을 넘기고, 대회 녹화본과 다른 파일에 저장한다.
    (가짜 녹화기는 mode 기본값을 늘 8 로 낸다 — 템플릿 내용은 test_practice_templates_pick_one_mission 이 본다)"""
    b = recboat
    b.wp_rec("start", course="practice", lane="dock")
    st = _wait(b, "main")
    assert "연습 dock" in st["screen"] and ".practice_template_dock.yaml" in st["screen"], st["screen"]
    b.wp_rec("send", kind="capture"); _wait(b, "mode")
    b.wp_rec("send", kind="accept"); _wait(b, "dwell")
    b.wp_rec("send", kind="accept"); _wait(b, "name")
    b.wp_rec("send", kind="accept"); _wait(b, "main")
    b.wp_rec("send", kind="save")
    st = _wait(b, "closing")
    out = os.path.join(b.home, "ssf_logs", "practice_dock.yaml")
    assert "practice_dock.yaml" in st["screen"], st["screen"]
    b.wp_rec("send", kind="close")
    end = _time.time() + 5
    while b.wp_rec("status")["running"] and _time.time() < end:
        _time.sleep(0.2)
    f = [x for x in b.waypoints() if x["name"] == "practice_dock.yaml"][0]
    assert len(f["modes"]) == 1 and f["kind"] == "연습(노트북)" and os.path.exists(out)
    assert not os.path.exists(os.path.join(b.home, "ssf_logs", "waypoints_recorded.yaml"))


def test_rec_kill_and_wrong_boat(recboat):
    b = recboat
    b.wp_rec("start", course="main")
    _wait(b, "main")
    b.wp_rec("kill")
    assert not b.wp_rec("status")["running"]
    other = core.Repo(b.repo, "B", home=b.home)
    with pytest.raises(core.ParamError, match="배가 다르다"):
        other.wp_rec("start", course="main")
    with pytest.raises(core.ParamError, match="레인"):
        b.wp_rec("start", course="tournament")


# ---------------------------------------------------------------- D455 영상 송신 (tmux) — tmux 있는 컴퓨터에서만

import ssf_cam  # noqa: E402

FAKE_STREAM = '''#!/usr/bin/env bash
trap 'echo "■ 중지 (Ctrl-C)"; exit 0' INT TERM
echo "=== 카메라 송신 ==="
echo "  대상    : $1:${CAM_PORT} (UDP/RTP H.264)"
echo "  모드    : $2"
while true; do sleep 0.2; done
'''


@pytest.fixture
def camboat(boat, tmp_path, monkeypatch):
    if not _sh.which("tmux"):
        pytest.skip("tmux 없음")
    tools = os.path.join(boat.repo, "tools")
    os.makedirs(tools, exist_ok=True)
    for f in ("ssf_cam_send.sh", "ssf_cam.py", "ssf_ros_net.sh"):
        shutil.copy(os.path.join(HERE, f), tools)
    with open(os.path.join(tools, "cam_stream.sh"), "w") as f:
        f.write(FAKE_STREAM)
    os.chmod(os.path.join(tools, "cam_stream.sh"), 0o755)
    sock = f"ssf_cam_test{os.getpid()}"
    monkeypatch.setenv("SSF_WP_TMUX_SOCKET", sock)
    monkeypatch.delenv("SSH_CLIENT", raising=False)
    monkeypatch.delenv("SSH_CONNECTION", raising=False)
    yield boat
    subprocess.run(["tmux", "-L", sock, "kill-server"], capture_output=True)


def _cam_wait(boat, text, sec=5):
    end = _time.time() + sec
    st = None
    while _time.time() < end:
        st = boat.cam("status")
        if any(text in ln for ln in st["log"]):
            return st
        _time.sleep(0.2)
    raise AssertionError(f"로그에 {text!r} 없음: {st}")


def test_cam_start_stop(camboat):
    b = camboat
    st = b.cam("status")
    assert not st["running"] and st["port"] == 5600 and st["mac_ip"] is None
    with pytest.raises(core.ParamError, match="IP 형식"):
        b.cam("start", "")
    with pytest.raises(core.ParamError, match="MacBook 주소"):
        b.cam("start", "127.0.0.1")
    with pytest.raises(core.ParamError, match="IP 형식"):
        b.cam("start", "1.2.3.4; rm -rf ~")
    b.cam("start", "100.93.158.11")
    st = _cam_wait(b, "대상    : 100.93.158.11:5600")
    assert st["running"]
    assert any("→ 100.93.158.11:5600 (v4l2)" in ln for ln in st["log"])
    assert any("흔들기 시험" in ln for ln in st["log"])
    with pytest.raises(core.ParamError, match="이미 송신 중"):
        b.cam("start", "100.93.158.11")
    st = b.cam("stop")
    assert not st["running"]
    with pytest.raises(core.ParamError, match="송신 중이 아니다"):
        b.cam("stop")


def test_cam_wrong_boat_and_ssh_client(camboat, monkeypatch):
    b = camboat
    other = core.Repo(b.repo, "B", home=b.home)
    with pytest.raises(core.ParamError, match="배가 다르다"):
        other.cam("start", "100.93.158.11")
    assert other.cam("status")["port"] == 5600   # 보기는 등록된 배(이 노트북 = A번배) 기준
    monkeypatch.setenv("SSH_CLIENT", "100.93.158.11 51234 22")
    assert ssf_cam.ssh_client_ip() == "100.93.158.11"
    b.cam("start")            # IP 를 안 주면 SSH 접속 상대(= MacBook)
    st = _cam_wait(b, "대상    : 100.93.158.11:5600")
    b.cam("stop")


def test_server_cam_routes(server):
    port, _ = server
    st, d = _req(port, "/api/cam")
    assert st == 200 and d["ok"] and d["running"] is False
    st, d = _req(port, "/api/cam/start", {"ip": "x"})
    assert st == 403          # 헤더 없음
    st, d = _req(port, "/api/cam/start", {"ip": "not-ip"}, {"X-SSF": "1"})
    assert d["ok"] is False and "IP" in d["error"]


# ---------------------------------------------------------------- 검토(11-10)에서 나온 경우들

def test_detect_prompt_skips_ros_log_lines():
    s = ("[Enter=추가 / l=목록 / d N=삭제 / r N=재캡처 / q=종료] > \n"
         "[WARN] [1794249093.9] [waypoint_recorder]: New publisher discovered on topic, offering incompatible QoS\n")
    assert wpm.detect_prompt(s) == "main"
    assert wpm.detect_prompt("  표본 20개 평균 → lat 35.1\n") == "busy"


def test_rec_accept_gated_copy_mode_and_saved_check(recboat):
    b = recboat
    b.wp_rec("start", course="tournament", lane="7")
    _wait(b, "main")
    st = b.wp_rec("status")
    assert st["what"] == "tournament:7"
    with pytest.raises(core.ParamError, match="mode·dwell"):
        b.wp_rec("send", kind="accept")            # 명령 줄의 Enter = 찍기 — 막혀야
    # 화면 올려 보기(copy-mode) 중에 저장 → copy-mode 먼저 빠져나와 저장돼야('찍기'가 되면 안 됨)
    import ssf_waypoints as w
    sess = w.session_name("A")
    subprocess.run(["tmux", "-L", os.environ["SSF_WP_TMUX_SOCKET"], "copy-mode", "-t", sess], check=True)
    b.wp_rec("send", kind="save")
    st = _wait(b, "closing")
    assert "WP0 추가됨" not in st["screen"]
    assert "이번에 저장된 것이 없다" in st["screen"]   # 0점 → 이전 파일을 이번 녹화로 쓰지 말라
    assert "ROS_LOCALHOST_ONLY=1" in st["screen"]      # 비대화형에서도 ROS 격리
    b.wp_rec("send", kind="close")


# ---------------------------------------------------------------- 동시·겹침(11-10 동시 시험)

def _apply_in_child(repo, home, key, val, start_at):
    import time as t
    while t.time() < start_at:
        pass
    core.Repo(repo, "A", home=home).apply(key, val)


def test_concurrent_apply_same_file_no_lost_update(boat):
    import multiprocessing as mp
    import time as t
    ctx = mp.get_context("fork")
    for i in range(6):
        v1, v2 = ("60.0", "0.6") if i % 2 == 0 else ("0.0", "0.4")
        at = t.time() + 0.3
        ps = [ctx.Process(target=_apply_in_child, args=(boat.repo, boat.home, "side_escape_deg", v1, at)),
              ctx.Process(target=_apply_in_child, args=(boat.repo, boat.home, "reverse_duration_sec", v2, at))]
        [p.start() for p in ps]
        [p.join() for p in ps]
        assert all(p.exitcode == 0 for p in ps)
        txt = _text(boat, "src/ship_direction/config/ship_direction.yaml")
        assert f"side_escape_deg: {v1}" in txt and f"reverse_duration_sec: {v2}" in txt, i
    assert not [f for f in os.listdir(os.path.join(boat.repo, "src/ship_direction/config")) if ".ssf_tmp" in f]


def test_lock_reentrant_and_exclusive(tmp_path):
    import ssf_lock
    import time as t
    order = []
    with ssf_lock.locked(str(tmp_path), "x"):
        with ssf_lock.locked(str(tmp_path), "x"):      # 같은 스레드 재진입
            order.append("inner")

    def other():
        with ssf_lock.locked(str(tmp_path), "x"):
            order.append("other")
    with ssf_lock.locked(str(tmp_path), "x"):
        th = threading.Thread(target=other)
        th.start()
        t.sleep(0.3)
        order.append("holder")                          # 다른 스레드는 아직 못 들어온다
    th.join()
    assert order == ["inner", "holder", "other"]
    with ssf_lock.locked(str(tmp_path), "x"):
        def late():
            with pytest.raises(ssf_lock.LockTimeout):
                with ssf_lock.locked(str(tmp_path), "x", timeout=0.3):
                    pass
        th = threading.Thread(target=late)
        th.start()
        th.join()


def test_rec_concurrent_capture_second_refused(recboat):
    b = recboat
    b.wp_rec("start", course="tournament", lane="c")
    _wait(b, "main")
    res = {}
    bar = threading.Barrier(2)

    def cap(i):
        bb = core.Repo(b.repo, "A", home=b.home)
        bar.wait()
        try:
            bb.wp_rec("send", kind="capture")
            res[i] = "sent"
        except core.ParamError as e:
            res[i] = str(e)
    ts = [threading.Thread(target=cap, args=(i,)) for i in range(2)]
    [x.start() for x in ts]
    [x.join() for x in ts]
    assert sorted(v == "sent" for v in res.values()) == [False, True], res
    assert any("다른 걸 묻고" in v for v in res.values()), res
    _time.sleep(0.5)
    assert b.wp_rec("status")["prompt"] == "mode"     # 두 번째 Enter 가 mode 를 기본값으로 넘기지 않았다
    b.wp_rec("kill")


def test_cam_concurrent_start_one_wins(camboat):
    b = camboat
    res = {}
    bar = threading.Barrier(2)

    def st(i):
        bb = core.Repo(b.repo, "A", home=b.home)
        bar.wait()
        try:
            bb.cam("start", "100.93.158.11")
            res[i] = "ok"
        except core.ParamError as e:
            res[i] = str(e)
    ts = [threading.Thread(target=st, args=(i,)) for i in range(2)]
    [x.start() for x in ts]
    [x.join() for x in ts]
    assert sorted(res.values(), key=lambda v: v != "ok")[0] == "ok"
    assert any("이미 송신 중" in v for v in res.values()), res
    b.cam("stop")


# ---------------------------------------------------------------- GPS·RTK 불

import ssf_gps  # noqa: E402


def test_gps_classify():
    now = 1000.0
    base = {"t": now - 0.5, "fix_ok": True, "fix_type": 3, "h_acc_m": 0.015, "num_sv": 18}
    assert ssf_gps.classify(None, now)[0] == "none"
    assert ssf_gps.classify({"t": now, "error": "x"}, now)[0] == "none"
    assert "감시 실패" in ssf_gps.classify({"t": now - 5, "error": "x"}, now)[1]
    assert "감시 실패" not in ssf_gps.classify({"t": now - 3600, "error": "x"}, now)[1]   # 지난 오류는 안 보인다
    lv, txt = ssf_gps.classify({"t": now, "waiting": True}, now)
    assert lv == "none" and "기다린다" in txt
    assert ssf_gps.classify(dict(base, carr=2, t=now - 5), now)[0] == "none"          # 오래됨
    lv, txt = ssf_gps.classify(dict(base, carr=2), now)
    assert lv == "fixed" and "±1.5 cm" in txt and "위성 18" in txt
    assert ssf_gps.classify(dict(base, carr=1, h_acc_m=0.4), now)[0] == "float"
    lv, txt = ssf_gps.classify(dict(base, carr=0, h_acc_m=2.3), now)
    assert lv == "single" and "±2.3 m" in txt
    assert ssf_gps.classify(dict(base, carr=0, fix_ok=False), now)[0] == "nofix"
    assert ssf_gps.classify(dict(base, carr=2, fix_type=1), now)[0] == "nofix"


def test_gps_status_reads_file_and_touches_heartbeat(boat, monkeypatch):
    import json as js
    import time as t
    monkeypatch.setattr(ssf_gps, "_ensure_watch", lambda *a: None)      # 감시 노드는 따로 시험(배)
    st = boat.gps()
    assert st["level"] == "none"
    assert os.path.exists(os.path.join(boat.home, "ssf_logs", ".gps_watch_A.hb"))
    with open(os.path.join(boat.home, "ssf_logs", "gps_status_A.json"), "w") as f:
        js.dump({"t": t.time(), "carr": 2, "fix_ok": True, "fix_type": 3, "h_acc_m": 0.02, "num_sv": 20,
                 "lat": 35.1, "lon": 128.5}, f)
    st = boat.gps()
    assert st["level"] == "fixed" and st["fixed"] and st["num_sv"] == 20


def test_server_gps_route(server, monkeypatch):
    monkeypatch.setattr(ssf_gps, "_ensure_watch", lambda *a: None)
    port, _ = server
    st, d = _req(port, "/api/gps")
    assert st == 200 and d["ok"] and d["level"] == "none"


# ---------------------------------------------------------------- 오늘 시험(tools/ssf_test_plan.yaml)

def test_plan_file_passes_schema():
    """실제 계획 파일이 스키마를 통과하고 id 가 겹치지 않는다."""
    p = core.load_test_plan()
    ids = [t["id"] for t in p["tests"]]
    assert ids and len(ids) == len(set(ids))
    assert p["title"] and p["date"]
    for t in p["tests"]:
        assert t["title"] and t["steps"] and all(s["text"] for s in t["steps"])
        for k in ("prep", "pass", "record", "params"):
            assert isinstance(t[k], list)


@pytest.mark.parametrize("b", ["A", "B"])
def test_plan_params_exist_in_config(boat, b):
    """params 의 pkg/key 가 그 배가 읽는 config yaml 에 있다(오타면 탭에서 '없다'로 뜬다). launch 가 덮는 키도 아님."""
    _, groups = boat.load(b)
    have = {(g[0], g[2]) for g in groups}
    n = 0
    for t in core.load_test_plan()["tests"]:
        for pk in t["params"]:
            pkg, key = pk.split("/")
            assert (pkg, key) in have, (b, t["id"], pk)
            assert (pkg, key) not in core.LAUNCH_OVERRIDES, pk
            n += 1
    assert n


def test_plan_cmd_placeholders():
    assert core.plan_cmd("ssf {b} wp practice avoid", "A") == "ssf a wp practice avoid"
    assert core.plan_cmd("boat:={B} x {b}", "b") == "boat:=B x b"
    shell = "tee ~/ssf_logs/k_$(date +%H%M).log ${HOME}"
    assert core.plan_cmd(shell, "A") == shell
    cmds = [s["cmd"] for t in core.load_test_plan()["tests"] for s in t["steps"] if "cmd" in s]
    assert any("{B}" in c for c in cmds) and any("{b}" in c for c in cmds)
    for c in cmds:
        for b in "AB":
            out = core.plan_cmd(c, b)
            assert "{b}" not in out and "{B}" not in out
    assert any("boat:=B" in core.plan_cmd(c, "B") for c in cmds)
    assert any(core.plan_cmd(c, "B").startswith("ssf b ") for c in cmds)


def _plan(**over):
    t = {"id": "x", "title": "T", "steps": [{"text": "s", "cmd": "ssf {b} wp"}], "params": ["ship_back/hold_dist_m"]}
    t.update(over)
    return {"date": "2026-11-12", "title": "t", "tests": [t]}


@pytest.mark.parametrize("doc,why", [
    ({"date": "d", "tests": [_plan()["tests"][0], _plan()["tests"][0]]}, "id 가 겹친다"),
    (_plan(title=""), "title 필요"),
    (_plan(id=None), "id 필요"),
    (_plan(steps=[]), "steps 필요"),
    (_plan(steps=[{"cmd": "ls"}]), "text 필요"),
    (_plan(steps=[{"text": "s", "cmd": "ssf {boat} wp"}]), "{boat}"),
    (_plan(params=["hold_dist_m"]), "pkg/key"),
    (_plan(params="ship_back/hold_dist_m"), "params 는"),
    (_plan(pas=["x"]), "모르는 항목 pas"),
    (_plan(prep="한 줄"), "prep 는"),
    ({"date": "d", "tests": []}, "tests"),
    ([1, 2], "맨 위"),
])
def test_plan_schema_rejects(doc, why):
    with pytest.raises(core.ParamError) as e:
        core.check_test_plan(doc)
    assert why in str(e.value)


def test_plan_schema_defaults_and_unquoted_date(tmp_path):
    f = tmp_path / "p.yaml"
    f.write_text("date: 2026-11-13\ntests:\n  - id: a\n    title: A\n    steps:\n      - text: s\n", encoding="utf-8")
    p = core.load_test_plan(str(f))
    assert p["date"] == "2026-11-13" and p["title"] == "" and p["note"] == ""
    t = p["tests"][0]
    assert t["prep"] == t["pass"] == t["record"] == t["params"] == [] and t["why"] == ""
    assert "cmd" not in t["steps"][0]
    f.write_text("tests: [\n", encoding="utf-8")
    with pytest.raises(core.ParamError, match="YAML 문법"):
        core.load_test_plan(str(f))
    with pytest.raises(core.ParamError, match="없다"):
        core.load_test_plan(str(tmp_path / "none.yaml"))


def test_server_plan_route(server, monkeypatch, tmp_path):
    port, _ = server
    st, d = _req(port, "/api/plan")
    assert st == 200 and d["ok"] is True
    assert [t["id"] for t in d["tests"]] == [t["id"] for t in core.load_test_plan()["tests"]]
    st, page = _req(port, "/")
    assert b'data-t="plan"' in page and b"/api/plan" in page
    bad = tmp_path / "bad.yaml"
    bad.write_text("tests:\n  - id: a\n", encoding="utf-8")
    monkeypatch.setattr(core, "PLAN_PATH", str(bad))
    st, d = _req(port, "/api/plan")
    assert st == 200 and d["ok"] is False and "title 필요" in d["error"]

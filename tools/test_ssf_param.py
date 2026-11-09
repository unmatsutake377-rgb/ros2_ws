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
    """임시 저장소(진짜 config yaml 사본 + git 커밋) + 집 폴더(1번배 등록)."""
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

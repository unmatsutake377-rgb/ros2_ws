#!/usr/bin/env python3
"""recorder_prompts(waypoint_recorder 입력 안내) 순수 로직 테스트.

python3 src/north_goal_angle/test/test_recorder_prompts.py  또는  tools/run_tests.sh north_goal_angle
계약서 docs/문제와작업/계약서_waypoint_recorder안내_20261104.md §8. input/print 는 주입해서 ROS·stdin 없이 돈다.

구조 테스트는 아래 _FIXTURE_YAML(고정 4개)로 만든 임시 파일을 쓴다 — 소스 config/waypoints.yaml 은 대회장·수조에서
편집되므로 개수·라벨을 단정하지 않는다(소스 yaml 은 "읽히고 ≥1개, 라벨 있음" + dwell 폴백표 일치만 본다).
yaml 모듈은 필요한 테스트 안에서만 import 하고 없으면 건너뛴다(Mac).
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.waypoint_loader import VALID_MODES  # noqa: E402
from north_goal_angle.recorder_prompts import (  # noqa: E402
    DEFAULT_DWELL_BY_MODE, MODE_DESC, ask_dwell, ask_mode, compare_with_template,
    describe_template, dwell_default, load_template, missing_modes, mode_menu,
    next_expected_line, template_defaults, wp_name,
)
from north_goal_angle.waypoint_recorder import parse_cli, strip_ros_args  # noqa: E402

_PKG = os.path.join(os.path.dirname(__file__), '..')
_WAYPOINTS_YAML = os.path.join(_PKG, 'config', 'waypoints.yaml')

# 고정 템플릿(소스 yaml 과 무관) — 구조 테스트 전용
_FIXTURE_YAML = (
    'waypoints:\n'
    '  - {구역: "WP0 게이트 시작", lat: 35.1862, lon: 128.5655, mode: 0, dwell: 3.0}\n'
    '  - {구역: "WP1 게이트 끝",   lat: 35.1863, lon: 128.5657, mode: 1, dwell: 3.0}\n'
    '  - {구역: "부표",            lat: 35.1868, lon: 128.5658, mode: 3, dwell: 3.0}\n'
    '  - {구역: "WP3 도킹",        lat: 35.1859, lon: 128.5655, mode: 7, dwell: 60.0}\n'
)
_FIXTURE_N = 4

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn()
        _p += 1
        print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


def _has_yaml():
    """pyyaml 없으면(Mac) 템플릿을 읽는 테스트는 건너뛴다(test_waypoint_recorder.py 패턴)."""
    try:
        import yaml  # noqa: F401
        return True
    except ImportError:
        print("     (yaml 모듈 없음 — 건너뜀)")
        return False


class _Script:
    """입력 대본 — 호출될 때마다 다음 답을 돌려주고 질문 횟수를 센다. 대본이 바닥나면 AssertionError."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.prompts = []

    def __call__(self, prompt):
        self.prompts.append(prompt)
        assert self.answers, f"대본보다 질문이 많다: {self.prompts}"
        return self.answers.pop(0)


def _write_tmp(text):
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8")
    f.write(text)
    f.close()
    return f.name


def _load_tmp(text):
    """임시 yaml 로 load_template 실행 후 파일 삭제 → (template, reason)."""
    p = _write_tmp(text)
    try:
        return load_template(p)
    finally:
        os.unlink(p)


def _fixture():
    """고정 템플릿 (rows, labels). yaml 없으면 None(호출자가 건너뜀)."""
    if not _has_yaml():
        return None
    t, reason = _load_tmp(_FIXTURE_YAML)
    assert t is not None, reason
    return t


def _silent(*_):
    pass


# ---------------- 번호표
def test_mode_desc_keys_equal_valid_modes():
    assert set(MODE_DESC) == set(VALID_MODES), (sorted(MODE_DESC), sorted(VALID_MODES))


def test_dwell_table_keys_equal_valid_modes():
    assert set(DEFAULT_DWELL_BY_MODE) == set(VALID_MODES)


def test_mode_menu_lists_every_valid_mode():
    menu = mode_menu(VALID_MODES)
    for m in VALID_MODES:
        assert f"  {m} = {MODE_DESC[m]}" in menu, menu


def test_mode_menu_missing_numbers_4_6():
    assert missing_modes(VALID_MODES) == [4, 6]
    menu = mode_menu(VALID_MODES)
    assert "없는 번호" in menu and "4, 6" in menu, menu


def test_mode_menu_missing_numbers_9_not_listed_but_rejected():
    """9 는 max 보다 커서 표의 '없는 번호' 엔 안 들어가지만 ask_mode 가 거부한다."""
    assert 9 not in missing_modes(VALID_MODES)
    s = _Script(["9", "7"])
    assert ask_mode(s, VALID_MODES, None, print_fn=_silent) == 7
    assert len(s.prompts) == 2


def test_mode_menu_asserts_on_key_mismatch():
    try:
        mode_menu({0, 1, 2})
    except AssertionError:
        return
    assert False, "키 불일치인데 AssertionError 가 안 났다"


def test_mode3_wording_not_assertive():
    assert "hold" in MODE_DESC[3] and "orbit" in MODE_DESC[3], MODE_DESC[3]


# ---------------- load_template (고정 yaml)
def test_load_template_fixture():
    t = _fixture()
    if t is None:
        return
    rows, labels = t
    assert len(rows) == _FIXTURE_N and len(labels) == _FIXTURE_N, (len(rows), len(labels))
    assert labels[0] == "WP0 게이트 시작", labels[0]
    assert rows[0][2] == 0 and rows[0][3] == 3.0
    assert rows[3][2] == 7 and rows[3][3] == 60.0


def test_load_template_source_yaml_readable():
    """소스 waypoints.yaml 은 '읽히고 ≥1개, 라벨 비어 있지 않음' 만 본다(개수·문구는 대회장에서 바뀐다)."""
    if not _has_yaml():
        return
    t, reason = load_template(_WAYPOINTS_YAML)
    assert t is not None, reason
    rows, labels = t
    assert len(rows) >= 1 and len(labels) == len(rows)
    assert all(lb for lb in labels), labels


def test_skip_dock_block_2026_order():
    """2026 순서: 위치유지 1 뒤 도킹 네 줄은 녹화 기본값에서 빠져, 그 뒤 탐색이 한 칸도 안 밀린다."""
    from north_goal_angle.recorder_prompts import skip_dock_block, dock_block_start
    rows = [[0, 0, 0, 3.0], [0, 0, 1, 3.0], [0, 0, 2, 0.5], [0, 0, 5, 1.0], [0, 0, 5, 1.0], [0, 0, 7, 1.0],
            [0, 0, 7, 1.0], [0, 0, 3, 0.5], [0, 0, 2, 0.5], [0, 0, 5, 1.0], [0, 0, 5, 1.0], [0, 0, 8, 3.0]]
    labels = ["WP0", "WP1", "WP2 위치유지 1", "도킹 접근 10m", "도킹 접근 6m", "WP8 도킹 입구", "WP9 도킹 이탈",
              "WP3 부표", "WP5b 위치유지 2", "WP6 회피 시작", "WP7 회피 끝", "WP10 종료"]
    assert dock_block_start(rows, labels) == 3
    r2, l2 = skip_dock_block(rows, labels)
    assert [r[2] for r in r2] == [0, 1, 2, 3, 2, 5, 5, 8], [r[2] for r in r2]
    assert l2[3] == "WP3 부표" and len(l2) == len(r2)
    # 도킹 연습 템플릿(네 줄뿐)은 그대로
    r3, l3 = skip_dock_block(rows[3:7], labels[3:7])
    assert len(r3) == 4
    # '도킹 접근' 이름이 아니면 건드리지 않는다(단독 mode 7 'WP3 도킹' 같은 템플릿)
    assert dock_block_start([[0, 0, 0, 1.0], [0, 0, 7, 1.0]], ["WP0", "WP3 도킹"]) is None


def test_source_yaml_recording_order_has_no_dock_lines():
    """소스 waypoints.yaml 녹화 기본값에 도킹 네 줄이 없고 mode 순서가 2026 규정 순서(도킹 제외)다."""
    if not _has_yaml():
        return
    t, reason = load_template(_WAYPOINTS_YAML)
    assert t is not None, reason
    rows, labels = t
    assert not any("도킹" in lb for lb in labels), labels
    assert [int(r[2]) for r in rows] == [0, 1, 2, 5, 3, 3, 3, 5, 2, 5, 5, 8], [int(r[2]) for r in rows]


def test_load_template_missing_file():
    t, reason = load_template("/nonexistent/dir/waypoints.yaml")
    assert t is None and "파일 없음" in reason, reason


def test_load_template_none_path():
    t, reason = load_template(None)
    assert t is None and reason


def test_load_template_loader_rejects_mode9():
    if not _has_yaml():
        return
    t, reason = _load_tmp(
        'waypoints:\n  - {구역: "x", lat: 35.18, lon: 128.56, mode: 9, dwell: 3.0}\n')
    assert t is None and "로더 검증 실패" in reason and "9" in reason, reason


def test_load_template_yaml_syntax_error():
    if not _has_yaml():
        return
    t, reason = _load_tmp('waypoints: [ {lat: 35.18, lon: 128.56, mode: 0, dwell: 3.0\n')
    assert t is None and "yaml 문법 오류" in reason, reason


def test_load_template_list_form_no_labels():
    if not _has_yaml():
        return
    t, reason = _load_tmp(
        'waypoints:\n  - [35.18, 128.56, 0, 3.0]\n  - [35.19, 128.57, 7, 60.0]\n')
    assert t is not None, reason
    rows, labels = t
    assert labels == ["", ""], labels
    assert rows[1][2] == 7


def test_describe_template_lines():
    t = _fixture()
    if t is None:
        return
    s = describe_template("rel/t.yaml", t, "")
    assert s.startswith("템플릿: /") and f"({_FIXTURE_N}개)" in s, s   # 절대경로 + 개수
    s2 = describe_template(None, None, "파일 없음: x")
    assert "템플릿 없음" in s2 and "파일 없음: x" in s2, s2


# ---------------- template_defaults / dwell_default
def test_template_defaults_inside():
    t = _fixture()
    if t is None:
        return
    assert template_defaults(t, 0) == ("WP0 게이트 시작", 0, 3.0)
    assert template_defaults(t, 3) == ("WP3 도킹", 7, 60.0)


def test_template_defaults_beyond_template():
    t = _fixture()
    if t is None:
        return
    assert template_defaults(t, _FIXTURE_N) is None
    assert template_defaults(t, -1) is None


def test_template_defaults_none_template():
    assert template_defaults(None, 0) is None


def test_dwell_default_priority_template_over_table():
    assert dwell_default(7, 12.5) == 12.5          # 템플릿 우선
    assert dwell_default(7, None) == 1.0           # 표(소스 yaml mode 7 = 1.0)
    assert dwell_default(99, None) is None         # 둘 다 없음 → None(재질문)


def test_dwell_table_matches_source_yaml():
    """폴백표 == 소스 config/waypoints.yaml 의 mode 별 dwell(드리프트 방지)."""
    if not _has_yaml():
        return
    import yaml
    with open(_WAYPOINTS_YAML, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    seen = {}
    for it in raw["waypoints"]:
        m, d = int(it["mode"]), float(it["dwell"])
        assert seen.setdefault(m, d) == d, f"yaml 안에서 mode {m} dwell 이 서로 다르다"
    for m, d in seen.items():
        assert DEFAULT_DWELL_BY_MODE.get(m) == d, (m, d, DEFAULT_DWELL_BY_MODE.get(m))


def test_hold_modes_dwell_half_second():
    """mode 2·3 dwell 0.5(계약서_전체코스_20261104 §10 추가 결정 3), 도킹·회피·토너먼트는 그대로."""
    assert DEFAULT_DWELL_BY_MODE[2] == 0.5 and DEFAULT_DWELL_BY_MODE[3] == 0.5
    assert DEFAULT_DWELL_BY_MODE[5] == 1.0 and DEFAULT_DWELL_BY_MODE[7] == 1.0
    assert DEFAULT_DWELL_BY_MODE[8] == 3.0
    if not _has_yaml():
        return
    import yaml
    with open(_WAYPOINTS_YAML, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    want = {2: 0.5, 3: 0.5, 5: 1.0, 7: 1.0, 8: 3.0}
    for it in raw["waypoints"]:
        m = int(it["mode"])
        if m in want:
            assert float(it["dwell"]) == want[m], (it.get("구역"), m, it["dwell"])


def test_next_expected_line():
    t = _fixture()
    if t is None:
        return
    assert next_expected_line(t, 0) == "다음 예상: WP0 게이트 시작 mode 0 dwell 3"
    assert next_expected_line(t, 2) == "다음 예상: WP2 부표 mode 3 dwell 3"
    assert next_expected_line(t, _FIXTURE_N).startswith(f"다음 예상: WP{_FIXTURE_N} (템플릿 밖")
    assert next_expected_line(None, 0).startswith("다음 예상: WP0 (템플릿 없음")
    assert next_expected_line(t, 1, prefix="다음에 찍을 곳").startswith("다음에 찍을 곳: WP1 게이트 끝")


def test_wp_name_no_double_number():
    assert wp_name(0, "WP0 게이트 시작") == "WP0 게이트 시작"
    assert wp_name(1, "WP10 토너먼트 회피") == "WP1 WP10 토너먼트 회피"   # 'WP10' 은 WP1 이 아니다
    assert wp_name(2, "부표") == "WP2 부표"
    assert wp_name(3, "") == "WP3"


# ---------------- ask_mode
def test_ask_mode_reasks_until_valid():
    """4·6·abc·엔터(기본 없음) 모두 재질문, 7 에서 채택. 캡처 취소(None 반환) 없음."""
    s = _Script(["4", "6", "abc", "", "7"])
    printed = []
    assert ask_mode(s, VALID_MODES, None, print_fn=printed.append) == 7
    assert len(s.prompts) == 5, s.prompts          # 재질문 4회
    assert any("없는 번호" in x for x in printed)
    assert any("mode(미션번호) 표" in x for x in printed)  # 번호표 재표시


def test_ask_mode_enter_takes_default():
    s = _Script([""])
    assert ask_mode(s, VALID_MODES, 7, print_fn=_silent) == 7
    assert len(s.prompts) == 1
    assert "엔터=" in s.prompts[0] and "7" in s.prompts[0], s.prompts[0]


def test_ask_mode_enter_without_default_reasks():
    s = _Script(["", "3"])
    assert ask_mode(s, VALID_MODES, None, print_fn=_silent) == 3
    assert len(s.prompts) == 2


def test_ask_mode_override_default():
    s = _Script(["2"])
    assert ask_mode(s, VALID_MODES, 0, print_fn=_silent) == 2


# ---------------- ask_dwell
def test_ask_dwell_enter_takes_default():
    s = _Script([""])
    assert ask_dwell(s, 60.0, print_fn=_silent) == 60.0
    assert "엔터=" in s.prompts[0] and "60" in s.prompts[0], s.prompts[0]


def test_ask_dwell_negative_reasks():
    s = _Script(["-5", "3"])
    assert ask_dwell(s, 60.0, print_fn=_silent) == 3.0
    assert len(s.prompts) == 2


def test_ask_dwell_enter_without_default_reasks():
    s = _Script(["", "abc", "nan", "0"])
    assert ask_dwell(s, None, print_fn=_silent) == 0.0   # 0 은 명시 입력이면 허용
    assert len(s.prompts) == 4


# ---------------- compare_with_template (고정 yaml)
def _rec_from_template(t):
    rows, labels = t
    return [[r[0], r[1], r[2], r[3], lb] for r, lb in zip(rows, labels)]


def test_compare_identical():
    t = _fixture()
    if t is None:
        return
    assert compare_with_template(_rec_from_template(t), t) == []


def test_compare_mode_differs():
    t = _fixture()
    if t is None:
        return
    rec = _rec_from_template(t)
    rec[1][2] = 2
    w = compare_with_template(rec, t)
    assert len(w) == 1 and "WP1 mode 다름" in w[0] and "기록 2 / 템플릿 1" in w[0], w


def test_compare_fewer_recorded():
    t = _fixture()
    if t is None:
        return
    rec = _rec_from_template(t)[:2]
    w = compare_with_template(rec, t)
    assert any("개수 다름" in x and f"기록 2개 / 템플릿 {_FIXTURE_N}개" in x for x in w), w
    assert any("기록 안 됨" in x and "WP2 부표" in x and "WP3 도킹" in x for x in w), w


def test_compare_more_recorded():
    t = _fixture()
    if t is None:
        return
    rec = _rec_from_template(t) + [[35.18, 128.56, 8, 60.0, "extra"]]
    w = compare_with_template(rec, t)
    assert any("개수 다름" in x and f"기록 {_FIXTURE_N + 1}개 / 템플릿 {_FIXTURE_N}개" in x for x in w), w
    assert any("1개 많음" in x and f"WP{_FIXTURE_N}" in x for x in w), w


def test_compare_none_template():
    assert compare_with_template([[35.18, 128.56, 0, 3.0, ""]], None) == []


# ---------------- parse_cli (waypoint_recorder.py 의 순수 부분)
def test_parse_cli_out_path_positional():
    ns = parse_cli(["out.yaml"])
    assert ns.out_path == "out.yaml" and ns.template is None and ns.unknown == []


def test_parse_cli_template_flag():
    ns = parse_cli(["--template=t.yaml"])
    assert ns.template == "t.yaml" and ns.out_path is None
    ns2 = parse_cli(["--template", "t2.yaml", "o.yaml"])
    assert ns2.template == "t2.yaml" and ns2.out_path == "o.yaml"


def test_parse_cli_ros_args_do_not_become_out_path():
    """예전 함정: `--ros-args` 가 출력 경로가 됐다. 이제 떼어내므로 out_path 기본값(None) 유지."""
    ns = parse_cli(["--ros-args", "-p", "x:=1"])
    assert ns.out_path is None and ns.template is None, vars(ns)
    ns2 = parse_cli(["o.yaml", "--ros-args", "-p", "x:=1", "--", "--template=t.yaml"])
    assert ns2.out_path == "o.yaml" and ns2.template == "t.yaml", vars(ns2)
    assert strip_ros_args(["a", "--ros-args", "b", "--", "c"]) == ["a", "c"]


def main():
    print("=== recorder_prompts 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

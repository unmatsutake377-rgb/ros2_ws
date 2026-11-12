"""recorder_prompts — waypoint_recorder 의 터미널 안내·입력 흐름(ROS 비의존 순수 로직).

왜: 코드를 모르는 팀원이 대회장에서 **엔터만 눌러도** 미션 순서·번호·dwell 이 맞게 들어가게 한다.
  · 번호표(mode_menu): 허용 mode 는 waypoint_loader.VALID_MODES 에서 가져오고, 4·6 같은 "없는 번호" 는 계산으로 낸다.
  · 템플릿(load_template): config/waypoints.yaml 을 읽어 i 번째 WP 의 구역명·mode·dwell 을 기본값으로 쓴다.
    못 읽으면 숨기지 않고 사유를 돌려준다(시작 화면에 "템플릿 없음(사유)" 로 드러낸다, CLAUDE.md §1-6).
  · ask_mode / ask_dwell: 틀린 번호·음수는 **캡처를 취소하지 않고 재질문**한다(위치 평균은 유지).
  · compare_with_template: q 저장 때 템플릿과 개수·순서(mode)가 다르면 경고만 하고 **저장은 항상** 한다.

입력·출력은 input_fn / print_fn 으로 주입받아 테스트한다(test_recorder_prompts.py). yaml 은 함수 안에서
지연 import 한다(Mac 엔 없을 수 있음 → 템플릿 없음으로 처리).
"""

import math
import os

from north_goal_angle.waypoint_loader import VALID_MODES, WaypointError, parse_waypoints

# mode(미션번호) 설명 — 키 집합은 반드시 VALID_MODES 와 같아야 한다(mode_menu 가 assert, 테스트로 고정).
# mode 3 은 launch 의 search_behavior 에 따라 행동이 달라지므로 단정하지 않는다(계약서 §0-⑩).
MODE_DESC = {
    0: "게이트 시작",
    1: "게이트 끝",
    2: "위치유지",
    3: "탐색 부표 — 기본(orbit)은 원 선회(빨강·초록 시계/노랑 반시계), search_behavior:=hold 면 위치유지. 여러 개 가능",
    5: "회피 구간·접근점(ship_avoid 담당 — 기본 lidar 면 ship_direction 순수 회피, 비전 없음)",
    7: "도킹",
    8: "토너먼트 회피 · 종합임무 마지막(WP10 종료 게이트 너머, 도착 뒤 정지)",
}

# 템플릿을 못 읽었을 때의 dwell 폴백표. 규정값이 아니라 **소스 config/waypoints.yaml 의 mode 별 dwell** 과
# 같아야 한다(테스트가 묶어 둔다 — yaml 을 바꾸면 여기도 같이 바꾼다).
DEFAULT_DWELL_BY_MODE = {0: 3.0, 1: 3.0, 2: 0.5, 3: 0.5, 5: 1.0, 7: 1.0, 8: 3.0}


# ---------------- 번호표 ----------------

def missing_modes(valid_modes):
    """0 ~ max(valid) 중 허용 밖 번호(예: {0,1,2,3,5,7,8} → [4, 6]). 사람이 자주 헷갈리는 번호를 표에 적어 주려고."""
    valid = set(valid_modes)
    if not valid:
        return []
    return sorted(set(range(max(valid) + 1)) - valid)


def mode_menu(valid_modes=VALID_MODES):
    """mode 번호표 문자열. MODE_DESC 키 집합이 valid_modes 와 다르면 AssertionError(설명 누락·잉여를 잡는다)."""
    valid = set(valid_modes)
    assert set(MODE_DESC) == valid, (
        f"MODE_DESC 키 {sorted(MODE_DESC)} 와 VALID_MODES {sorted(valid)} 가 다르다 — 둘을 같이 고쳐라")
    lines = ["mode(미션번호) 표 — 이 표에서만 고른다:"]
    for m in sorted(valid):
        d = DEFAULT_DWELL_BY_MODE.get(m)
        dwell_txt = f"dwell 기본 {d:g}초" if d is not None else "dwell 기본 없음"
        lines.append(f"  {m} = {MODE_DESC[m]}  ({dwell_txt})")
    gaps = missing_modes(valid)
    if gaps:
        lines.append(f"  없는 번호: {', '.join(str(m) for m in gaps)} — 넣으면 다시 물어본다")
    return "\n".join(lines)


# ---------------- 템플릿 ----------------

def load_template(path):
    """템플릿 yaml 읽기 → ((rows, labels), "") 또는 (None, 사유).

    rows 는 waypoint_loader.parse_waypoints 결과 [[lat, lon, mode, dwell], ...](검증 통과분만),
    labels 는 원본 dict 의 '구역'(없으면 'label', 없으면 ""). 리스트 형식 항목은 라벨 "".
    파일 없음·pyyaml 없음·yaml 문법 오류·로더 검증 실패 → (None, 사유 한 줄). 호출자는 사유를 화면에 찍고
    기본값 없이 진행한다(모르면 그럴듯한 값을 만들지 않는다).
    """
    if not path:
        return None, "경로 없음"
    if not os.path.isfile(path):
        return None, f"파일 없음: {path}"
    try:
        import yaml  # 지연 import — Mac 등 pyyaml 없는 환경에서도 recorder 자체는 돈다
    except ImportError:
        return None, "pyyaml 없음(템플릿 못 읽음)"
    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except OSError as e:
        return None, f"읽기 실패: {e}"
    except yaml.YAMLError as e:
        return None, f"yaml 문법 오류: {e}"
    try:
        rows = parse_waypoints(raw)
    except WaypointError as e:
        return None, f"로더 검증 실패: {e}"
    labels = []
    for it in raw["waypoints"]:
        if isinstance(it, dict):
            labels.append(str(it.get("구역", it.get("label", "")) or ""))
        else:
            labels.append("")
    return skip_dock_block(rows, labels), ""


# 도킹 네 줄('도킹 접근 10m'·'6m' mode 5 + 입구·이탈 mode 7)은 녹화하지 않고 tools/dock_wp.py 출력으로 끼운다.
# 2026 순서에선 이 네 줄이 위치유지 1 바로 뒤(코스 가운데)에 있어, 템플릿 i 번째 기본값을 그대로 쓰면 그 뒤 점이
# 전부 한 칸씩 어긋난다(탐색 자리에 '도킹 접근' 기본값). → 녹화 기본값에서는 이 네 줄을 뺀다.
DOCK_BLOCK_MODES = [5, 5, 7, 7]


def dock_block_start(rows, labels):
    """도킹 네 줄의 시작 index 또는 None. 첫 mode 7 앞 두 줄이 이름에 '도킹 접근' 이 든 mode 5 일 때만."""
    j = next((k for k, r in enumerate(rows) if int(r[2]) == 7), None)
    if j is None or j < 2 or j + 2 > len(rows):
        return None
    if [int(r[2]) for r in rows[j - 2:j + 2]] != DOCK_BLOCK_MODES:
        return None
    if not all("도킹 접근" in (labels[k] if k < len(labels) else "") for k in (j - 2, j - 1)):
        return None
    return j - 2


def skip_dock_block(rows, labels):
    """(rows, labels) 에서 도킹 네 줄을 뺀 녹화 순서. 네 줄뿐인 템플릿(도킹 연습)은 그대로 둔다."""
    k = dock_block_start(rows, labels)
    if k is None or len(rows) <= len(DOCK_BLOCK_MODES):
        return rows, labels
    keep = [i for i in range(len(rows)) if not (k <= i < k + len(DOCK_BLOCK_MODES))]
    return [rows[i] for i in keep], [labels[i] if i < len(labels) else "" for i in keep]


def describe_template(path, template, reason):
    """시작 화면 한 줄: '템플릿: <절대경로> (N개)' 또는 '템플릿 없음(사유)'."""
    if template is None:
        return f"템플릿 없음({reason}) — mode/dwell 기본값 없이 직접 입력한다"
    rows, _ = template
    return f"템플릿: {os.path.abspath(path)} ({len(rows)}개)"


def template_defaults(template, i):
    """i 번째 WP 의 (구역명, mode, dwell). 템플릿 없음·i 가 범위 밖('템플릿 밖')이면 None. i = 현재 len(recorded)."""
    if template is None:
        return None
    rows, labels = template
    if not (0 <= i < len(rows)):
        return None
    label = labels[i] if i < len(labels) else ""
    return label, int(rows[i][2]), float(rows[i][3])


def dwell_default(mode, template_dwell):
    """dwell 기본값 우선순위: 템플릿 > mode 별 폴백표 > None(기본 없음 → ask_dwell 이 재질문)."""
    if template_dwell is not None:
        return float(template_dwell)
    d = DEFAULT_DWELL_BY_MODE.get(mode)
    return float(d) if d is not None else None


def wp_name(i, label):
    """'WP{i} {구역}' — 구역명이 이미 'WP{i} …' 로 시작하면(우리 yaml 관례) 번호를 두 번 쓰지 않는다."""
    label = (label or "").strip()
    if not label:
        return f"WP{i}"
    if label.split(" ", 1)[0] == f"WP{i}":
        return label
    return f"WP{i} {label}"


def next_expected_line(template, i, prefix="다음 예상"):
    """'{prefix}: WP{i} {구역} mode {m} dwell {d}' 또는 템플릿 없음·밖 안내. 캡처 뒤엔 prefix='다음에 찍을 곳'."""
    d = template_defaults(template, i)
    if d is None:
        why = "템플릿 없음" if template is None else "템플릿 밖"
        return f"{prefix}: WP{i} ({why} — mode/dwell 을 직접 입력)"
    label, mode, dwell = d
    return f"{prefix}: {wp_name(i, label)} mode {mode} dwell {dwell:g}"


# ---------------- 질문(재질문 포함) ----------------

def ask_mode(input_fn, valid_modes, default=None, print_fn=print):
    """mode 를 물어 허용 번호를 돌려준다. 엔터+기본 → 기본 채택. 엔터+기본 없음, 정수 아님, 허용 밖 → 재질문.

    허용 밖 번호(4·6·9)를 넣으면 허용표와 번호표를 다시 보여 주고 다시 묻는다 — 캡처를 취소하지 않는다.
    """
    valid = set(valid_modes)
    hint = f"엔터={default}" if default is not None else "기본 없음 — 번호 입력"
    while True:
        raw = input_fn(f"  mode(미션번호)? [{hint}] ").strip()
        if raw == "":
            if default is not None:
                return int(default)
            print_fn("    기본값이 없다 — 표에서 번호를 골라 입력하세요.")
            print_fn(mode_menu(valid))
            continue
        try:
            m = int(raw)
        except ValueError:
            print_fn("    정수로 입력하세요.")
            continue
        if m not in valid:
            print_fn(f"    {m} 은 없는 번호 — 허용값 {sorted(valid)}. 다시 입력.")
            print_fn(mode_menu(valid))
            continue
        return m


def ask_dwell(input_fn, default=None, print_fn=print):
    """dwell(초)을 물어 0 이상 실수를 돌려준다. 엔터+기본 → 기본 채택. 엔터+기본 없음, 숫자 아님, 음수 → 재질문."""
    hint = f"엔터={default:g}" if default is not None else "기본 없음 — 초 입력"
    while True:
        raw = input_fn(f"  dwell(머무는 초)? [{hint}] ").strip()
        if raw == "":
            if default is not None:
                return float(default)
            print_fn("    기본값이 없다 — 머무는 초(0 이상)를 입력하세요.")
            continue
        try:
            d = float(raw)
        except ValueError:
            print_fn("    숫자로 입력하세요.")
            continue
        if not math.isfinite(d) or d < 0.0:
            print_fn(f"    {raw} 은 안 된다 — 0 이상 숫자. 다시 입력.")
            continue
        return d


# ---------------- 저장 전 비교 ----------------

def compare_with_template(recorded, template):
    """기록 [[lat, lon, mode, dwell, label], ...] 과 템플릿 비교 → 경고 문자열 목록(빈 목록 = 일치).

    개수 다름 / i 번째 mode 다름 / 템플릿에 있는데 미기록 / 템플릿보다 많음 을 적는다.
    경고일 뿐 저장은 호출자가 항상 한다(순서 강제 안 함 — 계약서 §9-1).
    """
    if template is None:
        return []
    rows, labels = template
    warns = []
    n, m = len(recorded), len(rows)
    if n != m:
        warns.append(f"개수 다름: 기록 {n}개 / 템플릿 {m}개")
    for i in range(min(n, m)):
        got, want = int(recorded[i][2]), int(rows[i][2])
        if got != want:
            label = labels[i] if i < len(labels) else ""
            warns.append(f"WP{i} mode 다름: 기록 {got} / 템플릿 {want} ({label or '이름 없음'})")
    if n < m:
        missing = ", ".join(
            f"{wp_name(i, labels[i] if i < len(labels) else '')}(mode {int(rows[i][2])})"
            for i in range(n, m))
        warns.append(f"템플릿에 있는데 기록 안 됨: {missing}")
    elif n > m:
        warns.append(f"템플릿보다 {n - m}개 많음: WP{m}~WP{n - 1} 은 템플릿 밖")
    return warns

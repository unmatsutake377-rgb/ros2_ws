"""웨이포인트 타임아웃 표 — ROS 비의존 순수 로직 (test_wp_timeout.py).

예전엔 north_goal_angle.py 가 '목록 순번'으로 타임아웃을 정했다(wp_idx 3·4·5 → 90 s, <=6 → 120 s,
그 밖 없음). 그래서 같은 회피 구간(mode 5)인 WP6 엔 걸리고 WP7 엔 안 걸렸고, WP7 이후엔
GPS 가 죽으면 다음 WP 로 넘어갈 수단이 없었다. 이제 **wp_mode 기준**으로 정한다
(계약서 docs/문제와작업/계약서_WP타임아웃모드기준_20261029.md, Davidson 결정 10-29).

ROS2 파라미터는 dict 를 못 받는다 → 병렬 배열 두 개:
    wp_timeout_modes: [0, 1, 2, 3, 5, 7]                         (int 배열)
    wp_timeout_sec:   [120.0, 120.0, 120.0, 90.0, 120.0, 150.0]  (double 배열 — 🚨 전부 .0)

규칙 (Davidson 결정):
  · 값은 **WP 하나당** 시간. 같은 모드 WP 가 연속이면 각자 따로 잰다(누적 아님).
  · 표에 없는 모드 = **타임아웃 없음**(None). 기본값을 따로 두지 않는다.
    → 지금 기본값에선 mode 8(토너먼트, 마지막 WP)이 없음.
  · 빈 표 · 값 ≤0 · NaN · inf · 배열 길이 불일치 · VALID_MODES 밖 · 중복 모드
    → WpTimeoutError. 노드는 출발 전에 멈춘다(waypoints.yaml 오류와 같은 방식).
    🚨 '없음'을 0 으로 적으면 안 된다 — remain=0 이라 틱마다 WP 를 건너뛰어 몇 초 만에 미션이 끝난다.
       없애고 싶으면 그 모드를 표에서 빼라.
"""

import math

from north_goal_angle.waypoint_loader import VALID_MODES

# 코드 기본값 — config/north_goal_angle.yaml 과 **똑같이** 둔다(대회 launch 는 yaml 을 싣지만,
# 단독 실행·yaml 누락 때는 이 값이 쓰인다. 둘이 다르면 test_wp_timeout.py 가 잡는다).
# mode 0·1·2·5 = 120 s, 3 = 90 s 는 옛 순번 규칙(WP0~6)과 같은 값 — WP0~6 동작 불변.
# mode 7 = 150 s: ship_dock 선형 최악 ≈129 s 보다 크고 dwell 60 s 보다 크다(WP8·9 → 최대 300 s).
DEFAULT_WP_TIMEOUT_MODES = [0, 1, 2, 3, 5, 7]
DEFAULT_WP_TIMEOUT_SEC = [120.0, 120.0, 120.0, 90.0, 120.0, 150.0]


class WpTimeoutError(ValueError):
    """타임아웃 파라미터가 잘못됐을 때. 메시지에 무엇이 왜 틀렸는지 담는다."""


def build_timeout_table(modes, secs, valid_modes=VALID_MODES):
    """병렬 배열 (modes, secs) → {mode: timeout_sec}. 잘못되면 WpTimeoutError.

    modes/secs 는 list·tuple·array.array 무엇이든 된다(rclpy 배열 파라미터 값).
    """
    if modes is None or secs is None:
        raise WpTimeoutError(f"타임아웃 파라미터가 없다(None): modes={modes!r}, sec={secs!r}")
    modes = list(modes)
    secs = list(secs)
    if len(modes) != len(secs):
        raise WpTimeoutError(
            f"wp_timeout_modes({len(modes)}개)와 wp_timeout_sec({len(secs)}개)의 길이가 다르다: "
            f"modes={modes}, sec={secs}")
    if not modes:
        raise WpTimeoutError(
            "타임아웃 표가 비었다 — 모든 WP 가 GPS 없이 영구 정체될 수 있다 "
            "(시간 초과 진행이 fix 없이 넘어가는 유일한 수단이다).")

    table = {}
    for i, (mode, sec) in enumerate(zip(modes, secs)):
        where = f"{i}번째 항목"
        if not isinstance(mode, int) or isinstance(mode, bool):
            raise WpTimeoutError(f"{where}: 모드 {mode!r} 는 정수여야 한다.")
        if mode not in valid_modes:
            raise WpTimeoutError(
                f"{where}: 모드 {mode} 는 허용값 {sorted(valid_modes)} 에 없다.")
        if mode in table:
            raise WpTimeoutError(f"{where}: 모드 {mode} 가 중복됐다.")
        if isinstance(sec, bool) or not isinstance(sec, (int, float)):
            raise WpTimeoutError(f"{where}: 모드 {mode} 의 시간 {sec!r} 이 숫자가 아니다.")
        sec = float(sec)
        if not math.isfinite(sec):
            raise WpTimeoutError(
                f"{where}: 모드 {mode} 의 시간 {sec} 은 유한한 수여야 한다 "
                f"(타임아웃을 없애려면 그 모드를 표에서 빼라).")
        if sec <= 0.0:
            raise WpTimeoutError(
                f"{where}: 모드 {mode} 의 시간 {sec} 이 0 이하다 — 틱마다 WP 를 건너뛴다 "
                f"(타임아웃을 없애려면 그 모드를 표에서 빼라).")
        table[mode] = sec
    return table


def timeout_for_mode(table, mode):
    """그 모드의 WP 하나당 타임아웃(초). 표에 없으면 None = 타임아웃 없음."""
    return table.get(mode)


def per_wp_timeouts(waypoints, table):
    """[[lat, lon, mode, dwell], ...] → WP 순서대로 타임아웃 목록(없으면 None)."""
    return [timeout_for_mode(table, wp[2]) for wp in waypoints]


def dwell_conflicts(waypoints, table):
    """타임아웃 ≤ dwell 인 WP 목록 [(idx, mode, timeout, dwell), ...].

    타임아웃 시계는 WP 진입부터 dwell 시간까지 포함해 잰다 → timeout ≤ dwell 이면
    '반경 안에서 dwell 초 머무르기'로는 절대 도착할 수 없고 늘 시간 초과로만 넘어간다.
    """
    out = []
    for idx, wp in enumerate(waypoints):
        mode, dwell = wp[2], wp[3]
        t = timeout_for_mode(table, mode)
        if t is not None and t <= dwell:
            out.append((idx, mode, t, dwell))
    return out

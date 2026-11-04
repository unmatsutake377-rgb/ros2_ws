"""LiDAR 로 부표 **방위** 추적 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_turn.lidar_track import lidar_track

왜 필요한가(계약서_ship_turn선회_20261104 §0-2): orbit 자세는 부표를 ±90° 에 두는데 카메라 HFOV 는
±40.16° 라 **설계대로 돌면 부표는 반드시 카메라 밖**이다. 그래서 ORBIT(및 APPROACH 말기)에선
직전 유효 방위 주변의 LiDAR 최근접 빔을 부표로 본다. 부표는 점 물체라 방위 매칭이 통한다.

프레임: 빔 각 a(deg) = angle_min + i·inc, 상대각 rel = a − forward_deg(정면, 노드의 LIDAR_FORWARD_DEG=80).
  → 좌 양수(ship_direction 의 a−80 과 같은 규약). angle_min 이 −180 이든 0 이든 clamp_pm180 으로 같다.

한계(테스트로 문서화):
  · 창 안에 더 가까운 반사(다른 부표·수조 벽·마스트)가 있으면 **가까운 쪽으로 갈아탄다.**
    max_m 과 "직전 rel 연속성"(호출자가 expected 를 직전 매칭값으로 넘김)으로 완화할 뿐 완전 제거는 못 한다.
  · LiDAR 높이 ≠ 부표 높이면 못 본다 → None → 호출자는 (A) 폴백(직전 조향 유지, lost_sec 뒤 SEARCH).
"""

import math


def _clamp_pm180(a):
    return (a + 180.0) % 360.0 - 180.0


def lidar_track(ranges, angle_min_deg, inc_deg, expected_rel_deg, window_deg, max_m, forward_deg):
    """예상 상대각 ±window 안에서 거리 ≤ max_m 인 **최근접 빔**을 부표로 본다.

    ranges          : LaserScan.ranges (m). inf/nan/0 이하는 무시
    angle_min_deg   : LaserScan.angle_min (deg 로 변환해 넘길 것)
    inc_deg         : LaserScan.angle_increment (deg)
    expected_rel_deg: 부표가 있을 것으로 보는 상대각(좌 양수) — 직전 유효 방위
    window_deg      : 탐색 반창(±). 창 밖은 아무리 가까워도 안 본다
    max_m           : 이보다 먼 반사는 부표로 안 본다(벽 배제)
    forward_deg     : LiDAR 프레임에서 정면인 빔 각(노드 상수 80)
    → (rel_deg, dist_m) 또는 None(창 안에 유효 반사 없음 / 입력 불량)
    """
    if ranges is None or expected_rel_deg is None:
        return None
    if not (math.isfinite(inc_deg) and inc_deg > 0.0 and math.isfinite(angle_min_deg)):
        return None
    if not (math.isfinite(window_deg) and window_deg >= 0.0 and math.isfinite(max_m) and max_m > 0.0):
        return None
    best_r = float('inf')
    best_rel = None
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and 0.0 < r <= max_m):
            continue
        if r >= best_r:
            continue
        rel = _clamp_pm180(angle_min_deg + i * inc_deg - forward_deg)
        if abs(_clamp_pm180(rel - expected_rel_deg)) > window_deg:
            continue
        best_r = r
        best_rel = rel
    if best_rel is None:
        return None
    return best_rel, best_r

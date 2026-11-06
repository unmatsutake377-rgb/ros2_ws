"""(C) 갭 후보 정렬 — 편 유지 히스테리시스 + 가로지름 벌점 (ROS 비의존, 순수 함수).

근거: docs/문제와작업/시뮬3자유도_발견_20261106.md 발견 8 (계약서_회피편선택_20261106.md).
기존 정렬은 매 틱 |후보 - 목표| 최소라 이력이 없다 → 두 가지 접촉이 나왔다.
  ① 편 바꿈: 피하던 중 반대편 틈이 0.2° 더 가까워지면 편을 바꿔 장애물 앞을 가로지른다.
  ② 가로지름: 장애물이 정면 살짝 옆인데 목표 쪽 틈을 골라 장애물 앞을 가로지른다.
그래서 정렬 키 첫째 항에 벌점(도)을 더한다. 벌점 둘 다 0.0 이면 기존 정렬과 같다.

candidates 형식은 ship_direction._compute 그대로: (angle_diff, -arc_len, angle, s, e).
각도는 전부 LiDAR 프레임(0~160, 80 = 정면).
"""

import math


def obstacle_blobs(binary, angle_array, distance_array):
    """팽창 **전** 이진 마스크(1 = 장애물)의 연속 구간마다 (중심각, 최근접 거리 m).

    중심각 = (angle[s] + angle[e]) / 2. 최근접 거리 = 그 구간 유한 거리 최솟값,
    전부 비유한이면 inf (모르면 그럴듯한 값을 지어내지 않는다).
    """
    blobs = []
    n = len(binary)
    i = 0
    while i < n:
        if binary[i] != 1:
            i += 1
            continue
        s = i
        while i + 1 < n and binary[i + 1] == 1:
            i += 1
        e = i
        nearest = math.inf
        for k in range(s, e + 1):
            r = distance_array[k]
            if math.isfinite(r):
                nearest = min(nearest, r)
        blobs.append(((angle_array[s] + angle_array[e]) / 2.0, nearest))
        i += 1
    return blobs


def rank_candidates(candidates, prev_side_angle, obstacles, side_hysteresis_deg,
                    cross_penalty_deg, forward_deg=80.0):
    """벌점을 더한 키로 정렬한 **새** 리스트. 원본 candidates 는 안 바꾼다.

    키 = (angle_diff + 벌점, -arc_len).
    - 편 유지: 후보 ≥ 2 이고 prev 가 있고 (후보 - 정면)*(prev - 정면) < 0 이면 +side_hysteresis_deg.
      (정면 == 후보 또는 정면 == prev 이면 곱이 0 → 무벌점)
    - 가로지름: 정면과 후보각 사이 **열린 구간** (lo, hi) 에 장애물 중심각이 하나라도 있으면
      +cross_penalty_deg. 거리 조건 없음 — 마스크 셀은 정의상 탐지 거리 안이다(cross_dist_m 삭제).
    """
    # 🚨 n 은 정렬 **전에** 잡는다. list.sort 도중엔 리스트가 비어 보여 len() 이 0 이다
    #    (CPython 이 정렬 중 리스트를 비워 둔다 — 시제품에서 실제로 걸린 버그).
    n = len(candidates)
    centers = [c for c, _ in obstacles]

    def penalty(angle):
        p = 0.0
        if n >= 2 and prev_side_angle is not None and \
                (angle - forward_deg) * (prev_side_angle - forward_deg) < 0:
            p += side_hysteresis_deg
        lo, hi = min(angle, forward_deg), max(angle, forward_deg)
        if any(lo < c < hi for c in centers):
            p += cross_penalty_deg
        return p

    return sorted(candidates, key=lambda x: (x[0] + penalty(x[2]), x[1]))


def next_side_memory(chosen_angle, n_candidates):
    """다음 틱 편 유지 기준. 후보 ≥ 2 일 때만 기억, 아니면 None(고를 편이 없었다)."""
    return chosen_angle if n_candidates >= 2 else None


# ---- 틈 없을 때 옆 장애물 비킴 — 발견 9 (계약서_옆비킴_현장조정_20261106) ----
# 스위치는 away_deg(노드 파라미터 side_escape_deg) 하나. 아래는 상수(결정 ①) — 바꾸려면 계약서부터.
SIDE_ESCAPE_SIDE_MIN_DEG = 15.0    # |중심 - 정면| 이보다 작으면 '정면' 장애물 → 비킴 안 함(후진)
SIDE_ESCAPE_SIDE_MAX_DEG = 60.0    # 이보다 크면 정횡 너머 → 선미 휘두름 위험, 비킴 안 함(결정 ③)
SIDE_ESCAPE_FRONT_HALF_DEG = 10.0  # 정면 창 ±
SIDE_ESCAPE_FRONT_CLEAR_M = 0.8    # 정면 창·정면→비킬 각 호 전체의 최소 여유
SIDE_ESCAPE_TGT_HALF_DEG = 15.0    # 비킬 각 창 ±
SIDE_ESCAPE_TGT_CLEAR_M = 1.0      # 비킬 각 창의 최소 여유
SIDE_ESCAPE_NO_CELL_M = 99.0       # 창 안에 유한 셀이 없으면 '비었다'로 본다


def _window_min(angle_array, distance_array, lo, hi):
    """lo ≤ a ≤ hi 셀의 거리 최솟값. inf·nan 은 측정 없음이라 건너뛰고, r ≤ 0 은 막힘으로 남긴다.
    셀이 하나도 없으면 SIDE_ESCAPE_NO_CELL_M."""
    m = SIDE_ESCAPE_NO_CELL_M
    for a, r in zip(angle_array, distance_array):
        if lo <= a <= hi and math.isfinite(r):
            m = min(m, r)
    return m


def side_escape(blobs, angle_array, distance_array, away_deg, forward_deg=80.0,
                side_min_deg=SIDE_ESCAPE_SIDE_MIN_DEG, side_max_deg=SIDE_ESCAPE_SIDE_MAX_DEG,
                front_half_deg=SIDE_ESCAPE_FRONT_HALF_DEG,
                front_clear_m=SIDE_ESCAPE_FRONT_CLEAR_M,
                tgt_half_deg=SIDE_ESCAPE_TGT_HALF_DEG, tgt_clear_m=SIDE_ESCAPE_TGT_CLEAR_M):
    """후보 0 개 틱에 후진 대신 낼 비킴 각(LiDAR 프레임) 또는 None(= 기존 후진).

    - away_deg ≤ 0(끔)·비유한·> forward_deg(목표각이 0~160 밖) 이면 None. blobs 비면 None.
    - 가장 가까운 덩어리 중심 c 가 정면에서 side_min_deg~side_max_deg(양끝 포함) 밖이면 None.
    - 장애물 반대편으로: c > 정면(좌현) → 정면 - away_deg(우), c < 정면 → 정면 + away_deg(좌).
    - 원시 거리 셋 다 통과해야 그 각: ① 정면 ±front_half_deg 최솟값 > front_clear_m
      ② 정면~비킬 각 **호 전체**(양끝 포함) 최솟값 > front_clear_m ③ 비킬 각 ±tgt_half_deg > tgt_clear_m.
    입력 리스트는 읽기만 한다.
    """
    if not math.isfinite(away_deg) or away_deg <= 0.0 or away_deg > forward_deg:
        return None
    if not blobs:
        return None
    center, _ = min(blobs, key=lambda b: b[1])
    off = abs(center - forward_deg)
    if not (side_min_deg <= off <= side_max_deg):
        return None
    tgt = forward_deg - away_deg if center > forward_deg else forward_deg + away_deg

    front = _window_min(angle_array, distance_array,
                        forward_deg - front_half_deg, forward_deg + front_half_deg)
    arc = _window_min(angle_array, distance_array, min(forward_deg, tgt), max(forward_deg, tgt))
    side = _window_min(angle_array, distance_array, tgt - tgt_half_deg, tgt + tgt_half_deg)
    if front > front_clear_m and arc > front_clear_m and side > tgt_clear_m:
        return tgt
    return None

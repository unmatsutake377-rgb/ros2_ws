"""카메라·LiDAR 퓨전 → /camera_obstacle_state '가짜 LiDAR' 배열 — 순수 로직(ROS·OpenCV 없음).

근거: docs/문제와작업/계약서_카메라장애물_20261111.md 보완 1 U2b.
생각(팀원 eunwootest 참고 코드에서 가져옴, 코드는 재사용 안 함): **카메라는 색·방위, LiDAR 는 거리**,
방위로 짝짓는다. 짝이 있으면 거리는 LiDAR 값 → "카메라는 방위각만, 거리는 LiDAR"(CLAUDE.md §3) 와 맞는다.
짝이 없을 때만 U2 의 행 기반 보수 추정(obstacle_cam_logic.bbox_range)으로 대체(fusion_fallback_row).

참고 코드에서 **반복하지 않는 것**(테스트로 고정):
    · 군집을 닫은 뒤 새 군집을 시작하지 않아 물체가 합쳐지던 버그 → 닫으면 현재 점으로 새로 시작
    · LiDAR 원 각도(정면 80°)와 카메라 상대방위(정면 0°)를 그대로 비교 → rel = a − 80 으로 바꾼다
    · 이미 도인 값에 math.degrees 재적용, time.time() → 여기엔 시간이 없다(순수 함수)

규약:
    · LiDAR 원 각도 a(deg) → 상대방위 rel = a − LIDAR_FORWARD_DEG(80), ±180 래핑, **좌 +**
      (ship_direction.py LIDAR_FORWARD_DEG·_merge_geofence 와 같은 규약 — 상수 공유 모듈화는 §1-8 로 금지,
       test_obstacle_fusion_logic 이 값 일치를 검사)
    · 군집 = (rel_left_deg, rel_right_deg, min_range_m, n_points), left = 최대 rel, right = 최소 rel. LiDAR 원점.
    · 래핑 이음매(rel ±180 = 바로 뒤)에서 한 물체가 군집 둘로 갈릴 수 있다 — 허용. 카메라는 정면 ±32° 만
      보므로 뒤쪽 군집은 짝지어질 일이 없다.
    · bbox 방위는 카메라 원점(U2 _bbox_cam: 바닥 행·틸트 정확식). 짝짓기 때 군집 끝을 카메라 원점으로 바꿔
      비교한다(카메라↔LiDAR 오프셋 시차 보정, 오프셋 0 이면 항등).
    · 짝이 있는 bbox 의 출력: 거리 = LiDAR min_range(이미 LiDAR 원점 → 원점 변환·range_scale **안 함**),
      방위 = 카메라 방위를 그 LiDAR 거리에 놓인 점으로 보고 LiDAR 원점으로 변환.
    · 프레임 앞단(파라미터 검사·max_bboxes·확인-N)·각폭 검사·격자 칠하기는 U2 와 **같은 헬퍼**
      (obstacle_cam_logic._frame_boxes / _bbox_cam / _finish_sector / _grid_array) — process_frame 과 같은 구조.

U2c — 붐 벽 규칙(현장 사진 2차: 붐 = 흰 부표 두 개 사이를 잇는 빨간 누운 원통, LiDAR 는 부표만 본다):
    · bbox 방위 구간(± fusion_margin_deg)과 겹치는 군집이 **2개 이상**이면, bbox 왼쪽 끝 방위에 가장 가까운
      군집 A 와 (A 를 뺀) 오른쪽 끝에 가장 가까운 군집 B 의 대표점(군집 가운데 방위·min_range, LiDAR 원점)을
      잇는 선분 = 붐. bbox 구간 안 각 출력 칸 방위의 LiDAR 원점 광선이 선분과 만나는 거리를 그 칸에 낸다
      (안 만나는 칸 = 기존 규칙: 겹치는 군집 중 가장 가까운 거리). 1개면 기존 규칙, 0개면 기존 대체.
    · |AB| > boom_max_span_m 이면 벽을 만들지 않는다(멀리 떨어진 두 부표를 잘못 잇는 것 방지) → 기존 규칙.
    · boom_wall_enabled False → 기존 규칙(U2b 와 비트 단위 같음).

scan 신선도는 호출자(U4 노드·U3 시뮬)가 판단한다: 묵었거나 없으면 scan_or_clusters=None 으로 넘긴다.
"""

import math

from color_shape_detector.obstacle_cam_logic import (
    _bbox_cam, _bbox_sector, _finish_sector, _frame_boxes, _grid_array, _num, _size_ok, cam_to_lidar,
    check_params, out_grid)

LIDAR_FORWARD_DEG = 80.0   # LiDAR 프레임 정면(ship_direction.py:39 와 같은 값 — 테스트가 일치 검사)
SOURCE_LIDAR = 'lidar'

# 퓨전 파라미터 기본값 — 계약서 보완 1 U2b. 공통 DEFAULT_PARAMS 와 합쳐 한 dict 로 넘긴다.
FUSION_DEFAULTS = {
    "fusion_margin_deg": 3.0,      # ⚠️ 실측 필요 — bbox 방위 구간을 양쪽으로 이만큼 넓혀 군집과 겹침 판정
    "cluster_gap_deg": 2.0,        # ⚠️ 실측 필요 — 인접 유효 점 방위 차가 이보다 크면 군집 끊음
    "cluster_gap_m": 0.3,          # ⚠️ 실측 필요 — 인접 유효 점 거리 차가 이보다 크면 군집 끊음
    "cluster_min_points": 2,       # ⚠️ 실측 필요 — 🚨 정수. 점이 이보다 적은 군집은 버림(스파이크)
    "fusion_fallback_row": True,   # ⚠️ 실측 필요 — 짝 없는 bbox 를 행 기반 거리로 낼지(False = 버림)
    "boom_wall_enabled": True,     # ⚠️ 실측 필요 — U2c 붐 벽(겹치는 군집 2개 이상 → 두 부표 잇는 선분 거리)
    "boom_max_span_m": 4.0,        # ⚠️ 실측 필요 — 붐 벽 두 부표 간 거리 상한(넘으면 안 이음)
}

# (최소, 최대, 최소 포함, 최대 포함)
_FUSION_RANGES = {
    "fusion_margin_deg": (0.0, 30.0, True, True),
    "cluster_gap_deg": (0.0, 30.0, False, True),
    "cluster_gap_m": (0.0, 5.0, False, True),
    "boom_max_span_m": (0.0, 20.0, False, True),
}
CLUSTER_MIN_POINTS_LIMIT = 1000


def check_fusion_params(p):
    """퓨전 파라미터 → 오류 문자열 리스트(빈 리스트 = 정상). 공통 파라미터는 check_params 가 따로 본다."""
    errs = []
    for k, (lo, hi, lo_in, hi_in) in _FUSION_RANGES.items():
        if k not in p:
            errs.append(f"{k} 없음")
            continue
        v = _num(p[k])
        if v is None:
            errs.append(f"{k}={p[k]!r} 유한 실수 아님")
            continue
        ok_lo = v >= lo if lo_in else v > lo
        ok_hi = v <= hi if hi_in else v < hi
        if not (ok_lo and ok_hi):
            errs.append(f"{k}={v} 범위 밖({lo}~{hi})")
    mp = p.get("cluster_min_points")
    if isinstance(mp, bool) or not isinstance(mp, int):
        errs.append(f"cluster_min_points={mp!r} 정수 아님")
    elif not (1 <= mp <= CLUSTER_MIN_POINTS_LIMIT):
        errs.append(f"cluster_min_points={mp} 범위 밖(1~{CLUSTER_MIN_POINTS_LIMIT})")
    for k in ("fusion_fallback_row", "boom_wall_enabled"):
        if not isinstance(p.get(k), bool):
            errs.append(f"{k}={p.get(k)!r} bool 아님")
    return errs


def _wrap180(a):
    """deg → [−180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------- (1) LiDAR 군집
def lidar_clusters(ranges, angle_min_rad, angle_inc_rad, p):
    """LaserScan ranges → 군집 목록 [(rel_left_deg, rel_right_deg, min_range_m, n_points), …] (스캔 순서).

    · 점 i: a = degrees(angle_min + i·inc), rel = wrap180(a − 80) (좌 +)
    · 건너뜀: 숫자 아님·inf·nan·0 이하·max_range_m 초과. 건너뛴 점은 군집을 직접 끊지 않는다 —
      다음 유효 점과의 방위 차가 cluster_gap_deg 를 넘으면 그때 끊긴다.
    · 직전 유효 점과 |Δrel| > cluster_gap_deg 또는 |Δr| > cluster_gap_m → 현재 군집을 닫고
      **이 점으로 새 군집을 시작**(닫힌 군집은 점 수 ≥ cluster_min_points 일 때만 남김).
    · 래핑 이음매(rel ±180)에선 Δrel ≈ 360 이라 군집이 갈린다 — 허용(정면 관심, 모듈 docstring).
    파라미터·스캔 머리(angle_min·inc 비유한, inc ≤ 0) 무효 → None(모름). 유효하지만 없음 → [].
    p 는 DEFAULT_PARAMS ∪ FUSION_DEFAULTS 전체(max_range_m 은 공통 키).
    """
    if check_params(p) or check_fusion_params(p):
        return None
    a_min, a_inc = _num(angle_min_rad), _num(angle_inc_rad)
    if a_min is None or a_inc is None or a_inc <= 0.0:     # ship_direction scan_callback 과 같은 기준(0·음수 폐기)
        return None
    try:
        rr = list(ranges)
    except TypeError:
        return None
    max_r = float(p["max_range_m"])
    gap_deg, gap_m = float(p["cluster_gap_deg"]), float(p["cluster_gap_m"])
    min_pts = p["cluster_min_points"]

    out = []
    cur = []        # [(rel, r)]

    def close():
        if len(cur) >= min_pts:
            rels = [a for a, _ in cur]
            out.append((max(rels), min(rels), min(r for _, r in cur), len(cur)))

    for i, raw in enumerate(rr):
        r = _num(raw)
        if r is None or r <= 0.0 or r > max_r:
            continue
        rel = _wrap180(math.degrees(a_min + i * a_inc) - LIDAR_FORWARD_DEG)
        if cur and (abs(rel - cur[-1][0]) > gap_deg or abs(r - cur[-1][1]) > gap_m):
            close()
            cur = []        # ★ 닫았으면 새로 시작(참고 코드는 여기서 계속 append → 물체가 합쳐졌다)
        cur.append((rel, r))
    close()
    return out


# ---------------------------------------------------------------- (2) 짝짓기
def _lidar_to_cam_bearing(rel_deg, r, cam_x_m, cam_y_m):
    """LiDAR 원점 (rel, r) 점 → 카메라 원점 방위[deg](좌 +). 카메라 위치 = LiDAR 기준 (x 앞, y 왼쪽)."""
    t = math.radians(rel_deg)
    x = r * math.cos(t) - cam_x_m
    y = r * math.sin(t) - cam_y_m
    if x == 0.0 and y == 0.0:
        return float(rel_deg)
    return math.degrees(math.atan2(y, x))


def _overlapping_clusters(bbox_bearing_left, bbox_bearing_right, clusters, p):
    """bbox 카메라 방위 구간 [right, left] ± fusion_margin_deg 와 겹치는 군집 → [(cam_center_deg, rel_center_deg,
    min_range_m), …] (입력 순서). 군집 끝(rel_left·rel_right)은 그 군집 min_range 에 놓인 점으로 보고 카메라 원점
    방위로 바꿔 비교한다(cam_x/y_from_lidar_m 시차 보정 — 오프셋 0 이면 그대로). 입력 무효 → [].
    """
    bl, br = _num(bbox_bearing_left), _num(bbox_bearing_right)
    m = _num(p.get("fusion_margin_deg"))
    if bl is None or br is None or m is None:
        return []
    lo, hi = min(bl, br) - m, max(bl, br) + m
    cx, cy = float(p.get("cam_x_from_lidar_m", 0.0)), float(p.get("cam_y_from_lidar_m", 0.0))
    out = []
    for c in clusters or ():
        try:
            cl, cr, rmin = _num(c[0]), _num(c[1]), _num(c[2])
        except (TypeError, IndexError):
            continue
        if cl is None or cr is None or rmin is None or rmin <= 0.0:
            continue
        a1 = _lidar_to_cam_bearing(cl, rmin, cx, cy)
        a2 = _lidar_to_cam_bearing(cr, rmin, cx, cy)
        c_lo, c_hi = min(a1, a2), max(a1, a2)
        if c_hi >= lo and c_lo <= hi:
            out.append(((c_lo + c_hi) / 2.0, (cl + cr) / 2.0, rmin))
    return out


def fuse_bbox_range(bbox_bearing_left, bbox_bearing_right, clusters, p):
    """bbox 카메라 방위 구간 [right, left] 를 fusion_margin_deg 만큼 넓혀, 겹치는 군집 중 가장 가까운
    min_range → (range_m, 'lidar'). 겹치는 군집이 없거나 입력 무효 → None.

    range_m 은 LiDAR 원점 거리 그대로(겹침 판정만 카메라 원점 — _overlapping_clusters).
    """
    ov = _overlapping_clusters(bbox_bearing_left, bbox_bearing_right, clusters, p)
    if not ov:
        return None
    return min(o[2] for o in ov), SOURCE_LIDAR


def _xy(rel_deg, r):
    t = math.radians(rel_deg)
    return r * math.cos(t), r * math.sin(t)


def boom_wall_segment(bbox_bearing_left, bbox_bearing_right, clusters, p):
    """U2c: 붐 벽 선분 ((x1, y1), (x2, y2)) LiDAR 원점(x 앞, y 왼쪽), 또는 None.

    겹치는 군집 < 2 / boom_wall_enabled False / 두 대표점 거리 > boom_max_span_m / 입력 무효 → None.
    A = bbox 왼쪽 끝(카메라 방위)에 가운데가 가장 가까운 군집, B = A 를 뺀 나머지 중 오른쪽 끝에 가장 가까운 군집.
    대표점 = (군집 가운데 rel 방위, min_range).
    """
    if p.get("boom_wall_enabled") is not True:
        return None
    ov = _overlapping_clusters(bbox_bearing_left, bbox_bearing_right, clusters, p)
    if len(ov) < 2:
        return None
    bl, br = float(bbox_bearing_left), float(bbox_bearing_right)
    ia = min(range(len(ov)), key=lambda i: abs(ov[i][0] - bl))
    ib = min((i for i in range(len(ov)) if i != ia), key=lambda i: abs(ov[i][0] - br))
    pa = _xy(ov[ia][1], ov[ia][2])
    pb = _xy(ov[ib][1], ov[ib][2])
    if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) > float(p["boom_max_span_m"]):
        return None
    return pa, pb


def ray_segment_range(bearing_deg, seg, origin=(0.0, 0.0)):
    """origin 에서 방위 bearing_deg(좌 +) 광선이 선분 seg 와 만나는 거리 t > 0, 안 만나면 None.

    평행(분모 0)·뒤쪽 교점(t ≤ 0)·선분 밖(s ∉ [0, 1]) → None.
    """
    (x1, y1), (x2, y2) = seg
    t_ = math.radians(bearing_deg)
    dx, dy = math.cos(t_), math.sin(t_)
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    wx, wy = x1 - origin[0], y1 - origin[1]
    t = (wx * ey - wy * ex) / den
    s = (wx * dy - wy * dx) / den
    if t <= 0.0 or s < -1e-9 or s > 1.0 + 1e-9:
        return None
    return t


def _wall_sector(cam, seg, r_fb, p):
    """붐 벽: bbox 카메라 방위 cam + 선분 seg(LiDAR 원점) → (lo, hi, seg, r_fb) LiDAR 원점 부채꼴, 또는 None.

    부채꼴 끝 = bbox 양 끝 카메라 광선이 선분과 만나는 점의 LiDAR 원점 방위(안 만나면 거리 r_fb 에 놓인 점,
    _lidar_range_sector 와 같은 방식). min_sector_deg 미만이면 가운데 기준으로 넓힌다. r_fb > max_range → None.
    """
    if not (r_fb <= float(p["max_range_m"])):
        return None
    cx, cy = float(p["cam_x_from_lidar_m"]), float(p["cam_y_from_lidar_m"])
    bs = []
    for b in (cam[0], cam[2]):
        s = ray_segment_range(b, seg, origin=(cx, cy))
        if s is None:
            s = _cam_range_for_lidar_range(b, r_fb, cx, cy)
        bs.append(cam_to_lidar(b, s, cx, cy)[0])
    lo, hi = min(bs), max(bs)
    ms = float(p["min_sector_deg"])
    if hi - lo < ms:
        c = (lo + hi) / 2.0
        lo, hi = c - ms / 2.0, c + ms / 2.0
    return lo, hi, seg, r_fb


def _paint_walls(arr, walls, p):
    """_grid_array 결과 arr 에 붐 벽 부채꼴을 칸마다 min 으로 칠한다(제자리). 칸 거리 = LiDAR 원점 광선-선분
    교점 거리(없으면 r_fb), 하한 min_range_out_m, max_range_m 초과 칸은 안 칠함."""
    a0, inc, n = out_grid(p)
    r_lo, r_hi = float(p["min_range_out_m"]), float(p["max_range_m"])
    for lo, hi, seg, r_fb in walls:
        k_lo = max(0, int(math.ceil((lo - a0) / inc - 1e-9)))
        k_hi = min(n - 1, int(math.floor((hi - a0) / inc + 1e-9)))
        if k_lo > k_hi:
            k = int(round(((lo + hi) / 2.0 - a0) / inc))
            if not (0 <= k < n):
                continue
            k_lo = k_hi = k
        for k in range(k_lo, k_hi + 1):
            t = ray_segment_range(a0 + k * inc, seg)
            r = r_fb if t is None else t
            if r > r_hi:
                continue
            r = max(r, r_lo)
            if r < arr[2 + k]:
                arr[2 + k] = r
    return arr


def _cam_range_for_lidar_range(b_deg, r_lidar, cam_x_m, cam_y_m):
    """카메라 방위 b 의 광선 위에서 LiDAR 원점 거리가 r_lidar 인 점까지의 카메라 거리.

    |C + s·u| = R 의 양근 s = −C·u + √((C·u)² − |C|² + R²). 해가 없거나(광선이 그 원에 안 닿음) s ≤ 0 이면
    R 로 근사(오프셋이 거리보다 작으면 생기지 않는다). 오프셋 0 이면 정확히 R.
    """
    t = math.radians(b_deg)
    cu = cam_x_m * math.cos(t) + cam_y_m * math.sin(t)
    disc = cu * cu - (cam_x_m * cam_x_m + cam_y_m * cam_y_m) + r_lidar * r_lidar
    if disc < 0.0:
        return r_lidar
    s = -cu + math.sqrt(disc)
    return s if s > 0.0 else r_lidar


def _lidar_range_sector(cam, r_lidar, p):
    """짝 있는 bbox: (b_left, b_center, b_right, _) 카메라 방위 + LiDAR 거리 → (lo, hi, r) LiDAR 원점 또는 None.

    거리는 r_lidar 그대로(변환·range_scale 없음). 방위만 그 거리에 놓인 점으로 보고 LiDAR 원점으로.
    넓히기·하한·max_range 는 U2 _finish_sector 와 같다.
    """
    cx, cy = float(p["cam_x_from_lidar_m"]), float(p["cam_y_from_lidar_m"])
    pts = []
    for b in cam[:3]:
        s = _cam_range_for_lidar_range(b, r_lidar, cx, cy)
        pts.append((cam_to_lidar(b, s, cx, cy)[0], r_lidar))
    return _finish_sector(pts, p)


def _as_clusters(scan_or_clusters, p):
    """None → None(LiDAR 정보 없음). ranges·angle_min·angle_increment 속성(LaserScan) → lidar_clusters.
    그 밖 = 군집 목록 그대로(항목 검사는 fuse_bbox_range 가 한다)."""
    if scan_or_clusters is None:
        return None
    if hasattr(scan_or_clusters, 'ranges'):
        return lidar_clusters(scan_or_clusters.ranges, getattr(scan_or_clusters, 'angle_min', None),
                              getattr(scan_or_clusters, 'angle_increment', None), p)
    try:
        return list(scan_or_clusters)
    except TypeError:
        return None


# ---------------------------------------------------------------- (3) 한 프레임 → 배열
def fused_detections_to_array(bboxes, width, height, scan_or_clusters, p, confirm=None):
    """한 프레임: bbox (x_l, x_r, y_bottom) 목록 + LiDAR → /camera_obstacle_state 배열 (U2 process_frame 의 퓨전판).

    p = DEFAULT_PARAMS ∪ FUSION_DEFAULTS 키를 다 가진 dict. scan_or_clusters = LaserScan(유사 객체) /
    lidar_clusters 결과 목록 / None(스캔 없음·묵음 — 호출자가 판단).
    · 파라미터(공통·퓨전)·이미지 크기 무효 → [] (창 안 건드림)
    · LiDAR 정보 없음(None·스캔 머리 무효) + fusion_fallback_row False → [] (거리를 모른다 — 입 다문다, 창 안 건드림)
      fusion_fallback_row True 면 전부 행 기반(= process_frame 과 같은 결과)
    · bbox 수 > max_bboxes → [] (프레임 이상, 창은 '본 것 없음' 으로 흐름) — U2 와 같은 헬퍼
    · 확인-N(confirm) 통과 bbox 마다: 각폭 > max_bbox_width_deg 면 버림(U2 와 같음),
      겹치는 군집 2개 이상 + 붐 벽 조건(boom_wall_segment) → 칸마다 선분 광선 거리(U2c, _paint_walls),
      그 밖 짝 있음 → LiDAR 거리(_lidar_range_sector), 짝 없음 → fallback True 면 행 기반(_bbox_sector), False 면 버림
    · 검출 없음·전부 버림 → 전부 inf(살아 있음)
    """
    if not _size_ok(width, height) or check_params(p) or check_fusion_params(p):
        return []
    clusters = _as_clusters(scan_or_clusters, p)
    fallback = p["fusion_fallback_row"]
    if clusters is None and not fallback:
        return []
    boxes = _frame_boxes(bboxes, width, height, p, confirm)
    if boxes is None:
        return []
    sectors = []
    walls = []
    for b in boxes:
        cam = _bbox_cam(b, width, height, p)
        if cam is None:
            continue
        fused = fuse_bbox_range(cam[0], cam[2], clusters, p) if clusters else None
        seg = boom_wall_segment(cam[0], cam[2], clusters, p) if fused is not None else None
        if seg is not None:
            ws = _wall_sector(cam, seg, fused[0], p)
            if ws is not None:
                walls.append(ws)
        elif fused is not None:
            sectors.append(_lidar_range_sector(cam, fused[0], p))
        elif fallback:
            sectors.append(_bbox_sector(b, width, height, p))
    arr = _grid_array(sectors, p)
    return _paint_walls(arr, walls, p) if walls else arr

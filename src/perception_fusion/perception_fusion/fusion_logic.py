"""perception_fusion 순수 로직 — ROS 없이 테스트한다(test_fusion_logic.py). 시각은 전부 `now` 인자.

계약서_인식융합_20261112 §4(형식)·§5(융합)·§10(결정), 근거 계약서_거리융합_20261112 §4(D455)·§5(융합 규칙).

좌표: 상대방위 rel(도, 0 = 정면, 왼쪽 +), 원점 = LiDAR. LiDAR 각 a 는 rel = a − 80 (a = 80 + rel, 부호 안 뒤집음).
출력 /fused_objects = Float32MultiArray
  [VER=1, STRIDE=8, N, N × (color_id, bearing_deg, range_m, conf, src_id, age_s, track_id, flags)]
  src_id 0 lidar · 1 depth · 2 fused · 3 row(1단계 안 냄) / flags bit0 어긋남(min 채택) · bit1 LiDAR stale
  color_id 0 모름 · 1 red · 2 green · 3 white · 4 yellow · 5 black · 6 orange · 7 blue
  (🚨 /avoid_detections cls 열거형과 다르다. 사본이 미션 노드 셋에 있다 — 상수 공유 모듈화 금지 §1-8,
   VER·STRIDE 머리 검사로 어긋남을 잡는다.)
거리 없는 물체는 안 넣는다. 빈 목록 = 살아 있고 물체 없음. LiDAR·뎁스 둘 다 stale = 발행 안 함(None).

신뢰도(소스별 최근 N 프레임, 질의 방위 ± fusion_margin 1° 칸 최솟값 시계열):
  conf = 유효율 × (1 − 0.5·min(MAD / conf_mad_ref_m, 1) − 0.5·점프율)
  MAD = 연속 유효값 차이 |Δr| 의 중앙값, 점프 = |Δr| ≥ conf_jump_m, 유효값 3개 미만이면 안정도 0.5.
  곱셈 식이다 — 덧셈 식은 반짝임 1프레임(유효율 0.1)을 0.5 넘게 통과시켜 기각됐다(fusion_sim).
"""

from collections import deque, namedtuple
import math

import numpy as np

# ---- 형식 상수 (미션 노드 셋에 사본 — 바꾸면 VER 을 올리고 넷을 같이) ----
VER = 1
STRIDE = 8
SRC_LIDAR = 0
SRC_DEPTH = 1
SRC_FUSED = 2
SRC_ROW = 3
FLAG_DISAGREE = 1
FLAG_LIDAR_STALE = 2
COLOR_IDS = {'red': 1, 'green': 2, 'white': 3, 'yellow': 4, 'black': 5, 'orange': 6, 'blue': 7}

LIDAR_FORWARD_DEG = 80.0      # LiDAR 프레임 정면 (ship_back·ship_turn·ship_gate 와 같은 규약)
IMAGE_ANGLE_INVALID = 10000.0  # 검출기 '안 보임' 센티넬 (basic_image_subscriber* 와 같은 값)

# 코드 기본값 == config/perception_fusion.yaml == 노드 declare_parameter 리터럴 (test_fusion_params 가 묶음)
DEFAULTS = {
    'depth_topic': '/camera/camera/depth/image_rect_raw',
    'depth_info_topic': '/camera/camera/depth/camera_info',
    'publish_hz': 10.0,
    'status_hz': 1.0,
    'lidar_stale_sec': 0.5,
    'depth_stale_sec': 0.5,
    'det_stale_sec': 0.7,
    'warn_period_sec': 5.0,
    'lidar_half_fov_deg': 90.0,
    'cluster_gap_deg': 2.0,
    'cluster_gap_m': 0.3,
    'cluster_min_points': 2,
    'fusion_margin_deg': 3.0,
    'color_match_deg': 3.0,
    'weight_min': 0.3,
    'weight_max': 0.7,
    'agree_abs_m': 0.3,
    'agree_rel': 0.1,
    'disagree_penalty': 0.1,
    'conf_window_frames': 10,
    'conf_mad_ref_m': 0.3,
    'conf_jump_m': 0.5,
    'depth_min_m': 0.5,
    'depth_max_m': 6.0,
    'depth_row_lo_frac': 0.4,
    'depth_row_hi_frac': 0.6,
    'depth_min_valid_frac': 0.3,
    'cam_x_from_lidar_m': 0.0,
    'cam_y_from_lidar_m': 0.0,
    'cam_yaw_offset_deg': 0.0,
}

Blob = namedtuple('Blob', 'lo hi bearing range_m')


def wrap180(a):
    """각(도, 스칼라·배열) → [−180, 180)."""
    return ((np.asarray(a, dtype=float) + 180.0) % 360.0) - 180.0


def color_id(name):
    """/image_color 문자열 → color_id. 모르는 값('none'·빈 문자열 포함) = 0."""
    return COLOR_IDS.get(str(name).strip().lower(), 0)


def detector_angle_valid(v):
    """검출기 방위가 유효한가: 10000 센티넬·nan·inf 는 무효."""
    v = float(v)
    return math.isfinite(v) and abs(v - IMAGE_ANGLE_INVALID) >= 1e-3


def detections_from(now, red, green, image, image_color, stale_sec):
    """검출기 상태 → [(방위, color_id)]. 각 입력은 (수신 시각, 값) 또는 None. 묵으면(stale_sec) 뺀다.

    /image_angle 은 /image_color 가 신선할 때만 그 색으로 붙인다(dock 검출기는 색을 안 낸다 → 0 = 안 씀).
    """
    out = []
    for det, cid in ((red, COLOR_IDS['red']), (green, COLOR_IDS['green'])):
        if det is not None and now - det[0] <= stale_sec:
            out.append((float(det[1]), cid))
    if image is not None and now - image[0] <= stale_sec:
        cid = 0
        if image_color is not None and now - image_color[0] <= stale_sec:
            cid = color_id(image_color[1])
        if cid:
            out.append((float(image[1]), cid))
    return out


# ------------------------------------------------------------ LiDAR

def scan_points(ranges, angle_min_rad, angle_inc_rad, range_min, half_fov_deg):
    """LaserScan → 정면 ±half_fov 유효점 (rel 배열, r 배열). inf/nan/≤range_min 무효.

    angle_increment 가 0·음수·nan 이면 None(스캔 자체를 못 믿는다 — 호출자가 버린다).
    """
    inc = float(angle_inc_rad)
    if not math.isfinite(inc) or inc <= 0.0 or not math.isfinite(float(angle_min_rad)):
        return None
    r = np.asarray(ranges, dtype=float)
    a = np.degrees(float(angle_min_rad) + np.arange(len(r)) * inc)
    rel = wrap180(a - LIDAR_FORWARD_DEG)
    rmin = float(range_min) if math.isfinite(float(range_min)) else 0.0
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(r) & (r > rmin) & (np.abs(rel) <= half_fov_deg)
    return rel[ok], r[ok]


# ------------------------------------------------------------ 군집

def cluster(rel, r, gap_deg, gap_m, min_points, reduce='min'):
    """방위순 정렬 뒤 이웃 간격 > gap_deg 또는 거리 차 > gap_m 에서 끊는다. min_points 미만 덩어리는 버림.

    거리 = 최솟값(LiDAR: 가까운 표면, ship_back _dist_at 과 같은 뜻) 또는 중앙값(뎁스 열 거리).
    방위 = 구간 가운데 (lo + hi) / 2.
    """
    rel = np.asarray(rel, dtype=float)
    r = np.asarray(r, dtype=float)
    if len(rel) == 0:
        return []
    o = np.argsort(rel, kind='stable')
    aa = rel[o].tolist()
    rr = r[o].tolist()
    out = []
    s = 0
    n = len(aa)
    for i in range(1, n + 1):
        if i == n or (aa[i] - aa[i - 1]) > gap_deg or abs(rr[i] - rr[i - 1]) > gap_m:
            if i - s >= min_points:
                seg = rr[s:i]
                rng = float(np.median(seg)) if reduce == 'median' else float(min(seg))
                lo, hi = aa[s], aa[i - 1]
                out.append(Blob(lo, hi, 0.5 * (lo + hi), rng))
            s = i
    return out


# ------------------------------------------------------------ D455 뎁스

def depth_image(data, height, width, step, is_bigendian):
    """sensor_msgs/Image(16UC1) 바이트 → uint16 (H, W). 길이·크기가 안 맞으면 None."""
    h, w, step = int(height), int(width), int(step)
    if h <= 0 or w <= 0 or step < 2 * w:
        return None
    try:
        buf = np.frombuffer(data, dtype=np.uint8)       # bytes·array.array('B') (rclpy uint8[])
    except TypeError:
        buf = np.asarray(data, dtype=np.uint8)           # 리스트로 오는 경우 대비
    if len(buf) < h * step:
        return None
    rows = buf[:h * step].reshape(h, step)[:, :2 * w]
    dt = np.dtype('>u2') if is_bigendian else np.dtype('<u2')
    return np.ascontiguousarray(rows).view(dt).reshape(h, w).astype(np.uint16)


def column_bearings(width, fx, cx):
    """열 u → 카메라 기준 방위(도, 왼쪽 +). 오른쪽 열(u > cx) = 음수."""
    u = np.arange(int(width), dtype=float)
    return -np.degrees(np.arctan2(u - float(cx), float(fx)))


def column_z(img, row_lo_frac, row_hi_frac, min_valid_frac):
    """열마다 행 띠 [lo, hi) 의 유효(0 아님) 픽셀 median z(m). 유효 픽셀이 하한 미만이면 nan(모름)."""
    h, w = img.shape
    lo = int(math.floor(round(row_lo_frac * h, 6)))
    hi = int(math.ceil(round(row_hi_frac * h, 6)))
    lo = min(max(lo, 0), h - 1)
    hi = min(max(hi, lo + 1), h)
    band = img[lo:hi].astype(float)
    band[band == 0.0] = np.nan                  # 0 = 모름 (D455 구멍)
    cnt = np.sum(np.isfinite(band), axis=0)
    need = max(1, int(math.ceil(min_valid_frac * (hi - lo) - 1e-9)))
    z = np.full(w, np.nan)
    ok = cnt >= need
    if ok.any():
        z[ok] = np.nanmedian(band[:, ok], axis=0) / 1000.0      # mm → m
    return z


def cam_to_lidar(bearing_deg, r, cam_x, cam_y, yaw_deg):
    """카메라 원점 (방위, 거리) → LiDAR 원점 (rel, r). 카메라 위치 (cam_x 앞 +, cam_y 왼쪽 +), 요 왼쪽 +."""
    th = np.radians(np.asarray(bearing_deg, dtype=float) + yaw_deg)
    px = cam_x + np.asarray(r, dtype=float) * np.cos(th)
    py = cam_y + np.asarray(r, dtype=float) * np.sin(th)
    return np.degrees(np.arctan2(py, px)), np.hypot(px, py)


def depth_points(img, fx, cx, p):
    """뎁스 영상 → LiDAR 원점 열 점 (rel, r). z ∈ [depth_min_m, depth_max_m] 만, r = z / cos(방위)."""
    z = column_z(img, p['depth_row_lo_frac'], p['depth_row_hi_frac'], p['depth_min_valid_frac'])
    b = column_bearings(img.shape[1], fx, cx)
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(z) & (z >= p['depth_min_m']) & (z <= p['depth_max_m'])
    b = b[ok]
    r = z[ok] / np.cos(np.radians(b))
    return cam_to_lidar(b, r, p['cam_x_from_lidar_m'], p['cam_y_from_lidar_m'], p['cam_yaw_offset_deg'])


# ------------------------------------------------------------ 신뢰도

def frame_bins(rel, r):
    """한 프레임 → 1° 칸 360개 최솟값(유효점 없으면 inf). 칸 k = floor(rel + 180)."""
    out = np.full(360, np.inf)
    rel = np.asarray(rel, dtype=float)
    if len(rel):
        bi = np.floor(rel + 180.0).astype(int) % 360
        np.minimum.at(out, bi, np.asarray(r, dtype=float))
    return out


def window_conf(hist, bearing, margin_deg, mad_ref_m, jump_m):
    """최근 프레임 칸들(hist)에서 bearing ± margin 칸 최솟값 시계열 → conf ∈ [0, 1] (머리말 식)."""
    if not hist:
        return 0.0
    lo = int(math.floor(bearing - margin_deg + 180.0))
    hi = int(math.floor(bearing + margin_deg + 180.0))
    idx = [k % 360 for k in range(lo, hi + 1)]
    v = np.array([h[idx].min() for h in hist])
    valid = np.isfinite(v)
    vr = float(valid.mean())
    vv = v[valid]
    if len(vv) >= 3:
        d = np.abs(np.diff(vv))
        mad = float(np.median(d))
        jr = float(np.mean(d >= jump_m))
        stab = 1.0 - 0.5 * min(mad / mad_ref_m, 1.0) - 0.5 * jr
    else:
        stab = 0.5
    return float(min(max(vr * stab, 0.0), 1.0))


class SourceTrack:
    """소스 하나(LiDAR 또는 뎁스)의 마지막 프레임 + 신뢰도용 최근 N 프레임 칸."""

    def __init__(self, n_frames):
        self.t = None
        self.rel = np.array([])
        self.r = np.array([])
        self.hist = deque(maxlen=int(n_frames))

    def push(self, t, rel, r):
        self.t = float(t)
        self.rel = np.asarray(rel, dtype=float)
        self.r = np.asarray(r, dtype=float)
        self.hist.append(frame_bins(self.rel, self.r))

    def fresh(self, now, stale_sec):
        return self.t is not None and (now - self.t) <= stale_sec


# ------------------------------------------------------------ 연관·융합

def associate(lblobs, dblobs, margin_deg):
    """LiDAR 군집 ↔ 뎁스 덩어리: 방위 구간이 ±margin 으로 겹치면 후보, 방위 차 작은 순으로 한 번씩.

    → [(l, d)] 짝 먼저, 그다음 짝 없는 l (l, None), 짝 없는 d (None, d).
    """
    cands = []
    for i, lb in enumerate(lblobs):
        for j, db in enumerate(dblobs):
            if lb.lo - margin_deg <= db.hi and db.lo <= lb.hi + margin_deg:
                cands.append((abs(lb.bearing - db.bearing), i, j))
    cands.sort()
    used_l, used_d, pairs = set(), set(), []
    for _, i, j in cands:
        if i in used_l or j in used_d:
            continue
        used_l.add(i)
        used_d.add(j)
        pairs.append((lblobs[i], dblobs[j]))
    pairs += [(lb, None) for i, lb in enumerate(lblobs) if i not in used_l]
    pairs += [(None, db) for j, db in enumerate(dblobs) if j not in used_d]
    return pairs


def fuse_pair(d_l, c_l, d_d, c_d, p):
    """LiDAR·뎁스 거리 하나씩 → (거리, src_id, conf, flags).

    일치(|dL − dD| ≤ max(agree_abs_m, agree_rel·min)) → w = 정규화 conf 를 [weight_min, weight_max] 로 자른
      가중평균, src fused, conf = max.
    어긋남 → **min**(평균은 물체 없는 거리를 만든다), src = min 을 낸 소스, conf = min − disagree_penalty,
      flags bit0.
    """
    tol = max(p['agree_abs_m'], p['agree_rel'] * min(d_l, d_d))
    if abs(d_l - d_d) <= tol:
        s = c_l + c_d
        w = 0.5 if s <= 0.0 else c_l / s
        w = min(max(w, p['weight_min']), p['weight_max'])
        return w * d_l + (1.0 - w) * d_d, SRC_FUSED, max(c_l, c_d), 0
    conf = max(0.0, min(c_l, c_d) - p['disagree_penalty'])
    if d_l <= d_d:
        return d_l, SRC_LIDAR, conf, FLAG_DISAGREE
    return d_d, SRC_DEPTH, conf, FLAG_DISAGREE


def assign_colors(objs, detections, margin_deg):
    """검출기 방위가 물체 구간 [lo − margin, hi + margin] 안이면 그 색. 방위 차 작은 순(동점 → 가까운 것),
    검출 하나는 물체 하나에만, 물체 하나는 색 하나만."""
    cands = []
    for k, (b, cid) in enumerate(detections):
        if not cid:
            continue
        for i, o in enumerate(objs):
            if o['lo'] - margin_deg <= b <= o['hi'] + margin_deg:
                cands.append((abs(o['bearing_deg'] - b), o['range_m'], k, i, cid))
    cands.sort()
    used_k, used_i = set(), set()
    for _, _, k, i, cid in cands:
        if k in used_k or i in used_i:
            continue
        used_k.add(k)
        used_i.add(i)
        objs[i]['color_id'] = cid


def build_objects(now, lidar, depth, detections, p):
    """→ (물체 목록 또는 None, 상태). None = LiDAR·뎁스 둘 다 stale → 발행하지 않는다.

    LiDAR stale 이면 뎁스만으로 계속하고 모든 물체에 flags bit1. 뎁스 stale 이면 LiDAR 만.
    물체 dict: color_id, bearing_deg, range_m, conf, src_id, age_s, track_id(-1), flags.
    """
    lok = lidar.fresh(now, p['lidar_stale_sec'])
    dok = depth.fresh(now, p['depth_stale_sec'])
    stats = {'lidar_ok': lok, 'depth_ok': dok, 'n': None, 'conf_l': None, 'conf_d': None, 'agree': None}
    if not lok and not dok:
        return None, stats
    half = p['lidar_half_fov_deg']
    margin = p['fusion_margin_deg']
    gap_deg, gap_m, min_pts = p['cluster_gap_deg'], p['cluster_gap_m'], p['cluster_min_points']
    lbl = cluster(lidar.rel, lidar.r, gap_deg, gap_m, min_pts) if lok else []
    dbl = []
    if dok:
        sel = np.abs(depth.rel) <= half
        dbl = cluster(depth.rel[sel], depth.r[sel], gap_deg, gap_m, min_pts, reduce='median')
    objs, c_ls, c_ds = [], [], []
    n_pair = n_agree = 0
    for lb, db in associate(lbl, dbl, margin):
        if lb is not None:
            c_l = window_conf(lidar.hist, lb.bearing, margin, p['conf_mad_ref_m'], p['conf_jump_m'])
            c_ls.append(c_l)
        if db is not None:
            c_d = window_conf(depth.hist, db.bearing, margin, p['conf_mad_ref_m'], p['conf_jump_m'])
            c_ds.append(c_d)
        if lb is not None and db is not None:
            rng, src, conf, flags = fuse_pair(lb.range_m, c_l, db.range_m, c_d, p)
            n_pair += 1
            n_agree += 0 if flags & FLAG_DISAGREE else 1
            bearing, lo, hi = lb.bearing, min(lb.lo, db.lo), max(lb.hi, db.hi)
            age = now - min(lidar.t, depth.t)
        elif lb is not None:
            rng, src, conf, flags = lb.range_m, SRC_LIDAR, c_l, 0
            bearing, lo, hi, age = lb.bearing, lb.lo, lb.hi, now - lidar.t
        else:
            rng, src, conf, flags = db.range_m, SRC_DEPTH, c_d, 0
            bearing, lo, hi, age = db.bearing, db.lo, db.hi, now - depth.t
        if not lok:
            flags |= FLAG_LIDAR_STALE
        objs.append({'color_id': 0, 'bearing_deg': float(bearing), 'range_m': float(rng),
                     'conf': float(conf), 'src_id': src, 'age_s': float(max(age, 0.0)),
                     'track_id': -1, 'flags': flags, 'lo': lo, 'hi': hi})
    assign_colors(objs, detections, p['color_match_deg'])
    objs.sort(key=lambda o: (o['range_m'], o['bearing_deg']))
    for o in objs:
        del o['lo'], o['hi']
    stats.update(n=len(objs),
                 conf_l=float(np.mean(c_ls)) if c_ls else None,
                 conf_d=float(np.mean(c_ds)) if c_ds else None,
                 agree=(n_agree / n_pair) if n_pair else None)
    return objs, stats


# ------------------------------------------------------------ 형식

FIELDS = ('color_id', 'bearing_deg', 'range_m', 'conf', 'src_id', 'age_s', 'track_id', 'flags')


def encode(objs):
    """물체 목록 → Float32MultiArray.data ([VER, STRIDE, N] + N × STRIDE)."""
    data = [float(VER), float(STRIDE), float(len(objs))]
    for o in objs:
        data += [float(o[k]) for k in FIELDS]
    return data


def decode(data):
    """encode 의 역. 머리(VER·STRIDE)·길이가 안 맞으면 None — 엉뚱한 칸을 거리로 읽지 않는다."""
    data = list(data)
    if len(data) < 3 or int(data[0]) != VER or int(data[1]) != STRIDE:
        return None
    n = int(data[2])
    if n < 0 or len(data) != 3 + STRIDE * n:
        return None
    return [dict(zip(FIELDS, data[3 + STRIDE * i: 3 + STRIDE * (i + 1)])) for i in range(n)]


# ------------------------------------------------------------ 파라미터 가드·상태·주기

def _num(v, kind):
    if isinstance(v, bool):
        return None
    try:
        x = int(v) if kind is int else float(v)
    except (TypeError, ValueError):
        return None
    if kind is int and float(v) != x:
        return None
    return x if (kind is int or math.isfinite(x)) else None


_SINGLE = {
    # 키: (타입, 조건, 설명)
    'publish_hz': (float, lambda x: x > 0.0, '> 0'),
    'status_hz': (float, lambda x: x > 0.0, '> 0'),
    'lidar_stale_sec': (float, lambda x: x > 0.0, '> 0'),
    'depth_stale_sec': (float, lambda x: x > 0.0, '> 0'),
    'det_stale_sec': (float, lambda x: x > 0.0, '> 0'),
    'warn_period_sec': (float, lambda x: x > 0.0, '> 0'),
    'lidar_half_fov_deg': (float, lambda x: 0.0 < x <= 180.0, '0 < x ≤ 180'),
    'cluster_gap_deg': (float, lambda x: x > 0.0, '> 0'),
    'cluster_gap_m': (float, lambda x: x > 0.0, '> 0'),
    'cluster_min_points': (int, lambda x: x >= 1, '정수 ≥ 1'),
    'fusion_margin_deg': (float, lambda x: x >= 0.0, '≥ 0'),
    'color_match_deg': (float, lambda x: x >= 0.0, '≥ 0'),
    'agree_abs_m': (float, lambda x: x >= 0.0, '≥ 0'),
    'agree_rel': (float, lambda x: x >= 0.0, '≥ 0'),
    'disagree_penalty': (float, lambda x: 0.0 <= x <= 1.0, '0 ≤ x ≤ 1'),
    'conf_window_frames': (int, lambda x: x >= 2, '정수 ≥ 2'),
    'conf_mad_ref_m': (float, lambda x: x > 0.0, '> 0'),
    'conf_jump_m': (float, lambda x: x > 0.0, '> 0'),
    'depth_min_valid_frac': (float, lambda x: 0.0 < x <= 1.0, '0 < x ≤ 1'),
    'cam_x_from_lidar_m': (float, lambda x: True, '유한한 수'),
    'cam_y_from_lidar_m': (float, lambda x: True, '유한한 수'),
    'cam_yaw_offset_deg': (float, lambda x: -180.0 <= x <= 180.0, '−180 ~ 180'),
}
_PAIRS = (
    ('weight_min', 'weight_max', lambda a, b: 0.0 <= a <= b <= 1.0, '0 ≤ weight_min ≤ weight_max ≤ 1'),
    ('depth_min_m', 'depth_max_m', lambda a, b: 0.0 <= a < b, '0 ≤ depth_min_m < depth_max_m'),
    ('depth_row_lo_frac', 'depth_row_hi_frac', lambda a, b: 0.0 <= a < b <= 1.0,
     '0 ≤ depth_row_lo_frac < depth_row_hi_frac ≤ 1'),
)


def check_params(raw):
    """선언값 dict → (쓸 값 dict, 이슈 문자열 목록). 무효 → 그 키(짝이면 둘 다) 기본값. 노드가 ERROR 로 낸다."""
    out = dict(DEFAULTS)
    issues = []
    for key in ('depth_topic', 'depth_info_topic'):
        v = raw.get(key, DEFAULTS[key])
        if isinstance(v, str) and v.startswith('/') and len(v) > 1:
            out[key] = v
        else:
            issues.append(f'{key}={v!r} 무효(/ 로 시작하는 토픽) → 기본값 {DEFAULTS[key]!r}')
    for key, (kind, ok, desc) in _SINGLE.items():
        v = raw.get(key, DEFAULTS[key])
        x = _num(v, kind)
        if x is None or not ok(x):
            issues.append(f'{key}={v!r} 무효({desc}) → 기본값 {DEFAULTS[key]!r}')
        else:
            out[key] = x
    for ka, kb, ok, desc in _PAIRS:
        a, b = _num(raw.get(ka, DEFAULTS[ka]), float), _num(raw.get(kb, DEFAULTS[kb]), float)
        if a is None or b is None or not ok(a, b):
            issues.append(f'{ka}={raw.get(ka)!r}, {kb}={raw.get(kb)!r} 무효({desc}) → 둘 다 기본값 '
                          f'{DEFAULTS[ka]!r}, {DEFAULTS[kb]!r}')
        else:
            out[ka], out[kb] = a, b
    for key in ('publish_hz', 'status_hz'):
        out[key] = float(out[key])
    return out, issues


def _fmt(v):
    return '-' if v is None else f'{v:.2f}'


def format_status(stats, wp_mode):
    """/fusion_status 한 줄. 예 'L0.82 D0.61 agree0.93 lidar_ok depth_ok n=3 m=2'."""
    s = (f"L{_fmt(stats['conf_l'])} D{_fmt(stats['conf_d'])} agree{_fmt(stats['agree'])} "
         f"{'lidar_ok' if stats['lidar_ok'] else 'lidar_stale'} "
         f"{'depth_ok' if stats['depth_ok'] else 'depth_stale'} "
         f"n={'-' if stats['n'] is None else stats['n']} m={wp_mode}")
    if stats['n'] is None:
        s += ' (발행 중단)'
    return s


def due(last_t, now, period):
    """주기 로그: 처음이거나 마지막 뒤 period 가 지났으면 True."""
    return last_t is None or (now - last_t) >= period

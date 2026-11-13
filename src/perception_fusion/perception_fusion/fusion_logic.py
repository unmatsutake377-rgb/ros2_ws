"""perception_fusion 순수 로직 — ROS 없이 테스트한다(test_fusion_logic.py). 시각은 전부 `now` 인자.

계약서_인식융합_20261112 §4(형식)·§5(융합)·§10(결정), 근거 계약서_거리융합_20261112 §4(D455)·§5(융합 규칙).

좌표: 상대방위 rel(도, 0 = 정면, 왼쪽 +), 원점 = LiDAR. LiDAR 각 a 는 rel = a − 80 (a = 80 + rel, 부호 안 뒤집음).
출력 /fused_objects = Float32MultiArray
  [VER=1, STRIDE=8, N, N × (color_id, bearing_deg, range_m, conf, src_id, age_s, track_id, flags)]
  src_id 0 lidar · 1 depth · 2 fused(LiDAR 가 기여한 합의 = LiDAR + D455) · 3 row(1단계 안 냄) ·
         4 mono(OAK 단안) · 5 tri(삼각측량)
    🚨 src 2 는 LiDAR 가 거리에 기여했을 때만(P1b-1). LiDAR 없는 합의는 D455 가 끼면 src 1(미션 노드가
       '채우기 전용'으로 다룬다 — 노드 LiDAR 가 보면 LiDAR), 카메라끼리면 src 4|5(미션 노드 기본 불허 G-7).
       미션 노드가 자기 LiDAR 대신 카메라 합의를 쓰는 길(G-5 우회)을 막는다.
  flags bit0 어긋남(disagree_policy 로 처리) · bit1 LiDAR stale ·
        bit2 lidar · bit3 depth · bit4 mono · bit5 tri = 거리에 기여한 소스(계약서_흰부표_거리융합_v3 P1).
        🚨 bit2~5 는 카메라 경로가 살아 있을 때(use_mono/use_triangulation 켜짐 + /oak_detections 신선)만 채운다.
           꺼졌거나 /oak_detections 가 없으면 0 — 출력이 P1 이전과 비트 동일(test_fusion_cam 골든).
  color_id 0 모름 · 1 red · 2 green · 3 white · 4 yellow · 5 black · 6 orange · 7 blue
  (🚨 /avoid_detections cls 열거형과 다르다. 사본이 미션 노드 셋에 있다 — 상수 공유 모듈화 금지 §1-8,
   VER·STRIDE 머리 검사로 어긋남을 잡는다.)
거리 없는 물체는 안 넣는다. 빈 목록 = 살아 있고 물체 없음. LiDAR·뎁스 둘 다 stale = 발행 안 함(None).
  (단안·삼각측량은 LiDAR 또는 뎁스가 살아 있을 때만 얹힌다 — 카메라만으로는 발행을 다시 열지 않는다.)

입력 /oak_detections (D1 image_subscriber_objects, 아직 없음) = Float32MultiArray
  [VER=1, STRIDE=6, N, N × (color_id, bearing_left_deg, v_norm_bottom, ang_width_deg, conf, age_s)]
  color_id = 위 /fused_objects 열거형. bearing_left_deg = 물체 가운데 열의 방위(camera_info, 좌 +),
  v_norm_bottom = (v_bottom − cy)/fy(아래 +). 🚨 둘 다 **왜곡 보정 뒤 정규 좌표**로 정의한다(D1 이 camera_info D 로
  cv2.undistortPoints 를 한 뒤 x = (u − cx)/fx 로 낸다) — 이 노드는 픽셀을 받지 않아 D 를 다시 쓸 수 없다.
  그래서 x = −tan(bearing_left), y = v_norm_bottom 를 바로 쓴다. age_s = 촬영 → 발행 지연(≥ 0).

신뢰도(소스별 최근 N 프레임, 질의 방위 ± fusion_margin 1° 칸 최솟값 시계열):
  conf = 유효율 × (1 − 0.5·min(MAD / conf_mad_ref_m, 1) − 0.5·점프율)
  MAD = 연속 유효값 차이 |Δr| 의 중앙값, 점프 = |Δr| ≥ conf_jump_m, 유효값 3개 미만이면 안정도 0.5.
  곱셈 식이다 — 덧셈 식은 반짝임 1프레임(유효율 0.1)을 0.5 넘게 통과시켜 기각됐다(fusion_sim).
"""

from collections import deque, namedtuple
from itertools import combinations
import math

import numpy as np

# ---- 형식 상수 (미션 노드 셋에 사본 — 바꾸면 VER 을 올리고 넷을 같이) ----
VER = 1
STRIDE = 8
SRC_LIDAR = 0
SRC_DEPTH = 1
SRC_FUSED = 2
SRC_ROW = 3
SRC_MONO = 4                  # 미션 노드 사본(FUSED_SRC_IDS)은 4·5 를 모른다 → allowed 밖 → LiDAR 로 (닫힌 쪽)
SRC_TRI = 5
FLAG_DISAGREE = 1
FLAG_LIDAR_STALE = 2
FLAG_SRC_LIDAR = 4            # bit2~5: 거리에 기여한 소스 (카메라 경로가 살아 있을 때만)
FLAG_SRC_DEPTH = 8
FLAG_SRC_MONO = 16
FLAG_SRC_TRI = 32
COLOR_IDS = {'red': 1, 'green': 2, 'white': 3, 'yellow': 4, 'black': 5, 'orange': 6, 'blue': 7}

# ---- /oak_detections 형식 (발행자 D1 — 사본은 D1 쪽에 둔다, 상수 공유 모듈화 금지 §1-8) ----
OAK_VER = 1
OAK_STRIDE = 6
OAK_FIELDS = ('color_id', 'bearing_left_deg', 'v_norm_bottom', 'ang_width_deg', 'conf', 'age_s')

# sensor_msgs/NavSatStatus·NavSatFix 상수 (ROS 를 import 하지 않으려고 재정의 — gps_guard.py 와 같은 값)
NAV_STATUS_NO_FIX = -1
NAV_STATUS_GBAS_FIX = 2       # vendored ublox: CARRIER_PHASE_FIXED(carrSoln 2) 일 때만 (ublox_firmware7plus.hpp:95-96)
NAV_COV_TYPE_UNKNOWN = 0
EARTH_R_M = 6371000.0         # 평면 근사 지구 반지름 (ship_avoid EARTH_R 와 같은 값)
MONO_HORIZON_DEG = 0.5        # 수면 교점: 내려다보는 각이 이보다 작으면 None (계약 단안 규격 3)

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
    'disagree_policy': 'lidar',
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
    # ---- P1 단안·삼각측량 (계약서_흰부표_거리융합_v3 B-2 + 수식 정밀화). 장착값 전부 ⚠️ 가정값 ----
    #   배별 값 통로(G-4 perception_fusion_A/B.yaml)는 이번 범위 밖 → 공용 값 하나.
    'use_mono': False,
    'use_triangulation': False,
    'oak_cam_h_m': 0.6,
    'oak_cam_h_sigma_m': 0.05,
    'oak_pitch_deg': 0.0,
    'oak_roll_deg': 0.0,
    'oak_x_from_lidar_m': 0.0,
    'oak_y_from_lidar_m': 0.0,
    'oak_yaw_offset_deg': 0.0,
    'oak_fy_px': 513.0,
    'mono_pitch_sigma_deg': 2.0,
    'mono_pixel_sigma': 2.0,
    'mono_max_range_m': 8.0,
    'imu_stale_sec': 0.5,
    'imu_pitch_sign': 1.0,
    'imu_roll_sign': 1.0,
    'gps_to_cam_x_m': 0.0,
    'gps_to_cam_y_m': 0.0,
    'tri_min_baseline_m': 0.8,
    'tri_min_parallax_deg': 5.0,
    'tri_max_dt_s': 3.0,
    'tri_assoc_deg': 5.0,
    'tri_pose_max_gap_s': 0.3,
    'gps_period_s': 1.0,
    'tri_interp_sigma_frac': 0.1,
    'tri_pending_max': 45,
    'tri_parallax_sigma_k': 2.0,
    'tri_max_rays': 8,
    'cam_max_rel_sigma': 0.3,
    'tri_yaw_sigma_deg': 0.5,
    'tri_rtk_max_sigma_m': 0.05,
    'tri_max_range_m': 15.0,
    'lidar_sigma_m': 0.05,
    'depth_sigma_k': 0.005,
    'fusion_gate_sigma': 3.0,
}

# disagree_policy: LiDAR·뎁스가 어긋날 때 무엇을 내나 (계약서_흰부표_거리융합_v3_20261114 G-5·P1a)
#   lidar = LiDAR 값·src 0 (카메라 값 버림, 기본) / min = 둘 중 가까운 값(11-12 원래 규칙)
DISAGREE_POLICIES = ('lidar', 'min')

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
    어긋남 → 평균 금지(물체 없는 거리를 만든다). conf = min(c_l, c_d) − disagree_penalty, flags bit0 —
      정책과 무관(어긋남 = 둘 중 하나는 틀렸다는 증거, 어느 쪽인지는 모른다).
      disagree_policy lidar(기본) → LiDAR 값, src lidar(카메라 값 버림, G-5).
      disagree_policy min → 가까운 값, src = min 을 낸 소스(11-12 규칙).
      모르는 정책 값은 lidar 로 본다(check_params 가 미리 ERROR 로 걸러 둔다).
    """
    tol = max(p['agree_abs_m'], p['agree_rel'] * min(d_l, d_d))
    if abs(d_l - d_d) <= tol:
        s = c_l + c_d
        w = 0.5 if s <= 0.0 else c_l / s
        w = min(max(w, p['weight_min']), p['weight_max'])
        return w * d_l + (1.0 - w) * d_d, SRC_FUSED, max(c_l, c_d), 0
    conf = max(0.0, min(c_l, c_d) - p['disagree_penalty'])
    if p.get('disagree_policy', 'lidar') != 'min' or d_l <= d_d:
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
            if o['color_id']:
                continue                # 카메라 소스(/oak_detections)가 이미 붙인 색 — 카메라 경로 없으면 늘 0
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


_SRC_BITS = {SRC_LIDAR: FLAG_SRC_LIDAR, SRC_DEPTH: FLAG_SRC_DEPTH, SRC_FUSED: FLAG_SRC_LIDAR | FLAG_SRC_DEPTH}


def build_objects(now, lidar, depth, detections, p, cams=None, cam_live=False):
    """→ (물체 목록 또는 None, 상태). None = LiDAR·뎁스 둘 다 stale → 발행하지 않는다.

    LiDAR stale 이면 뎁스만으로 계속하고 모든 물체에 flags bit1. 뎁스 stale 이면 LiDAR 만.
    물체 dict: color_id, bearing_deg, range_m, conf, src_id, age_s, track_id(-1), flags.
    cams     : 카메라 측정(mono_measurements·TriTracker.estimates 의 dict 목록) 또는 None. None = P1 이전과 같은 경로.
    cam_live : True 면 flags bit2~5(기여 소스)를 채운다. False 면 0 — 카메라 없으면 출력 비트 동일.
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
        c_d = None
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
                     'track_id': -1, 'flags': flags, 'lo': lo, 'hi': hi,
                     '_lr': None if lb is None else float(lb.range_m), '_bits': _SRC_BITS[src]})
    cam_stats = None
    if cams is not None:
        cam_stats = merge_cameras(objs, cams, now, p, lok)
    if cam_live:
        for o in objs:
            o['flags'] |= o['_bits']
    assign_colors(objs, detections, p['color_match_deg'])
    objs.sort(key=lambda o: (o['range_m'], o['bearing_deg']))
    for o in objs:
        del o['lo'], o['hi'], o['_lr'], o['_bits']
    stats.update(n=len(objs),
                 conf_l=float(np.mean(c_ls)) if c_ls else None,
                 conf_d=float(np.mean(c_ds)) if c_ds else None,
                 agree=(n_agree / n_pair) if n_pair else None)
    if cam_stats is not None:
        stats.update(cam_stats)
    return objs, stats


# ------------------------------------------------------------ 카메라 소스 융합 (P1)
#
# 카메라 측정 m = dict(src 4|5, bit, range_m, sigma, bearing_deg(LiDAR 원점), lo, hi, conf, color_id, t).
#   1) LiDAR 가 있는 물체: LiDAR 우선(G-5). 카메라 값은 연관·색·검증용 — 거리는 그대로 두고
#      |c − dL| ≤ fusion_gate_sigma·√(σc² + σL²) 이면 기여 비트만, 넘으면 bit0 + 감점 + disagree_policy
#      (lidar = LiDAR 값 유지, min = 카메라가 더 가까울 때만 그 값·src 4|5).
#   2) LiDAR 없는 물체(뎁스 단독 + 카메라, 카메라끼리): 쌍별 게이팅 → 가장 큰 합의 집합을 역분산 가중.
#      src(P1b-1): 합의에 D455 가 끼면 1(depth), 카메라끼리면 σ 가 더 작은 쪽 번호(4|5, 동점 4) — 가중값을
#      사실상 정한 소스를 말한다. src 2 는 LiDAR 기여 때만(머리말). 노드 사본은 4·5 를 모른다 → LiDAR 로.
#      셋 중 하나만 빠지면 그 둘로 내고 bit0(어긋남 있었음, 감점 없음). 전부 어긋나면 기존 규칙
#      (min + disagree_penalty + bit0, src = min 을 낸 소스). 혼자면 그 소스(src 1|4|5).
#   연관 = 방위 구간 겹침 ± fusion_margin_deg, 한 물체에 mono·tri 각 하나씩, 측정 하나는 한 물체에만.

def _gate_ok(ra, sa, rb, sb, k):
    return abs(ra - rb) <= k * math.sqrt(sa * sa + sb * sb)


def _attach(objs, idx, cams, free, margin):
    """objs[idx] 물체들에 free 카메라 측정을 방위 구간 겹침으로 붙인다 → {물체 i: [측정, …]}. free 에서 뺀다."""
    cands = []
    for i in idx:
        o = objs[i]
        for k in free:
            c = cams[k]
            if o['lo'] - margin <= c['hi'] and c['lo'] <= o['hi'] + margin:
                cands.append((abs(o['bearing_deg'] - c['bearing_deg']), o['range_m'], i, k))
    cands.sort()
    got, used_k, used_ik = {}, set(), set()
    for _, _, i, k in cands:
        src = cams[k]['src']
        if k in used_k or (i, src) in used_ik:
            continue
        used_k.add(k)
        used_ik.add((i, src))
        got.setdefault(i, []).append(cams[k])
    for k in used_k:
        free.discard(k)
    return got


def _color_from(cs):
    for c in sorted(cs, key=lambda c: c['src']):          # mono(4) 먼저 — 지금 프레임의 색
        if c['color_id']:
            return c['color_id']
    return 0


def lidar_priority(o, cs, now, p):
    """LiDAR 가 있는 물체 o 에 카메라 측정 cs → o 를 제자리에서 고친다(머리 주석 1)."""
    d_l = o['_lr']
    k, s_l = p['fusion_gate_sigma'], p['lidar_sigma_m']
    agree = [c for c in cs if _gate_ok(c['range_m'], c['sigma'], d_l, s_l, k)]
    dis = [c for c in cs if not _gate_ok(c['range_m'], c['sigma'], d_l, s_l, k)]
    for c in agree:
        o['_bits'] |= c['bit']
    if dis:
        o['flags'] |= FLAG_DISAGREE
        o['conf'] = float(max(0.0, min(o['conf'], min(c['conf'] for c in dis)) - p['disagree_penalty']))
        if p.get('disagree_policy', 'lidar') == 'min':
            c = min(dis, key=lambda c: (c['range_m'], c['src']))
            if c['range_m'] < o['range_m']:
                o['range_m'], o['src_id'], o['_bits'] = float(c['range_m']), c['src'], c['bit']
                o['age_s'] = float(max(now - c['t'], 0.0))


def group_src(ms):
    """LiDAR 없는 합의 집합의 src (P1b-1): D455 가 끼면 1, 아니면 σ 가 작은 카메라 소스 번호(동점 → 작은 번호)."""
    if any(m['src'] == SRC_DEPTH for m in ms):
        return SRC_DEPTH
    return min(ms, key=lambda m: (m['sigma'], m['src']))['src']


def fuse_group(members, p):
    """LiDAR 없는 물체의 측정들 → (거리, src, conf, flags, 기여 비트, 가장 이른 시각) (머리 주석 2).

    members = dict(range_m, sigma, conf, src, bit, t). 순서 = 동점 처리 순서.
    """
    n = len(members)
    if n == 1:
        m = members[0]
        return m['range_m'], m['src'], m['conf'], 0, m['bit'], m['t']
    k = p['fusion_gate_sigma']
    best = None
    for size in range(n, 1, -1):
        for sub in combinations(range(n), size):
            ms = [members[i] for i in sub]
            if not all(_gate_ok(a['range_m'], a['sigma'], b['range_m'], b['sigma'], k)
                       for a, b in combinations(ms, 2)):
                continue
            w = [1.0 / max(m['sigma'], 1e-6) ** 2 for m in ms]
            r = sum(wi * m['range_m'] for wi, m in zip(w, ms)) / sum(w)
            chi2 = sum(wi * (m['range_m'] - r) ** 2 for wi, m in zip(w, ms))
            if best is None or chi2 < best[0]:
                best = (chi2, r, ms)
        if best is not None:
            break
    if best is not None:
        _, r, ms = best
        bits = 0
        for m in ms:
            bits |= m['bit']
        flags = 0 if len(ms) == n else FLAG_DISAGREE
        return r, group_src(ms), max(m['conf'] for m in ms), flags, bits, min(m['t'] for m in ms)
    m = min(members, key=lambda m: m['range_m'])
    conf = max(0.0, min(x['conf'] for x in members) - p['disagree_penalty'])
    return m['range_m'], m['src'], conf, FLAG_DISAGREE, m['bit'], m['t']


def merge_cameras(objs, cams, now, p, lidar_ok):
    """카메라 측정을 물체 목록에 얹는다(제자리). → 상태 dict(mono_n, mono_used, tri_n, tri_used)."""
    margin = p['fusion_margin_deg']
    free = set(range(len(cams)))
    lid = [i for i, o in enumerate(objs) if o['_lr'] is not None]
    dep = [i for i, o in enumerate(objs) if o['_lr'] is None]
    for i, cs in _attach(objs, lid, cams, free, margin).items():
        objs[i]['color_id'] = _color_from(cs)
        lidar_priority(objs[i], cs, now, p)
    for i, cs in _attach(objs, dep, cams, free, margin).items():
        o = objs[i]
        d_mem = {'range_m': o['range_m'], 'sigma': max(p['depth_sigma_k'] * o['range_m'] ** 2, 1e-6),
                 'conf': o['conf'], 'src': SRC_DEPTH, 'bit': FLAG_SRC_DEPTH, 't': now - o['age_s']}
        r, src, conf, flags, bits, t0 = fuse_group([d_mem] + cs, p)
        o.update(range_m=float(r), src_id=src, conf=float(conf), age_s=float(max(now - t0, 0.0)),
                 color_id=_color_from(cs), _bits=bits)
        o['flags'] |= flags
        o['lo'] = min([o['lo']] + [c['lo'] for c in cs])
        o['hi'] = max([o['hi']] + [c['hi'] for c in cs])
    # 남은 카메라 측정끼리: mono ↔ tri 방위 겹침으로 짝, 나머지는 혼자
    monos = sorted((k for k in free if cams[k]['src'] == SRC_MONO), key=lambda k: cams[k]['range_m'])
    tris = {k for k in free if cams[k]['src'] == SRC_TRI}
    groups = []
    for k in monos:
        m = cams[k]
        best = None
        for j in tris:
            t = cams[j]
            if m['lo'] - margin <= t['hi'] and t['lo'] <= m['hi'] + margin:
                key = (abs(m['bearing_deg'] - t['bearing_deg']), j)
                if best is None or key < best:
                    best = key
        if best is None:
            groups.append([m])
        else:
            tris.discard(best[1])
            groups.append([m, cams[best[1]]])
    groups += [[cams[j]] for j in sorted(tris)]
    for cs in groups:
        r, src, conf, flags, bits, t0 = fuse_group(cs, p)
        if not lidar_ok:
            flags |= FLAG_LIDAR_STALE
        lead = cs[0]
        objs.append({'color_id': _color_from(cs), 'bearing_deg': float(lead['bearing_deg']), 'range_m': float(r),
                     'conf': float(conf), 'src_id': src, 'age_s': float(max(now - t0, 0.0)),
                     'track_id': -1, 'flags': flags, 'lo': min(c['lo'] for c in cs),
                     'hi': max(c['hi'] for c in cs), '_lr': None, '_bits': bits})
    return {'mono_n': sum(1 for c in cams if c['src'] == SRC_MONO),
            'mono_used': sum(1 for o in objs if o['_bits'] & FLAG_SRC_MONO),
            'tri_n': sum(1 for c in cams if c['src'] == SRC_TRI),
            'tri_used': sum(1 for o in objs if o['_bits'] & FLAG_SRC_TRI)}


# ------------------------------------------------------------ /oak_detections 파서

def parse_oak_detections(data):
    """/oak_detections data → (검출 dict 목록, 버린 줄 수, None) 또는 (None, 0, 사유). 순수 함수.

    머리 VER·STRIDE 불일치, N 이 0 이상 정수 아님, 길이 ≠ 3 + 6N, 숫자 아님 → 메시지 통째로 버림(노드 ERROR).
    줄 단위 무효(비유한, color_id 가 열거형 밖 정수, |방위| ≥ 90°, 폭 < 0, conf ∉ [0, 1], age_s < 0) → 그 줄만 버림.
    N = 0 = '살아 있고 물체 없음'.
    """
    try:
        vals = [float(x) for x in data]
    except (TypeError, ValueError):
        return None, 0, '숫자 배열 아님'
    if len(vals) < 3:
        return None, 0, f'길이 {len(vals)} < 머리 3'
    ver, stride, n = vals[0], vals[1], vals[2]
    if ver != OAK_VER:
        return None, 0, f'VER {ver:g} ≠ {OAK_VER}'
    if stride != OAK_STRIDE:
        return None, 0, f'STRIDE {stride:g} ≠ {OAK_STRIDE}'
    if not math.isfinite(n) or n < 0 or n != int(n):
        return None, 0, f'N {n!r} 이 0 이상 정수 아님'
    n = int(n)
    if len(vals) != 3 + n * OAK_STRIDE:
        return None, 0, f'길이 {len(vals)} ≠ 3 + {n}×{OAK_STRIDE}'
    cmax = max(COLOR_IDS.values())
    out, bad = [], 0
    for k in range(n):
        row = vals[3 + k * OAK_STRIDE: 3 + (k + 1) * OAK_STRIDE]
        d = dict(zip(OAK_FIELDS, row))
        cid = d['color_id']
        ok = (all(math.isfinite(v) for v in row) and cid == int(cid) and 0 <= cid <= cmax
              and abs(d['bearing_left_deg']) < 90.0 and d['ang_width_deg'] >= 0.0
              and 0.0 <= d['conf'] <= 1.0 and d['age_s'] >= 0.0)
        if not ok:
            bad += 1
            continue
        d['color_id'] = int(cid)
        out.append(d)
    return out, bad, None


# ------------------------------------------------------------ 단안 (수면 교점) — 계약 '단안 규격'
#
# 카메라 광선 c = (앞 1, 오른쪽 x, 아래 y) (x = −tan(bearing_left), y = v_norm_bottom, 왜곡 보정 뒤 정규 좌표).
# 수평 좌표(앞 f, 오른쪽 s, 아래 w) = Ry(θ)·Rx(ρ)·c. θ = 피치(아래 +), ρ = 롤(우현 아래 +), 둘 다 장착 + IMU.
#   Rx(ρ): (f, r, d) → (f, r cosρ − d sinρ, r sinρ + d cosρ)    (오른쪽 축이 아래로 기운다)
#   Ry(θ): (f, r, d) → (f cosθ − d sinθ, r, f sinθ + d cosθ)    (앞 축이 아래로 기운다)
#   (롤 먼저, 피치 다음 = 선체 ZYX 자세 순서. 장착값과 IMU 값은 더해서 쓴다 — 1차 근사, 작은 각에서 순서 무관)
# 수면 교점: w ≤ sin(0.5°)·|c| 이면 None. t = h / w, 수평 거리 = t·√(f² + s²), 방위 = atan2(−s, f) (좌 +).

def level_ray(x, y, pitch_deg, roll_deg):
    """정규 좌표 (x 오른쪽, y 아래) → 수평 좌표 (f 앞, s 오른쪽, w 아래). 머리 주석의 Ry(θ)·Rx(ρ)."""
    th, rho = math.radians(pitch_deg), math.radians(roll_deg)
    cr, sr = math.cos(rho), math.sin(rho)
    r1, d1 = x * cr - y * sr, x * sr + y * cr
    ct, st = math.cos(th), math.sin(th)
    return ct - d1 * st, r1, st + d1 * ct


def mono_range(x, y, cam_h, pitch_deg, roll_deg):
    """→ (수평 거리 m, 방위 도 좌 +, 카메라 원점·선체 수평 기준) 또는 None(수평선 위·근처)."""
    f, s, w = level_ray(x, y, pitch_deg, roll_deg)
    if w <= math.sin(math.radians(MONO_HORIZON_DEG)) * math.sqrt(1.0 + x * x + y * y):
        return None
    t = cam_h / w
    return t * math.hypot(f, s), math.degrees(math.atan2(-s, f))


def mono_sigma(x, y, cam_h, pitch_deg, roll_deg, p):
    """1차 전파(중심 차분): σd² = (∂d/∂θ σθ)² + (∂d/∂ρ σρ)² + (∂d/∂y σy)² + (d σh/h)². 미분 중 수평선 → inf.

    σθ = σρ = mono_pitch_sigma_deg(피치·롤 공통), σy = mono_pixel_sigma / oak_fy_px (바닥 행 ±px).
    """
    base = mono_range(x, y, cam_h, pitch_deg, roll_deg)
    if base is None:
        return math.inf
    d = base[0]
    terms = [d * p['oak_cam_h_sigma_m'] / cam_h]
    s_ang = p['mono_pitch_sigma_deg']
    s_y = p['mono_pixel_sigma'] / p['oak_fy_px']
    for (dp, dr, dy), sig in (((1e-3, 0.0, 0.0), s_ang), ((0.0, 1e-3, 0.0), s_ang), ((0.0, 0.0, 1e-5), s_y)):
        hi = mono_range(x, y + dy, cam_h, pitch_deg + dp, roll_deg + dr)
        lo = mono_range(x, y - dy, cam_h, pitch_deg - dp, roll_deg - dr)
        if hi is None or lo is None:
            return math.inf
        step = dp or dr or dy
        terms.append((hi[0] - lo[0]) / (2.0 * step) * sig)
    return math.sqrt(sum(t * t for t in terms))


def quat_pitch_roll(qx, qy, qz, qw):
    """sensor_msgs/Imu 자세(REP-103 FLU) → (피치 도 아래 +, 롤 도 우현 아래 +). FLU 에서 +y 축 회전 = 선수 숙임,
    +x 축 회전 = 왼쪽 들림 = 우현 내려감이라 부호가 그대로 맞는다(⚠️ iahrs 실발행 확인 전 — imu_*_sign)."""
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    roll = math.degrees(math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))
    return pitch, roll


def camera_attitude(att_buf, t, p):
    """→ (피치, 롤, imu 썼나) — **촬영 시각 t** 의 값(P1b-5). att_buf = TimeSeries((IMU 피치, IMU 롤)) 또는 None.

    t 앞뒤 imu_stale_sec 안 표본으로 보간(없으면 가까운 표본), 그것도 없으면 장착값만(노드가 WARN 1회).
    """
    br = None if att_buf is None else att_buf.bracket(t, p['imu_stale_sec'])
    if br is None:
        return p['oak_pitch_deg'], p['oak_roll_deg'], False
    a, b, w = br
    return (p['oak_pitch_deg'] + a[0] + w * (b[0] - a[0]), p['oak_roll_deg'] + a[1] + w * (b[1] - a[1]), True)


def _cam_xy(det):
    return -math.tan(math.radians(det['bearing_left_deg'])), det['v_norm_bottom']


def mono_measurements(dets, recv_t, now, att_buf, p):
    """/oak_detections 검출 → 카메라 측정 목록(src 4). 자세는 검출마다 촬영 시각(recv − age_s) 값.

    뺀다(모르면 입을 다문다): 묵은 검출(now − 촬영 > det_stale_sec)·수평선·σ 무한·mono_max_range_m 초과·
    σ/d > cam_max_rel_sigma(정보 없는 측정 — 3σ 게이트가 늘 '합의'가 되는 것을 막는다, P1b-6).
    방위·거리는 OAK 장착값으로 LiDAR 원점으로 옮긴다.
    """
    out = []
    h = p['oak_cam_h_m']
    for d in dets:
        t_cap = recv_t - d['age_s']
        if now - t_cap > p['det_stale_sec']:
            continue
        pitch_deg, roll_deg, _ = camera_attitude(att_buf, t_cap, p)
        x, y = _cam_xy(d)
        res = mono_range(x, y, h, pitch_deg, roll_deg)
        if res is None or res[0] > p['mono_max_range_m']:
            continue
        sig = mono_sigma(x, y, h, pitch_deg, roll_deg, p)
        if not math.isfinite(sig) or sig > p['cam_max_rel_sigma'] * res[0]:
            continue
        rel, r = cam_to_lidar(res[1], res[0], p['oak_x_from_lidar_m'], p['oak_y_from_lidar_m'],
                              p['oak_yaw_offset_deg'])
        rel, r = float(rel), float(r)
        half = 0.5 * d['ang_width_deg']
        out.append({'src': SRC_MONO, 'bit': FLAG_SRC_MONO, 'range_m': r, 'sigma': float(sig),
                    'bearing_deg': rel, 'lo': rel - half, 'hi': rel + half, 'conf': float(d['conf']),
                    'color_id': d['color_id'], 't': t_cap})
    return out


# ------------------------------------------------------------ GPS·자세 버퍼

def ll_to_local(lat, lon, lat0, lon0):
    """위경도 → 로컬 (동 x, 북 y) m. 평면 근사(등장방형): x = Δlon·cos(lat0)·R, y = Δlat·R (라디안, R = 6371 km).
    수 km 안에서 상대 오차 < 1e-3 — 삼각측량은 3 s 안 광선만 쓰므로 원점 위치와 무관."""
    k = math.pi / 180.0 * EARTH_R_M
    return (lon - lon0) * k * math.cos(math.radians(lat0)), (lat - lat0) * k


def fix_quality(status, cov, cov_type, max_sigma_m):
    """NavSatFix → (RTK FIXED 인가, 수평 σ m 또는 None).

    FIXED = status == STATUS_GBAS_FIX(2) **그리고** 공분산이 알려져 있고(type ≠ UNKNOWN) √max(cov[0], cov[4]) ≤
    tri_rtk_max_sigma_m. 근거: vendored ublox(ublox_firmware7plus.hpp:92-111) 가 GBAS_FIX 를 NavPVT
    CARRIER_PHASE_FIXED(carrSoln = 2, float 아님) 일 때만 쓰고 cov[0] = cov[4] = (hAcc/1000)² 를 넣는다.
    σ 문턱 0.05 m 는 mission_view RTK_FIXED_MAX_SIGMA(08-07 야외 실측 FIXED ±0.015 m · float ±0.10 m)와 같다.
    ⚠️ 다른 드라이버(GBAS_FIX 를 DGPS·float 에도 쓰는 구현)라면 σ 문턱이 유일한 방어선이다.
    """
    try:
        c0, c4 = float(cov[0]), float(cov[4])
    except (TypeError, ValueError, IndexError):
        return False, None
    if int(cov_type) == NAV_COV_TYPE_UNKNOWN or not (math.isfinite(c0) and math.isfinite(c4)) or c0 < 0 or c4 < 0:
        return False, None
    sigma = math.sqrt(max(c0, c4))
    return int(status) == NAV_STATUS_GBAS_FIX and sigma <= max_sigma_m, sigma


class TimeSeries:
    """(단조 시각, 값) 버퍼 — horizon_s 보다 오래된 것은 버린다. 값은 수 또는 수 튜플."""

    def __init__(self, horizon_s):
        self.horizon = float(horizon_s)
        self.buf = deque()

    def push(self, t, v):
        self.buf.append((float(t), v))
        while self.buf and self.buf[-1][0] - self.buf[0][0] > self.horizon:
            self.buf.popleft()

    def latest_t(self):
        return self.buf[-1][0] if self.buf else None

    def neighbors(self, t):
        """→ (t 이하 마지막 표본, t 초과 첫 표본) — 없으면 None."""
        prev = nxt = None
        for s in self.buf:
            if s[0] <= t:
                prev = s
            else:
                nxt = s
                break
        return prev, nxt

    def bracket(self, t, max_gap):
        """→ (a, b, w) 값 = a + w·(b − a) 또는 None. 앞뒤 표본이 있고 둘 다 t 에서 max_gap 안이면 선형 보간,
        아니면 가장 가까운 표본이 max_gap 안일 때만 그 값(w = 0). 그 밖 = 모름. (yaw·IMU 처럼 빠른 표본용)"""
        if not self.buf:
            return None
        prev, nxt = self.neighbors(t)
        if prev is not None and nxt is not None and t - prev[0] <= max_gap and nxt[0] - t <= max_gap:
            return prev[1], nxt[1], (t - prev[0]) / (nxt[0] - prev[0])
        near = min((s for s in (prev, nxt) if s is not None), key=lambda s: abs(s[0] - t))
        if abs(near[0] - t) <= max_gap:
            return near[1], near[1], 0.0
        return None


def yaw_at(yaw_buf, t, max_gap):
    """나침반 yaw(도, 시계 +) 보간 — 359°↔1° 를 짧은 쪽으로. 모르면 None."""
    br = yaw_buf.bracket(t, max_gap)
    if br is None:
        return None
    a, b, w = br
    return float((a + w * float(wrap180(b - a))) % 360.0)


# GPS 는 1 Hz(zed_f9p_rover.yaml rate 1.0) — 표본 간격 판정·대기 만료를 gps_period_s 의 배수로 둔다.
GPS_GAP_FACTOR = 1.5     # 보간에 쓸 앞뒤 표본 간격 상한 = 1.5·주기(한 표본 빠짐 = 2·주기 → 안 씀)
GPS_WAIT_FACTOR = 2.0    # 촬영 뒤 GPS 표본을 기다리는 최대 시간 = 2·주기(넘으면 'gps' 로 버림)


def gps_at(gps_buf, t, p):
    """촬영 시각 t 의 GPS (x, y, σ, fixed) — 앞뒤 표본 사이 **보간만**(외삽 안 함, P1b-2). 모르면 None.

    표본과 같은 시각이면 그 표본. 앞뒤 간격 ≤ GPS_GAP_FACTOR·gps_period_s.
    σ = √(σ_gps² + (tri_interp_sigma_frac·|v|·δ)²): |v| = 두 표본 사이 속도, δ = 가까운 표본까지 시간.
      등속 직선 보간이 틀리는 만큼(선회·가감속) — 예: 1.5 m/s·20°/s 선회에서 1 s 간격 현 중앙 편차 약 0.07 m.
    fixed = 쓴 표본이 모두 FIXED.
    """
    prev, nxt = gps_buf.neighbors(t)
    if prev is not None and abs(prev[0] - t) < 1e-9:
        x, y, sig, fx = prev[1]
        return x, y, sig, bool(fx)
    if prev is None or nxt is None or nxt[0] - prev[0] > GPS_GAP_FACTOR * p['gps_period_s']:
        return None
    a, b = prev[1], nxt[1]
    dt = nxt[0] - prev[0]
    w = (t - prev[0]) / dt
    v = math.hypot(b[0] - a[0], b[1] - a[1]) / dt
    delta = min(t - prev[0], nxt[0] - t)
    sig = math.sqrt(max(a[2], b[2]) ** 2 + (p['tri_interp_sigma_frac'] * v * delta) ** 2)
    return a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]), sig, bool(a[3] and b[3])


def gps_now(gps_buf, now, p):
    """지금 GPS (x, y, σ, fixed) — 마지막 두 표본 속도로 외삽(마지막 표본이 GPS_GAP_FACTOR·주기 안일 때만).
    σ 에 tri_interp_sigma_frac·|v|·(now − 마지막 표본) 을 더한다. 모르면 None."""
    if not gps_buf.buf:
        return None
    t1, b = gps_buf.buf[-1]
    dt = now - t1
    period = p['gps_period_s']
    if dt < 0.0 or dt > GPS_GAP_FACTOR * period:
        return None
    vx = vy = 0.0
    if len(gps_buf.buf) >= 2:
        t0, a = gps_buf.buf[-2]
        if 0.0 < t1 - t0 <= GPS_GAP_FACTOR * period:
            vx, vy = (b[0] - a[0]) / (t1 - t0), (b[1] - a[1]) / (t1 - t0)
    sig = math.sqrt(b[2] ** 2 + (p['tri_interp_sigma_frac'] * math.hypot(vx, vy) * dt) ** 2)
    return b[0] + vx * dt, b[1] + vy * dt, sig, bool(b[3])


def _lever(ax, ay, yaw_deg, fwd_m, left_m):
    """안테나 (ax, ay) + 선체 앞 fwd_m·왼쪽 left_m (yaw 로 회전) → 그 점의 로컬 좌표."""
    ps = math.radians(yaw_deg)
    return (ax + math.sin(ps) * fwd_m - math.cos(ps) * left_m,
            ay + math.cos(ps) * fwd_m + math.sin(ps) * left_m)


def current_pose(gps_buf, yaw_buf, now, p):
    """삼각측량 결과를 '지금' 카메라 기준으로 다시 잴 자세 → (cam_x, cam_y, σ, yaw) 또는 None.
    GPS 외삽(gps_now)이 FIXED 이고 yaw 가 tri_pose_max_gap_s 안일 때만."""
    g = gps_now(gps_buf, now, p)
    yaw = yaw_at(yaw_buf, now, p['tri_pose_max_gap_s'])
    if g is None or not g[3] or yaw is None:
        return None
    cx, cy = _lever(g[0], g[1], yaw, p['gps_to_cam_x_m'], p['gps_to_cam_y_m'])
    return cx, cy, g[2], yaw


# ------------------------------------------------------------ 삼각측량 — 계약 '삼각측량 규격'
#
# 좌표: 로컬 (동 x, 북 y) m, 나침반 방위 α(도, 북 0·시계 +) → 단위 벡터 u = (sin α, cos α).
# 선체 방위 b(좌 +) → α = yaw − b. 선체 앞 = (sin ψ, cos ψ), 왼쪽 = (−cos ψ, sin ψ).
# 카메라 위치 = GPS 안테나 + 앞·gps_to_cam_x + 왼쪽·gps_to_cam_y (레버암을 yaw 로 회전, ⚠️ 실측).
# 기선 = **안테나** 위치 차(제자리 회전에서 레버암이 만드는 가짜 기선을 막는다 — '제자리 회전 미발행').
# 시각(P1b-2·5): 광선은 촬영 시각 t_cap 의 yaw·피치·롤·GPS 로 만든다. GPS 는 t_cap 뒤 표본이 올 때까지
#   RayQueue 에서 기다렸다가(노드 publish_cb) 보간한다 — 1 Hz 에서 '이전 표본 유지'로 위치가 최대 1 s 묵던 문제.
# 출력: 교점 X(세계)를 **지금** 카메라 위치·yaw(current_pose)로 다시 재서 거리·방위를 낸다(광선이 GPS 를
#   기다리느라 최대 약 1.5 s 묵으므로 그 사이 회전·이동을 반영). σ_r² = gᵀ Σ_X g + σ_C², g = (X − C)/|X − C|.

Ray = namedtuple('Ray', 't ant_x ant_y cam_x cam_y alpha_deg yaw_deg pos_sigma color_id conf half_width')


def make_ray(det, t_cap, gps_buf, yaw_buf, att_buf, p):
    """검출 하나 + 촬영 시각 → (Ray, None) 또는 (None, 사유 'yaw'|'gps'|'rtk'). 자세·yaw·GPS 모두 t_cap 값."""
    yaw = yaw_at(yaw_buf, t_cap, p['tri_pose_max_gap_s'])
    if yaw is None:
        return None, 'yaw'
    g = gps_at(gps_buf, t_cap, p)
    if g is None:
        return None, 'gps'
    if not g[3]:
        return None, 'rtk'
    pitch_deg, roll_deg, _ = camera_attitude(att_buf, t_cap, p)
    x, y = _cam_xy(det)
    f, s, _ = level_ray(x, y, pitch_deg, roll_deg)
    hull_b = math.degrees(math.atan2(-s, f)) + p['oak_yaw_offset_deg']
    cx, cy = _lever(g[0], g[1], yaw, p['gps_to_cam_x_m'], p['gps_to_cam_y_m'])
    return Ray(float(t_cap), g[0], g[1], cx, cy, float((yaw - hull_b) % 360.0), yaw, g[2], det['color_id'],
               float(det['conf']), 0.5 * det['ang_width_deg']), None


class RayQueue:
    """촬영 시각 뒤 GPS 표본을 기다리는 검출 대기열(P1b-2). 메시지 단위 FIFO, 상한 tri_pending_max(넘으면 오래된
    것부터 'queue' 로 버림). drops = 버림 사유 카운터(노드가 주기 WARN 뒤 비운다)."""

    REASONS = ('yaw', 'gps', 'rtk', 'queue')

    def __init__(self):
        self.q = deque()
        self.drops = dict.fromkeys(self.REASONS, 0)

    def push(self, recv_t, dets, p):
        if dets:
            self.q.append((float(recv_t), list(dets)))
        while len(self.q) > p['tri_pending_max']:
            _, old = self.q.popleft()
            self.drops['queue'] += len(old)

    def pop_ready(self, now, gps_buf, yaw_buf, att_buf, p):
        """→ 프레임별 광선 목록들(시각순). 준비 = 마지막 GPS 표본 시각 ≥ 그 메시지의 가장 늦은 촬영 시각,
        만료 = 가장 이른 촬영 시각이 GPS_WAIT_FACTOR·gps_period_s 넘게 지남(그래도 만들어 보고 안 되면 버림)."""
        latest = gps_buf.latest_t()
        wait = GPS_WAIT_FACTOR * p['gps_period_s']
        frames = []
        while self.q:
            recv_t, dets = self.q[0]
            tcs = [recv_t - d['age_s'] for d in dets]
            ready = latest is not None and latest >= max(tcs)
            if not ready and now - min(tcs) <= wait:
                break
            self.q.popleft()
            rays = []
            for d, tc in zip(dets, tcs):
                ray, why = make_ray(d, tc, gps_buf, yaw_buf, att_buf, p)
                if ray is None:
                    self.drops[why] += 1
                else:
                    rays.append(ray)
            frames.append(rays)
        return frames

    def take_drops(self):
        out = dict(self.drops)
        self.drops = dict.fromkeys(self.REASONS, 0)
        return out


def _unit(alpha_deg):
    a = math.radians(alpha_deg)
    return math.sin(a), math.cos(a)


def tri_solve(pts):
    """광선 [(px, py, α도)] → 교점 (x, y) 또는 None. 2개 = 2×2 선형계 P1 + t1·u1 = P2 + t2·u2,
    3개 이상 = 수직거리 제곱합 최소(Σ(I − uuᵀ)X = Σ(I − uuᵀ)P, 2×2 를 직접 푼다). 평행·뒤쪽(t ≤ 0) → None."""
    if len(pts) < 2:
        return None
    us = [_unit(a) for _, _, a in pts]
    if len(pts) == 2:
        (p1x, p1y, _), (p2x, p2y, _) = pts
        u1, u2 = us
        det = -u1[0] * u2[1] + u2[0] * u1[1]
        if abs(det) < 1e-9:
            return None
        bx, by = p2x - p1x, p2y - p1y
        t1 = (bx * -u2[1] + u2[0] * by) / det
        X = (p1x + t1 * u1[0], p1y + t1 * u1[1])
    else:
        a11 = a12 = a22 = b1 = b2 = 0.0
        for (px, py, _), (ux, uy) in zip(pts, us):
            m11, m12, m22 = 1.0 - ux * ux, -ux * uy, 1.0 - uy * uy
            a11 += m11
            a12 += m12
            a22 += m22
            b1 += m11 * px + m12 * py
            b2 += m12 * px + m22 * py
        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-9:
            return None
        X = ((a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det)
    for (px, py, _), (ux, uy) in zip(pts, us):
        if (X[0] - px) * ux + (X[1] - py) * uy <= 0.0:
            return None
    return X


def tri_alpha_sigma_deg(p):
    """광선 세계 방위 σα(도) = √(atan(픽셀 σ/fy)² + yaw σ²)."""
    return math.sqrt(math.degrees(math.atan(p['mono_pixel_sigma'] / p['oak_fy_px'])) ** 2
                     + p['tri_yaw_sigma_deg'] ** 2)


def tri_fit(rays, p):
    """한 트랙의 광선들 → 교점 dict(x, y, cov(xx, xy, yy), newest Ray, n) 또는 None.

    기준 = 가장 새 광선 N. 짝 = N 과 시간차 ≤ tri_max_dt_s, 안테나 기선 ≥ tri_min_baseline_m,
    **잡음 여유를 뺀 시차** |α_N − α_i| − tri_parallax_sigma_k·√2·σα ≥ tri_min_parallax_deg 인 광선들(P1b-3:
    문턱 근처에서 잡음이 시차를 부풀린 쌍만 골라 거리를 짧게 내던 선택 편향을 막는다).
    짝 없으면 None(정면 접근·정지·제자리 회전). 짝 하나 = 2광선 교점, 둘 이상 = 최소제곱. 앞쪽(t > 0)만.
    Σ_X = 각 광선 α(σα)·위치(GPS σ, 동·북)에 대한 중심 차분 야코비안 J 로 J·diag(σ²)·Jᵀ.
    """
    if len(rays) < 2:
        return None
    rays = sorted(rays, key=lambda r: r.t)
    n_ray = rays[-1]
    s_a = tri_alpha_sigma_deg(p)
    margin = p['tri_parallax_sigma_k'] * math.sqrt(2.0) * s_a
    pair = [r for r in rays[:-1]
            if n_ray.t - r.t <= p['tri_max_dt_s']
            and math.hypot(n_ray.ant_x - r.ant_x, n_ray.ant_y - r.ant_y) >= p['tri_min_baseline_m']
            and abs(float(wrap180(n_ray.alpha_deg - r.alpha_deg))) - margin >= p['tri_min_parallax_deg']]
    if not pair:
        return None
    used = pair + [n_ray]
    pts = [(r.cam_x, r.cam_y, r.alpha_deg) for r in used]
    X = tri_solve(pts)
    if X is None:
        return None
    cxx = cxy = cyy = 0.0
    for i, r in enumerate(used):
        for j, step, sig in ((2, 1e-3, s_a), (0, 1e-4, r.pos_sigma), (1, 1e-4, r.pos_sigma)):
            if sig <= 0.0:
                continue
            hi = [list(q) for q in pts]
            lo = [list(q) for q in pts]
            hi[i][j] += step
            lo[i][j] -= step
            xh, xl = tri_solve([tuple(q) for q in hi]), tri_solve([tuple(q) for q in lo])
            if xh is None or xl is None:
                return None
            jx = (xh[0] - xl[0]) / (2.0 * step) * sig
            jy = (xh[1] - xl[1]) / (2.0 * step) * sig
            cxx += jx * jx
            cxy += jx * jy
            cyy += jy * jy
    return {'x': X[0], 'y': X[1], 'cov': (cxx, cxy, cyy), 'newest': n_ray, 'n': len(used)}


def tri_measure(fit, cam_x, cam_y, cam_sigma, yaw_deg, p):
    """교점 fit 을 카메라 (cam_x, cam_y, σ, yaw) 기준 측정 dict(src 5) 로. 거리 > tri_max_range_m·
    σ/d > cam_max_rel_sigma 면 None. 방위 = yaw − 교점 방위(좌 +) → OAK 장착값으로 LiDAR 원점."""
    dx, dy = fit['x'] - cam_x, fit['y'] - cam_y
    d = math.hypot(dx, dy)
    if d <= 0.0 or d > p['tri_max_range_m']:
        return None
    gx, gy = dx / d, dy / d
    cxx, cxy, cyy = fit['cov']
    sig = math.sqrt(max(gx * gx * cxx + 2.0 * gx * gy * cxy + gy * gy * cyy, 0.0) + cam_sigma ** 2)
    if sig > p['cam_max_rel_sigma'] * d:
        return None
    hull_b = float(wrap180(yaw_deg - math.degrees(math.atan2(dx, dy))))
    rel, rr = cam_to_lidar(hull_b, d, p['oak_x_from_lidar_m'], p['oak_y_from_lidar_m'], 0.0)
    rel, rr = float(rel), float(rr)
    n_ray = fit['newest']
    return {'src': SRC_TRI, 'bit': FLAG_SRC_TRI, 'range_m': rr, 'sigma': sig, 'bearing_deg': rel,
            'lo': rel - n_ray.half_width, 'hi': rel + n_ray.half_width, 'conf': n_ray.conf,
            'color_id': n_ray.color_id, 't': n_ray.t, 'x': fit['x'], 'y': fit['y'], 'n_rays': fit['n']}


def tri_estimate(rays, p):
    """tri_fit → 가장 새 광선의 카메라 기준 측정(그 위치 σ 는 Σ_X 에 이미 있음). 검산·테스트용."""
    fit = tri_fit(rays, p)
    if fit is None:
        return None
    n_ray = fit['newest']
    return tri_measure(fit, n_ray.cam_x, n_ray.cam_y, 0.0, n_ray.yaw_deg, p)


def tri_stale_sec(p):
    """삼각측량 결과를 낼 수 있는 마지막 광선 나이 상한 = det_stale_sec + GPS_GAP_FACTOR·gps_period_s
    (광선이 GPS 표본을 기다리느라 늦게 만들어지는 만큼 늘린다)."""
    return p['det_stale_sec'] + GPS_GAP_FACTOR * p['gps_period_s']


class TriTracker:
    """색 + 세계 방위로 연관한 트랙별 광선 버퍼. 시각은 전부 인자(순수).

    P1b-4: 트랙당 광선 ≤ tri_max_rays(넘으면 가운데 광선 중 이웃 간 시간·기선 간격이 가장 작은 것을 솎는다 —
    가장 오래된 것(시차 최대)과 가장 새 것은 남긴다). 교점은 새 광선이 붙은 트랙만 다시 푼다(dirty).
    """

    def __init__(self):
        self.tracks = []          # dict(color, alpha, t, rays deque, dirty, fit)

    def _prune(self, now, max_dt):
        self.tracks = [tr for tr in self.tracks if now - tr['t'] <= max_dt]

    def add(self, rays, p):
        """광선 목록(한 프레임) → 트랙에 붙임. 같은 색, 마지막 α 와 차 ≤ tri_assoc_deg, 시간차 ≤ tri_max_dt_s 중
        α 차 작은 순으로 트랙 하나에 광선 하나. 못 붙으면 새 트랙."""
        if not rays:
            return
        max_dt = p['tri_max_dt_s']
        self._prune(max(r.t for r in rays), max_dt)
        cands = []
        for k, r in enumerate(rays):
            for i, tr in enumerate(self.tracks):
                da = abs(float(wrap180(r.alpha_deg - tr['alpha'])))
                if tr['color'] == r.color_id and 0.0 <= r.t - tr['t'] <= max_dt and da <= p['tri_assoc_deg']:
                    cands.append((da, i, k))
        cands.sort()
        used_i, used_k = set(), set()
        for _, i, k in cands:
            if i in used_i or k in used_k:
                continue
            used_i.add(i)
            used_k.add(k)
            self._push(self.tracks[i], rays[k], p)
        for k, r in enumerate(rays):
            if k not in used_k:
                tr = {'color': r.color_id, 'alpha': r.alpha_deg, 't': r.t, 'rays': deque(),
                      'dirty': True, 'fit': None}
                self._push(tr, r, p)
                self.tracks.append(tr)

    @staticmethod
    def _push(tr, r, p):
        max_dt = p['tri_max_dt_s']
        rays = tr['rays']
        rays.append(r)
        tr['alpha'], tr['t'], tr['dirty'] = r.alpha_deg, r.t, True
        while rays and r.t - rays[0].t > max_dt:
            rays.popleft()
        if len(rays) > p['tri_max_rays']:
            lst = list(rays)
            b_min = p['tri_min_baseline_m']
            while len(lst) > p['tri_max_rays']:
                i = min(range(1, len(lst) - 1),
                        key=lambda i: ((lst[i + 1].t - lst[i - 1].t) / max_dt
                                       + math.hypot(lst[i + 1].ant_x - lst[i - 1].ant_x,
                                                    lst[i + 1].ant_y - lst[i - 1].ant_y) / b_min, i))
                del lst[i]
            tr['rays'] = deque(lst)

    def estimates(self, now, pose, p):
        """→ 측정 목록(src 5). pose = current_pose(...) — None 이면(GPS 묵음·비FIXED·yaw 없음) 아무것도 안 낸다.
        마지막 광선이 tri_stale_sec 넘게 묵은 트랙도 내지 않는다."""
        self._prune(now, p['tri_max_dt_s'] + tri_stale_sec(p))
        if pose is None:
            return []
        out = []
        for tr in self.tracks:
            if now - tr['t'] > tri_stale_sec(p):
                continue
            if tr['dirty']:
                tr['fit'] = tri_fit(list(tr['rays']), p)
                tr['dirty'] = False
            if tr['fit'] is None:
                continue
            m = tri_measure(tr['fit'], pose[0], pose[1], pose[2], pose[3], p)
            if m is not None:
                out.append(m)
        return out


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
    'oak_cam_h_m': (float, lambda x: x > 0.0, '> 0'),
    'oak_cam_h_sigma_m': (float, lambda x: x >= 0.0, '≥ 0'),
    'oak_pitch_deg': (float, lambda x: -60.0 <= x <= 60.0, '−60 ~ 60'),
    'oak_roll_deg': (float, lambda x: -45.0 <= x <= 45.0, '−45 ~ 45'),
    'oak_x_from_lidar_m': (float, lambda x: True, '유한한 수'),
    'oak_y_from_lidar_m': (float, lambda x: True, '유한한 수'),
    'oak_yaw_offset_deg': (float, lambda x: -180.0 <= x <= 180.0, '−180 ~ 180'),
    'oak_fy_px': (float, lambda x: x > 0.0, '> 0'),
    'mono_pitch_sigma_deg': (float, lambda x: x >= 0.0, '≥ 0'),
    'mono_pixel_sigma': (float, lambda x: x >= 0.0, '≥ 0'),
    'mono_max_range_m': (float, lambda x: x > 0.0, '> 0'),
    'imu_stale_sec': (float, lambda x: x > 0.0, '> 0'),
    'imu_pitch_sign': (float, lambda x: x in (-1.0, 1.0), '1.0 또는 −1.0'),
    'imu_roll_sign': (float, lambda x: x in (-1.0, 1.0), '1.0 또는 −1.0'),
    'gps_to_cam_x_m': (float, lambda x: True, '유한한 수'),
    'gps_to_cam_y_m': (float, lambda x: True, '유한한 수'),
    'tri_min_baseline_m': (float, lambda x: x > 0.0, '> 0'),
    'tri_min_parallax_deg': (float, lambda x: 0.0 < x < 90.0, '0 < x < 90'),
    'tri_max_dt_s': (float, lambda x: x > 0.0, '> 0'),
    'tri_assoc_deg': (float, lambda x: x > 0.0, '> 0'),
    'tri_pose_max_gap_s': (float, lambda x: x > 0.0, '> 0'),
    'gps_period_s': (float, lambda x: x > 0.0, '> 0'),
    'tri_interp_sigma_frac': (float, lambda x: x >= 0.0, '≥ 0'),
    'tri_pending_max': (int, lambda x: x >= 1, '정수 ≥ 1'),
    'tri_parallax_sigma_k': (float, lambda x: x >= 0.0, '≥ 0'),
    'tri_max_rays': (int, lambda x: x >= 3, '정수 ≥ 3'),
    'cam_max_rel_sigma': (float, lambda x: x > 0.0, '> 0'),
    'tri_yaw_sigma_deg': (float, lambda x: x >= 0.0, '≥ 0'),
    'tri_rtk_max_sigma_m': (float, lambda x: x > 0.0, '> 0'),
    'tri_max_range_m': (float, lambda x: x > 0.0, '> 0'),
    'lidar_sigma_m': (float, lambda x: x > 0.0, '> 0'),
    'depth_sigma_k': (float, lambda x: x > 0.0, '> 0'),
    'fusion_gate_sigma': (float, lambda x: x > 0.0, '> 0'),
}
_BOOLS = ('use_mono', 'use_triangulation')
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
    v = raw.get('disagree_policy', DEFAULTS['disagree_policy'])
    pol = v.strip().lower() if isinstance(v, str) else None      # ship_back distance_source 와 같은 관대함
    if pol in DISAGREE_POLICIES:
        out['disagree_policy'] = pol
    else:
        issues.append(f"disagree_policy={v!r} 무효(lidar|min) → 기본값 {DEFAULTS['disagree_policy']!r}"
                      '(어긋나면 LiDAR 값)')
    for key in _BOOLS:
        v = raw.get(key, DEFAULTS[key])
        if isinstance(v, bool):
            out[key] = v
        else:
            issues.append(f'{key}={v!r} 무효(true|false) → 기본값 {DEFAULTS[key]!r}(카메라 거리 안 씀)')
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


def _cnt(used, n):
    return '-' if n is None else f'{used}/{n}'


def format_status(stats, wp_mode):
    """/fusion_status 한 줄. 예 'L0.82 D0.61 agree0.93 lidar_ok depth_ok n=3 m=2'.

    use_mono/use_triangulation 이 하나라도 켜져 있으면(stats['cam'] 있음) 뒤에 카메라 토큰을 붙인다 —
      'oak_ok|oak_stale mono=<기여 물체>/<측정> tri=<기여 물체>/<추정> imu_ok|imu_mount rtk_fixed|rtk_no'
      (꺼진 소스는 'mono=off'·'tri=off'). 둘 다 꺼지면 P1 이전과 같은 문자열.
    """
    s = (f"L{_fmt(stats['conf_l'])} D{_fmt(stats['conf_d'])} agree{_fmt(stats['agree'])} "
         f"{'lidar_ok' if stats['lidar_ok'] else 'lidar_stale'} "
         f"{'depth_ok' if stats['depth_ok'] else 'depth_stale'} "
         f"n={'-' if stats['n'] is None else stats['n']} m={wp_mode}")
    cam = stats.get('cam')
    if cam is not None:
        mono = 'off' if not cam['use_mono'] else _cnt(stats.get('mono_used', 0), stats.get('mono_n'))
        tri = 'off' if not cam['use_tri'] else _cnt(stats.get('tri_used', 0), stats.get('tri_n'))
        s += (f" {'oak_ok' if cam['oak_ok'] else 'oak_stale'} mono={mono} tri={tri} "
              f"{'imu_ok' if cam['imu_ok'] else 'imu_mount'} {'rtk_fixed' if cam['rtk_fixed'] else 'rtk_no'}")
    if stats['n'] is None:
        s += ' (발행 중단)'
    return s


def due(last_t, now, period):
    """주기 로그: 처음이거나 마지막 뒤 period 가 지났으면 True."""
    return last_t is None or (now - last_t) >= period

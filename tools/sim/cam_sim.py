#!/usr/bin/env python3
"""tournament_sim --arena photo 보조 — 현장 사진 경기장·합성 카메라·전략 C 시제품 (시뮬 전용, 배 코드 아님).

근거: docs/문제와작업/계약서_카메라장애물_20261111.md U3·보완 1, 팀원 경기장 사진(11-11 메인 세션 판독).

세계 (--arena photo, 로컬 x = 동, y = 북 = 진행 방향 = 경기장 **긴 변** 방향(팀원 확인 11-11))
  · 경기장 = 평행사변형. 아래·위 변은 수평(y = 0, y = arena_len), 긴 변은 shear(기본 3 m / 30 m ≈ 5.7°)만큼 동쪽으로 기운다:
    꼭짓점 (−w/2, 0), (w/2, 0), (w/2 + sh, L), (−w/2 + sh, L). 이 폴리곤이 지오펜스(north geofence_polygon).
  · 경기장 좌표 (u, v): u = 가로(−w/2 ~ w/2), v = 긴 변 방향(0 ~ L). 세계 x = u + sh·v/L, y = v.
  · 꼭짓점 4곳 = 세운 큰 빨간 원통 부표(반경 POST_R, 높이 POST_H) — LiDAR 에 보임, 카메라에 빨강.
  · 꼭짓점 사이 로프(작은 노란 부이) = LiDAR 에 안 보인다고 가정 → 모델 안 함(지오펜스가 경계).
  · 붐 6개 = 빨간 누운 원통(폭 BOOM_W 0.2 m, 높이 BOOM_H 0.2 m, 길이 2.2~2.8 m), 양 끝에 흰 둥근 부표(반경 WHITE_R).
    대부분 긴 변과 나란, 하나는 비스듬(≈18°), 왼쪽에 두 붐이 나란히(중심 간격 1.6 m → 통로 ≈1.2~1.4 m).
    LiDAR 가시성 --boom-lidar {none, p, always} (p: 스캔마다 붐마다 확률 --boom-lidar-p 0.3). 흰 부표는 늘 보임.
  · 붐에 안 붙은 흰 부표 10개.
  · 배치 = 손으로 정한 사진 근사 + 시드 지터(위치 ±0.4 m, 각 ±5°, 길이 ±0.3 m), 별도 RNG — 기존 세계 RNG 와 무관.
  · 두 배가 경기장 하나를 같이 쓴다(레인 분리 없음, 팀원 확인 11-11). 우리 배 = 시드별 u = ±2, 상대 = 반대쪽.
    상대 배 = 대본 궤적: A*(0.2 m 격자, 부표·붐·경계에서 OPP_CLEAR 떨어짐) + 시선 단축 → 장애물을 관통하지 않는다.

합성 카메라 (전략 B·C, 매 틱 = 10 Hz)
  · 카메라 = 배 중심 + cam_x(선수 방향), 높이 cam_h_true, 틸트 cam_tilt_true + 피치 AR(1) 잡음(σ 1.5°, τ 1 s).
  · 빨간 물체(붐·꼭짓점 기둥)의 수면 윤곽·윗면 점을 핀홀 투영(fx = fy = vision_geom.fx_from_hfov, cx = W//2, cy = H/2 —
    U2 순수 함수와 같은 가정) → bbox (x_l, x_r, y_bottom). 화면 밖 자르기(바닥 닿음은 y = H), 뒤쪽 점 버림.
  · 가림(단순): 더 가까운 물체(빨강·흰 부표·상대 배) 방위 구간이 그 물체 방위 구간의 80 % 이상을 덮으면 안 보임.
  · 최소 크기: 폭 ≥ 4 px, 높이 ≥ 3 px, 면적 ≥ min_area_px(40).
  · 검출률 pdet(물체·프레임마다), 오탐 사건 --cam-fp(프레임당 시작 확률, 사건은 --cam-fp-dur s 지속, 종류 small/large/both):
    small = 수면 띠 무작위 폭 8~40 px, large = 바닥 닿음(y = H) 폭 60~240 px. 끊김 --cam-outage T0:DUR.
  · bbox + 시뮬 LiDAR 스캔 → **실제 U2/U2b/U2c 순수 함수** fused_detections_to_array + ConfirmWindow(3/5) → 배열
    → sd.camera_obstacle_cb(msg). ship_direction stale 판정은 rosstub Clock(노드 모듈 time) 기준.

전략 C 시제품 (src 아님, /candidate_angle 을 시뮬이 매 틱 ship_direction 에 주입 → 실제 ship_direction → motor_control 체인)
  FORWARD(주입 없음 = north 의 20000 → (C) 회피 + 카메라 병합) → 앞쪽 ±c_cone_deg·c_trigger_m 안 LiDAR 군집이
  아직 '확인' 안 됐으면 STOP(50000, 속도 < c_stop_speed 0.15 m/s 또는 c_stop_s 2.5 s — 흐름 0.1 m/s 라 0 은 안 된다)
  → SWEEP(SPIN 6000/5000 으로 ψ0 + sweep, ψ0 − sweep 까지)
  — 그동안 카메라 배열·LiDAR·경계(폴리곤)를 세계 방위 칸(1°)에 min 으로 모은다, 카메라 시야에 든 군집은 '확인' →
  가장 열린 방위 선택(점수 = min(±c_corr_deg 안 최소 여유거리, c_clear_cap 3.5 m) − c_goal_w·|목표 방위 차|°)
  → ALIGN(SPIN, 오차 < 8°) → COMMIT(candidate 상대각,
  c_commit_m 또는 6 s, 앞 0.8 m 장애물이면 중단) → FORWARD + c_cooldown_s 동안 다시 안 멈춤.
  카메라 끊김 중에는 트리거하지 않는다(= B). 선회가 [I] 경계 가드로 막혀 SWEEP 이 c_sweep_timeout 넘으면 포기.
"""
import heapq
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", "..", "src"))
if os.path.join(SRC, "color_shape_detector") not in sys.path:
    sys.path.insert(0, os.path.join(SRC, "color_shape_detector"))

from color_shape_detector.obstacle_cam_logic import DEFAULT_PARAMS, ConfirmWindow  # noqa: E402
from color_shape_detector.obstacle_fusion_logic import (  # noqa: E402
    FUSION_DEFAULTS, fused_detections_to_array, lidar_clusters)
from color_shape_detector.vision_geom import fx_from_hfov  # noqa: E402

IMG_W, IMG_H = 640, 360
WHITE_R = 0.22        # 흰 부표 반경(지름 0.44 — 사진 판독 0.4~0.5 m 가정)
POST_R, POST_H = 0.30, 1.0   # 꼭짓점 빨간 세운 원통 ⚠️ 가정
BOOM_W, BOOM_H = 0.2, 0.2    # 누운 원통 지름 ≈ 수면 위 높이 ⚠️ 가정
OPP_CLEAR = 0.65      # 상대 배 경로가 장애물·경계에서 떨어지는 거리
LIDAR_MAX = 20.0
MIN_AREA_PX = 40.0    # vision.yaml /** min_area_px

# 사진 근사 배치 (u, v[m], 긴 변 기준 각[deg, + = 시계], 붐 길이[m]) — 경기장 12 × 30 m 기준
PHOTO_BOOMS = [(-4.4, 10.0, 0.0, 2.5), (-2.8, 10.4, 0.0, 2.5),      # 왼쪽 나란한 두 붐(좁은 통로)
               (0.3, 7.6, 0.0, 2.5), (2.4, 13.6, 0.0, 2.5),
               (-1.1, 17.2, 18.0, 2.5),                              # 비스듬한 붐
               (1.2, 21.2, 0.0, 2.5)]
PHOTO_WHITE = [(4.6, 5.6), (-2.0, 6.3), (-5.0, 14.6), (4.8, 10.6), (-0.6, 13.4),
               (4.2, 17.6), (-4.0, 21.0), (3.6, 23.2), (-1.8, 23.6), (0.9, 16.2)]


# ───────────────────────── 기하 ─────────────────────────
def rect_from_center(cx, cy, hdg_deg, L, W):
    """중심·나침반 방향(긴 축)·길이·폭 → 꼭짓점 4개(np.array, 순서대로)."""
    p = math.radians(hdg_deg)
    fx, fy = math.sin(p), math.cos(p)
    lx, ly = -fy, fx
    hl, hw = L / 2.0, W / 2.0
    return [np.array([cx + sf * hl * fx + sl * hw * lx, cy + sf * hl * fy + sl * hw * ly])
            for sf, sl in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def pt_seg_dist(px, py, ax, ay, bx, by):
    ex, ey = bx - ax, by - ay
    L2 = ex * ex + ey * ey
    s = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / L2))
    return math.hypot(px - (ax + s * ex), py - (ay + s * ey))


def point_in_convex(px, py, poly):
    sgn = 0
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        c = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if c != 0:
            s = 1 if c > 0 else -1
            if sgn == 0:
                sgn = s
            elif s != sgn:
                return False
    return True


def poly_outside_dist(px, py, poly):
    """볼록 폴리곤 밖 거리(안이면 0)."""
    if point_in_convex(px, py, poly):
        return 0.0
    n = len(poly)
    return min(pt_seg_dist(px, py, *poly[i], *poly[(i + 1) % n]) for i in range(n))


def poly_inside_dist(px, py, poly):
    """폴리곤 안에서 경계까지 거리(밖이면 0)."""
    if not point_in_convex(px, py, poly):
        return 0.0
    n = len(poly)
    return min(pt_seg_dist(px, py, *poly[i], *poly[(i + 1) % n]) for i in range(n))


def ray_poly_dist(x, y, hdg_deg, poly):
    """안에서 나침반 방향 광선이 경계에 닿는 거리(없으면 inf)."""
    p = math.radians(hdg_deg)
    dx, dy = math.sin(p), math.cos(p)
    best = math.inf
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if t > 0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best


# ───────────────────────── 세계 ─────────────────────────
def build_photo_world(seed, g):
    """사진 경기장. 반환 dict: poly, circles[(x, y, r, kind)], booms[{c4, cx, cy, L, hdg, seg}], posts, side,
    our_x, start, psi0, wp_pts, opp_start, opp_goal, uv2xy."""
    if g.get("travel_axis", "long") != "long":
        raise NotImplementedError("--travel-axis short 미구현 — 진행 방향 = 긴 변(팀원 확인 11-11)")
    aw, al, sh = g["arena_w"], g["arena_len"], g["photo_shear"]
    sx, sy = aw / 12.0, al / 30.0            # 사진 근사는 12 × 30 기준 → 비례

    def uv2xy(u, v):
        return u + sh * v / al, v

    poly = [uv2xy(-aw / 2, 0.0), uv2xy(aw / 2, 0.0), uv2xy(aw / 2, al), uv2xy(-aw / 2, al)]
    rng0 = random.Random(seed)                # 기존 build_world 와 같은 첫 뽑기(레인 편) — 같은 시드면 같은 편
    side = rng0.choice((-1.0, 1.0))
    psi_jit = rng0.uniform(-3.0, 3.0)
    rng = random.Random(seed * 7919 + 17)     # 배치 지터 전용
    tilt = math.degrees(math.atan2(sh, al))   # 긴 변 나침반 방향
    circles, booms, posts = [], [], []
    for (px, py) in poly:
        circles.append((px, py, POST_R, "post"))
        posts.append((px, py))
    for (u0, v0, a0, L0) in PHOTO_BOOMS:
        for _ in range(50):
            u = u0 * sx + rng.uniform(-0.4, 0.4)
            v = v0 * sy + rng.uniform(-0.4, 0.4)
            a = a0 + rng.uniform(-5.0, 5.0)
            L = L0 + rng.uniform(-0.3, 0.3)
            cx, cy = uv2xy(u, v)
            hdg = tilt + a
            ok = all(pt_seg_dist(cx, cy, *b["seg"][0], *b["seg"][1]) > 1.2 for b in booms) or not booms
            if ok:
                break
        p = math.radians(hdg)
        ux, uy = math.sin(p), math.cos(p)
        h = L / 2.0 + WHITE_R                 # 흰 부표 중심 = 붐 끝 + 반경
        e1 = (cx + h * ux, cy + h * uy)
        e2 = (cx - h * ux, cy - h * uy)
        booms.append(dict(cx=cx, cy=cy, L=L, hdg=hdg, c4=rect_from_center(cx, cy, hdg, L, BOOM_W),
                          seg=((cx + L / 2 * ux, cy + L / 2 * uy), (cx - L / 2 * ux, cy - L / 2 * uy))))
        circles.append((e1[0], e1[1], WHITE_R, "white_end"))
        circles.append((e2[0], e2[1], WHITE_R, "white_end"))
    our_u = side * 2.0 * sx
    start = np.array(uv2xy(our_u, g["start_y"] - 1.0))
    opp_u = -our_u
    opp_start = uv2xy(opp_u, g["start_y"] - 1.0)
    for (u0, v0) in PHOTO_WHITE:
        for _ in range(50):
            u = u0 * sx + rng.uniform(-0.4, 0.4)
            v = v0 * sy + rng.uniform(-0.4, 0.4)
            x, y = uv2xy(u, v)
            if all(math.hypot(x - c[0], y - c[1]) > 1.0 for c in circles) and \
               all(pt_seg_dist(x, y, *b["seg"][0], *b["seg"][1]) > 0.8 for b in booms) and \
               math.hypot(x - start[0], y - start[1]) > 3.0 and \
               math.hypot(x - opp_start[0], y - opp_start[1]) > 3.0:
                circles.append((x, y, WHITE_R, "white"))
                break
    wp_pts = [("T0 출발 근처",) + uv2xy(our_u, g["start_y"] + 1.0),
              ("T1 결승선 너머",) + uv2xy(our_u, g["finish_y"] + g["wp_beyond"])]
    W = dict(seed=seed, side=side, our_x=float(start[0]), toward_opp=-side, start=start,
             psi0=(tilt + psi_jit) % 360.0, poly=poly, circles=circles, booms=booms, posts=posts,
             wp_pts=wp_pts, opp_start=opp_start, opp_goal=uv2xy(opp_u, g["finish_y"] + g["wp_beyond"]),
             goal_xy=wp_pts[-1][1:], tilt=tilt, uv2xy=None)
    W["cx"] = np.array([c[0] for c in circles])
    W["cy"] = np.array([c[1] for c in circles])
    W["cr"] = np.array([c[2] for c in circles])
    W["buoys"] = [(c[0], c[1]) for c in circles]
    W["n_white"] = sum(c[3].startswith("white") for c in circles)
    W["boom_segs"] = [np.array([[b["c4"][i][0], b["c4"][i][1], b["c4"][(i + 1) % 4][0], b["c4"][(i + 1) % 4][1]]
                                for i in range(4)]) for b in booms]
    return W


def obstacle_clear(x, y, W):
    """(x, y) 에서 가장 가까운 장애물(원·붐 사각형·경계) 표면까지 거리."""
    d = poly_inside_dist(x, y, W["poly"])
    for (cx, cy, r, _) in W["circles"]:
        d = min(d, math.hypot(x - cx, y - cy) - r)
    for b in W["booms"]:
        c = b["c4"]
        dd = min(pt_seg_dist(x, y, c[i][0], c[i][1], c[(i + 1) % 4][0], c[(i + 1) % 4][1]) for i in range(4))
        if point_in_convex(x, y, [tuple(p) for p in c]):
            dd = 0.0
        d = min(d, dd)
    return d


def opp_path(W, g, res=0.2, clear=OPP_CLEAR):
    """상대 배 경로: A*(8방향) + 시선 단축 → (xs, ys) 0.1 m 간격, 목표 뒤로 12 m 직진 연장."""
    xs_all = [p[0] for p in W["poly"]]
    x0, x1 = min(xs_all) - 0.5, max(xs_all) + 0.5
    y0, y1 = -0.5, g["arena_len"] + 0.5
    nx, ny = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    free = np.zeros((nx, ny), dtype=bool)
    for i in range(nx):
        for j in range(ny):
            free[i, j] = obstacle_clear(x0 + i * res, y0 + j * res, W) > clear
    s = (int(round((W["opp_start"][0] - x0) / res)), int(round((W["opp_start"][1] - y0) / res)))
    gl = (int(round((W["opp_goal"][0] - x0) / res)), int(round((W["opp_goal"][1] - y0) / res)))

    def nearest_free(c):
        best, bd = c, 1e9
        for i in range(max(0, c[0] - 10), min(nx, c[0] + 11)):
            for j in range(max(0, c[1] - 10), min(ny, c[1] + 11)):
                if free[i, j]:
                    d = (i - c[0]) ** 2 + (j - c[1]) ** 2
                    if d < bd:
                        best, bd = (i, j), d
        return best
    s, gl = nearest_free(s), nearest_free(gl)
    openq = [(0.0, s)]
    came = {s: None}
    cost = {s: 0.0}
    while openq:
        _, c = heapq.heappop(openq)
        if c == gl:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                n = (c[0] + di, c[1] + dj)
                if not (0 <= n[0] < nx and 0 <= n[1] < ny) or not free[n]:
                    continue
                nc = cost[c] + math.hypot(di, dj)
                if nc < cost.get(n, 1e18):
                    cost[n] = nc
                    came[n] = c
                    heapq.heappush(openq, (nc + math.hypot(gl[0] - n[0], gl[1] - n[1]), n))
    if gl not in came:
        pts = [W["opp_start"], W["opp_goal"]]
    else:
        cells = []
        c = gl
        while c is not None:
            cells.append(c)
            c = came[c]
        cells.reverse()
        pts = [(x0 + i * res, y0 + j * res) for i, j in cells]

        def los(a, b):
            n_ = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res / 2)))
            for k in range(n_ + 1):
                t = k / n_
                if obstacle_clear(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, W) <= clear:
                    return False
            return True
        sm = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not los(pts[i], pts[j]):
                j -= 1
            sm.append(pts[j])
            i = j
        pts = sm
    pts = [W["opp_start"]] + pts[1:] + [(pts[-1][0], pts[-1][1] + 12.0)]
    xs, ys = [], []
    for a, b in zip(pts, pts[1:]):
        n_ = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.1))
        for k in range(n_):
            xs.append(a[0] + (b[0] - a[0]) * k / n_)
            ys.append(a[1] + (b[1] - a[1]) * k / n_)
    xs.append(pts[-1][0])
    ys.append(pts[-1][1])
    return np.array(xs), np.array(ys)


# ───────────────────────── LiDAR ─────────────────────────
def lidar_photo(W, x, y, psi, scan_rel, boom_vis):
    """원(반경 각각) + 보이는 붐 4변. scan_rel = 상대각 배열(좌 +). 반환 np 배열(LIDAR_MAX 넘으면 inf)."""
    th = np.radians(psi - scan_rel)
    dx, dy = np.sin(th), np.cos(th)
    ox = W["cx"] - x
    oy = W["cy"] - y
    t_ca = dx[:, None] * ox[None, :] + dy[:, None] * oy[None, :]
    d2 = (ox ** 2 + oy ** 2)[None, :] - t_ca ** 2
    r2 = (W["cr"] ** 2)[None, :]
    hit = (d2 <= r2) & (t_ca > 0)
    t = np.where(hit, t_ca - np.sqrt(np.maximum(r2 - d2, 0.0)), np.inf)
    rng = t.min(axis=1)
    for bi, vis in enumerate(boom_vis):
        if not vis:
            continue
        for ax, ay, bx, by in W["boom_segs"][bi]:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            ok = np.abs(den) > 1e-9
            dd = np.where(ok, den, 1.0)
            tt = ((ax - x) * ey - (ay - y) * ex) / dd
            uu = ((ax - x) * dy - (ay - y) * dx) / dd
            m = ok & (tt > 0) & (uu >= 0.0) & (uu <= 1.0)
            rng = np.where(m & (tt < rng), tt, rng)
    rng[rng > LIDAR_MAX] = np.inf
    return rng


# ───────────────────────── 카메라 ─────────────────────────
def _red_objects(W):
    """빨간 물체별 3D 점(수면 z = 0 윤곽, 윗면) — 투영용. [(kind, idx, water_pts(N×2), top_pts(M×3))]."""
    out = []
    for i, b in enumerate(W["booms"]):
        c4 = b["c4"]
        wp = []
        for a, bb in ((c4[0], c4[3]), (c4[1], c4[2]), (c4[0], c4[1]), (c4[2], c4[3])):
            n = max(2, int(np.linalg.norm(bb - a) / 0.05))
            for k in range(n + 1):
                wp.append(a + (bb - a) * k / n)
        (sx_, sy_), (ex_, ey_) = b["seg"]
        n = max(2, int(b["L"] / 0.05))
        top = [(sx_ + (ex_ - sx_) * k / n, sy_ + (ey_ - sy_) * k / n, BOOM_H) for k in range(n + 1)]
        out.append(("boom", i, np.array(wp), np.array(top)))
    for i, (px, py) in enumerate(W["posts"]):
        ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        wp = np.column_stack([px + POST_R * np.cos(ang), py + POST_R * np.sin(ang)])
        top = np.column_stack([wp, np.full(len(ang), POST_H)])
        out.append(("post", i, wp, top))
    return out


class CamSim:
    """합성 카메라 + 실제 퓨전 순수 함수. 프레임마다 step() → (배열 or None(끊김), 진단 dict)."""

    def __init__(self, W, seed, hfov, p_over=None, cam_x=0.5, cam_h_err=0.0, cam_tilt_err=0.0,
                 pitch_sigma=1.5, pitch_tau=1.0, pdet=0.9, fp=0.0, fp_kind="both", fp_dur=1.0,
                 outage=None, dt=0.1):
        self.W = W
        self.p = dict(DEFAULT_PARAMS)
        self.p.update(FUSION_DEFAULTS)
        self.p["hfov_deg"] = float(hfov)
        self.p["cam_x_from_lidar_m"] = float(cam_x)
        self.p.update(p_over or {})
        self.h_true = self.p["cam_height_m"] + cam_h_err        # 참값 = 설정 + 오차
        self.tilt_true = self.p["cam_tilt_deg"] + cam_tilt_err
        self.f = fx_from_hfov(IMG_W, float(hfov))
        self.cam_x = cam_x
        self.objs = _red_objects(W)
        self.rng = random.Random(seed * 104729 + 7)
        self.a = math.exp(-dt / pitch_tau)
        self.pitch_sigma = pitch_sigma
        self.pitch = self.rng.gauss(0.0, pitch_sigma)
        self.pdet, self.fp, self.fp_kind, self.fp_dur = pdet, fp, fp_kind, fp_dur
        self.fp_left = 0
        self.fp_box = None
        self.outage = outage
        self.dt = dt
        self.cw = ConfirmWindow(3, 5, 10.0)
        self.in_outage = False

    # --- 투영 ---
    def _proj(self, pts_xy, z, cxw, cyw, psi, tilt):
        p = math.radians(psi)
        dx = pts_xy[:, 0] - cxw
        dy = pts_xy[:, 1] - cyw
        fwd = dx * math.sin(p) + dy * math.cos(p)
        left = -dx * math.cos(p) + dy * math.sin(p)
        drop = self.h_true - z
        t = math.radians(tilt)
        zc = fwd * math.cos(t) + drop * math.sin(t)
        yc = drop * math.cos(t) - fwd * math.sin(t)
        ok = zc > 0.05
        u = IMG_W // 2 + self.f * (-left[ok]) / zc[ok]
        v = IMG_H / 2.0 + self.f * yc[ok] / zc[ok]
        return u, v

    @staticmethod
    def _interval(pts_xy, cxw, cyw, psi):
        """점들의 상대방위 구간(좌 +)과 최소 거리 — 카메라(또는 LiDAR) 원점."""
        rel = np.degrees(np.arctan2(-(pts_xy[:, 0] - cxw) * math.cos(math.radians(psi))
                                    + (pts_xy[:, 1] - cyw) * math.sin(math.radians(psi)),
                                    (pts_xy[:, 0] - cxw) * math.sin(math.radians(psi))
                                    + (pts_xy[:, 1] - cyw) * math.cos(math.radians(psi))))
        r = np.hypot(pts_xy[:, 0] - cxw, pts_xy[:, 1] - cyw)
        return float(rel.min()), float(rel.max()), float(r.min())

    def true_red_intervals(self, x, y, psi, rmax=10.0):
        """LiDAR 원점(배 중심) 기준 빨간 물체 방위 구간 [(lo, hi, rmin)] — cam_false 판정용."""
        out = []
        for _, _, wp, _ in self.objs:
            lo, hi, rmin = self._interval(wp, x, y, psi)
            if rmin <= rmax and hi - lo < 180.0:
                out.append((lo, hi, rmin))
        return out

    def frame(self, t, x, y, psi, scan, opp_corners=None):
        """한 프레임. 반환 (arr | None, info). None = 끊김(발행 안 함)."""
        info = dict(n_vis=0, n_box=0, fp=False)
        out_now = self.outage is not None and self.outage[0] <= t < self.outage[0] + self.outage[1]
        if out_now:
            if not self.in_outage:
                self.cw.reset()
            self.in_outage = True
            return None, info
        if self.in_outage:
            self.cw.reset()
            self.in_outage = False
        self.pitch = self.a * self.pitch + math.sqrt(1 - self.a ** 2) * self.rng.gauss(0.0, self.pitch_sigma)
        tilt = self.tilt_true + self.pitch
        p = math.radians(psi)
        cxw, cyw = x + self.cam_x * math.sin(p), y + self.cam_x * math.cos(p)
        # 가림용 구간: 흰 부표 + 상대 배 + 빨간 물체
        occ = []
        for (bx, by, r, kind) in self.W["circles"]:
            if kind == "post":
                continue
            d = math.hypot(bx - cxw, by - cyw)
            if d < 0.3 or d > 15.0:
                continue
            c_rel = math.degrees(math.atan2(-(bx - cxw) * math.cos(p) + (by - cyw) * math.sin(p),
                                            (bx - cxw) * math.sin(p) + (by - cyw) * math.cos(p)))
            hw = math.degrees(math.asin(min(1.0, r / d)))
            occ.append((c_rel - hw, c_rel + hw, d - r))
        if opp_corners is not None:
            oc = np.array([[c[0], c[1]] for c in opp_corners])
            lo, hi, rmin = self._interval(oc, cxw, cyw, psi)
            if hi - lo < 180.0:
                occ.append((lo, hi, rmin))
        cands = []
        for kind, i, wp, top in self.objs:
            lo, hi, rmin = self._interval(wp, cxw, cyw, psi)
            if rmin > 15.0 or hi - lo >= 180.0:
                continue
            u1, v1 = self._proj(wp, 0.0, cxw, cyw, psi, tilt)
            u2, v2 = self._proj(top[:, :2], top[:, 2], cxw, cyw, psi, tilt)
            if len(u1) == 0:
                continue
            uu = np.concatenate([u1, u2])
            vv = np.concatenate([v1, v2])
            x_l, x_r = float(uu.min()), float(uu.max())
            y_top, y_bot = float(vv.min()), float(v1.max())
            if x_r < 0 or x_l > IMG_W or y_top > IMG_H or y_bot < 0:
                continue
            x_l, x_r = max(0.0, x_l), min(float(IMG_W), x_r)
            y_bot_c = min(float(IMG_H), y_bot)
            y_top_c = max(0.0, y_top)
            if x_r - x_l < 4 or y_bot_c - y_top_c < 3 or (x_r - x_l) * (y_bot_c - y_top_c) < MIN_AREA_PX:
                continue
            cands.append((rmin, lo, hi, (x_l, x_r, y_bot_c)))
            occ.append((lo, hi, rmin))
        boxes = []
        for rmin, lo, hi, box in cands:
            span = hi - lo
            cover = []
            for (a, b, r) in occ:
                if r < rmin - 0.05 and b > lo and a < hi:
                    cover.append((max(a, lo), min(b, hi)))
            cover.sort()
            tot, cur = 0.0, None
            for a, b in cover:
                if cur is None or a > cur[1]:
                    if cur is not None:
                        tot += cur[1] - cur[0]
                    cur = [a, b]
                else:
                    cur[1] = max(cur[1], b)
            if cur is not None:
                tot += cur[1] - cur[0]
            if span > 1e-6 and tot / span >= 0.8:
                continue
            info["n_vis"] += 1
            if self.rng.random() < self.pdet:
                boxes.append(box)
        # 오탐 사건
        if self.fp_left <= 0 and self.fp > 0.0 and self.rng.random() < self.fp:
            self.fp_left = max(1, int(round(self.fp_dur / self.dt)))
            kind = self.fp_kind if self.fp_kind != "both" else self.rng.choice(("small", "large"))
            hz = IMG_H / 2.0 - self.f * math.tan(math.radians(self.p["cam_tilt_deg"]))
            if kind == "small":
                w = self.rng.uniform(8.0, 40.0)
                xc = self.rng.uniform(w / 2, IMG_W - w / 2)
                self.fp_box = [xc - w / 2, xc + w / 2, self.rng.uniform(hz + 5.0, IMG_H - 5.0)]
            else:
                w = self.rng.uniform(60.0, 240.0)
                xc = self.rng.uniform(w / 2, IMG_W - w / 2)
                self.fp_box = [xc - w / 2, xc + w / 2, float(IMG_H)]
        if self.fp_left > 0:
            self.fp_left -= 1
            jit = self.rng.uniform(-4.0, 4.0)
            self.fp_box[0] += jit
            self.fp_box[1] += jit
            boxes.append(tuple(self.fp_box))
            info["fp"] = True
        info["n_box"] = len(boxes)
        arr = fused_detections_to_array(boxes, IMG_W, IMG_H, scan, self.p, self.cw)
        return arr, info


def cam_false(arr, red_iv, slack_deg=3.0, close_frac=0.5):
    """배열의 유한 칸 중 참 빨간 물체(10 m 안) 방위 구간 ±slack_deg 밖이거나, 거리가 그 물체 참 최소거리의
    close_frac 배보다 가까운 칸(크게 당겨진 거리 — 앞쪽 흰 부표와 잘못 짝지은 경우 등)이 있나.
    (행 기반 대체의 보수 편향 0.85·피치 오차 정도는 거짓으로 안 센다.)"""
    if not arr:
        return False
    a0, inc = arr[0], arr[1]
    for k, r in enumerate(arr[2:]):
        if not math.isfinite(r):
            continue
        a = a0 + k * inc
        if not any(lo - slack_deg <= a <= hi + slack_deg and r >= close_frac * rmin for lo, hi, rmin in red_iv):
            return True
    return False


def contact_note(t, x, y, psi, bm, cam_hist):
    """붐 접촉 진단 문자열: 접촉 시 붐 최근접점의 상대방위(좌 +)·붐과 선수 방향 사잇각, 직전 3 s 동안 카메라 배열이
    그 붐(세계 위치)을 덮은 프레임 수/카메라 프레임 수. 'cam k/n'."""
    (ax, ay), (bx, by) = bm["seg"]
    ex, ey = bx - ax, by - ay
    L2 = ex * ex + ey * ey
    s_ = max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
    qx, qy = ax + s_ * ex, ay + s_ * ey
    p = math.radians(psi)
    rel = math.degrees(math.atan2(-(qx - x) * math.cos(p) + (qy - y) * math.sin(p),
                                  (qx - x) * math.sin(p) + (qy - y) * math.cos(p)))
    cross = abs(wrap(bm["hdg"] - psi))
    cross = min(cross, 180.0 - cross)
    seen = n = 0
    for (t_, x_, y_, psi_, arr) in cam_hist:
        if arr is None:
            continue
        n += 1
        a0, inc = (arr[0], arr[1]) if arr else (0.0, 1.0)
        hit = False
        for k, r in enumerate(arr[2:]):
            if not math.isfinite(r):
                continue
            hdg = math.radians(psi_ - (a0 + k * inc))
            px, py = x_ + r * math.sin(hdg), y_ + r * math.cos(hdg)
            if pt_seg_dist(px, py, ax, ay, bx, by) < 0.8:
                hit = True
                break
        seen += int(hit)
    return f"t{t:.1f} rel{rel:+.0f}° x{cross:.0f}° cam{seen}/{n}"


# ───────────────────────── 전략 C 시제품 ─────────────────────────
C_DEFAULTS = dict(c_trigger_m=3.0, c_sweep_deg=45.0, c_cone_deg=30.0, c_corr_deg=12.0, c_commit_m=1.5,
                  c_commit_s=6.0, c_cooldown_s=4.0, c_stop_s=2.5, c_stop_speed=0.15, c_sweep_timeout=20.0,
                  c_check_r=0.7, c_goal_w=0.03, c_clear_cap=3.5)
SPIN_RIGHT, SPIN_LEFT, STOP_HOLD, CAND_INVALID = 5000.0, 6000.0, 50000.0, 20000.0


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class StrategyC:
    """정지·선회 탐색 시제품. step() → 이번 틱 /candidate_angle 주입값(None = 주입 없음, north 값 그대로)."""

    def __init__(self, W, cfg=None, hfov=63.9):
        self.W = W
        self.c = dict(C_DEFAULTS, **(cfg or {}))
        self.hfov = hfov
        self.state = "FORWARD"
        self.checked = []
        self.stops = 0
        self.t_state = 0.0
        self.cool_until = -1.0
        self.bins = None
        self.log = []

    def _set(self, s, t):
        self.state, self.t_state = s, t

    def _clusters(self, scan, p):
        cl = lidar_clusters(scan.ranges, scan.angle_min, scan.angle_increment, p) or []
        return [((c[0] + c[1]) / 2.0, c[2]) for c in cl]

    def _world(self, x, y, psi, rel, r):
        hdg = math.radians(psi - rel)
        return x + r * math.sin(hdg), y + r * math.cos(hdg)

    def _accumulate(self, x, y, psi, arr, scan_rel, ranges):
        def put(hdg, r):
            k = int(round(hdg)) % 360
            if r < self.bins[k]:
                self.bins[k] = r
        if arr:
            a0, inc = arr[0], arr[1]
            for k, r in enumerate(arr[2:]):
                if math.isfinite(r):
                    put(psi - (a0 + k * inc), r)
        fin = np.isfinite(ranges) & (np.abs(scan_rel) <= 90.0)
        for rel, r in zip(scan_rel[fin], ranges[fin]):
            put(psi - rel, float(r))

    def _choose(self, x, y):
        psi0 = self.psi0
        gx, gy = self.W["goal_xy"]
        goal = math.degrees(math.atan2(gx - x, gy - y))
        best, best_h = -1e9, psi0
        sw = self.c["c_sweep_deg"]
        for d in range(int(-sw), int(sw) + 1):
            h = psi0 + d
            clr = min(self.c["c_clear_cap"], ray_poly_dist(x, y, h, self.W["poly"]) - 0.6)
            for e in range(-int(self.c["c_corr_deg"]), int(self.c["c_corr_deg"]) + 1):
                clr = min(clr, self.bins[int(round(h + e)) % 360])
            sc = clr - self.c["c_goal_w"] * abs(wrap(h - goal))
            if sc > best:
                best, best_h = sc, h
        return best_h % 360.0

    def step(self, t, x, y, psi, speed, scan, scan_rel, ranges, arr, cam_ok, p):
        c = self.c
        if self.state == "FORWARD":
            if not cam_ok or t < self.cool_until:
                return None
            for rel, r in self._clusters(scan, p):
                if abs(rel) <= c["c_cone_deg"] and r <= c["c_trigger_m"]:
                    wx, wy = self._world(x, y, psi, rel, r)
                    if all(math.hypot(wx - a, wy - b) > c["c_check_r"] for a, b in self.checked):
                        self.stops += 1
                        self.log.append(f"t{t:.1f}:stop@({wx:.1f},{wy:.1f})")
                        self._set("STOP", t)
                        return STOP_HOLD
            return None
        if self.state == "STOP":
            if (speed < c["c_stop_speed"] and t - self.t_state > 0.5) or t - self.t_state > c["c_stop_s"]:
                self.psi0 = psi
                self.bins = [math.inf] * 360
                self.sweep_targets = [psi + c["c_sweep_deg"], psi - c["c_sweep_deg"]]
                self._set("SWEEP", t)
            else:
                return STOP_HOLD
        if self.state == "SWEEP":
            if cam_ok:
                self._accumulate(x, y, psi, arr, scan_rel, ranges)
                for rel, r in self._clusters(scan, p):
                    if abs(rel) <= self.hfov / 2.0 and r <= c["c_trigger_m"] + 1.5:
                        self.checked.append(self._world(x, y, psi, rel, r))
            if t - self.t_state > c["c_sweep_timeout"]:
                self.log.append(f"t{t:.1f}:sweep_timeout")
                self._set("FORWARD", t)
                self.cool_until = t + c["c_cooldown_s"]
                return CAND_INVALID
            tgt = self.sweep_targets[0]
            err = wrap(tgt - psi)               # + = 오른쪽(시계)으로 돌아야 함
            if abs(err) < 5.0:
                self.sweep_targets.pop(0)
                if not self.sweep_targets:
                    self.align = self._choose(x, y)
                    self._set("ALIGN", t)
                else:
                    tgt = self.sweep_targets[0]
                    err = wrap(tgt - psi)
            if self.state == "SWEEP":
                return SPIN_RIGHT if err > 0 else SPIN_LEFT
        if self.state == "ALIGN":
            err = wrap(self.align - psi)
            if abs(err) < 8.0 or t - self.t_state > 10.0:
                self.commit0 = (x, y)
                self._set("COMMIT", t)
            else:
                return SPIN_RIGHT if err > 0 else SPIN_LEFT
        if self.state == "COMMIT":
            fin = np.isfinite(ranges) & (np.abs(scan_rel) <= 25.0)
            blocked = bool(fin.any() and float(ranges[fin].min()) < 0.8)
            if math.hypot(x - self.commit0[0], y - self.commit0[1]) >= c["c_commit_m"] or \
               t - self.t_state > c["c_commit_s"] or blocked:
                self._set("FORWARD", t)
                self.cool_until = t + c["c_cooldown_s"]
                return CAND_INVALID
            rel = wrap(psi - self.align)        # 좌 + (ψ 시계 + → 목표가 왼쪽이면 ψ − h > 0)
            return rel % 360.0
        return None

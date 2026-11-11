#!/usr/bin/env python3
"""도리도리회피 시뮬 — 낮은 장애물(LiDAR 불가시) + 노란부표 1 m 간격 통로.

계약서: docs/문제와작업/계약서_도리도리회피_20261112.md (초안, 미승인)

  · 실제 ship_direction·motor_control 노드 코드를 rosstub 로 돌린다(avoid_sim.py 와 같은 틀).
  · 세계: 통로 좌우 노란부표(x=±W/2, y=0..L, 1 m 간격, LiDAR·카메라 모두 보임)
          + 낮은 빨간 장애물(통로 안, **LiDAR 불가시**, 카메라만 보임).
  · 선체 접촉 = 직사각형 1.66 × 0.57 m(1번배 실측) ↔ 원. 회전 중심 = 선체 중심(⚠️ 실물 피벗 미확인).
  · 전략:
      S0 lidar_only   현행 — mode 5, ship_direction (C) 갭 선택만 (카메라 지도 없음)
      S1 cam_fwd      전방 카메라 지도(주행 중 계속) → 가상 장애물을 스캔에 min 병합 → (C)
      S2 dori90       S1 + 칸(1 m)마다 정지 → 좌 90 → 복귀 → 우 90 → 복귀 (멈춘 순간만 촬영 추가)
      S3 dori60       S2 를 ±60°
      S4 dori60_plan  S3 + 지도 위 경로탐색(Dijkstra) → 전방 주시점으로 /candidate_angle 각도((B) 경로)
  · 가상 장애물 병합은 계약서 단위 ① 을 흉내 낸다: 스캔 ranges = min(LiDAR, 가상). (노드에선 (C) 병합 지점)

재현:  python3 tools/sim/doridori_sim.py            (기본 20 시드 × 폭 3/4 m × τ 0/0.5)
       python3 tools/sim/doridori_sim.py --seeds 5 --widths 3
의존: numpy, matplotlib. ROS 불필요. ⚠️ 모든 센서·부표 치수는 가정값 — 실측 후 교체.
"""
import argparse
import csv
import heapq
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import rosstub                                   # noqa: E402
rosstub.install()
from rosstub import Clock                        # noqa: E402
import kaboat_full_course_sim as K               # noqa: E402
from fastworld import FastWorld                  # noqa: E402

SD = rosstub.load_node(os.path.join(ROOT, "src/ship_direction/ship_direction/ship_direction.py"), "ship_direction_real",
                       extra_path=os.path.join(ROOT, "src/ship_direction"))
MC = rosstub.load_node(os.path.join(ROOT, "src/motor_control/motor_control/motor_control.py"), "motor_control_real")
std = sys.modules["std_msgs.msg"]
Float32, Int32 = std.Float32, std.Int32

AVOID_MODE = 5
DT = K.DT
STOP_HOLD, SPIN_R, SPIN_L, INVALID = 50000.0, 5000.0, 6000.0, 20000.0

# ── 선체·세계 (⚠️ 가정 — 실측 필요) ─────────────────────────────
HALF_L, HALF_W = 0.83, 0.285      # 1번배 1.66 × 0.57
YELLOW_R = 0.12                   # ⚠️ 노란부표 반경 가정
LOW_R = 0.20                      # ⚠️ 낮은 장애물 반경 가정
COURSE_L = 14.0

# ── 센서 (⚠️ 가정) ──────────────────────────────────────────────
CAM_HFOV = 80.32                  # vision.yaml hfov_deg (OAK 실측 전)
CAM_MAX = 8.0
CAM_H = 0.60                      # ⚠️ 카메라 높이
CAM_VFOV = 55.0                   # ⚠️ 세로 화각
CAM_DMIN = CAM_H / math.tan(math.radians(CAM_VFOV / 2.0))   # 근거리 사각 ≈ 1.15 m
CAM_LAT = 0.08                    # s, 촬영→처리 지연(보상 안 함)
SIG_BEAR = 0.4                    # deg, 픽셀+IMU
SIG_POS = 0.02                    # m, RTK fix
RS_HFOV, RS_MAX = 87.0, 6.0       # D455 뎁스
P_MISCLS = 0.03                   # 색 오분류(빨강↔노랑)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing(x, y, tx, ty):
    return math.degrees(math.atan2(tx - x, ty - y)) % 360.0


class Scan:
    def __init__(self, ranges):
        self.angle_min = math.radians(K.SCAN_ANGLE_MIN_DEG)
        self.angle_increment = math.radians(K.SCAN_INC_DEG)
        self.ranges = ranges


def build_world(seed, width, n_low=4):
    rng = random.Random(seed * 7919 + int(width * 10))
    yellow = []
    for k in range(int(COURSE_L) + 1):
        yellow += [(-width / 2, float(k), YELLOW_R), (width / 2, float(k), YELLOW_R)]
    low = []
    side = rng.choice((-1, 1))
    cmax = width / 2 - YELLOW_R - LOW_R - 0.05
    for k in range(n_low):
        y = 2.5 + 3.0 * k + rng.uniform(-0.5, 0.5)
        c = side * rng.uniform(0.0, cmax)
        low.append((c, y, LOW_R))
        side = -side
    return yellow, low


def hull_clear(x, y, psi, ox, oy, orad):
    """직사각형 선체 ↔ 원 최소거리(음수 = 침범)."""
    s, c = math.sin(math.radians(psi)), math.cos(math.radians(psi))
    dx, dy = ox - x, oy - y
    fwd = dx * s + dy * c
    lat = dx * c - dy * s
    qx = min(max(fwd, -HALF_L), HALF_L)
    qy = min(max(lat, -HALF_W), HALF_W)
    return math.hypot(fwd - qx, lat - qy) - orad


class LowMap:
    """낮은 장애물 지도: 세계 좌표 정적 점 칼만(트랙마다). 확정 = 2회 이상 갱신."""

    def __init__(self, vote=False):
        self.tr = []           # dict(mu, P, n, red)
        self.vote = vote

    def update(self, z, R, red=True):
        best, bd = None, 1e9
        for t in self.tr:
            S = t["P"] + R
            d = z - t["mu"]
            m = float(d @ np.linalg.solve(S, d))
            e = float(np.hypot(*d))
            if (m < 13.8 or e < 0.5) and m < bd:
                best, bd = t, m
        if best is None:
            self.tr.append(dict(mu=z.copy(), P=R.copy(), n=1, red=int(red)))
            return
        S = best["P"] + R
        Kg = best["P"] @ np.linalg.inv(S)
        best["mu"] = best["mu"] + Kg @ (z - best["mu"])
        best["P"] = (np.eye(2) - Kg) @ best["P"]
        best["n"] += 1
        best["red"] += int(red)

    def confirmed(self):
        if self.vote:   # 색 다수결: 빨강 비율 ≥ 0.6 + 3회 이상
            return [t for t in self.tr if t["n"] >= 3 and t["red"] / t["n"] >= 0.6]
        return [t for t in self.tr if t["n"] >= 2 and t["red"] >= 1]


def range_sigma(d, rel, moving, use_rs):
    """거리 측정 σ — D455 뎁스 + 수면 기준(지평면) 공식 역분산 결합."""
    inv = 0.0
    if use_rs and abs(rel) <= RS_HFOV / 2 and d <= RS_MAX:
        s = 0.0047 * d * d * 1.5
        inv += 1.0 / s ** 2
    phi = math.atan2(CAM_H, d)
    sp = math.radians(2.0 if moving else 0.5)          # 피치 잡음: 주행 중 파도 2°, 정지 0.5°
    s = CAM_H / math.sin(phi) ** 2 * sp
    inv += 1.0 / s ** 2
    return 1.0 / math.sqrt(inv)


def observe(rng, lm, low, yellow, x, y, psi, w_dps, yaw_bias, moving, use_rs, stats):
    """카메라 한 프레임: 화각·사각·최대거리 안의 물체 → 빨강으로 분류되면 지도 갱신."""
    yaw_m = psi + yaw_bias + rng.gauss(0, 0.3)
    px, py = x + rng.gauss(0, SIG_POS), y + rng.gauss(0, SIG_POS)
    p_det = 0.9 if abs(w_dps) < 5.0 else 0.5            # 회전 중 모션 블러
    objs = [(o, True) for o in low] + [(o, False) for o in yellow]
    for (ox, oy, _), is_low in objs:
        d = math.hypot(ox - x, oy - y)
        rel = wrap180(psi - bearing(x, y, ox, oy))       # CCW+ (좌 양수)
        if abs(rel) > CAM_HFOV / 2 or d > CAM_MAX or d < CAM_DMIN:
            continue
        if rng.random() > p_det:
            continue
        red = is_low if rng.random() > P_MISCLS else (not is_low)
        if not red and not lm.vote:
            continue
        if red and not is_low:
            stats["phantom_obs"] += 1
        rel_m = rel + rng.gauss(0, SIG_BEAR) + w_dps * CAM_LAT     # 지연 → 회전 중 방위 편향
        sr = range_sigma(d, rel, moving, use_rs)
        r_m = max(0.3, d + rng.gauss(0, sr))
        wb = math.radians(yaw_m - rel_m)
        z = np.array([px + r_m * math.sin(wb), py + r_m * math.cos(wb)])
        u = np.array([math.sin(wb), math.cos(wb)]); v = np.array([u[1], -u[0]])
        sb = r_m * math.radians(SIG_BEAR + 0.3)
        R = sr ** 2 * np.outer(u, u) + sb ** 2 * np.outer(v, v) + (SIG_POS ** 2) * np.eye(2)
        lm.update(z, R, red)


def virtual_scan(lm, x, y, psi, infl, wall=None):
    tr = lm.confirmed()
    objs = [(t["mu"][0], t["mu"][1], LOW_R + infl) for t in tr] + (wall or [])
    if not objs:
        return None
    w = FastWorld(objs)
    return w.lidar_scan(x, y, psi, max_range=20.0)


def plan_path(lm, yellow, width, x, y, goal, infl_low, infl_y):
    """격자 Dijkstra(0.1 m). 반환: 경로 점 목록(세계) 또는 None."""
    res = 0.1
    x0, x1 = -width / 2 - 0.5, width / 2 + 0.5
    y0, y1 = -3.0, COURSE_L + 3.0
    nx, ny = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    gx = x0 + np.arange(nx) * res
    gy = y0 + np.arange(ny) * res
    XX, YY = np.meshgrid(gx, gy, indexing="ij")
    cost = np.ones((nx, ny))
    block = np.zeros((nx, ny), bool)
    obs = [(ox, oy, r + HALF_W + infl_y) for (ox, oy, r) in yellow]
    obs += [(t["mu"][0], t["mu"][1], LOW_R + HALF_W + infl_low) for t in lm.confirmed()]
    for ox, oy, rr in obs:
        d = np.hypot(XX - ox, YY - oy)
        block |= d < rr
        cost += 3.0 * np.exp(-np.maximum(d - rr, 0) / 0.3)       # 멀리 떨어질수록 싸다
    block |= (XX < -width / 2) | (XX > width / 2)
    si = (int(round((x - x0) / res)), int(round((y - y0) / res)))
    si = (min(max(si[0], 0), nx - 1), min(max(si[1], 0), ny - 1))
    block[si] = False
    gj = int(round((goal[1] - y0) / res))
    dist = np.full((nx, ny), np.inf); prev = {}
    dist[si] = 0.0; pq = [(0.0, si)]
    nb = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    end = None
    while pq:
        dcur, (i, j) = heapq.heappop(pq)
        if dcur > dist[i, j]:
            continue
        if j >= gj:
            end = (i, j); break
        for di, dj in nb:
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and not block[a, b]:
                nd = dcur + math.hypot(di, dj) * cost[a, b]
                if nd < dist[a, b]:
                    dist[a, b] = nd; prev[(a, b)] = (i, j); heapq.heappush(pq, (nd, (a, b)))
    if end is None:
        return None
    path = [end]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    return [(gx[i], gy[j]) for i, j in path]


def run(seed, width, strat, tau=0.0, current=0.05, use_rs=True, infl_low=0.20, max_t=400.0,
        vote=False, wall=False, spin_guard=False, guard_cont=False, sweep_margin=0.10):
    rng = random.Random(seed * 31 + hash(strat) % 997)
    yellow, low = build_world(seed, width)
    lidar_world = FastWorld(yellow)                     # LiDAR 는 노란부표만 본다
    goal = (0.0, COURSE_L + 1.5)
    x, y, psi = 0.0, -1.5, 0.0
    v = w = 0.0
    cur_dir = rng.uniform(0, 360)
    cvx, cvy = current * math.sin(math.radians(cur_dir)), current * math.cos(math.radians(cur_dir))
    yaw_bias = rng.gauss(0, 1.0)                        # ⚠️ 카메라-IMU 정렬 오차(미측정)
    Clock.t = 0.0
    sd = SD.ShipDirection(); mc = MC.MotorController()
    sd.wp_mode_cb(Int32(AVOID_MODE))
    lm = LowMap(vote)
    # 통로 벽: LiDAR 로 잡은 노란부표 이웃 쌍(1 m) 사이를 0.15 m 간격 가상 점으로 메운다 (계약 단위 ①-b)
    wall_pts = None
    if wall:
        wall_pts = [(sx * width / 2, yy, 0.08) for sx in (-1, 1)
                    for yy in np.arange(0.0, COURSE_L + 1e-6, 0.15)]
    n_skip = 0
    stats = dict(phantom_obs=0)
    use_cam = strat != "lidar_only"
    dori = {"dori90": 90.0, "dori60": 60.0, "dori60_plan": 60.0}.get(strat)
    plan = strat == "dori60_plan"
    next_cell = 0.0
    # 도리도리 상태: None | dict(seq=[(kind, target_rel)], i, t0, ref)
    ds = None
    hits_low = hits_y = hits_spin = 0
    hit_flag = {}
    min_clear_low = math.inf
    path = None; last_plan = -1.0
    t = 0.0; ok = False; spin_time = 0.0; n_dori = 0
    seen_before = {i: None for i in range(len(low))}
    events = []
    out_ch = False
    while t < max_t:
        if y > COURSE_L + 1.0:
            ok = True; break
        moving = abs(v) > 0.05 or abs(w) > 5.0
        yaw_meas = psi + yaw_bias
        # ── 카메라 ──
        if use_cam and (dori is None or ds is None or ds.get("capture")):
            observe(rng, lm, low, yellow, x, y, psi, w, yaw_bias, moving, use_rs, stats)
            if ds is not None:
                ds["capture"] = False
        # ── 도리도리 진입 ──
        if dori is not None and ds is None and y >= next_cell and y < COURSE_L and spin_guard:
            r_sw = math.hypot(HALF_L, HALF_W) + sweep_margin
            obs_ = [(ox, oy, rr) for (ox, oy, rr) in yellow] + [(tt["mu"][0], tt["mu"][1], LOW_R) for tt in lm.confirmed()]
            if any(math.hypot(ox - x, oy - y) - rr < r_sw for (ox, oy, rr) in obs_):
                next_cell = y + 0.5                     # 반 칸 더 가서 다시 시도
                n_skip += 1
        if dori is not None and ds is None and y >= next_cell and y < COURSE_L:
            ds = dict(i=0, t0=t, ref=yaw_meas, capture=False,
                      seq=[("stop", 0.0), ("hold", 0.0), ("spin", dori), ("hold", dori),
                           ("spin", 0.0), ("spin", -dori), ("hold", -dori), ("spin", 0.0)])
            next_cell = math.floor(y) + 1.0
            n_dori += 1
        # ── 명령 결정 ──
        cand = INVALID
        if ds is not None:
            kind, tgt = ds["seq"][ds["i"]]
            err = wrap180((ds["ref"] - tgt) - yaw_meas)      # 목표 나침반 방위 = ref − rel(좌 양수)
            done = False
            if kind == "stop":
                cand = STOP_HOLD; done = abs(v) < 0.02 and abs(w) < 1.0 and t - ds["t0"] > 0.3
            elif kind == "hold":
                cand = STOP_HOLD; done = t - ds["t0"] > 0.5
                if done:
                    ds["capture"] = True
            else:
                if abs(err) < 4.0:
                    cand = STOP_HOLD; done = abs(w) < 2.0
                else:
                    cand = SPIN_R if err > 0 else SPIN_L     # 나침반 증가 = 우선회
                    spin_time += DT
                    if guard_cont:                            # 회전 중 연속 가드: 지금 위치 기준 회전 원 검사
                        r_sw = math.hypot(HALF_L, HALF_W) + sweep_margin
                        obs_ = [(ox, oy, rr) for (ox, oy, rr) in yellow] + \
                               [(tt["mu"][0], tt["mu"][1], LOW_R) for tt in lm.confirmed()]
                        if any(math.hypot(ox - x, oy - y) - rr < r_sw for (ox, oy, rr) in obs_):
                            cand = STOP_HOLD; ds = None; n_skip += 1
            if done and ds is not None:
                ds["i"] += 1; ds["t0"] = t
                if ds["i"] >= len(ds["seq"]):
                    ds = None
        elif plan:
            if t - last_plan > 0.5:
                path = plan_path(lm, yellow, width, x, y, goal, infl_low, K.CLEARANCE if hasattr(K, "CLEARANCE") else 0.25)
                last_plan = t
            if path:
                look = None
                for (px_, py_) in path:
                    if math.hypot(px_ - x, py_ - y) >= 1.2:
                        look = (px_, py_); break
                if look is None:
                    look = path[-1]
                rel = wrap180(yaw_meas - bearing(x, y, *look))
                rel = max(-75.0, min(75.0, rel))
                cand = rel % 360.0
        sd.candidate_callback(Float32(cand))
        # ── 스캔: LiDAR(노랑만) + 가상 장애물 min 병합 ──
        scan = lidar_world.lidar_scan(x, y, psi, max_range=20.0)
        if use_cam:
            vs = virtual_scan(lm, x, y, psi, infl_low, wall_pts)
            if vs is not None:
                scan = np.minimum(scan, vs)
        sd.scan_callback(Scan(scan.tolist()))
        sd.yaw_error_callback(Float32((bearing(x, y, *goal) - yaw_meas) % 360.0))
        sd.watchdog_cb(); sd.control_cb()
        desired = sd.pubs["/desired_angle"].last.data
        mc.angle_callback(Float32(desired))
        if sd.pubs["/obstacle_distance_array"].last is not None:
            mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
        mc.failsafe_callback(sd.pubs["/failsafe_level"].last)
        mc.timer_callback()
        code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
        TR, TL = 1500 - pr, 1500 - pl
        v_cmd, w_cmd = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
        if tau > 0:
            v += (v_cmd - v) * DT / tau; w += (w_cmd - w) * DT / tau
        else:
            v, w = v_cmd, w_cmd
        psi = (psi + w * DT) % 360.0
        x += (v * math.sin(math.radians(psi)) + cvx) * DT
        y += (v * math.cos(math.radians(psi)) + cvy) * DT
        t += DT; Clock.t = t
        spinning = cand in (SPIN_R, SPIN_L)
        if 0.0 < y < COURSE_L and abs(x) > width / 2:
            out_ch = True
        for i, (ox, oy, r) in enumerate(low):
            c = hull_clear(x, y, psi, ox, oy, r)
            min_clear_low = min(min_clear_low, c)
            if c < 0 and not hit_flag.get(("l", i)):
                hit_flag[("l", i)] = True; hits_low += 1
                events.append(("L", round(t, 1), i, round(x, 2), round(y, 2), round(psi), cand, None))
                if spinning: hits_spin += 1
            if seen_before[i] is None and oy - y < 0.5:
                tr = lm.confirmed()
                seen_before[i] = any(math.hypot(tt["mu"][0] - ox, tt["mu"][1] - oy) < 0.6 for tt in tr)
        for i, (ox, oy, r) in enumerate(yellow):
            if abs(oy - y) > 2.0:
                continue
            if hull_clear(x, y, psi, ox, oy, r) < 0 and not hit_flag.get(("y", i)):
                hit_flag[("y", i)] = True; hits_y += 1
                sn, cs = math.sin(math.radians(psi)), math.cos(math.radians(psi))
                events.append(("Y", round(t, 1), i, round(x, 2), round(y, 2), round(psi), cand,
                               round((ox - x) * sn + (oy - y) * cs, 2)))
                if spinning: hits_spin += 1
    errs = []
    for (ox, oy, _) in low:
        tr = lm.confirmed()
        if tr:
            d = min(math.hypot(tt["mu"][0] - ox, tt["mu"][1] - oy) for tt in tr)
            if d < 0.6: errs.append(d)
    phantom = sum(1 for tt in lm.confirmed()
                  if min(math.hypot(tt["mu"][0] - ox, tt["mu"][1] - oy) for (ox, oy, _) in low) > 0.6)
    missed = sum(1 for v_ in seen_before.values() if v_ is False)
    return dict(ok=ok, t=t, hits_low=hits_low, hits_y=hits_y, hits_spin=hits_spin,
                min_clear_low=min_clear_low, map_err=(float(np.mean(errs)) if errs else float("nan")),
                missed=missed, phantom=phantom, n_dori=n_dori, spin_time=spin_time, events=events, n_skip=n_skip, out=out_ch)


sys.path.insert(0, os.path.join(ROOT, "src", "ship_avoid"))
from ship_avoid.core import AvoidCore, AvoidParams, hull_clear as core_hull_clear  # noqa: E402,F401
from ship_avoid.lidar_buoys import cluster_buoys                                   # noqa: E402


def run_core(seed, width, tau=0.0, current=0.05, use_rs=True, max_t=400.0, cam_dmin=None, params=None):
    """S5 core — src/ship_avoid/core.py (실제 미션 노드 두뇌) 를 그대로 돌린다.
    칸 = 낮은 빨간 장애물 하나 통과. 가상 장애물(낮은 장애물+노란부표 벽)은 스캔에 min 병합(계약 단위 ①)."""
    rng = random.Random(seed * 31 + 4242)
    yellow, low = build_world(seed, width)
    lidar_world = FastWorld(yellow)
    goal = (0.0, COURSE_L + 1.5)
    x, y, psi = 0.0, -1.5, 0.0
    v = w = 0.0
    cur_dir = rng.uniform(0, 360)
    cvx, cvy = current * math.sin(math.radians(cur_dir)), current * math.cos(math.radians(cur_dir))
    yaw_bias = rng.gauss(0, 1.0)
    dmin = CAM_DMIN if cam_dmin is None else cam_dmin
    Clock.t = 0.0
    sd = SD.ShipDirection(); mc = MC.MotorController()
    sd.wp_mode_cb(Int32(AVOID_MODE))
    core = AvoidCore(params or AvoidParams())
    hits_low = hits_y = hits_spin = 0
    hit_flag = {}
    min_clear_low = math.inf
    out_ch = False
    t = 0.0; ok = False
    activated = False
    events = []
    while t < max_t:
        if y > COURSE_L + 1.0:
            ok = True; break
        yaw_m = (psi + yaw_bias + rng.gauss(0, 0.3)) % 360.0
        px, py = x + rng.gauss(0, SIG_POS), y + rng.gauss(0, SIG_POS)
        core.set_pose(t, px, py, yaw_m, yaw_rate=w + rng.gauss(0, 0.5))
        moving = abs(v) > 0.05 or abs(w) > 5.0
        core.set_pitch(rng.gauss(0, 2.0 if moving else 0.5))       # 측정 피치 = 참값 0 + 잡음
        raw = lidar_world.lidar_scan(x, y, psi, max_range=20.0)
        bl = cluster_buoys(raw.tolist(), K.SCAN_ANGLE_MIN_DEG, K.SCAN_INC_DEG, buoy_r=YELLOW_R)
        core.set_buoys([(px + d * math.sin(math.radians(yaw_m - a)), py + d * math.cos(math.radians(yaw_m - a)))
                        for a, d in bl])
        if not activated:
            activated = core.activate(t)
        # 카메라 → dets
        dets = []
        p_det = 0.9 if abs(w) < 5.0 else 0.5
        for (ox, oy, _), is_low in [(o, True) for o in low] + [(o, False) for o in yellow]:
            d = math.hypot(ox - x, oy - y)
            rel = wrap180(psi - bearing(x, y, ox, oy))
            if abs(rel) > CAM_HFOV / 2 or d > CAM_MAX or d < dmin or rng.random() > p_det:
                continue
            red = is_low if rng.random() > P_MISCLS else (not is_low)
            v_norm = math.tan(math.atan2(CAM_H, d))                  # 참 피치 0: 측정 피치 잡음은 core 가 더함
            rr = float("nan")
            if use_rs and abs(rel) <= RS_HFOV / 2 and d <= RS_MAX:
                rr = d + rng.gauss(0, 0.0047 * d * d * 1.5)
            dets.append((0 if red else 2, rel + rng.gauss(0, SIG_BEAR) + w * CAM_LAT, v_norm, rr, 1.0))
        core.add_detections(t, px, py, yaw_m, dets)
        cand = core.step(t)
        if cand is None:
            cand = CANDIDATE_INVALID_SIM
        sd.candidate_callback(Float32(cand))
        scan = raw
        vo = core.virtual_obstacles()
        if vo:
            scan = np.minimum(scan, FastWorld(vo).lidar_scan(x, y, psi, max_range=20.0))
        sd.scan_callback(Scan(scan.tolist()))
        sd.yaw_error_callback(Float32((bearing(x, y, *goal) - yaw_m) % 360.0))
        sd.watchdog_cb(); sd.control_cb()
        desired = sd.pubs["/desired_angle"].last.data
        mc.angle_callback(Float32(desired))
        if sd.pubs["/obstacle_distance_array"].last is not None:
            mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
        mc.failsafe_callback(sd.pubs["/failsafe_level"].last)
        mc.timer_callback()
        code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
        TR, TL = 1500 - pr, 1500 - pl
        v_cmd, w_cmd = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
        if tau > 0:
            v += (v_cmd - v) * DT / tau; w += (w_cmd - w) * DT / tau
        else:
            v, w = v_cmd, w_cmd
        psi = (psi + w * DT) % 360.0
        x += (v * math.sin(math.radians(psi)) + cvx) * DT
        y += (v * math.cos(math.radians(psi)) + cvy) * DT
        t += DT; Clock.t = t
        spinning = cand in (SPIN_R, SPIN_L)
        if 0.0 < y < COURSE_L and abs(x) > width / 2:
            out_ch = True
        for i, (ox, oy, r) in enumerate(low):
            c = hull_clear(x, y, psi, ox, oy, r)
            min_clear_low = min(min_clear_low, c)
            if c < 0 and not hit_flag.get(("l", i)):
                hit_flag[("l", i)] = True; hits_low += 1; hits_spin += int(spinning)
                events.append(("L", round(t, 1), i, round(x, 2), round(y, 2), round(psi), cand, core.status()))
        for i, (ox, oy, r) in enumerate(yellow):
            if abs(oy - y) <= 2.0 and hull_clear(x, y, psi, ox, oy, r) < 0 and not hit_flag.get(("y", i)):
                hit_flag[("y", i)] = True; hits_y += 1; hits_spin += int(spinning)
                events.append(("Y", round(t, 1), i, round(x, 2), round(y, 2), round(psi), cand, core.status()))
    conf = core.map.confirmed()
    errs = [min(math.hypot(tr.x - ox, tr.y - oy) for tr in conf) for (ox, oy, _) in low if conf]
    errs = [e for e in errs if e < 0.6]
    phantom = sum(1 for tr in conf if min(math.hypot(tr.x - ox, tr.y - oy) for (ox, oy, _) in low) > 0.6)
    return dict(ok=ok, t=t, hits_low=hits_low, hits_y=hits_y, hits_spin=hits_spin, min_clear_low=min_clear_low,
                map_err=(float(np.mean(errs)) if errs else float("nan")), missed=len(low) - len(errs),
                phantom=phantom, n_dori=core.n_dori, spin_time=0.0, events=events, n_skip=core.n_skip,
                out=out_ch, cells=core.cells)


CANDIDATE_INVALID_SIM = INVALID
STRATS = ["lidar_only", "cam_fwd", "dori90", "dori60", "dori60_plan", "core"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--widths", type=float, nargs="+", default=[3.0, 4.0])
    ap.add_argument("--taus", type=float, nargs="+", default=[0.0, 0.5])
    ap.add_argument("--strats", nargs="+", default=STRATS)
    ap.add_argument("--no-rs", action="store_true", help="D455 뎁스 없음(단안 지평면만)")
    ap.add_argument("--current", type=float, default=0.05)
    ap.add_argument("--infl-low", type=float, default=0.20)
    ap.add_argument("--tag", default="")
    ap.add_argument("--v2", action="store_true", help="대책 켬: 색 다수결 + 통로 벽 + 회전 여유 가드")
    ap.add_argument("--v3", action="store_true", help="v2 + 회전 중 연속 가드 + 회전 여유 0.25 m")
    a = ap.parse_args()
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    out_csv = os.path.join(HERE, "out", f"doridori_sim{a.tag}.csv")
    rows = []
    print(f"도리도리 회피 시뮬 — 시드 {a.seeds}, 폭 {a.widths}, τ {a.taus}, D455 {'없음' if a.no_rs else '있음'}, "
          f"{'대책 v3' if a.v3 else ('대책 v2' if a.v2 else '대책 없음 v1')}, 조류 {a.current} m/s, 낮은장애물 추가여유 {a.infl_low} m, 카메라 근거리 사각 {CAM_DMIN:.2f} m")
    print(f"{'폭':>4} {'τ':>4} {'전략':<12} {'완주':>5} {'낮은접촉':>8} {'노랑접촉':>8} {'회전중접촉':>10} "
          f"{'무접촉':>6} {'시간중앙':>8} {'>120s':>6} {'최소여유중앙':>12} {'지도오차':>8} {'놓침':>4} {'유령':>4} {'이탈':>4} {'스킵':>4}")
    for W in a.widths:
        for tau in a.taus:
            for s in a.strats:
                R = []
                for seed in range(1, a.seeds + 1):
                    r = run_core(seed, W, tau=tau, current=a.current, use_rs=not a.no_rs) if s == "core" else run(seed, W, s, tau=tau, current=a.current, use_rs=not a.no_rs, infl_low=a.infl_low,
                            vote=a.v2 or a.v3, wall=a.v2 or a.v3, spin_guard=a.v2 or a.v3,
                            guard_cont=a.v3, sweep_margin=0.25 if a.v3 else 0.10)
                    r.update(width=W, tau=tau, strat=s, seed=seed); R.append(r); rows.append(r)
                n = len(R)
                ts = sorted(r["t"] for r in R if r["ok"])
                tm = ts[len(ts) // 2] if ts else float("nan")
                mcl = sorted(r["min_clear_low"] for r in R)[n // 2]
                me = [r["map_err"] for r in R if not math.isnan(r["map_err"])]
                print(f"{W:4.1f} {tau:4.1f} {s:<12} {sum(r['ok'] for r in R):>3}/{n:<2}"
                      f" {sum(r['hits_low'] for r in R) / n:8.2f} {sum(r['hits_y'] for r in R) / n:8.2f}"
                      f" {sum(r['hits_spin'] for r in R) / n:10.2f} {sum(1 for r in R if r['hits_low'] + r['hits_y'] == 0):>6}"
                      f" {tm:8.0f} {sum(1 for r in R if r['t'] > 120):>6} {mcl:12.2f}"
                      f" {(np.mean(me) if me else float('nan')):8.2f} {sum(r['missed'] for r in R):>4} {sum(r['phantom'] for r in R):>4}"
                      f" {sum(r['out'] for r in R):>4} {sum(r['n_skip'] for r in R):>4}")
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        keys = ["width", "tau", "strat", "seed", "ok", "t", "hits_low", "hits_y", "hits_spin", "min_clear_low",
                "map_err", "missed", "phantom", "n_dori", "spin_time", "n_skip", "out"]
        wr = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore"); wr.writeheader(); wr.writerows(rows)
    print(f"CSV → {out_csv}")


if __name__ == "__main__":
    main()

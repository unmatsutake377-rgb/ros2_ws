#!/usr/bin/env python3
"""토너먼트 2D 시뮬 — 실제 노드 코드(north_goal_angle·ship_goal_angle·ship_direction·motor_control)를 rosstub 으로
돌리고, 대본 궤적으로 움직이는 상대 배(1.7×0.55 m 직사각형)를 LiDAR 에 넣는다. 계약서_토너먼트분리_20261109 §1 T2·§6 시뮬.

실행
  python3 tools/sim/tournament_sim.py --scenario side_by_side --seeds 5 --hull A
  python3 tools/sim/tournament_sim.py --scenario all --seeds 20 --hull B --csv tools/sim/out/tournament_B.csv
  python3 tools/sim/tournament_sim.py --scenario all --seeds 20 --param ship_direction.side_lock_enabled=true \
      --csv tools/sim/out/tournament_A_sidelock.csv          # 옆 잠금 끔/켬 비교 — 같은 시드면 같은 세계
  --param 노드.키=값 (반복). 노드가 선언하지 않는 키(예: 옆 잠금 구현 전 side_lock_*)는 시작 때 경고만 내고 진행
  (rosstub 은 선언 안 된 키를 그냥 무시한다 → 그 값은 효과 없음).

파라미터 (실제 launch 와 같은 순서 — 뒤가 이긴다)
  north_goal_angle ← north_goal_angle.yaml → north_goal_angle_tournament.yaml → {course: tournament, waypoints_file}
                     → geofence_polygon = 시뮬 경기장 사각형(로컬 → 위경도, full_course_sim.Geo) → --param
  ship_goal_angle  ← ship_goal_angle.yaml → --param
  ship_direction   ← ship_direction.yaml → ship_direction_tournament.yaml → {dwa_hull: --hull} → --param
  motor_control    ← motor_control.yaml → motor_control_tournament.yaml → --param
  미션 노드(gate·back·turn·dock)·검출기는 mode 8 에서 침묵이라 띄우지 않는다(계약서 §0).
  웨이포인트 파일(시뮬이 임시 폴더에 생성): 전부 mode 8, dwell 0.5 — T0 = 우리 레인 출발선 + 1 m, T1 = 결승선 + 3 m.

세계 (로컬 x = 동, y = 북 = 진행 방향, 원점 = 경기장 아래 변 가운데. 위경도 원점 = waypoints_tournament.yaml T0)
  · 경기장 x ∈ [−6, 6], y ∈ [0, 30] (--arena-len/--arena-wid). 경계 폴리곤 = 지오펜스(north geofence_polygon 으로 켬).
    --fence-inset d: 지오펜스 좌우 변만 d m 안쪽(현장 대책 시험). 경계 이탈 지표는 늘 실제 경기장 기준.
  · 출발선 y = 2.0, 결승선 y = 26.0 (--start-y/--finish-y). 기본 결승선을 26 m 가 아니라 24 m 거리로 둔 이유:
    결승 WP(결승선 + 3 m)가 경기장 안에 있어야 해서(30 m 경기장, 경계 2 m 하드 정지 — 계약서 §5-3).
  · 우리 배: 시드별 왼쪽/오른쪽 레인(x = ±2.0), 출발선 1 m 뒤, 헤딩 0 ± 3°, 정지 상태. 경기장 가운데 쪽이 상대 레인.
  · 장애물 부표(반경 0.2 m) 6~10개: 하나는 side_by_side 상대 레인 위(y 10~17, 모든 시나리오에 같이 있음),
    2~3개는 우리 레인 위(레인 중심 ±0.8 m — 회피가 실제로 일어나게), 나머지는 x ∈ [−5.2, 5.2] 고르게.
    모두 y ∈ [출발선+3, 결승선−1.5], 서로 1.5 m 이상, 우리 출발점 3 m 밖.
    세계(부표·레인)는 시드만으로 정해진다 → 같은 시드면 시나리오·파라미터가 달라도 같은 부표 배치.
  · 상대 배(대본 궤적, 우리 배에 반응 없음, 부표와 충돌 물리 없음 — 통과한다). v_ref = 우리 배 순항 속도
    (fossen 이면 boat3dof.steady_state(hull, base_pwm), legacy 면 K_V·(1500−base_pwm)). 가속은 1차(τ 2 s).
      side_by_side : 옆 레인(중심 간격 1.5~2.5 m) 같은 출발선, 속도 v_ref×(0.95~1.05). 레인 위 부표를 우리 쪽으로
                     0.8~1.3 m 비켜 지나감(y ±4 m 구간 올림 코사인) → 그 구간에서 우리 쪽으로 붙는다.
      slow_ahead   : 우리 레인(±0.3 m) 4~7 m 앞에서 v_ref×(0.4~0.6) → 우리가 추월.
      fast_behind  : 옆(1.2~2.0 m) 같은 출발선에서 3~5 s 늦게 출발, v_ref×(1.5~1.8) → 뒤에서 와서 옆으로 추월.
      cut_in       : 옆(2.2~3.0 m), 0~1 m 앞에서 v_ref×(1.2~1.35). y 7~12 m 에서 25~35° 로 우리 레인(±0.3 m)에 끼어듦.
      none         : 상대 배 없음(기준).
  · LiDAR: full_course_sim.lidar() 그대로(360°/0.5°, 20 m, 잡음 없음, 위치 = 배 중심). 부표 원 + 상대 배 4변 선분.

지표 (시뮬 정의, 실측 아님) — '경기 중' = 출발 ~ 결승선 통과(못 하면 끝까지). '코스 끝' = north 가 마지막 WP 완료 → 50000.
  결승선 통과 뒤 코스 끝까지 걸린 시간(finish_to_end_s)도 적는다(결승 WP 반경에 못 들고 맴돌면 길어진다).
  · 장애물 접촉 = 직사각형 선체(full_course_sim.rect_hull_dist, A 1.66×0.55 / B 1.77×0.522)와 부표 원이 겹치기 시작한 횟수.
  · 상대 접촉 = 두 직사각형이 겹치기 시작한 횟수(0.1 m 멀어져야 다음 회).
  · 책임 추정(🚨 단순 휴리스틱 — 규정 판정 아님): 접촉 2 s 전 상대 위치를 우리 선수 방향에 투영해 관계를 정한다
    (> +1.7 m = 상대가 앞 → 우리가 추월 중, < −1.7 m = 상대가 추월 중; ±1.7 m 안이어도 2 s 동안 앞뒤 상대속도가
    0.3 m/s 넘으면 빠른 쪽이 추월 중, 아니면 나란히). 2 s 동안 침로 변화량 = 틱별 |Δψ| 합.
    나란히 → 침로를 더 바꾼 쪽(차이 5° 미만이면 '불명'). 추월 → 뒤(추월하는) 배. 단 앞 배가 15° 이상, 뒤 배보다 10° 넘게 더
    바꿨으면 앞 배(진로 방해). (규정: 추월 접촉 = 뒤 배 / 선두 진로 방해 = 선두 / 나란히 = 침로 바꾼 쪽)
  · 경계 이탈 = 선체 네 모서리의 경기장 사각형 밖 최대 거리(m). 경기 중·전체 따로.
  · 결승 = 배 중심이 결승선을 북쪽으로 넘은 시각. 결승 뒤 정지 = 코스 끝(50000) 뒤 --post(10) s 더 돌려
    마지막 위치의 결승선 너머 거리, 코스 끝 이후 이동 거리, 속도 < 0.05 m/s 가 된 시각.
  · STOP_HOLD 시간 = /desired_angle ≥ 50000 인 시간(경기 중 / 코스 끝 뒤 따로).
  · 기록 = 결승 시간 + 15 s × (장애물 접촉 + 상대 접촉, 경기 중). 결승 못 하면 없음.
    (record_fault_s = 상대 접촉 중 '우리 책임' 추정만 더한 값, 참고용)
  · 상대 배가 /scan 에 보이나: 상대 배(중심 거리) 6 m 안 틱 중 상대 배 선분에 맞은 광선이 있는 틱 비율(전체 360° / ship_direction
    이 읽는 정면 ±80°), 상대 광선 최소 거리 vs 배 중심→상대 직사각형 실제 최소 거리.
  · 로그 수(rosstub 은 throttle 무시 → 틱 수에 가깝다): ship_direction 'DWA 가능한 후보 없음'(폴백), '🚧🛑 경계'(하드 정지),
    '옆 잠금' 문구(구현 뒤 생기면).

한계: 상대 배는 대본 궤적(반응·회피 없음, 부표 통과), 운동·추력 계수는 가정값(boat3dof), 충돌 물리 없음(겹쳐도 통과,
    횟수만 셈), LiDAR 잡음·가림·물보라 없음, GPS σ 3 cm·IMU 참값, 책임 추정은 휴리스틱.
"""
import argparse
import csv
import math
import os
import random
import statistics
import sys
import tempfile
import time as _walltime
import traceback
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import full_course_sim as F                      # noqa: E402  rosstub 설치·실제 노드 로드·lidar/Geo/rect_hull_dist 재사용
import rosstub                                   # noqa: E402
import yaml                                      # noqa: E402
from rosstub import Clock                        # noqa: E402

SRC, OUT_DIR, K = F.SRC, F.OUT_DIR, F.K
Float64, Int32 = F.Float64, F.Int32
DT = F.DT
STOP_HOLD = 50000.0

# ── 선언된 파라미터 기록 (rosstub 은 선언 안 된 키를 조용히 버린다 → 경고용). rosstub.py 는 안 고치고 런타임 감싸기 ──
_orig_declare = rosstub.Node.declare_parameter


def _declare_recording(self, name, default=None):
    self.__dict__.setdefault("_declared", set()).add(name)
    return _orig_declare(self, name, default)


rosstub.Node.declare_parameter = _declare_recording

# ───────────────────────── 파라미터 파일 (launch 와 같은 순서) ─────────────────────────
PARAM_FILES = {
    "north_goal_angle": ["north_goal_angle/config/north_goal_angle.yaml",
                         "north_goal_angle/config/north_goal_angle_tournament.yaml"],
    "ship_goal_angle": ["ship_goal_angle/config/ship_goal_angle.yaml"],
    "ship_direction": ["ship_direction/config/ship_direction.yaml",
                       "ship_direction/config/ship_direction_tournament.yaml"],
    "motor_control": ["motor_control/config/motor_control.yaml",
                      "motor_control/config/motor_control_tournament.yaml"],
}
WP_TEMPLATE = os.path.join(SRC, "north_goal_angle", "config", "waypoints_tournament.yaml")


def _yaml_params(rel, node):
    with open(os.path.join(SRC, rel), encoding="utf-8") as f:
        d = yaml.safe_load(f) or {}
    return dict((d.get("/" + node) or {}).get("ros__parameters") or {})


def course_overrides():
    """{노드: 코스 yaml(공통 뒤에 붙는 것)이 덮어쓰는 키} — 선언 여부 경고용."""
    out = {}
    for node, files in PARAM_FILES.items():
        for rel in files[1:]:
            out.setdefault(node, {}).update(_yaml_params(rel, node))
    return out


def build_params(wp_file, hull, poly_ll, overrides):
    P = {}
    for node, files in PARAM_FILES.items():
        p = {}
        for rel in files:
            p.update(_yaml_params(rel, node))
        P[node] = p
    P["north_goal_angle"].update(course="tournament", waypoints_file=wp_file)
    P["north_goal_angle"]["geofence_polygon"] = list(poly_ll)
    P["ship_direction"]["dwa_hull"] = hull
    for node, kv in (overrides or {}).items():
        P.setdefault(node, {}).update(kv)
    rosstub.PARAMS.clear()
    rosstub.PARAMS.update({k: dict(v) for k, v in P.items()})
    return P


# ───────────────────────── 기하 ─────────────────────────
GEO0 = (35.1862375, 128.5655118)          # waypoints_tournament.yaml T0 (자리표시값) — 위경도 변환 원점만
OPP_L, OPP_B = 1.7, 0.55
LANE_X = 2.0
SCENARIOS = ["none", "side_by_side", "slow_ahead", "fast_behind", "cut_in"]
NEAR_OPP_M = 6.0
OVERTAKE_VREL = 0.3                        # 책임 휴리스틱: 겹친 채 이보다 빠르게 앞질러 가면 '추월 중'(m/s)
FRONT_MASK = np.abs(F.SCAN_REL) <= 80.0   # ship_direction 이 읽는 정면 ±80° (0~160°)


def rect_corners(x, y, psi, hl, hb):
    p = math.radians(psi); fx, fy = math.sin(p), math.cos(p); lx, ly = -fy, fx
    return [np.array([x + sf * hl * fx + sl * hb * lx, y + sf * hl * fy + sl * hb * ly])
            for sf, sl in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def rect_rect_dist(ca, cb):
    """두 볼록 사각형(꼭짓점 4개, 순서대로) 최단 거리. 겹치면 0."""
    ea = [(ca[i], ca[(i + 1) % 4]) for i in range(4)]
    eb = [(cb[i], cb[(i + 1) % 4]) for i in range(4)]
    best = float("inf")
    for p1, p2 in ea:
        for q1, q2 in eb:
            d = F.seg_seg_dist(p1, p2, q1, q2)
            if d <= 0.0:
                return 0.0
            best = min(best, d)
    for poly, pt in ((ca, cb[0]), (cb, ca[0])):       # 한쪽이 다른 쪽 안에 통째로
        s = [(poly[(i + 1) % 4][0] - poly[i][0]) * (pt[1] - poly[i][1])
             - (poly[(i + 1) % 4][1] - poly[i][1]) * (pt[0] - poly[i][0]) for i in range(4)]
        if all(v >= 0 for v in s) or all(v <= 0 for v in s):
            return 0.0
    return best


def outside_dist(px, py, aw, al):
    return math.hypot(max(0.0, abs(px) - aw / 2.0), max(0.0, py - al, -py))


class OppPath:
    """y 로 매개된 경로 x(y) 를 촘촘히 샘플 → 호 길이 s(t) 로 위치·헤딩."""

    def __init__(self, xs, ys, v, t_delay=0.0, tau=2.0):
        self.P = np.column_stack([xs, ys])
        seg = np.linalg.norm(np.diff(self.P, axis=0), axis=1)
        self.S = np.concatenate([[0.0], np.cumsum(seg)])
        self.v, self.td, self.tau = v, t_delay, tau

    def s_at(self, t):
        tt = max(0.0, t - self.td)
        return self.v * (tt - self.tau * (1.0 - math.exp(-tt / self.tau)))

    def pose(self, t):
        s = min(self.s_at(t), self.S[-1] - 1e-6)
        i = int(np.searchsorted(self.S, s, side="right") - 1)
        i = max(0, min(i, len(self.S) - 2))
        a, b = self.P[i], self.P[i + 1]
        r = (s - self.S[i]) / max(self.S[i + 1] - self.S[i], 1e-9)
        p = a + (b - a) * r
        hdg = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.0
        return float(p[0]), float(p[1]), hdg


def build_world(seed, g):
    rng = random.Random(seed)
    side = rng.choice((-1.0, 1.0))
    our_x = side * LANE_X
    toward_opp = -side                                # 경기장 가운데 쪽이 상대 레인
    gap_sbs = rng.uniform(1.5, 2.5)
    yc = rng.uniform(10.0, 17.0)
    start = np.array([our_x, g["start_y"] - 1.0])
    psi0 = rng.uniform(-3.0, 3.0) % 360.0
    buoys = [(our_x + toward_opp * gap_sbs, yc)]
    n = rng.randint(6, 10); n_lane = 1 + rng.randint(2, 3); tries = 0
    while len(buoys) < n and tries < 5000:
        tries += 1
        if len(buoys) < n_lane:                       # 2~3개는 우리 레인 위(±0.8 m) — 회피가 실제로 일어나게
            px_ = our_x + rng.uniform(-0.8, 0.8)
        else:
            px_ = rng.uniform(-g["arena_w"] / 2 + 0.8, g["arena_w"] / 2 - 0.8)
        p = (px_, rng.uniform(g["start_y"] + 3.0, g["finish_y"] - 1.5))
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) >= 1.5 for q in buoys) and \
           math.hypot(p[0] - start[0], p[1] - start[1]) > 3.0:
            buoys.append(p)
    arr = np.array(buoys)
    return dict(seed=seed, side=side, our_x=our_x, toward_opp=toward_opp, gap_sbs=gap_sbs, yc=yc,
                start=start, psi0=psi0, buoys=buoys, bx=arr[:, 0], by=arr[:, 1])


def build_opp(W, scen, seed, g, v_ref):
    if scen == "none":
        return None, {}
    r = random.Random(seed * 1000 + SCENARIOS.index(scen))
    ys = np.arange(g["start_y"] - 1.0, g["arena_len"] + 12.0, 0.1)
    ox, to = W["our_x"], W["toward_opp"]
    y0 = g["start_y"] - 1.0
    if scen == "side_by_side":
        lane = ox + to * W["gap_sbs"]; A = r.uniform(0.8, 1.3); yc = W["yc"]
        bump = np.where(np.abs(ys - yc) < 4.0, 0.5 * (1 + np.cos(np.pi * (ys - yc) / 4.0)), 0.0)
        xs = lane - to * A * bump                     # 레인 위 부표를 우리 쪽으로 비켜 감
        v = v_ref * r.uniform(0.95, 1.05); info = dict(opp_gap=W["gap_sbs"], opp_bump=A, opp_v=v)
        return OppPath(xs, ys, v), info
    if scen == "slow_ahead":
        lane = ox + r.uniform(-0.3, 0.3); ahead = r.uniform(4.0, 7.0)
        ys2 = ys[ys >= y0 + ahead]; v = v_ref * r.uniform(0.4, 0.6)
        return OppPath(np.full_like(ys2, lane), ys2, v), dict(opp_gap=lane - ox, opp_ahead=ahead, opp_v=v)
    if scen == "fast_behind":
        gap = r.uniform(1.2, 2.0); td = r.uniform(3.0, 5.0); v = v_ref * r.uniform(1.5, 1.8)
        return OppPath(np.full_like(ys, ox + to * gap), ys, v, t_delay=td), dict(opp_gap=gap, opp_delay=td, opp_v=v)
    if scen == "cut_in":
        gap = r.uniform(2.2, 3.0); ahead = r.uniform(0.0, 1.0); v = v_ref * r.uniform(1.2, 1.35)
        y_cut = r.uniform(7.0, 12.0); ang = r.uniform(25.0, 35.0); x_end = ox + r.uniform(-0.3, 0.3)
        x_start = ox + to * gap
        dy_cut = abs(x_start - x_end) / math.tan(math.radians(ang))
        ys2 = ys[ys >= y0 + ahead]
        frac = np.clip((ys2 - y_cut) / dy_cut, 0.0, 1.0)
        xs = x_start + (x_end - x_start) * frac
        return OppPath(xs, ys2, v), dict(opp_gap=gap, opp_ahead=ahead, opp_v=v, opp_ycut=y_cut, opp_ang=ang)
    raise ValueError(scen)


def write_wp_file(W, g, geo):
    pts = [("T0 출발 근처", W["our_x"], g["start_y"] + 1.0),
           ("T1 결승선 너머", W["our_x"], g["finish_y"] + g["wp_beyond"])]
    d = os.path.join(tempfile.gettempdir(), "ssf_tournament_sim")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f"wp_seed{W['seed']}_pid{os.getpid()}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        f.write("waypoints:\n")
        for label, x, y in pts:
            lat, lon = geo.ll(x, y)
            f.write(f'  - {{구역: "{label}", lat: {lat:.9f}, lon: {lon:.9f}, mode: 8, dwell: 0.5}}\n')
    return path


_VREF = {}


def v_ref_for(dyn, hull, base_pwm):
    key = (dyn, hull, base_pwm)
    if key not in _VREF:
        if dyn == "fossen":
            import boat3dof
            _VREF[key] = float(boat3dof.steady_state(hull, base_pwm, base_pwm)[0])
        else:
            _VREF[key] = K.K_V * (1500 - base_pwm)
    return _VREF[key]


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def blame(hist, now):
    """접촉 직전 2 s 휴리스틱 → (관계, 우리 침로변화°, 상대 침로변화°, 책임 '우리'/'상대'/'불명')."""
    h = list(hist)
    d_our = sum(abs(wrap(b[3] - a[3])) for a, b in zip(h, h[1:]))
    d_opp = sum(abs(wrap(b[6] - a[6])) for a, b in zip(h, h[1:]))

    def along(e):                                    # 상대 위치의 우리 선수 방향 성분(m, + = 상대가 앞)
        _, x_, y_, p_, ox_, oy_, _ = e
        return (ox_ - x_) * math.sin(math.radians(p_)) + (oy_ - y_) * math.cos(math.radians(p_))
    a0, a1 = along(h[0]), along(h[-1])
    v_rel = (a1 - a0) / max(h[-1][0] - h[0][0], DT)   # + = 상대가 우리보다 앞으로 빨리 감
    if a0 > OPP_L or (abs(a0) <= OPP_L and v_rel < -OVERTAKE_VREL):
        rel, rear, lead, d_rear, d_lead = "우리추월", "우리", "상대", d_our, d_opp
    elif a0 < -OPP_L or (abs(a0) <= OPP_L and v_rel > OVERTAKE_VREL):
        rel, rear, lead, d_rear, d_lead = "상대추월", "상대", "우리", d_opp, d_our
    else:
        rel = "나란히"
        who = "불명" if abs(d_our - d_opp) < 5.0 else ("우리" if d_our > d_opp else "상대")
        return rel, d_our, d_opp, who
    who = lead if (d_lead >= 15.0 and d_lead > d_rear + 10.0) else rear
    return rel, d_our, d_opp, who


# ───────────────────────── 한 런 ─────────────────────────
def run(scenario, seed, hull="A", dyn="fossen", overrides=None, geom=None, t_max=150.0, post=10.0,
        gps_sigma=0.03, record=False):
    g = dict(DEFAULT_GEOM, **(geom or {}))
    geo = F.Geo(*GEO0)
    W = build_world(seed, g)
    aw, al = g["arena_w"], g["arena_len"]
    fi = g["fence_inset"]                 # 지오펜스 폴리곤을 좌우 변에서만 안쪽으로(위·아래 변 = 경기장). 이탈 판정은 실제 경기장
    poly_xy = [(-aw / 2 + fi, 0.0), (aw / 2 - fi, 0.0), (aw / 2 - fi, al), (-aw / 2 + fi, al)]
    poly_ll = [v for (px, py) in poly_xy for v in geo.ll(px, py)]
    wp_file = write_wp_file(W, g, geo)
    P = build_params(wp_file, hull, poly_ll, overrides)
    base_pwm = int(P["motor_control"].get("base_pwm", 1360))
    v_ref = v_ref_for(dyn, hull, base_pwm)
    opp, opp_info = build_opp(W, scenario, seed, g, v_ref)
    Clock.t = 0.0
    rng = random.Random(10_000 + seed)
    out = dict(scenario=scenario, seed=seed, hull=hull, dyn=dyn, side="L" if W["side"] < 0 else "R",
               n_buoys=len(W["buoys"]), v_ref=round(v_ref, 3), exception="", **{k: round(v, 2) for k, v in opp_info.items()})
    hl, hb = F.HULL_DIMS[hull][0] / 2.0, F.HULL_DIMS[hull][1] / 2.0

    nga = F.NGA.NorthGoalAngle(); sga = F.SGA.ShipGoalAngle(); sd = F.SD.ShipDirection(); mc = F.MC.MotorController()
    nodes = dict(north_goal_angle=nga, ship_goal_angle=sga, ship_direction=sd, motor_control=mc)
    ov_all = course_overrides()
    for nd, kv in (overrides or {}).items():
        ov_all.setdefault(nd, {}).update(kv)
    out["undeclared"] = ";".join(f"{nd}.{k}" for nd, kv in sorted(ov_all.items()) if nd in nodes
                                 for k in sorted(kv) if k not in getattr(nodes[nd], "_declared", set()))
    nga.boat_mode_cb(Int32(2))
    n_wps = len(nga.waypoints)

    x, y = float(W["start"][0]), float(W["start"][1]); psi = W["psi0"]; v = w = 0.0
    boat3 = None
    if dyn == "fossen":
        import boat3dof
        boat3 = boat3dof.Boat3DOF(hull, x, y, psi)
    W_b = dict(bx=W["bx"], by=W["by"], seg_arr=np.zeros((0, 4)), dock=dict(O=np.array([1e6, 1e6])))
    W_o = dict(bx=np.array([1e6]), by=np.array([1e6]), seg_arr=None, dock=None)
    t = 0.0; k = 0
    t_fin = None; t_end = None; end_xy = None; t_stopped = None; post_len = 0.0
    obst_on = {}; obst_n = 0; obst_post = 0
    opp_on = False; opp_n = 0; opp_post = 0; contacts = []; blame_n = {"우리": 0, "상대": 0, "불명": 0}
    hist = deque(maxlen=21)
    bnd_race = bnd_all = 0.0
    sh_race = sh_post = 0.0
    near_ticks = seen_any = seen_front = 0
    scan_min_opp = true_min_opp = hull_min_opp = float("inf")
    rec = dict(t=[], x=[], y=[], psi=[], v=[], ox=[], oy=[], opsi=[], des=[]) if record else None
    ok = True
    try:
        while t < t_max and (t_end is None or t < t_end + post):
            # ---- 상대 배 + LiDAR ----
            rng_b = F.lidar(W_b, x, y, psi)
            op = opp.pose(t) if opp is not None else None
            if op is not None:
                oc = rect_corners(op[0], op[1], op[2], OPP_L / 2, OPP_B / 2)
                W_o["seg_arr"] = np.array([[oc[i][0], oc[i][1], oc[(i + 1) % 4][0], oc[(i + 1) % 4][1]] for i in range(4)])
                W_o["dock"] = dict(O=np.array([x, y]))
                rng_o = F.lidar(W_o, x, y, psi)
                ranges = np.minimum(rng_b, rng_o)
                d_true = F.rect_hull_dist(op[0], op[1], op[2], x, y, OPP_L / 2, OPP_B / 2)
                true_min_opp = min(true_min_opp, d_true)
                hit = np.isfinite(rng_o)
                if hit.any():
                    scan_min_opp = min(scan_min_opp, float(rng_o[hit].min()))
                if math.hypot(op[0] - x, op[1] - y) < NEAR_OPP_M:
                    near_ticks += 1
                    seen_any += int(hit.any()); seen_front += int((hit & FRONT_MASK).any())
            else:
                ranges = rng_b
            sd.scan_callback(F.Scan(ranges.tolist()))
            yaw = Float64(psi)
            nga.yaw_cb(yaw); sga.yaw_callback(yaw)
            if k % 10 == 3:                                       # GPS 1 Hz
                lat, lon = geo.ll(x + rng.gauss(0, gps_sigma), y + rng.gauss(0, gps_sigma))
                nga.gps_cb(F.NavSatFix(lat, lon, cov=max(gps_sigma, 1e-3) ** 2))
            if k % 50 == 0:
                nga.boat_mode_cb(Int32(2))
            # ---- north 0.5 s (full_course_sim 과 같은 배선 순서) ----
            if k % 5 == 0:
                c0 = {p_: nga.pubs[p_].count for p_ in ("/wp_mode", "/north_goal_angle_tp", "/geofence_state",
                                                        "/candidate_angle", "/goal_distance")}
                nga.timer_cb()
                if nga.pubs["/goal_distance"].count > c0["/goal_distance"]:
                    mc.goal_distance_cb(nga.pubs["/goal_distance"].last)
                if nga.pubs["/wp_mode"].count > c0["/wp_mode"]:
                    sd.wp_mode_cb(nga.pubs["/wp_mode"].last)
                if nga.pubs["/north_goal_angle_tp"].count > c0["/north_goal_angle_tp"]:
                    sga.goal_angle_callback(nga.pubs["/north_goal_angle_tp"].last)
                if nga.pubs["/geofence_state"].count > c0["/geofence_state"]:
                    sd.geofence_cb(nga.pubs["/geofence_state"].last)
                if nga.pubs["/candidate_angle"].count > c0["/candidate_angle"]:
                    m = nga.pubs["/candidate_angle"].last
                    sd.candidate_callback(m); nga.candidate_cb(m)
            c = sga.pubs["/yaw_error"].count
            sga.publish_yaw_error()
            if sga.pubs["/yaw_error"].count > c:
                sd.yaw_error_callback(sga.pubs["/yaw_error"].last)
            sd.watchdog_cb(); sd.control_cb()
            dl = sd.pubs["/desired_angle"].last
            if dl is not None:
                mc.angle_callback(dl)
            if sd.pubs["/obstacle_distance_array"].last is not None:
                mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
            if sd.pubs["/failsafe_level"].last is not None:
                mc.failsafe_callback(sd.pubs["/failsafe_level"].last)
            mc.timer_callback()
            code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
            px, py = x, y
            if boat3 is not None:
                x, y, psi = boat3.step(pl, pr, DT); v = boat3.speed
            else:
                TR, TL = 1500 - pr, 1500 - pl
                v_cmd, w_cmd = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
                v += (v_cmd - v) * DT / F.TAU; w += (w_cmd - w) * DT / F.TAU
                psi = (psi + w * DT) % 360.0
                x += v * math.sin(math.radians(psi)) * DT; y += v * math.cos(math.radians(psi)) * DT
            t += DT; k += 1; Clock.t = t
            racing = t_fin is None            # 경기 중 = 결승선 통과 전 (접촉·경계·STOP_HOLD 경기 중 지표)
            after_end = t_end is not None     # 코스 끝(50000) 뒤
            # ---- 판정 ----
            if dl is not None and dl.data >= STOP_HOLD:
                if racing: sh_race += DT
                elif after_end: sh_post += DT
            if t_fin is None and py < g["finish_y"] <= y:
                t_fin = t
            if nga.wp_idx >= n_wps and t_end is None:
                t_end = t; end_xy = (x, y)
            elif after_end:
                post_len += math.hypot(x - px, y - py)
                if t_stopped is None and abs(v) < 0.05:
                    t_stopped = t
            corners = rect_corners(x, y, psi, hl, hb)
            bo = max(outside_dist(cx, cy, aw, al) for cx, cy in corners)
            bnd_all = max(bnd_all, bo)
            if racing:
                bnd_race = max(bnd_race, bo)
            for i, (bx_, by_) in enumerate(W["buoys"]):
                rd = F.rect_hull_dist(x, y, psi, bx_, by_, hl, hb)
                on = rd < F.BUOY_R
                if on and not obst_on.get(i):
                    if racing: obst_n += 1
                    else: obst_post += 1
                obst_on[i] = on if on else (rd < F.BUOY_R + 0.1 and obst_on.get(i, False))
            if op is not None:
                op2 = opp.pose(t)
                hist.append((t, x, y, psi, op2[0], op2[1], op2[2]))
                if math.hypot(op2[0] - x, op2[1] - y) < 3.0:
                    dd = rect_rect_dist(corners, rect_corners(op2[0], op2[1], op2[2], OPP_L / 2, OPP_B / 2))
                else:
                    dd = 9.9
                hull_min_opp = min(hull_min_opp, dd)
                on = dd <= 0.0
                if on and not opp_on:
                    if racing:
                        opp_n += 1
                        rel, d_our, d_opp, who = blame(hist, t)
                        blame_n[who] += 1
                        contacts.append(f"t{t:.1f}:{rel}:우{d_our:.0f}°/상{d_opp:.0f}°→{who}")
                    else:
                        opp_post += 1
                opp_on = on if on else (dd < 0.1 and opp_on)
            if record:
                for kk, vv in (("t", t), ("x", x), ("y", y), ("psi", psi), ("v", v),
                               ("ox", op2[0] if op is not None else None), ("oy", op2[1] if op is not None else None),
                               ("opsi", op2[2] if op is not None else None), ("des", None if dl is None else dl.data)):
                    rec[kk].append(vv)
    except Exception:
        ok = False
        out["exception"] = traceback.format_exc().strip().splitlines()[-1]
        out["traceback"] = traceback.format_exc()
    sd_lines = [ln for lv, ln in sd.get_logger().lines]
    errs = sorted({f"{nm}:{ln.splitlines()[0][:60]}" for nm, n_ in nodes.items() for lv, ln in n_.get_logger().lines if lv == "ERROR"})
    contact_pen = obst_n + opp_n
    out.update(
        ok=ok, t_sim=round(t, 1), finished=t_fin is not None, t_finish=None if t_fin is None else round(t_fin, 1),
        t_course_end=None if t_end is None else round(t_end, 1),
        finish_to_end_s=None if (t_fin is None or t_end is None) else round(t_end - t_fin, 1),
        obst_contacts=obst_n, opp_contacts=opp_n, opp_fault_ours=blame_n["우리"], opp_fault_theirs=blame_n["상대"],
        opp_fault_unknown=blame_n["불명"], contacts_post=obst_post + opp_post, contact_detail=" | ".join(contacts),
        bnd_out_race_m=round(bnd_race, 2), bnd_out_max_m=round(bnd_all, 2),
        y_course_end=None if end_xy is None else round(end_xy[1], 2),
        stop_past_finish_m=None if t_end is None else round(y - g["finish_y"], 2),
        post_travel_m=None if t_end is None else round(post_len, 2),
        post_final_speed=None if t_end is None else round(abs(v), 3),
        stopped=t_end is not None and abs(v) < 0.05,
        t_to_stop_s=None if t_stopped is None else round(t_stopped - t_end, 1),
        post_observed_s=None if t_end is None else round(t - t_end, 1),
        stop_hold_race_s=round(sh_race, 1), stop_hold_post_s=round(sh_post, 1),
        record_s=None if t_fin is None else round(t_fin + 15.0 * contact_pen, 1),
        record_fault_s=None if t_fin is None else round(t_fin + 15.0 * (obst_n + blame_n["우리"]), 1),
        opp_near_ticks=near_ticks, opp_seen_any=seen_any, opp_seen_front=seen_front,
        opp_scan_min_m=None if not math.isfinite(scan_min_opp) else round(scan_min_opp, 2),
        opp_true_min_m=None if not math.isfinite(true_min_opp) else round(true_min_opp, 2),
        opp_hull_min_m=None if not math.isfinite(hull_min_opp) else round(hull_min_opp, 2),
        dwa_fallback_lines=sum("DWA 가능한 후보 없음" in ln for ln in sd_lines),
        geofence_stop_lines=sum("🚧🛑" in ln for ln in sd_lines),
        side_lock_lines=sum("옆 잠금" in ln and not ln.startswith("옆 잠금 켬") for ln in sd_lines),
        node_errors=" | ".join(errs)[:300],
    )
    out["rec"] = rec
    try:
        os.remove(wp_file)
    except OSError:
        pass
    return out


DEFAULT_GEOM = dict(arena_len=30.0, arena_w=12.0, start_y=2.0, finish_y=26.0, wp_beyond=3.0, fence_inset=0.0)
CSV_COLS = ["scenario", "seed", "hull", "dyn", "params", "side", "n_buoys", "v_ref", "ok", "exception", "finished",
            "t_finish", "t_course_end", "finish_to_end_s", "record_s", "record_fault_s", "obst_contacts", "opp_contacts",
            "opp_fault_ours", "opp_fault_theirs", "opp_fault_unknown", "contacts_post", "contact_detail",
            "bnd_out_race_m", "bnd_out_max_m", "y_course_end", "stop_past_finish_m", "post_travel_m",
            "post_final_speed", "stopped", "t_to_stop_s", "post_observed_s", "stop_hold_race_s", "stop_hold_post_s",
            "opp_near_ticks", "opp_seen_any", "opp_seen_front", "opp_scan_min_m", "opp_true_min_m", "opp_hull_min_m",
            "opp_gap", "opp_bump", "opp_ahead", "opp_delay", "opp_ycut", "opp_ang", "opp_v",
            "dwa_fallback_lines", "geofence_stop_lines", "side_lock_lines", "undeclared", "node_errors", "t_sim", "wall_s"]


def _worker(job):
    scen, seed, kw = job
    t0 = _walltime.perf_counter()
    r = run(scen, seed, **kw)
    r["wall_s"] = round(_walltime.perf_counter() - t0, 1)
    r.pop("rec", None)
    return r


def _med(vals):
    vals = [v for v in vals if v is not None]
    return None if not vals else statistics.median(vals)


def _f(v, fmt="{:.1f}"):
    return "-" if v is None else fmt.format(v)


def _fe(S):
    v = [r["finish_to_end_s"] for r in S if r["finish_to_end_s"] is not None]
    return "-" if not v else f"{statistics.median(v):.1f}({max(v):.1f})"


def summary_table(R):
    hdr = ["시나리오", "런", "예외", "결승", "결승s중앙", "기록s중앙", "장애물접촉(합)", "상대접촉 합(우리/상대/불명)",
           "경계이탈 경기중 런(최대m)", "경계이탈 전체(결승 뒤 포함) 런(최대m)", "결승뒤정지 관측", "결승선너머m 중앙", "코스끝뒤이동m 중앙", "STOP_HOLD경기중s 중앙", "결승→코스끝s 중앙(최대)",
           "상대보임 정면/360/근접틱", "상대 스캔최소/실제최소m"]
    rows = []
    for scen in SCENARIOS:
        S = [r for r in R if r["scenario"] == scen]
        if not S:
            continue
        n = len(S)
        oc = sum(r["opp_contacts"] for r in S)
        bnd = [r["bnd_out_race_m"] for r in S]
        sc_min = [r["opp_scan_min_m"] for r in S if r["opp_scan_min_m"] is not None]
        tr_min = [r["opp_true_min_m"] for r in S if r["opp_true_min_m"] is not None]
        rows.append([
            scen, str(n), str(sum(not r["ok"] for r in S)), f"{sum(r['finished'] for r in S)}/{n}",
            _f(_med([r["t_finish"] for r in S])), _f(_med([r["record_s"] for r in S])),
            f"{sum(r['obst_contacts'] for r in S)}",
            "-" if scen == "none" else f"{oc}({sum(r['opp_fault_ours'] for r in S)}/{sum(r['opp_fault_theirs'] for r in S)}/{sum(r['opp_fault_unknown'] for r in S)})",
            f"{sum(b > 0.0 for b in bnd)}({max(bnd):.2f})",
            f"{sum(r['bnd_out_max_m'] > 0.0 for r in S)}({max(r['bnd_out_max_m'] for r in S):.2f})",
            f"{sum(r['stopped'] for r in S)}/{sum(r['t_course_end'] is not None for r in S)}",
            _f(_med([r["stop_past_finish_m"] for r in S]), "{:.2f}"), _f(_med([r["post_travel_m"] for r in S]), "{:.2f}"),
            _f(_med([r["stop_hold_race_s"] for r in S])),
            _fe(S),
            "-" if scen == "none" else f"{sum(r['opp_seen_front'] for r in S)}/{sum(r['opp_seen_any'] for r in S)}/{sum(r['opp_near_ticks'] for r in S)}",
            "-" if scen == "none" else f"{_f(min(sc_min) if sc_min else None, '{:.2f}')}/{_f(min(tr_min) if tr_min else None, '{:.2f}')}",
        ])
    w = [max(len(h), *(len(r_[i]) for r_ in rows)) for i, h in enumerate(hdr)]
    lines = [" | ".join(h.ljust(w[i]) for i, h in enumerate(hdr))]
    lines.append("-+-".join("-" * x for x in w))
    for r_ in rows:
        lines.append(" | ".join(c.ljust(w[i]) for i, c in enumerate(r_)))
    return "\n".join(lines)


def check_declared(overrides, hull):
    """노드가 선언하지 않는 덮어쓰기 키(코스 yaml + --param)를 한 번 경고. 진행은 계속."""
    g = dict(DEFAULT_GEOM)
    geo = F.Geo(*GEO0)
    W = build_world(0, g)
    wp = write_wp_file(W, g, geo)
    build_params(wp, hull, [], overrides)
    Clock.t = 0.0
    nodes = dict(north_goal_angle=F.NGA.NorthGoalAngle(), ship_goal_angle=F.SGA.ShipGoalAngle(),
                 ship_direction=F.SD.ShipDirection(), motor_control=F.MC.MotorController())
    try:
        os.remove(wp)
    except OSError:
        pass
    src = {nd: dict(kv) for nd, kv in course_overrides().items()}
    for nd, kv in (overrides or {}).items():
        src.setdefault(nd, {}).update(kv)
    bad = []
    for nd, kv in sorted(src.items()):
        if nd not in nodes:
            bad.append(f"{nd}.* (이 시뮬은 {nd} 를 띄우지 않음)")
            continue
        dec = getattr(nodes[nd], "_declared", set())
        bad += [f"{nd}.{k}" for k in sorted(kv) if k not in dec]
    for b in bad:
        print(f"[tournament_sim] 경고: {b} 는 노드가 선언하지 않는 파라미터 → 값 무시(효과 없음), 계속 진행", file=sys.stderr)
    return bad


def main():
    ap = argparse.ArgumentParser(description="토너먼트 2D 시뮬 (실제 노드 코드 + 대본 상대 배)")
    ap.add_argument("--scenario", default="all", choices=SCENARIOS + ["all"])
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seed0", type=int, default=1)
    ap.add_argument("--hull", default="A", choices=["A", "B"])
    ap.add_argument("--dyn", default="fossen", choices=["fossen", "legacy"])
    ap.add_argument("--param", action="append", default=[], metavar="노드.키=값",
                    help="노드 파라미터 덮어쓰기(반복), 예 --param ship_direction.side_lock_enabled=true")
    ap.add_argument("--csv", default=os.path.join(OUT_DIR, "tournament_sim.csv"))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--t-max", type=float, default=150.0)
    ap.add_argument("--post", type=float, default=10.0, help="코스 끝(50000) 뒤 더 돌리는 초")
    ap.add_argument("--arena-len", type=float, default=DEFAULT_GEOM["arena_len"])
    ap.add_argument("--arena-wid", type=float, default=DEFAULT_GEOM["arena_w"])
    ap.add_argument("--start-y", type=float, default=DEFAULT_GEOM["start_y"])
    ap.add_argument("--finish-y", type=float, default=DEFAULT_GEOM["finish_y"])
    ap.add_argument("--wp-beyond", type=float, default=DEFAULT_GEOM["wp_beyond"], help="결승 WP = 결승선 + 이 값(m)")
    ap.add_argument("--fence-inset", type=float, default=DEFAULT_GEOM["fence_inset"],
                    help="지오펜스 폴리곤 좌우 변을 경기장보다 이만큼(m) 안쪽에 둔다(현장 대책 시험). 이탈은 실제 경기장 기준")
    ap.add_argument("--verbose", action="store_true", help="런마다 한 줄")
    a = ap.parse_args()
    ov = F.parse_param_overrides(a.param)
    check_declared(ov, a.hull)
    geom = dict(arena_len=a.arena_len, arena_w=a.arena_wid, start_y=a.start_y, finish_y=a.finish_y, wp_beyond=a.wp_beyond,
                fence_inset=a.fence_inset)
    kw = dict(hull=a.hull, dyn=a.dyn, overrides=ov, geom=geom, t_max=a.t_max, post=a.post)
    scens = SCENARIOS if a.scenario == "all" else [a.scenario]
    jobs = [(s, sd, kw) for s in scens for sd in range(a.seed0, a.seed0 + a.seeds)]
    t0 = _walltime.perf_counter()
    if a.jobs > 1 and len(jobs) > 1:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(min(a.jobs, len(jobs))) as pool:
            R = pool.map(_worker, jobs, chunksize=1)
    else:
        R = [_worker(j) for j in jobs]
    wall = _walltime.perf_counter() - t0
    ptag = " ".join(a.param)
    os.makedirs(os.path.dirname(os.path.abspath(a.csv)), exist_ok=True)
    with open(a.csv, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(CSV_COLS)
        for r in R:
            r["params"] = ptag
            wr.writerow(["" if r.get(c) is None else r.get(c) for c in CSV_COLS])
    print(f"토너먼트 시뮬: 선체 {a.hull}, 운동 {a.dyn}, 시드 {a.seed0}~{a.seed0 + a.seeds - 1}, "
          f"--param [{ptag or '없음'}] — {len(R)} 런 {wall:.0f} s (병렬 {a.jobs}) → {a.csv}")
    if a.verbose:
        for r in R:
            print(f"  {r['scenario']:12s} s{r['seed']:<3d} {r['side']} 결승 {_f(r['t_finish'])}s 끝 {_f(r['t_course_end'])}s "
                  f"장애물 {r['obst_contacts']} 상대 {r['opp_contacts']} [{r['contact_detail']}] 경계 {r['bnd_out_race_m']:.2f} "
                  f"정지 {r['stopped']} 너머 {_f(r['stop_past_finish_m'], '{:.2f}')}m STOP_HOLD {r['stop_hold_race_s']}s "
                  f"상대스캔최소 {_f(r['opp_scan_min_m'], '{:.2f}')} 실제 {_f(r['opp_true_min_m'], '{:.2f}')} "
                  f"보임 {r['opp_seen_front']}/{r['opp_seen_any']}/{r['opp_near_ticks']} DWA폴백 {r['dwa_fallback_lines']} "
                  f"경계정지 {r['geofence_stop_lines']} {r['exception']}")
    print(summary_table(R))
    print("  책임 = 단순 휴리스틱(접촉 2 s 전 관계·침로 변화량), 규정 판정 아님. 기록 = 결승 + 15 s × (장애물 + 상대 접촉).")
    bad = [r for r in R if not r["ok"]]
    for r in bad[:3]:
        print(f"예외 {r['scenario']} seed {r['seed']}:\n{r.get('traceback', r['exception'])}")
    und = sorted({r["undeclared"] for r in R if r.get("undeclared")})
    if und:
        print(f"  ⚠️ 노드가 선언 안 한(무시된) 파라미터: {', '.join(und)}")


if __name__ == "__main__":
    main()

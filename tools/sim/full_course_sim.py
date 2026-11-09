#!/usr/bin/env python3
"""전체 코스 2D 시뮬레이션 — 실제 노드 코드 9개를 rosstub 으로 연결해 waypoints.yaml 의 미션 순서를 끝까지 돈다.

실행
  python3 tools/sim/full_course_sim.py --variant A --plot-seed 1           # 대표 런 그림 1장 (+ 시간축 출력)
  python3 tools/sim/full_course_sim.py --variant AB --seeds 20 --seed0 1   # 몬테카를로 → tools/sim/out/full_course_sim.csv
  python3 tools/sim/full_course_sim.py --variant A --seeds 200 --seed0 1000 --csv tools/sim/out/full_course_sim_A200.csv
  python3 tools/sim/full_course_sim.py --variant AB --faults --seeds 20  # 고장 주입 (a)~(e) + 기준 런 → full_course_sim_faults.csv
  python3 tools/sim/full_course_sim.py --variant AB --seeds 20 --dwell-label "WP6=3,WP7=3" --csv tools/sim/out/full_course_sim_dwell_diag.csv
      # 보조 진단(현재 설정 아님): WP6·WP7 dwell 만 3 s 로 바꾼 스크래치 사본 → WP7 도착 뒤 도킹 구간만 따로 본다
  변형 A = north 가 --wp-file(없으면 --world-wp-file)을 그대로 읽는다. 기본 yaml 의 도킹 네 줄은 자리표시값(가운데 칸).
  변형 B = 기본 waypoints.yaml 의 도킹 네 줄(접근 10m·6m mode 5, WP8 입구·WP9 이탈 mode 7, ⚠️ 현장 교체)을
           tools/dock_wp.py 가 목표 칸 입구(벽에서 0.5 m)로 찍은 네 줄로 바꾼 파일(목표 칸별로 스크래치 fullcourse/ 에 생성).
           세계 배치는 변형과 무관하게 --world-wp-file 기준(전후 비교는 5faf86b waypoints 사본으로 고정).
  페일세이프 기록(모든 런): ship_direction /failsafe_level L1·L2 진입 수·시간·첫 진입, motor_control 명령 워치독 중립(로그 문구),
           north GPS stale/fix 없음·IMU stale 경고, ship_goal_angle IMU·목표각 stale 경고, /desired_angle 50000 시간(모드별).
  옵션: --wp-file <yaml>(north_goal_angle 이 읽을 파일; 세계 배치는 늘 --world-wp-file 기준), --gps-sigma 0.03,
        --current VX VY(조류 m/s, 기본 0 0), --t-max 1500, --jobs N(병렬 프로세스)

실제로 돌리는 노드 코드 (한 줄도 안 바꾼다. 파라미터는 아래 — 대회 launch 와 값은 같다)
  north_goal_angle  ← north_goal_angle.yaml + waypoints_file (launch 가 둘 다 넘김)
  ship_goal_angle   ← ship_goal_angle.yaml (launch 가 넘김)
  ship_direction, motor_control, ship_gate, ship_dock ← 코드 기본값
  ship_back         ← {active_wp_modes: [2, 3]}  (search_behavior 기본 hold)
  ship_turn         ← {enabled: False}           (hold 에서 침묵 — 발행 0 회인지 검사한다)
  ⚠️ 11-05부터 대회 launch 는 위 6개 노드에 각 패키지 yaml 도 넘긴다(계약서_launch_yaml_blackbox).
     지금 yaml 값 == 코드 기본값이라 결과는 같지만, 현장에서 yaml 을 고치면 이 시뮬과 달라진다.
  검출기 3종의 모드 게이트·확정기는 실제 순수 모듈(color_shape_detector.mode_gate.ModeGate,
  dock_logic.DetectionConfirmer)을 쓴다. 영상 처리(cv2)만 아래 카메라 모델로 바꾼다.

토픽 배선 (틱 0.1 s 마다 발행값을 실제 이름의 구독 콜백으로 손으로 넘긴다)
  /scan → ship_direction·ship_gate·ship_back·ship_turn·ship_dock     (10 Hz)
  /imu/yaw → north_goal_angle·ship_goal_angle·ship_turn                (10 Hz, 참값 — yaw_mux 출력 가정)
  /ublox_gps_node/fix (NavSatFix, 1 Hz, RTK status 2, 잡음 σ) → north_goal_angle
  /boat_mode = 2(AUTO) → north_goal_angle (t=0 부터 AUTO 가정)
  north_goal_angle(0.5 s) → /wp_mode → ship_direction·ship_gate·ship_back·ship_turn·ship_dock·검출기 3종
                          → /north_goal_angle_tp → ship_goal_angle, /geofence_state → ship_direction,
                          → /goal_distance → ship_dock (C4 TRANSIT→INIT, north 순서대로 /wp_mode 보다 먼저)
                          → /wp_remaining_time → ship_dock (C4 WP8→WP9 전환 감지, /wp_mode 뒤)
                          → /candidate_angle(20000, mode 5·8) → ship_direction
  검출기 gate → /red_angle·/green_angle → ship_gate;  turn → /image_angle·/image_color → ship_back·ship_turn;
  검출기 dock → /image_angle → ship_back·ship_turn·ship_dock
  ship_goal_angle → /yaw_error → ship_direction
  ship_gate·ship_back·ship_turn·ship_dock → /candidate_angle → ship_direction (발행자를 틱마다 기록)
  /candidate_angle(모든 발행자, north 자기 발행 포함) → north_goal_angle.candidate_cb
                          (C2 dwell 대기: mode 2·3 은 담당 노드가 손 뗀 동안만 dwell)
  ship_direction → /desired_angle·/obstacle_distance_array·/failsafe_level → motor_control → Motor_run → 운동 모델
  한 틱 안의 순서: 센서 → north(주기 도래 시) → 검출기 → ship_goal_angle → 미션 노드 4개 → ship_direction → motor_control.
  /candidate_angle: ship_direction 은 마지막 값을 들고 있되, 1.0 s(candidate_stale_sec) 넘게 갱신이 없으면
  20000 으로 대체하고, /wp_mode 가 바뀌면 20000·수신 시각 없음으로 초기화한다(C1, e0b1a06).

모델로 바꾼 것 (전부 가정)
  · 운동: gate_sim 과 같은 K_V(1.2 m/s ÷ 140 PWM), K_W 0.22 °/s/PWM, 1차 지연 τ 0.5 s, 조류는 옵션(기본 0).
    --dyn fossen 이면 3자유도 모델(boat3dof.py: 질량·수력 계수·추진기 위치, Gazebo 모델과 같은 계수, 가정값).
    후진도 같은 K_V. 충돌 물리 없음(통과) — 접촉은 횟수만 센다.
  · LiDAR: 360°/0.5° (fastworld 와 같은 규약, 80° = 정면), 최대 20 m, 원형 부표(반경 0.2 m)+도크 선분 레이캐스트, 잡음 없음.
    LiDAR 위치 = 배 중심.
  · 카메라(검출기): gate_sim·dock_sim 규약 — HFOV 63.9°, 부표 0.5~12 m(도크 표식 0.7~12 m), 방위 왼쪽 양수, 잡음 σ 0.5°,
    각 토픽에 가장 가까운 가시 대상 1개(= 가장 큰 컨투어). 가림(occlusion) 없음. 검출기 프레임 10 Hz.
    색 대상: gate = 빨강·초록, turn = 모드별 색(실제 wp_color_map: 2 → white, 3 → red), dock = 목표 칸 표식(빨강 사각)만.
    실제 검출기 동작 재현: 모드 게이트(ModeGate, 비활성이면 발행 안 함), 확정 프레임(gate·turn 1, dock 3),
    놓치면 grace 2 s 동안 마지막 유효 각을 다시 발행, 그 뒤 10000.
  · GPS: 1 Hz(zed_f9p_rover.yaml rate 1.0), 로컬 평면 근사 위경도, 축별 가우스 잡음 σ(기본 0.03 m = RTK), 공분산 σ².
  · IMU: /imu/yaw = 참 헤딩(장착 오프셋·잡음 없음).
  · 접촉: 부표 = 배 중심 거리 − 0.2 < half_width 0.29 (gate_sim 과 같음). 도크 = 선체 중심선(±0.83 m, LOA 1.66)이
    정면 벽을 넘거나, 칸막이와 0.29 m 안. 접촉은 '닿기 시작'을 한 번으로 센다.

세계 배치 (--world-wp-file, 기본 src/north_goal_angle/config/waypoints.yaml; 원점 = WP0, x = 동, y = 북)
  · 게이트 4쌍: WP0→WP1 선분의 1/8·3/8·5/8·7/8 지점이 중심, 폭 3~6 m 무작위, 선분에 수직, 빨강 = 진행 방향 좌현.
  · WP2 위치에 흰 부표(위치유지), WP3 초록·WP4 빨강·WP5 흰 부표(구역 이름대로, WP 좌표 그대로).
  · 장애물(주황, 검출기 대상 색 아님): WP6→WP7(처음 두 mode 5) 선분의 10~90 % 구간에 6~10개,
    도크(첫 mode 7)→WP10 선 위 WP10 앞 3~20 m 구간에 6~10개,
    경로선 ±3 m, 서로 1.5 m 이상, 다른 부표와 1.5 m 이상.
  · 도크(dock_sim 과 같은 규격): 정면 벽 3 m(1 m 칸 3개) + 칸막이 0.5 m 4개. 정면 벽 중심 = WP8 좌표,
    정면(물 쪽)이 WP7 방향을 향함. 목표 칸(표식) 좌·중·우 무작위.
  · geofence: north_goal_angle.yaml 의 geofence_polygon 이 [] → 꺼짐(노드가 빈 배열 발행). 경계 없음.
  · 출발: WP1→WP0 연장선 위 WP0 앞 8 m, 좌우 ±1 m, 헤딩 = WP0 방향 ±20°, 정지 상태. t=0 에 모든 노드 생성.

판정 (실측 아님, 시뮬 정의)
  · 게이트 기하 통과 = 궤적이 빨강–초록 선분을 진행 방향으로 가로지름. /gates_passed = ship_gate 마지막 발행값.
  · 위치유지(규정) = mode 2 동안 흰 부표(WP2) 5 m 이내 연속 5 s. 노드 판정(HOLD→DONE) 도 따로 적는다.
  · 탐색 = mode 3 동안 ship_back 이 HOLD→DONE 했고 그때 가장 가까운 부표가 빨강(turn 검출기 mode 3 대상 색).
    규정 기하(빨강 5 m 이내 연속 5 s)도 따로 적는다.
  · 포기 = ship_back 이 ERROR '포기'(거리 unknown 10 s 또는 HOLD 총 60 s)를 남기고 DONE — 성공으로 세지 않는다.
  · 도킹 = ship_dock 이 CONTACT 에 들어간 순간 배 중심이 목표 칸 중심 ±0.5 m(칸 폭) 안 → '칸 안 접안'.
    후진 이탈 = BACKOFF→DONE 이 거리(2.5 m) 도달로 끝남. SEARCH 타임아웃 = SEARCH→DONE.
  · 코스 시간 = WP10(마지막 WP) 2.5 m 안에 처음 든 시각. 그 뒤 dwell 완료를 --tail 초까지만 기다린다.
"""
import argparse
import csv
import math
import os
import random
import re
import subprocess
import sys
import time as _walltime
import traceback
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(ROOT, "src")
OUT_DIR = os.path.join(HERE, "out")
SCRATCH_B = "/private/tmp/claude-501/-Users-leesongeon-Desktop-SSF------/2f38bfd9-14de-463e-9b5f-06f132318c47/scratchpad/fullcourse"
WP_DEFAULT = os.path.join(SRC, "north_goal_angle", "config", "waypoints.yaml")
sys.path.insert(0, HERE)

import yaml                                      # noqa: E402
import rosstub                                   # noqa: E402
rosstub.install()
from rosstub import Clock                        # noqa: E402

# ───────────────────────── 스텁 보강 (노드 코드는 그대로, ROS·외부 패키지만 흉내) ─────────────────────────
_senm = sys.modules["sensor_msgs.msg"]


class NavSatFix:
    def __init__(self, lat=0.0, lon=0.0, cov=0.0009, status=2):
        self.latitude, self.longitude = lat, lon
        self.status = types.SimpleNamespace(status=status)
        self.position_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov]


_senm.NavSatFix = NavSatFix
_ament = types.ModuleType("ament_index_python"); _amp = types.ModuleType("ament_index_python.packages")
_amp.get_package_share_directory = lambda pkg: os.path.join(SRC, pkg)
_amp.PackageNotFoundError = Exception
_ament.packages = _amp
sys.modules.setdefault("ament_index_python", _ament); sys.modules.setdefault("ament_index_python.packages", _amp)
_geopy = types.ModuleType("geopy"); _geopy.distance = types.SimpleNamespace(distance=None)   # north 의 calc_dist(미사용)용
sys.modules.setdefault("geopy", _geopy)
rosstub.Pub.get_subscription_count = lambda self: 1      # /geofence_state 구독자(ship_direction) 있음

import kaboat_full_course_sim as K               # noqa: E402  (운동 상수: gate_sim 과 같은 출처)

std = sys.modules["std_msgs.msg"]
Float32, Float64, Int32, String = std.Float32, std.Float64, std.Int32, std.String


def _load(rel, name, pkg):
    return rosstub.load_node(os.path.join(SRC, rel), name, extra_path=os.path.join(SRC, pkg))


NGA = _load("north_goal_angle/north_goal_angle/north_goal_angle.py", "nga_real", "north_goal_angle")
SGA = _load("ship_goal_angle/ship_goal_angle/ship_goal_angle.py", "sga_real", "ship_goal_angle")
SD = _load("ship_direction/ship_direction/ship_direction.py", "sd_real", "ship_direction")
MC = _load("motor_control/motor_control/motor_control.py", "mc_real", "motor_control")
SG = _load("ship_gate/ship_gate/ship_gate.py", "sg_real", "ship_gate")
SB = _load("ship_back/ship_back/ship_back.py", "sb_real", "ship_back")
ST = _load("ship_turn/ship_turn/ship_turn.py", "st_real", "ship_turn")
SK = _load("ship_dock/ship_dock/ship_dock.py", "sk_real", "ship_dock")
sys.path.insert(0, os.path.join(SRC, "color_shape_detector"))
from color_shape_detector.mode_gate import ModeGate              # noqa: E402
from color_shape_detector.dock_logic import DetectionConfirmer   # noqa: E402
from color_shape_detector.turn_color_logic import (             # noqa: E402
    build_wp_color_map, detection_disabled, resolve_turn_color)

with open(os.path.join(SRC, "north_goal_angle/config/north_goal_angle.yaml"), encoding="utf-8") as _f:
    P_NGA = yaml.safe_load(_f)["/north_goal_angle"]["ros__parameters"]
with open(os.path.join(SRC, "ship_goal_angle/config/ship_goal_angle.yaml"), encoding="utf-8") as _f:
    P_SGA = yaml.safe_load(_f)["/ship_goal_angle"]["ros__parameters"]
with open(os.path.join(SRC, "color_shape_detector/config/vision.yaml"), encoding="utf-8") as _f:
    _V = yaml.safe_load(_f)
VIS_ALL = _V["/**"]["ros__parameters"]
VIS = {k: dict(VIS_ALL, **_V[k]["ros__parameters"]) for k in ("image_subscriber_gate", "image_subscriber_turn", "image_subscriber_dock")}

# ───────────────────────── 상수 ─────────────────────────
DT = K.DT                         # 0.1 s
TAU = 0.5
HALF_W = K.BOAT_HALF_W            # 0.29 (ship_direction half_width 기본값과 같음)
BUOY_R = K.BUOY_R                 # 0.2
HULL_HALF_L = 0.83
# 직사각형 선체 접촉(보조 지표, 11-06 계약서_옆비킴_현장조정 결정 ⑥): 기본 접촉은 중심 원(반경 HALF_W)이라
# 선미 휘두름을 못 본다. 여기서는 선체를 LOA×BOA 직사각형(중심 = 배 위치 = LiDAR 가정)으로 보고 부표 원과 겹치면 접촉.
# 치수는 sim/vrx make_boat.BOATS(A 1.66×0.55, B 1.77×0.522). 기존 접촉 수치는 바꾸지 않는다(별도 카운터).
HULL_DIMS = {"A": (1.66, 0.55), "B": (1.77, 0.522)}


def rect_hull_dist(x, y, psi, bx, by, half_l, half_b):
    """부표 중심에서 방향 psi(나침반 °) 직사각형 선체까지 최단 거리(안이면 0)."""
    p = math.radians(psi)
    fx, fy = math.sin(p), math.cos(p)
    dx, dy = bx - x, by - y
    u = dx * fx + dy * fy                 # 선수 방향 성분
    w = dx * (-fy) + dy * fx              # 왼쪽 성분
    return math.hypot(max(abs(u) - half_l, 0.0), max(abs(w) - half_b, 0.0))

HFOV = float(VIS_ALL["hfov_deg"])
CAM_MIN, CAM_MAX, CAM_NOISE = 0.5, 12.0, 0.5
DOCK_CAM_MIN = 0.7
LIDAR_MAX = 20.0
INVALID = 10000.0
# basic_image_subscriberturn.py wp_color_map — vision.yaml turn_color_mode2/3 에서 노드와 같은
# 규칙으로 만든다(C6). 무효 색이면 노드처럼 그 모드는 빈 목록(검출·발행 없음) + 경고 한 줄.


def _turn_color(name):
    c, err = resolve_turn_color(VIS["image_subscriber_turn"][name])
    if err:
        print(f"[full_course_sim] 경고: {name}: {err}", file=sys.stderr)
    return c


TURN_COLOR_MAP = build_wp_color_map(_turn_color("turn_color_mode2"),
                                    _turn_color("turn_color_mode3"))
MODE_NAME = {0: "게이트 0", 1: "게이트 1", 2: "위치유지 2", 3: "부표 3", 5: "회피 5", 7: "도킹 7", 8: "토너먼트 8"}
MODE_COL = {0: "#1f77b4", 1: "#17becf", 2: "#bcbd22", 3: "#9467bd", 5: "#ff7f0e", 7: "#d62728", 8: "#2ca02c"}
R_EARTH = 6371000.0
SCAN_A = K.SCAN_ANGLE_MIN_DEG + np.arange(int(round(360.0 / K.SCAN_INC_DEG))) * K.SCAN_INC_DEG
SCAN_REL = SCAN_A - 80.0          # 상대각, 좌 양수


def wrap(a): return (a + 180.0) % 360.0 - 180.0


# ───────────────────────── 좌표 ─────────────────────────
class Geo:
    def __init__(self, lat0, lon0): self.lat0, self.lon0 = lat0, lon0

    def xy(self, lat, lon):
        return (math.radians(lon - self.lon0) * R_EARTH * math.cos(math.radians(self.lat0)),
                math.radians(lat - self.lat0) * R_EARTH)

    def ll(self, x, y):
        return (self.lat0 + math.degrees(y / R_EARTH),
                self.lon0 + math.degrees(x / (R_EARTH * math.cos(math.radians(self.lat0)))))


def load_wps(path):
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)["waypoints"]
    return [dict(label=w.get("구역", ""), lat=float(w["lat"]), lon=float(w["lon"]), mode=int(w["mode"]), dwell=float(w["dwell"])) for w in raw]


# ───────────────────────── 세계 ─────────────────────────
def build_world(world_wps, seed):
    rng = random.Random(seed)
    geo = Geo(world_wps[0]["lat"], world_wps[0]["lon"])
    P = [np.array(geo.xy(w["lat"], w["lon"])) for w in world_wps]
    W = dict(geo=geo, P=P, buoys=[], segs=[])

    def add_buoy(p, color, kind):
        W["buoys"].append(dict(x=float(p[0]), y=float(p[1]), color=color, kind=kind))

    # 게이트
    d01 = P[1] - P[0]; L01 = np.linalg.norm(d01); u = d01 / L01; left = np.array([-u[1], u[0]])
    W["gates"] = []
    for k in range(4):
        c = P[0] + u * L01 * (k + 0.5) / 4.0; w = rng.uniform(3.0, 6.0)
        red, green = c + left * w / 2, c - left * w / 2
        W["gates"].append(dict(c=c, w=w, red=red, green=green))
        add_buoy(red, "red", f"gate{k}"); add_buoy(green, "green", f"gate{k}")
    W["gate_u"] = u
    # 미션 부표 (구역 이름대로)
    by_mode = [w["mode"] for w in world_wps]
    i2 = by_mode.index(2); add_buoy(P[i2], "white", "hold")
    W["hold_idx"] = len(W["buoys"]) - 1
    col_kw = {"초록": "green", "빨강": "red", "하양": "white", "흰": "white"}
    for i, w in enumerate(world_wps):
        if w["mode"] == 3:
            col = next((v for kk, v in col_kw.items() if kk in w["label"]), "red")
            add_buoy(P[i], col, f"search_{col}")
    # 도크: 정면 벽 중심 = 첫 mode 7 WP, 정면은 직전 WP 쪽
    i7 = by_mode.index(7)
    O = P[i7]; n = P[i7 - 1] - O; n = n / np.linalg.norm(n); t = np.array([n[1], -n[0]])
    bay = rng.choice((-1.0, 0.0, 1.0))
    W["dock"] = dict(O=O, n=n, t=t, bay=bay, marker=O + t * bay)
    W["segs"].append((O - 1.5 * t, O + 1.5 * t, "front"))
    for s in (-1.5, -0.5, 0.5, 1.5):
        W["segs"].append((O + s * t, O + s * t + 0.5 * n, "div"))

    # 장애물
    def scatter(a, b, t_lo, t_hi, tag, n_min=6, n_max=10):
        d = b - a; L = np.linalg.norm(d); uu = d / L; ll = np.array([-uu[1], uu[0]])
        N = rng.randint(n_min, n_max); placed = 0; tries = 0
        while placed < N and tries < 2000:
            tries += 1
            s = rng.uniform(t_lo, t_hi); p = a + uu * s + ll * rng.uniform(-3.0, 3.0)
            if all(math.hypot(p[0] - q["x"], p[1] - q["y"]) >= 1.5 for q in W["buoys"]) and \
               np.linalg.norm(p - W["dock"]["O"]) > 5.0:
                add_buoy(p, "orange", tag); placed += 1
    i5 = [i for i, m in enumerate(by_mode) if m == 5]
    a, b = P[i5[0]], P[i5[1]]; L = np.linalg.norm(b - a)       # WP6→WP7 (뒤의 mode 5 는 도킹 접근점)
    scatter(a, b, 0.1 * L, 0.9 * L, "obst_5")
    i8 = by_mode.index(8)
    a, b = P[i7], P[i8]; L = np.linalg.norm(b - a)    # 도크(첫 mode 7)→WP10. 옛 yaml 은 WP9 = 도크라 같다
    scatter(a, b, max(0.0, L - 20.0), L - 3.0, "obst_8")
    arr = np.array([[q["x"], q["y"]] for q in W["buoys"]])
    W["bx"], W["by"] = arr[:, 0], arr[:, 1]
    W["seg_arr"] = np.array([[p[0], p[1], q[0], q[1]] for p, q, _ in W["segs"]])
    # 출발
    back = -u; lat_off = rng.uniform(-1.0, 1.0)
    s0 = P[0] + back * 8.0 + left * lat_off
    W["start"] = s0
    brg = math.degrees(math.atan2(P[0][0] - s0[0], P[0][1] - s0[1]))
    W["psi0"] = (brg + rng.uniform(-20.0, 20.0)) % 360.0
    return W


def lidar(W, x, y, psi):
    th = np.radians(psi - SCAN_REL); dx, dy = np.sin(th), np.cos(th)
    ox = W["bx"] - x; oy = W["by"] - y
    t_ca = dx[:, None] * ox[None, :] + dy[:, None] * oy[None, :]
    d2 = (ox ** 2 + oy ** 2)[None, :] - t_ca ** 2
    r2 = BUOY_R ** 2
    hit = (d2 <= r2) & (t_ca > 0)
    t = np.where(hit, t_ca - np.sqrt(np.maximum(r2 - d2, 0.0)), np.inf)
    rng = t.min(axis=1)
    near = math.hypot(x - W["dock"]["O"][0], y - W["dock"]["O"][1]) < LIDAR_MAX + 3.0
    if near:
        for ax, ay, bx, by in W["seg_arr"]:
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


# LiDAR 잡음 모델 (--lidar-noise, 합성 — 실측 물보라 특성 아님). 계약서_도킹물보라필터_20261109 결정 5.
#   유한 거리에 σ 3 cm 가우스 + 광선 5 % 결측(inf) + 정면 ±30° 광선의 1 % 에 0.3~1.5 m 고립 물보라.
#   물보라는 '고립 한 점' 모델 — 덩어리 물보라는 다루지 않는다(ship_direction obst_median_kernel 근거와 같은 모델).
NOISE_SIGMA_M, NOISE_DROP, NOISE_SPRAY, NOISE_SPRAY_HALF_DEG = 0.03, 0.05, 0.01, 30.0
NOISE_SPRAY_RANGE_M = (0.3, 1.5)


def add_lidar_noise(rng, nrng):
    """lidar() 결과(np 배열)에 잡음을 얹은 새 배열. nrng = np.random.Generator(시드 고정 → 재현 가능)."""
    r = np.array(rng, dtype=float)
    fin = np.isfinite(r)
    r[fin] += nrng.normal(0.0, NOISE_SIGMA_M, int(fin.sum()))
    r[nrng.random(r.size) < NOISE_DROP] = np.inf
    rel = (SCAN_REL + 180.0) % 360.0 - 180.0
    spray = (np.abs(rel) <= NOISE_SPRAY_HALF_DEG) & (nrng.random(r.size) < NOISE_SPRAY)
    r[spray] = np.minimum(r[spray], nrng.uniform(NOISE_SPRAY_RANGE_M[0], NOISE_SPRAY_RANGE_M[1], int(spray.sum())))
    return np.maximum(r, 0.05)


class Scan:
    def __init__(self, ranges):
        self.angle_min = math.radians(K.SCAN_ANGLE_MIN_DEG)
        self.angle_increment = math.radians(K.SCAN_INC_DEG)
        self.ranges = ranges


def seg_seg_dist(p1, p2, q1, q2):
    def pt_seg(p, a, b):
        ab = b - a; L2 = float(ab @ ab)
        s = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, float((p - a) @ ab) / L2))
        return float(np.linalg.norm(p - (a + s * ab)))
    d1, d2 = p2 - p1, q2 - q1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) > 1e-12:
        r = q1 - p1
        s = (r[0] * d2[1] - r[1] * d2[0]) / den; u = (r[0] * d1[1] - r[1] * d1[0]) / den
        if 0 <= s <= 1 and 0 <= u <= 1:
            return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def cross_seg(p, q, a, b):
    """선분 pq 가 ab 를 가로지르나."""
    def orient(o, s, r): return (s[0] - o[0]) * (r[1] - o[1]) - (s[1] - o[1]) * (r[0] - o[0])
    return orient(p, q, a) * orient(p, q, b) < 0 and orient(a, b, p) * orient(a, b, q) < 0


# ───────────────────────── 검출기 모델 (영상 처리만 모델, 게이트·확정기는 실제 모듈) ─────────────────────────
class Det:
    def __init__(self, vis, confirm_frames):
        self.gate = ModeGate(vis["active_wp_modes"], stale_sec=float(vis.get("wp_mode_stale_sec", 2.0)))
        cw = int(vis.get("confirm_window", confirm_frames))
        self.conf = {}
        self.cf, self.cw = confirm_frames, cw
        self.grace = float(vis.get("grace_period_s", 2.0))
        self.last = {}           # key → (angle, t)

    def confirmer(self, key):
        if key not in self.conf:
            self.conf[key] = DetectionConfirmer(self.cf, confirm_window=self.cw)
        return self.conf[key]

    def on_mode_change(self):
        """노드 wp_mode_callback 과 같은 리셋 — /wp_mode **값이 바뀐** 틱에만 부른다(계약서_검출기모드리셋_20261106).
        유예(last)·확정 창(실제 DetectionConfirmer.reset)·turn 의 마지막 색을 비운다. 같은 값 반복엔 부르지 않는다."""
        self.last.clear()
        for c in self.conf.values():
            c.reset()
        if hasattr(self, "last_color"):
            del self.last_color

    def frame(self, chan, det_key, angle, t):
        """채널 하나의 프레임 처리 → 발행값(None = 이 채널 발행 안 함)."""
        c = self.confirmer(chan)
        confirmed = c.update(det_key) if angle is not None else c.update(None)
        if angle is not None and confirmed is not None:
            self.last[chan] = (angle, t)
            return angle
        lv = self.last.get(chan)
        if lv is not None and (t - lv[1]) <= self.grace:
            return lv[0]
        return INVALID


def cam_closest(x, y, psi, pts, rng, dmin=CAM_MIN):
    best = None
    for (bx, by) in pts:
        d = math.hypot(bx - x, by - y)
        if d < dmin or d > CAM_MAX:
            continue
        rel_cw = wrap(math.degrees(math.atan2(bx - x, by - y)) - psi)
        if abs(rel_cw) <= HFOV / 2 and (best is None or d < best[0]):
            best = (d, -rel_cw)
    return None if best is None else (best[0], best[1] + rng.gauss(0.0, CAM_NOISE))


# ───────────────────────── 한 런 ─────────────────────────
def node_params(wp_file, overrides=None):
    rosstub.PARAMS.clear()
    rosstub.PARAMS["north_goal_angle"] = dict(P_NGA, waypoints_file=wp_file)
    rosstub.PARAMS["ship_goal_angle"] = dict(P_SGA)
    for n in ("ship_direction", "motor_control", "ship_gate", "ship_dock"):
        rosstub.PARAMS[n] = {}
    rosstub.PARAMS["ship_back"] = {"active_wp_modes": [2, 3]}
    rosstub.PARAMS["ship_turn"] = {"enabled": False}
    for node, kv in (overrides or {}).items():         # --param 노드.키=값 (11-06, 계약서_DWA 결정 11)
        rosstub.PARAMS.setdefault(node, {}).update(kv)


def parse_param_overrides(items):
    """['ship_direction.dwa_enabled=true', ...] → {'ship_direction': {'dwa_enabled': True}} (값은 yaml 해석)."""
    out = {}
    for it in items or []:
        key, _, val = it.partition("=")
        node, _, name = key.partition(".")
        if not node or not name or not _:
            raise SystemExit(f"--param 형식은 노드.키=값: {it!r}")
        out.setdefault(node, {})[name] = yaml.safe_load(val)
    return out


def apply_dwell(path, overrides):
    """보조 진단용: 구역 이름이 키로 시작하는 WP 의 dwell 만 바꾼 사본을 스크래치에 쓴다(원본은 안 건드림)."""
    if not overrides:
        return path
    os.makedirs(SCRATCH_B, exist_ok=True)
    tag = "_".join(f"{k}-{v:g}" for k, v in sorted(overrides.items()))
    out = os.path.join(SCRATCH_B, os.path.basename(path).replace(".yaml", f"_dwell_{tag}.yaml"))
    lines = []
    with open(path, encoding="utf-8") as f:
        for ln in f.read().splitlines():
            m = re.search(r'구역:\s*"([^"]*)"', ln)
            if m and not ln.lstrip().startswith("#"):
                for key, val in overrides.items():
                    if m.group(1).startswith(key + " "):
                        ln = re.sub(r"dwell:\s*[0-9.]+", f"dwell: {float(val):.1f}", ln)
            lines.append(ln)
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return out


def run(seed, variant="A", wp_file=None, world_wp_file=WP_DEFAULT, gps_sigma=0.03, current=(0.0, 0.0),
        t_max=1500.0, tail=90.0, record=False, dwell_override=None, fault=None, dyn="legacy", hull=None,
        node_overrides=None, lidar_noise=False):
    world_wps = load_wps(world_wp_file)
    nrng = np.random.default_rng(20_000 + seed) if lidar_noise else None
    W = build_world(world_wps, seed)
    geo = W["geo"]; dock = W["dock"]
    if wp_file is None:
        wp_file = variant_b_file(W) if variant == "B" else world_wp_file
    wp_file = apply_dwell(wp_file, dwell_override)
    wps = load_wps(wp_file)
    WPXY = [np.array(geo.xy(w["lat"], w["lon"])) for w in wps]
    node_params(wp_file, node_overrides)
    Clock.t = 0.0
    vtag = variant + ("" if not dwell_override else "-dwell(" + ",".join(f"{k}={v:g}" for k, v in dwell_override.items()) + ")")
    out = dict(seed=seed, variant=vtag, wp_file=wp_file, bay=dock["bay"], exception="")
    rng = random.Random(10_000 + seed)

    nga = NGA.NorthGoalAngle(); sga = SGA.ShipGoalAngle(); sd = SD.ShipDirection(); mc = MC.MotorController()
    sg = SG.ShipGate(); sb = SB.ShipBack(); st = ST.ShipTurn(); sk = SK.ShipDock()
    det_gate = Det(VIS["image_subscriber_gate"], int(VIS["image_subscriber_gate"]["confirm_frames"]))
    det_turn = Det(VIS["image_subscriber_turn"], int(VIS["image_subscriber_turn"]["confirm_frames"]))
    det_dock = Det(VIS["image_subscriber_dock"], int(VIS["image_subscriber_dock"]["confirm_frames"]))
    nodes = dict(north_goal_angle=nga, ship_goal_angle=sga, ship_direction=sd, motor_control=mc,
                 ship_gate=sg, ship_back=sb, ship_turn=st, ship_dock=sk)
    cand_pubs = [("north_goal_angle", nga), ("ship_gate", sg), ("ship_back", sb), ("ship_turn", st), ("ship_dock", sk)]
    cnt = {name: n.pubs["/candidate_angle"].count for name, n in cand_pubs}
    nga.boat_mode_cb(Int32(2))                     # ssf_bridge /boat_mode = AUTO

    reds = [(b["x"], b["y"]) for b in W["buoys"] if b["color"] == "red"]
    greens = [(b["x"], b["y"]) for b in W["buoys"] if b["color"] == "green"]
    whites = [(b["x"], b["y"]) for b in W["buoys"] if b["color"] == "white"]
    by_color = {"red": reds, "green": greens, "white": whites}

    x, y = float(W["start"][0]), float(W["start"][1]); psi = W["psi0"]; v = w = 0.0
    boat3 = None
    if dyn == "fossen":
        import boat3dof
        boat3 = boat3dof.Boat3DOF(hull or (variant if variant in ("A", "B") else "A"), x, y, psi)
    t = 0.0; k = 0
    wp_idx = nga.wp_idx; wp_enter = {0: 0.0}; wp_how = {}
    mode_pubs = {}                 # mode → {노드: 틱 수}
    multi_pub_ticks = {}           # mode → 같은 틱 발행자 2개 이상 횟수
    last_cand = None
    contact_on = {}; contacts = {"gate": 0, "hold": 0, "search": 0, "obst_5": 0, "obst_8": 0, "dock": 0}
    contacts_by_mode = {}
    prev_xy = np.array([x, y])
    gate_cross = [0] * 4; gate_wrong = [0] * 4
    hold_run = 0.0; hold_best = 0.0; srch_run = {}; srch_best = {}
    sb_events = []; sk_events = []; prev_sb = sb.state; prev_sk = sk.state
    sb_log_idx = 0
    contacts_sb_state = {}
    _hd = HULL_DIMS.get(hull or variant[:1], HULL_DIMS["A"]); rect_hl, rect_hb = _hd[0] / 2.0, _hd[1] / 2.0
    contacts_rect = {"gate": 0, "hold": 0, "search": 0, "obst_5": 0, "obst_8": 0}; rect_on = {}
    dock_contact = None; dock_mode7_t0 = None; dock_dist_at = {}
    t_last_reach = None; img_pub_conflict = 0
    rec = dict(t=[], x=[], y=[], psi=[], v=[], mode=[], wp=[], sk=[], sb=[], cand=[], des=[], dd=[]) if record else None
    n_wps = len(wps)
    i_last = n_wps - 1
    first_mode = {}
    pub_mode = -1                  # 노드들이 실제로 받은 마지막 /wp_mode (north 는 전진 '전'에 현재 모드를 발행한다)
    # ---- 페일세이프 기록 (틱 배열) ----
    TK = dict(t=[], lvl=[], des=[], v=[], mode=[], wp=[], wd=[], ctot=[], gps=[], sga_imu=[], sga_goal=[], n_imu=[])
    log_idx = {"nga": 0, "sga": 0, "mc": 0}
    # ---- 고장 주입 ----
    f_kind = None if fault is None else fault["kind"]
    f_t0 = None if fault is None or f_kind == "node3" else float(fault["t0"])
    f_dur = None if fault is None else FAULT_DUR[f_kind]
    f_node = None; f_cand0 = None
    ok = True
    try:
        while t < t_max:
            cur_mode = pub_mode
            if f_kind == "node3" and f_t0 is None and pub_mode in NODE_GROUP_MODES[fault["group"]]:
                f_t0 = t + fault["offset"]
            fa = f_t0 is not None and f_t0 <= t < f_t0 + f_dur
            if fa and f_node is None:
                f_node = "/scan" if f_kind.startswith("scan") else ("GPS" if f_kind == "gps10" else ("/imu/yaw" if f_kind == "imu3" else MODE_OWNER.get(pub_mode, "north_goal_angle")))
                f_cand0 = last_cand
            # ---- 센서 ----
            _r = lidar(W, x, y, psi)
            scan = Scan((add_lidar_noise(_r, nrng) if nrng is not None else _r).tolist())
            if not (fa and f_kind in ("scan2", "scan5")):
                for cb in (sd.scan_callback, sg.scan_cb, sb.scan_cb, st.scan_cb, sk.scan_cb):
                    cb(scan)
            yaw = Float64(psi)
            if not (fa and f_kind == "imu3"):
                nga.yaw_cb(yaw); sga.yaw_callback(yaw); st.yaw_cb(yaw)
            if k % 10 == 3:                           # GPS 1 Hz
                lat, lon = geo.ll(x + rng.gauss(0, gps_sigma), y + rng.gauss(0, gps_sigma))
                if not (fa and f_kind == "gps10"):
                    nga.gps_cb(NavSatFix(lat, lon, cov=max(gps_sigma, 1e-3) ** 2))
            silent = f_node if (fa and f_kind == "node3") else None
            if k % 50 == 0:
                nga.boat_mode_cb(Int32(2))
            tick_cand = []
            # ---- north_goal_angle 0.5 s ----
            if k % 5 == 0 and silent != "north_goal_angle":
                c0 = {p: nga.pubs[p].count for p in ("/wp_mode", "/north_goal_angle_tp",
                                                     "/geofence_state", "/candidate_angle",
                                                     "/goal_distance", "/wp_remaining_time")}
                nga.timer_cb()
                if nga.pubs["/goal_distance"].count > c0["/goal_distance"]:
                    sk.goal_distance_cb(nga.pubs["/goal_distance"].last)   # 거리 → /wp_mode 순서
                    # motor_control 목표 근접 감속 (계약서_목표근접감속_20261106)
                    mc.goal_distance_cb(nga.pubs["/goal_distance"].last)
                if nga.pubs["/wp_mode"].count > c0["/wp_mode"]:
                    m = nga.pubs["/wp_mode"].last
                    sd.wp_mode_cb(m); sg.wp_mode_cb(m); sb.wp_cb(m); st.wp_cb(m); sk.wp_mode_cb(m)
                    mode_chg = m.data != pub_mode          # 값 변화만 — 0.5 s 반복 발행엔 리셋 안 함
                    for d in (det_gate, det_turn, det_dock):
                        d.gate.update(m.data, Clock.t)
                        if mode_chg:
                            d.on_mode_change()
                    det_turn.wp_mode = m.data
                    pub_mode = m.data
                if nga.pubs["/wp_remaining_time"].count > c0["/wp_remaining_time"]:
                    sk.wp_remaining_cb(nga.pubs["/wp_remaining_time"].last)   # /wp_mode 뒤
                if nga.pubs["/north_goal_angle_tp"].count > c0["/north_goal_angle_tp"]:
                    sga.goal_angle_callback(nga.pubs["/north_goal_angle_tp"].last)
                if nga.pubs["/geofence_state"].count > c0["/geofence_state"]:
                    sd.geofence_cb(nga.pubs["/geofence_state"].last)
                if nga.pubs["/candidate_angle"].count > c0["/candidate_angle"]:
                    tick_cand.append(("north_goal_angle", nga.pubs["/candidate_angle"].last))
                if nga.wp_idx != wp_idx:
                    # north 의 _log_wp_timeout / _log_wp_arrive 문구에만 맞춘다(dwell 대기 등 다른 로그 배제)
                    new = [ln for lv, ln in nga.get_logger().lines[-6:]]
                    re_to = re.compile(rf"WP{wp_idx}\(mode \d+\) 시간 초과 [0-9.]+s → 다음 WP 이동")
                    re_ar = re.compile(rf"✔ WP{wp_idx} 완료 \(반경 [0-9.]+m\) → 다음 WP 이동")
                    how = "시간초과" if any(re_to.search(ln) for ln in new) else (
                        "도착" if any(re_ar.search(ln) for ln in new) else "?")
                    wp_how[wp_idx] = how
                    wp_idx = nga.wp_idx; wp_enter[wp_idx] = t
                cur_mode = pub_mode
            first_mode.setdefault(cur_mode, t)
            # ---- 검출기 (10 Hz) ----
            if det_gate.gate.is_active(Clock.t):
                r = cam_closest(x, y, psi, reds, rng); g = cam_closest(x, y, psi, greens, rng)
                sg.red_angle_cb(Float32(det_gate.frame("red", "red", None if r is None else r[1], Clock.t)))
                sg.green_angle_cb(Float32(det_gate.frame("green", "green", None if g is None else g[1], Clock.t)))
            img = None; col = None
            wm = getattr(det_turn, "wp_mode", -1)
            # C6: 노드와 같이 무효 색 모드는 검출·발행 없음
            if det_turn.gate.is_active(Clock.t) and not detection_disabled(wm, TURN_COLOR_MAP):
                cols = TURN_COLOR_MAP.get(wm, ["red", "green", "white"])
                best = None
                for cname in cols:
                    a = cam_closest(x, y, psi, by_color[cname], rng)
                    if a is not None and (best is None or a[0] < best[0]):
                        best = (a[0], a[1], cname)
                if best is None:
                    img = det_turn.frame("img", None, None, Clock.t)
                else:
                    img = det_turn.frame("img", best[2], best[1], Clock.t)
                    if img == best[1]:
                        det_turn.last_color = best[2]
                col = "none" if img == INVALID else getattr(det_turn, "last_color", "none")
            if det_dock.gate.is_active(Clock.t):
                if img is not None:
                    img_pub_conflict += 1
                mk = dock["marker"]; rel_pos = np.array([x, y]) - dock["O"]
                a = cam_closest(x, y, psi, [tuple(mk)], rng, dmin=DOCK_CAM_MIN) if float(rel_pos @ dock["n"]) > 0.05 else None
                img = det_dock.frame("dock", ("red", "Square") if a is not None else None, None if a is None else a[1], Clock.t); col = None
            if img is not None:
                msg = Float32(img)
                sb.angle_cb(msg); st.angle_cb(msg); sk.image_angle_cb(msg)
                if col is not None:
                    st.color_cb(String(col))
            # ---- ship_goal_angle ----
            c = sga.pubs["/yaw_error"].count
            sga.publish_yaw_error()
            if sga.pubs["/yaw_error"].count > c:
                sd.yaw_error_callback(sga.pubs["/yaw_error"].last)
            # ---- 미션 노드 ----
            for nm_, nd_ in (("ship_gate", sg), ("ship_back", sb), ("ship_turn", st), ("ship_dock", sk)):
                if silent != nm_:
                    nd_.timer_cb()
            for name, n in cand_pubs[1:]:
                p = n.pubs["/candidate_angle"]
                if p.count > cnt[name]:
                    tick_cand.append((name, p.last))
            for name, n in cand_pubs:
                cnt[name] = n.pubs["/candidate_angle"].count
            mp = mode_pubs.setdefault(cur_mode, {})
            names = sorted(set(nm for nm, _ in tick_cand))
            for nm in names:
                mp[nm] = mp.get(nm, 0) + 1
            if len(names) > 1:
                multi_pub_ticks[cur_mode] = multi_pub_ticks.get(cur_mode, 0) + 1
            for nm, m in tick_cand:
                sd.candidate_callback(m); nga.candidate_cb(m); last_cand = m.data
            # ---- ship_direction → motor_control ----
            sd.watchdog_cb(); sd.control_cb()
            if sd.pubs["/desired_angle"].last is not None:
                mc.angle_callback(sd.pubs["/desired_angle"].last)
            if sd.pubs["/obstacle_distance_array"].last is not None:
                mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
            if sd.pubs["/failsafe_level"].last is not None:
                mc.failsafe_callback(sd.pubs["/failsafe_level"].last)
            mc.timer_callback()
            # 로그 기반 감시 (노드가 남긴 경고 문구로 판정)
            flags = {}
            for key, nd_ in (("nga", nga), ("sga", sga), ("mc", mc)):
                lines = nd_.get_logger().lines
                for lv_, ln_ in lines[log_idx[key]:]:
                    if lv_ not in ("WARN", "ERROR"):
                        continue
                    if key == "mc" and "/desired_angle stale" in ln_: flags["wd"] = True
                    if key == "nga" and ("fix stale" in ln_ or "여전히 stale" in ln_ or "fix 없음" in ln_ or "fix 상실" in ln_): flags["gps"] = True
                    if key == "nga" and ("/imu/yaw stale" in ln_ or "/imu/yaw 없음" in ln_): flags["n_imu"] = True
                    if key == "sga" and "IMU stale" in ln_: flags["sga_imu"] = True
                    if key == "sga" and "목표각 stale" in ln_: flags["sga_goal"] = True
                log_idx[key] = len(lines)
            code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
            if boat3 is not None:
                x, y, psi = boat3.step(pl, pr, DT, current); v = boat3.u; w = boat3.yaw_rate_dps
            else:
                TR, TL = 1500 - pr, 1500 - pl
                v_cmd, w_cmd = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
                v += (v_cmd - v) * DT / TAU; w += (w_cmd - w) * DT / TAU
                psi = (psi + w * DT) % 360.0
                x += (v * math.sin(math.radians(psi)) + current[0]) * DT
                y += (v * math.cos(math.radians(psi)) + current[1]) * DT
            t += DT; k += 1; Clock.t = t
            pos = np.array([x, y])
            # ---- 판정 ----
            for i, g in enumerate(W["gates"]):
                if cross_seg(prev_xy, pos, g["red"], g["green"]):
                    if float((pos - prev_xy) @ W["gate_u"]) > 0: gate_cross[i] += 1
                    else: gate_wrong[i] += 1
            prev_xy = pos
            for i, b in enumerate(W["buoys"]):
                dd = math.hypot(x - b["x"], y - b["y"]) - BUOY_R
                on = dd < HALF_W
                if on and not contact_on.get(("b", i)):
                    kind = b["kind"]; key = "gate" if kind.startswith("gate") else ("search" if kind.startswith("search") else kind)
                    contacts[key] += 1; contacts_by_mode[cur_mode] = contacts_by_mode.get(cur_mode, 0) + 1
                    if key in ("hold", "search"):        # 미션 부표 접촉 때 ship_back 상태(원인 진단용)
                        ks = f"{kind}@WP{nga.wp_idx}(m{cur_mode}):{sb.state}"
                        contacts_sb_state[ks] = contacts_sb_state.get(ks, 0) + 1
                contact_on[("b", i)] = on if on else (dd < HALF_W + 0.1 and contact_on.get(("b", i), False))
                rd = rect_hull_dist(x, y, psi, b["x"], b["y"], rect_hl, rect_hb)
                ron = rd < BUOY_R
                if ron and not rect_on.get(i):
                    kind = b["kind"]; key = "gate" if kind.startswith("gate") else ("search" if kind.startswith("search") else kind)
                    contacts_rect[key] = contacts_rect.get(key, 0) + 1
                rect_on[i] = ron if ron else (rd < BUOY_R + 0.1 and rect_on.get(i, False))
            if math.hypot(x - dock["O"][0], y - dock["O"][1]) < 4.0:
                h = np.array([math.sin(math.radians(psi)), math.cos(math.radians(psi))])
                bow, stern = pos + h * HULL_HALF_L, pos - h * HULL_HALF_L
                for j, (a_, b_, kind) in enumerate(W["segs"]):
                    dd = seg_seg_dist(stern, bow, a_, b_)
                    on = dd < (0.02 if kind == "front" else HALF_W)
                    if on and not contact_on.get(("s", j)):
                        contacts["dock"] += 1; contacts_by_mode[cur_mode] = contacts_by_mode.get(cur_mode, 0) + 1
                    contact_on[("s", j)] = on
            # 위치유지·탐색 기하
            if cur_mode == 2:
                hb = W["buoys"][W["hold_idx"]]
                hold_run = hold_run + DT if math.hypot(x - hb["x"], y - hb["y"]) <= 5.0 else 0.0
                hold_best = max(hold_best, hold_run)
            if cur_mode == 3:
                for b in W["buoys"]:
                    if b["kind"].startswith("search"):
                        kk = b["kind"]
                        srch_run[kk] = srch_run.get(kk, 0.0) + DT if math.hypot(x - b["x"], y - b["y"]) <= 5.0 else 0.0
                        srch_best[kk] = max(srch_best.get(kk, 0.0), srch_run[kk])
            # 미션 노드 상태 전이
            if sb.state != prev_sb:
                near = min(W["buoys"], key=lambda b: math.hypot(x - b["x"], y - b["y"]))
                # 포기 DONE(unknown 10 s·HOLD 총 60 s, ship_back ERROR '포기') 은 성공과 따로 센다
                sb_new = sb.get_logger().lines[sb_log_idx:]
                why = ""
                if sb.state == "DONE" and any(lv_ == "ERROR" and "포기" in ln_ for lv_, ln_ in sb_new):
                    why = "포기"
                sb_events.append((round(t, 1), cur_mode, prev_sb, sb.state, near["kind"], round(math.hypot(x - near["x"], y - near["y"]), 2), why))
                prev_sb = sb.state
            sb_log_idx = len(sb.get_logger().lines)
            dist_dock = math.hypot(x - dock["O"][0], y - dock["O"][1])
            if cur_mode == 7 and dock_mode7_t0 is None:
                dock_mode7_t0 = t; dock_dist_at["mode7"] = dist_dock
            if sk.state != prev_sk:
                sk_events.append((round(t, 1), cur_mode, prev_sk, sk.state, round(dist_dock, 1)))
                if sk.state == "CONTACT" and dock_contact is None:
                    lat_e = float((pos - dock["marker"]) @ dock["t"])
                    dock_contact = dict(t=t, lat=lat_e, head=abs(wrap(psi - (math.degrees(math.atan2(-dock["n"][0], -dock["n"][1]))))))
                prev_sk = sk.state
            if nga.wp_idx == i_last and math.hypot(x - WPXY[i_last][0], y - WPXY[i_last][1]) <= 2.5 and t_last_reach is None:
                t_last_reach = t
            dl = sd.pubs["/desired_angle"].last
            TK["t"].append(t); TK["lvl"].append(sd.failsafe_level); TK["des"].append(None if dl is None else dl.data)
            TK["v"].append(v); TK["mode"].append(cur_mode); TK["wp"].append(nga.wp_idx); TK["ctot"].append(sum(contacts.values()))
            for kk_ in ("wd", "gps", "sga_imu", "sga_goal", "n_imu"):
                TK[kk_].append(bool(flags.get(kk_)))
            if record:
                for kk_, vv_ in (("t", t), ("x", x), ("y", y), ("psi", psi), ("v", v), ("mode", cur_mode), ("wp", nga.wp_idx),
                                 ("sk", sk.state), ("sb", sb.state), ("cand", last_cand), ("des", sd.pubs["/desired_angle"].last.data if sd.pubs["/desired_angle"].last else None),
                                 ("dd", dist_dock)):
                    rec[kk_].append(vv_)
            if nga.wp_idx >= n_wps:
                break
            if t_last_reach is not None and t - t_last_reach > tail:
                break
    except Exception:
        ok = False
        out["exception"] = traceback.format_exc().strip().splitlines()[-1]
        out["traceback"] = traceback.format_exc()
    # ---- 요약 ----
    out.update(t_end=round(t, 1), finished_all=nga.wp_idx >= n_wps, t_course=None if t_last_reach is None else round(t_last_reach, 1),
               wp_reached_last=nga.wp_idx, n_wps=n_wps)
    seg = []
    for i in range(n_wps):
        if i in wp_enter:
            t1 = wp_enter.get(i + 1, t)
            seg.append(dict(i=i, mode=wps[i]["mode"], label=wps[i]["label"], t0=round(wp_enter[i], 1), dur=round(t1 - wp_enter[i], 1), how=wp_how.get(i, "진행중")))
    out["segments"] = seg
    out["timeouts"] = sum(1 for s in seg if s["how"] == "시간초과")
    out["gates_geo"] = sum(1 for c in gate_cross if c > 0)
    out["gates_wrongdir"] = sum(gate_wrong)
    gp = sg.pubs["/gates_passed"].last
    out["gates_node"] = gp.data if gp is not None else -1
    out["hold_geo"] = hold_best >= 5.0
    out["hold_best_s"] = round(hold_best, 1)
    m2 = [e for e in sb_events if e[1] == 2]
    done2 = [e for e in m2 if e[3] == "DONE"]
    if not done2:
        out["hold_node"] = "미완"
    elif done2[0][6] == "포기":
        out["hold_node"] = "포기"
    else:
        out["hold_node"] = "HOLD→DONE" if done2[0][2] == "HOLD" else "SEARCH 타임아웃"
    m3 = [e for e in sb_events if e[1] == 3]
    done3 = [e for e in m3 if e[3] == "DONE"]
    if done3:
        e = done3[0]
        out["search_node"] = ("포기" if e[6] == "포기" else ("HOLD→DONE" if e[2] == "HOLD" else "SEARCH 타임아웃"))
        out["search_at"] = e[4] if e[2] == "HOLD" else ""
    else:
        out["search_node"] = "미완"; out["search_at"] = ""
    out["search_ok"] = out["search_node"] == "HOLD→DONE" and out["search_at"] == "search_red"
    out["search_geo_red"] = srch_best.get("search_red", 0.0) >= 5.0
    out["search_best"] = {kk: round(vv, 1) for kk, vv in srch_best.items()}
    out["dock_contact"] = dock_contact is not None
    out["dock_inbay"] = dock_contact is not None and abs(dock_contact["lat"]) <= 0.5
    out["dock_lat"] = None if dock_contact is None else round(dock_contact["lat"], 2)
    out["dock_backout"] = _backoff_by_dist(sk_events)
    out["dock_search_timeout"] = any(e[2] == "SEARCH" and e[3] == "DONE" for e in sk_events)
    out["dock_dist_mode7"] = None if "mode7" not in dock_dist_at else round(dock_dist_at["mode7"], 1)
    out["sk_events"] = sk_events; out["sb_events"] = sb_events
    out["contacts"] = contacts; out["contacts_total"] = sum(contacts.values())
    out["contacts_avoid"] = contacts["obst_5"] + contacts["obst_8"]
    out["contacts_rect"] = contacts_rect; out["contacts_rect_avoid"] = contacts_rect["obst_5"] + contacts_rect["obst_8"]
    out["contacts_by_mode"] = contacts_by_mode
    out["contacts_sb_state"] = contacts_sb_state
    out["mode_pubs"] = mode_pubs; out["multi_pub_ticks"] = multi_pub_ticks
    out["turn_published"] = st.pubs["/candidate_angle"].count
    out["img_pub_conflict"] = img_pub_conflict
    errs = {}
    for name, n in nodes.items():
        for lv, ln in n.get_logger().lines:
            if lv == "ERROR":
                errs.setdefault(name, set()).add(ln.splitlines()[0][:70])
    out["node_errors"] = {kk: sorted(vv) for kk, vv in errs.items()}
    out["ok"] = ok
    out.update(failsafe_summary(TK))
    if fault is not None:
        out.update(fault_metrics(TK, f_kind, f_t0, f_dur, f_node, f_cand0, wps))
    out["rec"] = rec; out["W"] = W if record else None; out["wps"] = wps if record else None
    return out


FAULT_DUR = {"scan2": 2.0, "scan5": 5.0, "gps10": 10.0, "imu3": 3.0, "node3": 3.0}
FAULT_NAME = {"scan2": "(a) /scan 2 s 끊김", "scan5": "(b) /scan 5 s 끊김", "gps10": "(c) GPS fix 10 s 끊김",
              "imu3": "(d) /imu/yaw 3 s 끊김", "node3": "(e) 담당 미션 노드 발행 정지 3 s"}
NODE_GROUP_MODES = {"gate": (0, 1), "back": (2, 3), "dock": (7,), "north": (5, 8)}
NODE_GROUP_OFFSET = {"gate": (1.0, 8.0), "back": (2.0, 40.0), "dock": (1.0, 20.0), "north": (5.0, 60.0)}
MODE_OWNER = {0: "ship_gate", 1: "ship_gate", 2: "ship_back", 3: "ship_back", 7: "ship_dock", 5: "north_goal_angle", 8: "north_goal_angle"}


def _edges(flags):
    n = 0; prev = False
    for f in flags:
        if f and not prev: n += 1
        prev = f
    return n


def failsafe_summary(TK):
    L = TK["lvl"]; n1 = n2 = 0; prev = 0; first = None
    for i, lv in enumerate(L):
        if lv >= 1 and prev < 1: n1 += 1
        if lv >= 2 and prev < 2: n2 += 1
        if lv >= 1 and first is None: first = i
        prev = lv
    stop_by_mode = {}
    for d, m in zip(TK["des"], TK["mode"]):
        if d is not None and d >= 50000.0:
            stop_by_mode[m] = stop_by_mode.get(m, 0.0) + DT
    return dict(fs_l1_n=n1, fs_l2_n=n2, fs_l1_s=round(sum(DT for lv in L if lv == 1), 1), fs_l2_s=round(sum(DT for lv in L if lv >= 2), 1),
                fs_first_t=None if first is None else round(TK["t"][first], 1),
                fs_first_wp=None if first is None else TK["wp"][first], fs_first_mode=None if first is None else TK["mode"][first],
                mc_wd_n=_edges(TK["wd"]), mc_wd_s=round(sum(DT for f in TK["wd"] if f), 1),
                gps_stale_n=_edges(TK["gps"]), sga_imu_stale_n=_edges(TK["sga_imu"]), sga_goal_stale_n=_edges(TK["sga_goal"]),
                north_imu_stale_n=_edges(TK["n_imu"]),
                stop_hold_s=round(sum(stop_by_mode.values()), 1), stop_hold_by_mode={m: round(v, 1) for m, v in sorted(stop_by_mode.items())})


def fault_metrics(TK, kind, t0, dur, node, cand0, wps):
    out = dict(fault=kind, fault_t0=None if t0 is None else round(t0, 1), fault_node=node, fault_cand0=cand0)
    if t0 is None:
        out["fault_applied"] = False
        return out
    T = TK["t"]; L = TK["lvl"]; t1 = t0 + dur
    idx = [i for i, t in enumerate(T) if t0 <= t < t1]
    if not idx:
        out["fault_applied"] = False
        return out
    i0 = idx[0]
    out.update(fault_applied=True, fault_wp=TK["wp"][i0], fault_mode=TK["mode"][i0])
    def first_t(cond, lim):
        for i in range(i0, len(T)):
            if T[i] > lim: break
            if cond(i): return round(T[i] - t0, 1)
        return None
    out["t_L1"] = first_t(lambda i: L[i] >= 1, t1 + 5.0)
    out["t_L2"] = first_t(lambda i: L[i] >= 2, t1 + 5.0)
    win = [i for i, t in enumerate(T) if t0 <= t < t1 + 1.0]
    out["max_lvl"] = max(L[i] for i in win)
    out["wd_ticks"] = sum(1 for i in win if TK["wd"][i])
    out["t_wd"] = first_t(lambda i: TK["wd"][i], t1 + 5.0)
    out["stop_s"] = round(sum(DT for i in idx if TK["des"][i] is not None and TK["des"][i] >= 50000.0), 1)
    pre = [TK["v"][i] for i, t in enumerate(T) if t0 - 3.0 <= t < t0]
    out["v_before"] = round(sum(pre) / len(pre), 2) if pre else None
    out["v_in_mean"] = round(sum(TK["v"][i] for i in idx) / len(idx), 2)
    late = [i for i in idx if T[i] >= t0 + min(dur, 1.5)]
    out["v_in_late"] = round(sum(TK["v"][i] for i in late) / len(late), 2) if late else None
    rec = None
    if out["max_lvl"] >= 1:
        for i in range(i0, len(T)):
            if T[i] >= t1 and L[i] == 0:
                rec = round(T[i] - t1, 1); break
    out["recover_s"] = rec
    def ct(tt):
        j = min(range(len(T)), key=lambda i: abs(T[i] - tt)); return TK["ctot"][j]
    out["contacts_fault"] = ct(t1 + 3.0) - ct(t0)
    out["sga_goal_stale"] = any(TK["sga_goal"][i] for i in win)
    out["sga_imu_stale"] = any(TK["sga_imu"][i] for i in win)
    out["gps_stale_flag"] = any(TK["gps"][i] for i in range(i0, len(T)) if T[i] < t1 + 3.0)
    return out


def _backoff_by_dist(sk_events):
    """BACKOFF→DONE 이 backoff_dist(2.5 m) 도달로 끝났나 — 이벤트 시각 간격으로 판정(타임아웃 8 s 미만)."""
    t_b = None
    for e in sk_events:
        if e[3] == "BACKOFF": t_b = e[0]
        if e[2] == "BACKOFF" and e[3] == "DONE" and t_b is not None:
            return (e[0] - t_b) < 8.0 - 1e-6
    return False


# ───────────────────────── 변형 B: 도킹 네 줄을 목표 칸에 맞게 dock_wp.py 로 교체 ─────────────────────────
_B_CACHE = {}


def dock_block(src_lines):
    """기본 yaml 줄 목록에서 도킹 네 줄(접근 10m·6m mode 5, WP8 입구·WP9 이탈 mode 7)의 줄 번호 4개.

    첫 mode 7 항목과 그 앞 두 항목·뒤 한 항목이 mode [5, 5, 7, 7] 이어야 한다(계약서 C3 구조). 아니면 AssertionError.
    """
    items = [(k, int(yaml.safe_load(ln.strip())[0]["mode"]))
             for k, ln in enumerate(src_lines) if ln.strip().startswith("- {")]
    j = next(q for q, (_, m) in enumerate(items) if m == 7)
    assert j >= 2 and [m for _, m in items[j - 2:j + 2]] == [5, 5, 7, 7], \
        f"도킹 네 줄 구조가 아니다: {[m for _, m in items]}"
    return [k for k, _ in items[j - 2:j + 2]]


def variant_b_file(W):
    """기본 waypoints.yaml(WP_DEFAULT)의 도킹 네 줄(⚠️ 현장 교체)을 목표 칸 dock_wp.py 출력 네 줄로 교체한 사본."""
    bay = W["dock"]["bay"]
    if bay in _B_CACHE:
        return _B_CACHE[bay]
    os.makedirs(SCRATCH_B, exist_ok=True)
    tag = {-1.0: "m1", 0.0: "0", 1.0: "p1"}[bay]
    path = os.path.join(SCRATCH_B, f"waypoints_B_bay{tag}.yaml")
    geo = W["geo"]; d = W["dock"]
    entr = d["marker"] + 0.5 * d["n"]                      # 칸 입구 중심(칸막이 끝 선 위)
    lat, lon = geo.ll(*entr)
    out_deg = math.degrees(math.atan2(d["n"][0], d["n"][1])) % 360.0
    res = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "dock_wp.py"),
                          "--bay", f"{lat:.9f}", f"{lon:.9f}", "--out-deg", f"{out_deg:.3f}"],
                         capture_output=True, text=True, check=True)
    lines = [ln.strip() for ln in res.stdout.splitlines() if ln.strip().startswith("- {")]
    assert len(lines) == 4, res.stdout                     # 접근 10m·6m, WP8 입구, WP9 이탈
    with open(WP_DEFAULT, encoding="utf-8") as f:
        src = f.read().splitlines()
    blk = dock_block(src)
    head = src[:blk[0]]
    if head and head[-1].strip().startswith("# ↓"):        # 자리표시값 설명 줄은 뺀다
        head = head[:-1]
    note = (f"  # ↓ tools/dock_wp.py --bay {lat:.9f} {lon:.9f} --out-deg {out_deg:.3f}"
            f" (full_course_sim 변형 B, 목표 칸 {bay:+.0f})")
    outl = head + [note] + ["  " + ln for ln in lines] + src[blk[-1] + 1:]
    tmp = f"{path}.{os.getpid()}.tmp"                    # 병렬 워커가 같은 파일을 쓰므로 원자적 교체
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(outl) + "\n")
    os.replace(tmp, path)
    _B_CACHE[bay] = path
    return path


# ───────────────────────── 출력 ─────────────────────────
def wilson(k, n, z=1.96):
    if n == 0: return (float("nan"), float("nan"))
    p = k / n; den = 1 + z * z / n; c = (p + z * z / (2 * n)) / den; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


CSV_COLS = ["variant", "seed", "bay", "ok", "exception", "t_course", "t_end", "finished_all", "timeouts",
            "gates_geo", "gates_node", "gates_wrongdir", "hold_geo", "hold_best_s", "hold_node",
            "search_ok", "search_node", "search_at", "search_geo_red", "dock_dist_mode7", "dock_search_timeout",
            "dock_contact", "dock_inbay", "dock_lat", "dock_backout", "contacts_total", "contacts_avoid",
            "c_gate", "c_hold", "c_search", "c_obst5", "c_obst8", "c_dock", "turn_published", "multi_pub_ticks",
            "fs_l1_n", "fs_l2_n", "fs_l1_s", "fs_l2_s", "fs_first_t", "fs_first_wp", "fs_first_mode", "mc_wd_n", "mc_wd_s",
            "gps_stale_n", "north_imu_stale_n", "sga_imu_stale_n", "sga_goal_stale_n", "stop_hold_s", "stop_hold_by_mode",
            "segments", "sk_events", "node_errors", "mode_pubs"]
FAULT_COLS = ["fault", "fault_applied", "fault_t0", "fault_wp", "fault_mode", "fault_node", "fault_cand0", "t_L1", "t_L2", "max_lvl", "t_wd", "wd_ticks",
              "stop_s", "v_before", "v_in_mean", "v_in_late", "recover_s", "contacts_fault", "sga_goal_stale", "sga_imu_stale", "gps_stale_flag",
              "base_t_course", "d_t_course", "mission_continued", "same_outcome"]


def to_row(r):
    c = r["contacts"]
    seg = " | ".join(f"WP{s['i']}(m{s['mode']}) {s['dur']:.0f}s {s['how']}" for s in r["segments"])
    ske = " ; ".join(f"{e[0]:.0f}s m{e[1]} {e[2]}→{e[3]} d{e[4]}" for e in r["sk_events"])
    return [r["variant"], r["seed"], r["bay"], int(r["ok"]), r["exception"], r["t_course"], r["t_end"], int(r["finished_all"]), r["timeouts"],
            r["gates_geo"], r["gates_node"], r["gates_wrongdir"], int(r["hold_geo"]), r["hold_best_s"], r["hold_node"],
            int(r["search_ok"]), r["search_node"], r["search_at"], int(r["search_geo_red"]), r["dock_dist_mode7"], int(r["dock_search_timeout"]),
            int(r["dock_contact"]), int(r["dock_inbay"]), r["dock_lat"], int(r["dock_backout"]), r["contacts_total"], r["contacts_avoid"],
            c["gate"], c["hold"], c["search"], c["obst_5"], c["obst_8"], c["dock"], r["turn_published"], str(r["multi_pub_ticks"]),
            r["fs_l1_n"], r["fs_l2_n"], r["fs_l1_s"], r["fs_l2_s"], r["fs_first_t"], r["fs_first_wp"], r["fs_first_mode"], r["mc_wd_n"], r["mc_wd_s"],
            r["gps_stale_n"], r["north_imu_stale_n"], r["sga_imu_stale_n"], r["sga_goal_stale_n"], r["stop_hold_s"], str(r["stop_hold_by_mode"]),
            seg, ske, str(r["node_errors"]), str({m: sorted(v) for m, v in r["mode_pubs"].items()})] + \
           ([r.get(kk) for kk in FAULT_COLS] if "fault" in r else [])


def summarize(R, label):
    n = len(R)
    def S(key): return sum(1 for r in R if r[key])
    def ci(k): lo, hi = wilson(k, n); return f"{k}/{n} [{lo * 100:.0f}–{hi * 100:.0f}%]"
    tc = sorted(r["t_course"] for r in R if r["t_course"] is not None)
    tend = sorted(r["t_end"] for r in R)
    med = lambda a: (a[len(a) // 2] if len(a) % 2 else 0.5 * (a[len(a) // 2 - 1] + a[len(a) // 2])) if a else float("nan")
    lines = [f"[{label}] {n} 런"]
    lines.append(f"  예외 {sum(1 for r in R if not r['ok'])} | ship_turn 발행 합 {sum(r['turn_published'] for r in R)} | "
                 f"/candidate_angle 같은 틱 발행자 2개 이상: {sum(sum(r['multi_pub_ticks'].values()) for r in R)} 틱 | /image_angle 충돌 {sum(r['img_pub_conflict'] for r in R)}")
    g4 = sum(1 for r in R if r["gates_geo"] == 4); gm = sum(1 for r in R if r["gates_node"] == r["gates_geo"])
    lines.append(f"  게이트 4/4 기하 통과 {ci(g4)} | 기하 통과 합 {sum(r['gates_geo'] for r in R)}/{4 * n} | /gates_passed = 기하 {gm}/{n} | 역방향 가로지름 {sum(r['gates_wrongdir'] for r in R)}")
    lines.append(f"  위치유지 규정(5 m·5 s) {ci(S('hold_geo'))} | 노드 HOLD→DONE {sum(1 for r in R if r['hold_node'] == 'HOLD→DONE')}, SEARCH 타임아웃 {sum(1 for r in R if r['hold_node'] == 'SEARCH 타임아웃')}, 포기 {sum(1 for r in R if r['hold_node'] == '포기')}")
    at = {}
    for r in R:
        at[r["search_at"] or r["search_node"]] = at.get(r["search_at"] or r["search_node"], 0) + 1
    lines.append(f"  탐색(빨강 앞 HOLD→DONE) {ci(S('search_ok'))} | 빨강 기하 5 m·5 s {S('search_geo_red')}/{n} | mode 3 종료 대상 {at}")
    lines.append(f"  도킹 칸 안 접안 {ci(S('dock_inbay'))} | CONTACT {S('dock_contact')}/{n} | 후진 이탈 {S('dock_backout')}/{n} | SEARCH 타임아웃 {S('dock_search_timeout')}/{n} | "
                 f"mode 7 진입 시 도크 거리 중앙값 {med(sorted(r['dock_dist_mode7'] for r in R if r['dock_dist_mode7'] is not None)):.1f} m")
    lines.append(f"  코스 시간(WP10 2.5 m 도달) 중앙값 {med(tc):.0f} s (도달 {len(tc)}/{n}, ≤900 s {sum(1 for v in tc if v <= 900)}/{n}) | 런 종료 중앙값 {med(tend):.0f} s | 전 WP 완료 {S('finished_all')}/{n}")
    lines.append(f"  WP 시간 초과 합 {sum(r['timeouts'] for r in R)} (런당 중앙값 {med(sorted(r['timeouts'] for r in R))}) | 접촉 합 {sum(r['contacts_total'] for r in R)} "
                 f"(회피 장애물 {sum(r['contacts_avoid'] for r in R)}, 게이트 {sum(r['contacts']['gate'] for r in R)}, 위치유지 {sum(r['contacts']['hold'] for r in R)}, "
                 f"탐색 부표 {sum(r['contacts']['search'] for r in R)}, 도크 {sum(r['contacts']['dock'] for r in R)})")
    if all("contacts_rect" in r for r in R):
        lines.append(f"  직사각형 선체 접촉(보조, 선미 포함) 회피 장애물 {sum(r['contacts_rect_avoid'] for r in R)}, "
                     f"게이트 {sum(r['contacts_rect']['gate'] for r in R)}, 위치유지 {sum(r['contacts_rect']['hold'] for r in R)}, "
                     f"탐색 부표 {sum(r['contacts_rect']['search'] for r in R)}")
    cs = {}
    for r in R:
        for kk, vv in r.get("contacts_sb_state", {}).items():
            cs[kk] = cs.get(kk, 0) + vv
    if cs:
        lines.append("  미션 부표 접촉 때 ship_back 상태: " + ", ".join(f"{kk} {vv}" for kk, vv in sorted(cs.items())))
    # WP 별 시간초과·소요
    nw = max(len(r["segments"]) for r in R)
    parts = []
    for i in range(nw):
        ss = [r["segments"][i] for r in R if len(r["segments"]) > i]
        if not ss: continue
        parts.append(f"WP{i}(m{ss[0]['mode']}) {med(sorted(s['dur'] for s in ss)):.0f}s 초과{sum(1 for s in ss if s['how'] == '시간초과')}")
    lines.append("  WP별 소요 중앙값·시간초과 수: " + ", ".join(parts))
    fs_runs = sum(1 for r in R if r["fs_l1_n"] or r["fs_l2_n"] or r["mc_wd_n"])
    lines.append(f"  페일세이프(고장 없음): 거짓 정지·감속 런 {fs_runs}/{n} (L1 진입 합 {sum(r['fs_l1_n'] for r in R)}·{sum(r['fs_l1_s'] for r in R):.1f} s, "
                 f"L2 진입 합 {sum(r['fs_l2_n'] for r in R)}·{sum(r['fs_l2_s'] for r in R):.1f} s, motor 워치독 중립 {sum(r['mc_wd_n'] for r in R)}회) | "
                 f"north GPS stale/fix 없음 경고 {sum(r['gps_stale_n'] for r in R)}회, north IMU stale {sum(r['north_imu_stale_n'] for r in R)}회, "
                 f"ship_goal_angle IMU stale {sum(r['sga_imu_stale_n'] for r in R)}회·목표각 stale {sum(r['sga_goal_stale_n'] for r in R)}회")
    sbm = {}
    for r in R:
        for m, v in r["stop_hold_by_mode"].items(): sbm.setdefault(m, []).append(v)
    lines.append(f"  /desired_angle 50000(STOP_HOLD) 런당 중앙값 {med(sorted(r['stop_hold_s'] for r in R)):.1f} s — 모드별 중앙값(해당 런): "
                 + ", ".join(f"m{m} {med(sorted(v)):.1f} s({len(v)}런)" for m, v in sorted(sbm.items())))
    errs = {}
    for r in R:
        for nm, es in r["node_errors"].items():
            for e in es: errs[(nm, e)] = errs.get((nm, e), 0) + 1
    if errs:
        lines.append("  노드 ERROR 로그: " + "; ".join(f"{nm}: {e} ×{c}" for (nm, e), c in errs.items()))
    pubs = {}
    for r in R:
        for m, d in r["mode_pubs"].items():
            pubs.setdefault(m, set()).update(d.keys())
    lines.append("  모드별 /candidate_angle 발행자: " + ", ".join(f"m{m}: {sorted(v)}" for m, v in sorted(pubs.items())))
    return "\n".join(lines)


def timeline_text(r):
    L = [f"  시간축 (seed {r['seed']}, 변형 {r['variant']}, 목표 칸 {r['bay']:+.0f}):"]
    for s in r["segments"]:
        L.append(f"    WP{s['i']:>2} m{s['mode']} {s['label']:<16} 진입 {s['t0']:6.1f}s  소요 {s['dur']:6.1f}s  {s['how']}")
    L.append("    ship_dock 전이: " + " ; ".join(f"{e[0]:.1f}s(m{e[1]}) {e[2]}→{e[3]} 도크까지 {e[4]} m" for e in r["sk_events"]))
    L.append("    ship_back 전이: " + " ; ".join(f"{e[0]:.1f}s(m{e[1]}) {e[2]}→{e[3]}{'(' + e[6] + ')' if e[6] else ''} 최근접 {e[4]} {e[5]} m" for e in r["sb_events"]))
    return "\n".join(L)


def plot(r, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
    matplotlib.rcParams["axes.unicode_minus"] = False
    W, rec, wps = r["W"], r["rec"], r["wps"]
    fig = plt.figure(figsize=(14, 12.5))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.2, 1.0], hspace=0.22)
    ax = fig.add_subplot(gs[0])
    X, Y, M = np.array(rec["x"]), np.array(rec["y"]), rec["mode"]
    i0 = 0
    for i in range(1, len(X) + 1):
        if i == len(X) or M[i] != M[i0]:
            ax.plot(X[i0:i + 1], Y[i0:i + 1], color=MODE_COL.get(M[i0], "k"), lw=1.6)
            i0 = i
    colmap = {"red": "crimson", "green": "seagreen", "white": "white", "orange": "#ff7f0e"}
    for b in W["buoys"]:
        ax.add_patch(plt.Circle((b["x"], b["y"]), 0.45 if b["kind"] in ("hold",) or b["kind"].startswith("search") else 0.3,
                                fc=colmap[b["color"]], ec="k", lw=0.7, zorder=4))
    for g in W["gates"]:
        ax.plot([g["red"][0], g["green"][0]], [g["red"][1], g["green"][1]], color="k", ls=":", lw=0.7)
    for a_, b_, kind in W["segs"]:
        ax.plot([a_[0], b_[0]], [a_[1], b_[1]], color="#222", lw=3 if kind == "front" else 2)
    mk = W["dock"]["marker"]; ax.plot(mk[0], mk[1], "s", color="crimson", ms=8, zorder=5)
    WPXY = [W["geo"].xy(w["lat"], w["lon"]) for w in wps]
    for i, (wx, wy) in enumerate(WPXY):
        ax.plot(wx, wy, "+", color="k", ms=9, mew=1.4)
        ax.annotate(f"WP{i}", (wx, wy), xytext=(4, 4 + (8 if i == 9 or (i > 0 and WPXY[i] == WPXY[i - 1]) else 0)), textcoords="offset points", fontsize=7.5)
    ax.plot(X[0], Y[0], "o", color="k", ms=6); ax.annotate("출발", (X[0], Y[0]), xytext=(4, -10), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal"); ax.grid(alpha=0.2); ax.set_xlabel("x [m] (동)"); ax.set_ylabel("y [m] (북)")
    handles = [Line2D([0], [0], color=MODE_COL[m], lw=3, label=f"mode {MODE_NAME[m]}") for m in sorted(MODE_COL) if m in set(M)]
    handles += [Line2D([0], [0], marker="o", ls="none", mfc="#ff7f0e", mec="k", label="장애물"),
                Line2D([0], [0], marker="s", ls="none", color="crimson", label="도크 목표 표식"),
                Line2D([0], [0], color="#222", lw=3, label="도크 벽·칸막이")]
    ax.legend(handles=handles, fontsize=8, loc="best", framealpha=0.9)
    c = r["contacts"]
    ax.set_title(f"변형 {r['variant']} seed {r['seed']} — 게이트 기하 {r['gates_geo']}/4 (/gates_passed {r['gates_node']}), 위치유지 {'달성' if r['hold_geo'] else '미달'}, "
                 f"탐색 {'성공' if r['search_ok'] else '실패'}({r['search_node']} {r['search_at']}), 도킹 {'칸 안 접안' if r['dock_inbay'] else ('접안(칸 밖)' if r['dock_contact'] else '미접안')}, "
                 f"접촉 {r['contacts_total']}, 코스 {r['t_course']} s, WP 시간초과 {r['timeouts']}", fontsize=9.5)
    # 시간축
    ax2 = fig.add_subplot(gs[1])
    T = np.array(rec["t"])
    ax2.plot(T, rec["dd"], color="#d62728", lw=1.3, label="도크(WP8)까지 거리 [m]")
    ax2.plot(T, np.array(rec["v"]) * 10, color="#1f77b4", lw=0.9, alpha=0.8, label="속도 ×10 [m/s]")
    ax2.set_ylim(-12, max(80, max(rec["dd"]) + 5))
    i0 = 0
    for i in range(1, len(T) + 1):
        if i == len(T) or M[i] != M[i0]:
            ax2.axvspan(T[i0], T[min(i, len(T) - 1)], ymin=0.0, ymax=0.06, color=MODE_COL.get(M[i0], "k"))
            i0 = i
    SKC = {"TRANSIT": "#d0d0d0", "INIT": "#9e9e9e", "SEARCH": "#8e44ad", "ALIGN": "#e67e22", "APPROACH": "#1f77b4", "CONTACT": "#d62728", "BACKOFF": "#2ca02c", "DONE": "#555555"}
    i0 = None
    for i in range(len(T) + 1):
        act = i < len(T) and M[i] == 7
        if act and i0 is None: i0 = i
        if (not act or (i < len(T) and i0 is not None and rec["sk"][i] != rec["sk"][i0])) and i0 is not None:
            ax2.axvspan(T[i0], T[min(i, len(T) - 1)], ymin=0.07, ymax=0.13, color=SKC[rec["sk"][i0]])
            if T[min(i, len(T) - 1)] - T[i0] > 12:
                ax2.text((T[i0] + T[min(i, len(T) - 1)]) / 2, -8.2, rec["sk"][i0], ha="center", fontsize=7, color="white", weight="bold")
            i0 = i if act else None
    for s in r["segments"]:
        ax2.axvline(s["t0"], color="#999", lw=0.6, ls=":")
        ax2.text(s["t0"], ax2.get_ylim()[1] * 0.97, f"WP{s['i']}", fontsize=6.5, rotation=90, va="top")
    ax2.axhline(900, color="k")
    ax2.set_xlabel("시간 [s]  (아래 띠: mode 색 / 그 위 띠: mode 7 동안 ship_dock 상태)"); ax2.legend(fontsize=8, loc="upper right"); ax2.grid(alpha=0.2)
    if r["t_course"]:
        ax2.axvline(r["t_course"], color="g", lw=1.2); ax2.text(r["t_course"], 5, " WP10 도달", color="g", fontsize=8)
    ax2.axvline(900, color="k", lw=1.0, ls="--"); ax2.text(900, 60, " 900 s", fontsize=8)
    ax2.set_title("시간축 — 도크까지 거리, 속도, mode, ship_dock 상태", fontsize=10)
    fig.suptitle("전체 코스 2D 시뮬 — 실제 north_goal_angle·ship_goal_angle·ship_direction·motor_control·ship_gate·ship_back·ship_turn·ship_dock 코드, "
                 "카메라·LiDAR·GPS·운동은 모델 (tools/sim/full_course_sim.py)", fontsize=10.5, y=0.995)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    print("저장:", path)


def _worker(args):
    seed, variant, kw = args
    t0 = _walltime.perf_counter()
    r = run(seed, variant, **kw)
    r["wall"] = _walltime.perf_counter() - t0
    r.pop("rec", None); r.pop("W", None); r.pop("wps", None)
    return r


def make_fault(kind, seed):
    fr = random.Random(seed * 1000 + list(FAULT_DUR).index(kind))
    if kind == "node3":
        g = ["gate", "back", "dock", "north"][seed % 4]
        return dict(kind=kind, group=g, offset=fr.uniform(*NODE_GROUP_OFFSET[g]))
    return dict(kind=kind, t0=fr.uniform(10.0, 1000.0))


def _fworker(args):
    seed, variant, kw, fault = args
    t0 = _walltime.perf_counter()
    r = run(seed, variant, fault=fault, **kw)
    r["wall"] = _walltime.perf_counter() - t0
    r.pop("rec", None); r.pop("W", None); r.pop("wps", None)
    return r


def fault_batch(a, variants, kw):
    import multiprocessing as mp
    seeds = list(range(a.seed0, a.seed0 + max(1, a.seeds)))
    jobs = [(s, vv, kw, None) for vv in variants for s in seeds]
    jobs += [(s, vv, kw, make_fault(kind, s)) for vv in variants for kind in FAULT_DUR for s in seeds]
    t0 = _walltime.perf_counter()
    with mp.get_context("spawn").Pool(a.jobs) as pool:
        R = pool.map(_fworker, jobs, chunksize=1)
    wall = _walltime.perf_counter() - t0
    base = {(r["variant"], r["seed"]): r for r in R if "fault" not in r}
    for r in R:
        if "fault" not in r: continue
        b = base[(r["variant"], r["seed"])]
        r["base_t_course"] = b["t_course"]
        r["d_t_course"] = None if (r["t_course"] is None or b["t_course"] is None) else round(r["t_course"] - b["t_course"], 1)
        r["mission_continued"] = r["t_course"] is not None and r["wp_reached_last"] >= b["wp_reached_last"]
        keyf = lambda q: (q["gates_geo"], q["hold_geo"], q["search_ok"], q["dock_inbay"], q["dock_contact"])
        r["same_outcome"] = keyf(r) == keyf(b)
    path = os.path.join(OUT_DIR, "full_course_sim_faults.csv") if a.csv == os.path.join(OUT_DIR, "full_course_sim.csv") else a.csv
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(CSV_COLS + FAULT_COLS + ["wall_s"])
        for r in R:
            row = to_row(r)
            if "fault" not in r: row += [""] * len(FAULT_COLS)
            wr.writerow(row + [f"{r['wall']:.1f}"])
    print(f"고장 주입 묶음 {len(R)} 런 실행 시간 {wall:.0f} s → {path}")
    for vv in variants:
        B = [r for r in R if "fault" not in r and r["variant"] == vv]
        print(summarize(B, f"변형 {vv} 기준(고장 없음)"))
        for kind in FAULT_DUR:
            F = [r for r in R if r.get("fault") == kind and r["variant"] == vv]
            print(fault_summary(F, f"변형 {vv} {FAULT_NAME[kind]}"))
    bad = [r for r in R if not r["ok"]]
    for r in bad[:3]:
        print(f"예외 seed {r['seed']} {r['variant']} {r.get('fault')}: {r.get('traceback', r['exception'])}")


def fault_summary(F, label):
    A_ = [r for r in F if r.get("fault_applied")]
    n = len(A_)
    def med(v):
        v = sorted(x for x in v if x is not None)
        return "—" if not v else (f"{v[len(v) // 2]:.1f}" + (f" ({v[0]:.1f}~{v[-1]:.1f})" if len(v) > 1 else ""))
    L = [f"[{label}] 적용 {n}/{len(F)}"]
    L.append(f"  L1 걸림 {sum(1 for r in A_ if r['t_L1'] is not None)}/{n} (고장 시작→L1 {med([r['t_L1'] for r in A_])} s) | "
             f"L2 걸림 {sum(1 for r in A_ if r['t_L2'] is not None)}/{n} (→L2 {med([r['t_L2'] for r in A_])} s) | "
             f"motor 워치독 중립 {sum(1 for r in A_ if r['wd_ticks'])}/{n} | 고장 중 STOP_HOLD {med([r['stop_s'] for r in A_])} s")
    L.append(f"  속도 고장 전 {med([r['v_before'] for r in A_])} → 고장 중 평균 {med([r['v_in_mean'] for r in A_])}, 1.5 s 뒤 {med([r['v_in_late'] for r in A_])} m/s | "
             f"복구(고장 끝→L0) {med([r['recover_s'] for r in A_])} s | 고장~+3 s 접촉 {sum(r['contacts_fault'] for r in A_)} | "
             f"미션 이어짐 {sum(1 for r in A_ if r['mission_continued'])}/{n}, 결과 기준과 같음 {sum(1 for r in A_ if r['same_outcome'])}/{n}, "
             f"코스 시간 차 {med([r['d_t_course'] for r in A_])} s")
    md = {}
    for r in A_: md[r["fault_mode"]] = md.get(r["fault_mode"], 0) + 1
    nodes = {}
    for r in A_: nodes[r["fault_node"]] = nodes.get(r["fault_node"], 0) + 1
    L.append(f"  고장 시 mode 분포 {dict(sorted(md.items()))} | 대상 {nodes} | ship_goal_angle 목표각 stale {sum(1 for r in A_ if r['sga_goal_stale'])}, IMU stale {sum(1 for r in A_ if r['sga_imu_stale'])}, north GPS stale 경고 {sum(1 for r in A_ if r['gps_stale_flag'])}")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="전체 코스 2D 시뮬 (실제 노드 코드)")
    ap.add_argument("--variant", default="A", choices=["A", "B", "AB"])
    ap.add_argument("--seeds", type=int, default=0)
    ap.add_argument("--seed0", type=int, default=1)
    ap.add_argument("--plot-seed", type=int, default=None)
    ap.add_argument("--wp-file", default=None, help="north_goal_angle 이 읽을 웨이포인트 파일(지정 시 변형 B 자동 생성 대신 이 파일)")
    ap.add_argument("--world-wp-file", default=WP_DEFAULT)
    ap.add_argument("--gps-sigma", type=float, default=0.03)
    ap.add_argument("--current", type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument("--t-max", type=float, default=1500.0)
    ap.add_argument("--tail", type=float, default=90.0)
    ap.add_argument("--csv", default=os.path.join(OUT_DIR, "full_course_sim.csv"))
    ap.add_argument("--png", default=None)
    ap.add_argument("--dwell-label", default="", help='보조 진단: 구역 이름 접두어별 dwell 덮어쓰기, 예 "WP6=3,WP7=3" (스크래치 사본에만)')
    ap.add_argument("--faults", action="store_true", help="고장 주입 묶음: 변형별 --seeds 개 × (a)~(e) + 기준 런 → tools/sim/out/full_course_sim_faults.csv")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--dyn", default="legacy", choices=["legacy", "fossen"],
                    help="운동 모델: legacy = K_V·K_W 1차 지연(기존) / fossen = 3자유도(boat3dof.py, Gazebo 모델과 같은 계수)")
    ap.add_argument("--hull", default=None, choices=["A", "B"], help="fossen 선체 계수(기본 = variant 글자)")
    ap.add_argument("--lidar-noise", action="store_true",
                    help="LiDAR 잡음(σ3cm·결측5%·정면±30° 고립 물보라1%) — 합성, add_lidar_noise 참조")
    ap.add_argument("--param", action="append", default=[], metavar="노드.키=값",
                    help="노드 파라미터 덮어쓰기(반복 가능), 예 --param ship_direction.dwa_enabled=true")
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    dov = {kv.split("=")[0].strip(): float(kv.split("=")[1]) for kv in a.dwell_label.split(",") if "=" in kv} or None
    kw = dict(wp_file=a.wp_file, world_wp_file=a.world_wp_file, gps_sigma=a.gps_sigma, current=tuple(a.current), t_max=a.t_max, tail=a.tail, dyn=a.dyn, hull=a.hull,
              dwell_override=dov, node_overrides=parse_param_overrides(a.param), lidar_noise=a.lidar_noise)
    variants = ["A", "B"] if a.variant == "AB" else [a.variant]
    if a.plot_seed is not None:
        for vv in variants:
            t0 = _walltime.perf_counter()
            r = run(a.plot_seed, vv, record=True, **kw)
            wall = _walltime.perf_counter() - t0
            print(f"[변형 {vv} seed {a.plot_seed}] 한 런 실행 시간 {wall:.1f} s (시뮬 {r['t_end']:.0f} s), 예외: {r['exception'] or '없음'}")
            if not r["ok"]: print(r.get("traceback", ""))
            print(summarize([r], f"변형 {vv} seed {a.plot_seed}"))
            print(timeline_text(r))
            png = a.png or os.path.join(OUT_DIR, "full_course_sim.png" if vv == "A" else f"full_course_sim_{vv}.png")
            if dov and not a.png:
                png = png.replace(".png", "_dwell.png")
            plot(r, png)
    if a.faults:
        return fault_batch(a, variants, kw)
    if a.seeds > 0:
        import multiprocessing as mp
        jobs = [(s, vv, kw) for vv in variants for s in range(a.seed0, a.seed0 + a.seeds)]
        t0 = _walltime.perf_counter()
        if a.jobs > 1:
            with mp.get_context("spawn").Pool(a.jobs) as pool:
                R = pool.map(_worker, jobs, chunksize=1)
        else:
            R = [_worker(j) for j in jobs]
        wall = _walltime.perf_counter() - t0
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f); wr.writerow(CSV_COLS + ["wall_s"])
            for r in R: wr.writerow(to_row(r) + [f"{r['wall']:.1f}"])
        print(f"{len(R)} 런 실행 시간 {wall:.0f} s (병렬 {a.jobs}, 런당 CPU 중앙값 {sorted(r['wall'] for r in R)[len(R) // 2]:.1f} s) → {a.csv}")
        for vv in variants:
            print(summarize([r for r in R if r["variant"].split("-")[0] == vv], f"변형 {vv}{'' if not dov else ' dwell ' + a.dwell_label}, seed {a.seed0}~{a.seed0 + a.seeds - 1}"))
        bad = [r for r in R if not r["ok"]]
        for r in bad[:3]:
            print(f"예외 seed {r['seed']} {r['variant']}: {r.get('traceback', r['exception'])}")


if __name__ == "__main__":
    main()

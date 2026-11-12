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
  세계·도크 (11-12, 옵션을 안 주면 옛 결과와 같다):
        --world tools/sim/worlds/arena_2025.yaml   규정 그림(2025-11) 배치 — 게이트 5쌍·장애물 격자·위치유지·탐색·도크 12 m
                                                   (웨이포인트 = 그 파일의 waypoints_file, --write-world-wp 로 생성)
        --dock-bay-w 2 --dock-gap 1.5 --dock-depth 3   도크 칸 폭·칸막이 두께·깊이(기본 1·0·0.5 = 옛 형상, 세계 파일 값 우선)
        --dock-target -1|0|1|random                    목표 칸(기본 random = 시드)
        --dock-align wp|target                         target = 정렬·WP8·WP9 를 목표 칸 축으로 옮긴 사본
        형상은 dock_geometry.py(순수 함수, test_dock_geometry.py). 도크 상세 요약 줄(분류·접안 깊이)은 이 옵션을 줄 때만.

실제로 돌리는 노드 코드 (한 줄도 안 바꾼다. 파라미터는 아래 — 대회 launch 와 값은 같다)
  north_goal_angle  ← north_goal_angle.yaml + waypoints_file (launch 가 둘 다 넘김)
  ship_goal_angle   ← ship_goal_angle.yaml (launch 가 넘김)
  ship_direction, motor_control, ship_gate, ship_dock ← 코드 기본값
  ship_back·ship_turn ← --search-behavior 에 따라 launch search_behavior_params 와 같은 값
      hold : ship_back {active_wp_modes: [2, 3]}, ship_turn {enabled: False} (침묵 — 발행 0 회인지 검사한다)
      orbit: ship_back {active_wp_modes: [2]},    ship_turn {enabled: True}
      (계약서_탐색orbit_WP넘김_20261111 C4. 기본값은 대회 launch 기본과 같게 둔다.)
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
  · mode 2 WP 마다 흰 부표(위치유지 — 2026 순서는 2회), WP3 초록·WP4 빨강·WP5 흰 부표(구역 이름대로, WP 좌표 그대로).
  · 장애물(주황, 검출기 대상 색 아님): WP6→WP7(이름에 '회피' 가 든 mode 5 두 점) 선분의 10~90 % 구간에 6~10개.
    옛 순서(도킹 바로 뒤 WP10 mode 8)일 때만 도크(첫 mode 7)→WP10 선 위 WP10 앞 3~20 m 구간에 6~10개 더.
    경로선 ±3 m, 서로 1.5 m 이상, 다른 부표와 1.5 m 이상.
  · 도크(dock_sim 과 같은 규격): 정면 벽 3 m(1 m 칸 3개) + 칸막이 0.5 m 4개. 정면 벽 중심 = WP8 좌표,
    정면(물 쪽)이 직전 WP(도킹 접근 6m) 방향을 향함. 목표 칸(표식) 좌·중·우 무작위.
    --dock-* 로 칸 폭·칸막이 두께(덩어리)·깊이를 바꾸면 dock_geometry: 뒷벽(front) + 칸 옆벽·양 끝 벽(div) + 덩어리 끝면(cap).
    표식 = 목표 칸 뒷벽 가운데. 접촉 검사 반경 = max(4.0, 반길이 + 깊이 + 0.83 + 1). 칸 안 = |lat| ≤ 칸폭/2.
  · 위치유지 판정은 위치유지 WP 마다 따로(그 WP 동안 그 부표 5 m·5 s), 전부 채워야 성공.
  · geofence: north_goal_angle.yaml 의 geofence_polygon 이 [] → 꺼짐(노드가 빈 배열 발행). 경계 없음.
  · 출발: WP1→WP0 연장선 위 WP0 앞 8 m, 좌우 ±1 m, 헤딩 = WP0 방향 ±20°, 정지 상태. t=0 에 모든 노드 생성.

판정 (실측 아님, 시뮬 정의)
  · 게이트 기하 통과 = 궤적이 빨강–초록 선분을 진행 방향으로 가로지름. /gates_passed = ship_gate 마지막 발행값.
  · 위치유지(규정) = mode 2 동안 흰 부표(WP2) 5 m 이내 연속 5 s. 노드 판정(HOLD→DONE) 도 따로 적는다.
  · 탐색(hold) = mode 3 동안 ship_back 이 HOLD→DONE 했고 그때 가장 가까운 부표가 빨강(turn 검출기 mode 3 대상 색).
    규정 기하(빨강 5 m 이내 연속 5 s)도 따로 적는다.
  · 탐색(orbit) = ship_turn 이 ORBIT 에서 진행각 orbit_target_deg 를 채워 DONE(안전 타임아웃 DONE 은 실패)
    + 기하: mode 3 동안 공지 색 부표(turn_color_mode3) 8 m 안에서 배 위치 방위의 부호 있는 누적이
    규정 방향(빨강·초록 시계 / 노랑·흰색·검정 반시계 — 검정은 2026 OT 11-12)으로 ≥ 350°. 둘 다 맞아야 성공.
  · 포기 = ship_back 이 ERROR '포기'(거리 unknown 10 s 또는 HOLD 총 60 s)를 남기고 DONE — 성공으로 세지 않는다.
  · 도킹 = ship_dock 이 CONTACT 에 들어간 순간 배 중심이 목표 칸 중심 ±칸폭/2(기본 0.5 m) 안 → '칸 안 접안'.
    분류(dock_outcome): 그 순간 ship_dock 과 같은 전방 섹터(±15°, median 5) 최소거리 광선이 뒷벽이면 '안쪽 벽', 칸막이(div·cap)면
    목표 칸 밖일 때 '거짓 CONTACT@칸막이 끝'. CONTACT 없으면 SEARCH 타임아웃 / APPROACH 중단 / ALIGN 진동 / 기타.
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
import tempfile
import time as _walltime
import traceback
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(ROOT, "src")
OUT_DIR = os.path.join(HERE, "out")
# 변형 B 웨이포인트(자동 생성) 임시 폴더 — OS 임시 폴더 기준(옛 값은 Mac 세션 scratchpad 절대 경로라 Ubuntu 에서 PermissionError)
SCRATCH_B = os.path.join(tempfile.gettempdir(), "ssf_full_course_sim")
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
from dock_geometry import DOCK_DEFAULT, dock_geometry, bay_offsets, extent as dock_extent   # noqa: E402

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


# ───────────────────────── 세계 파일 (--world, 명시 배치) ─────────────────────────
# 형식은 tools/sim/worlds/arena_2025.yaml 머리말. 없으면(기본) 아래 build_world 의 옛 규칙 그대로.
#   origin(위경도 = 로컬 원점), waypoints(xy 로컬), waypoints_file(위경도 사본, --write-world-wp 로 생성),
#   gates(빨강·초록 쌍, 지나는 순서), buoys(명시 부표), obstacle_grid(주황 장애물 행), random_obstacles(false 면
#   회피 선분 무작위 장애물 끔), dock(center·facing_deg·bay_w·gap·depth·n_bays), arena(그림용 경계, 선택).
def load_world(path):
    with open(path, encoding="utf-8") as f:
        w = yaml.safe_load(f)
    w["_path"] = os.path.abspath(path)
    wf = w.get("waypoints_file")
    w["_wp_path"] = os.path.join(os.path.dirname(w["_path"]), wf) if wf else None
    return w


def world_geo(world):
    return Geo(float(world["origin"]["lat"]), float(world["origin"]["lon"]))


def world_wp_text(world):
    """세계 파일의 waypoints(로컬 xy) → north_goal_angle 이 읽는 waypoints.yaml 형식 문자열(위경도 9자리)."""
    geo = world_geo(world)
    name = os.path.basename(world.get("_path", "world.yaml"))
    lines = [f"# 자동 생성 — tools/sim/worlds/{name} 의 waypoints(로컬 xy)를 origin 기준 평면 변환. 손으로 고치지 말 것.",
             f"#   python3 tools/sim/full_course_sim.py --world tools/sim/worlds/{name} --write-world-wp",
             "#   시뮬 전용 좌표(합성, 실측 아님). 라벨·mode·dwell 은 대회 템플릿(KABOAT 2026 OT 순서)과 같다.",
             "waypoints:"]
    for w in world["waypoints"]:
        lat, lon = geo.ll(float(w["xy"][0]), float(w["xy"][1]))
        lines.append(f'  - {{구역: "{w["구역"]}", lat: {lat:.9f}, lon: {lon:.9f}, '
                     f'mode: {int(w["mode"])}, dwell: {float(w["dwell"]):.1f}}}')
    return "\n".join(lines) + "\n"


def resolve_dock_cfg(world=None, dock_cfg=None):
    """도크 치수: 코드 기본(DOCK_DEFAULT = 옛 1 m 칸) ← 세계 파일 dock ← CLI(--dock-*) 순으로 덮어쓴다. 목표 칸 None = 무작위."""
    d = dict(DOCK_DEFAULT, target=None)
    if world is not None and world.get("dock"):
        for k in ("bay_w", "gap", "depth", "n_bays"):
            if k in world["dock"]:
                d[k] = world["dock"][k]
    for k, v in (dock_cfg or {}).items():
        if v is not None:
            d[k] = v
    d["bay_w"], d["gap"], d["depth"], d["n_bays"] = float(d["bay_w"]), float(d["gap"]), float(d["depth"]), int(d["n_bays"])
    return d


# ───────────────────────── 세계 ─────────────────────────
def build_world(world_wps, seed, dock_cfg=None, world=None):
    """dock_cfg = resolve_dock_cfg() 결과(None 이면 기본 = 옛 형상), world = load_world() 결과(None 이면 옛 규칙)."""
    rng = random.Random(seed)
    dcfg = dock_cfg if dock_cfg is not None else resolve_dock_cfg(world)
    geo = world_geo(world) if world is not None else Geo(world_wps[0]["lat"], world_wps[0]["lon"])
    P = [np.array(geo.xy(w["lat"], w["lon"])) for w in world_wps]
    W = dict(geo=geo, P=P, buoys=[], segs=[])
    if world is not None and world.get("arena"):
        W["arena"] = [tuple(map(float, q)) for q in world["arena"]]

    def add_buoy(p, color, kind):
        W["buoys"].append(dict(x=float(p[0]), y=float(p[1]), color=color, kind=kind))

    # 게이트
    d01 = P[1] - P[0]; L01 = np.linalg.norm(d01); u = d01 / L01; left = np.array([-u[1], u[0]])
    W["gates"] = []
    if world is not None and world.get("gates"):
        for k, g in enumerate(world["gates"]):
            red, green = np.array(g["red"], dtype=float), np.array(g["green"], dtype=float)
            W["gates"].append(dict(c=(red + green) / 2, w=float(np.linalg.norm(red - green)), red=red, green=green))
            add_buoy(red, "red", f"gate{k}"); add_buoy(green, "green", f"gate{k}")
    else:
        for k in range(4):
            c = P[0] + u * L01 * (k + 0.5) / 4.0; w = rng.uniform(3.0, 6.0)
            red, green = c + left * w / 2, c - left * w / 2
            W["gates"].append(dict(c=c, w=w, red=red, green=green))
            add_buoy(red, "red", f"gate{k}"); add_buoy(green, "green", f"gate{k}")
    W["gate_u"] = u
    # 미션 부표 (구역 이름대로)
    by_mode = [w["mode"] for w in world_wps]
    W["hold_by_wp"] = {}                     # 위치유지 WP 순번 → 부표 index (2026 규정: 2회)
    for i2 in [i for i, m in enumerate(by_mode) if m == 2]:
        add_buoy(P[i2], "white", "hold"); W["hold_by_wp"][i2] = len(W["buoys"]) - 1
    col_kw = {"초록": "green", "빨강": "red", "하양": "white", "흰": "white", "검정": "black"}
    for i, w in enumerate(world_wps):
        if w["mode"] == 3:
            col = next((v for kk, v in col_kw.items() if kk in w["label"]), "red")
            add_buoy(P[i], col, f"search_{col}")
    if world is not None:                    # 명시 부표·장애물 격자
        for b in world.get("buoys") or []:
            add_buoy(b["xy"], b["color"], b["kind"])
        for row in world.get("obstacle_grid") or []:
            for gx in row["x"]:
                add_buoy((gx, row["y"]), "orange", "obst_5")
    # 도크: 뒷벽(칸 안쪽 벽) 중심 O, 물 쪽 n. 기본 = 첫 mode 7 WP 가 O, 정면은 직전 WP 쪽(옛 규칙).
    #   세계 파일 dock 이 있으면 center·facing_deg. 형상은 dock_geometry(순수 함수) — 기본 치수면 옛 선분과 같다.
    i7 = by_mode.index(7)
    if world is not None and world.get("dock"):
        O = np.array(world["dock"]["center"], dtype=float); f_ = math.radians(float(world["dock"]["facing_deg"]))
        n = np.array([math.sin(f_), math.cos(f_)])
    else:
        O = P[i7]; n = P[i7 - 1] - O; n = n / np.linalg.norm(n)
    geom = dock_geometry(O, n, dcfg["bay_w"], dcfg["gap"], dcfg["depth"], dcfg["n_bays"])
    t = geom["t"]
    # 목표 칸: 난수는 늘 같은 순서로 뽑는다(지정해도 기본 동작의 난수 흐름 유지)
    bay = rng.choice((-1.0, 0.0, 1.0)) if dcfg["n_bays"] == 3 else rng.choice(bay_offsets(dcfg["n_bays"]))
    if dcfg.get("target") is not None:
        bay = float(dcfg["target"])
    W["dock"] = dict(O=O, n=n, t=t, bay=bay, marker=O + t * (bay * geom["pitch"]), geom=geom,
                     bay_w=geom["bay_w"], depth=geom["depth"], pitch=geom["pitch"], half=geom["half"],
                     reach=max(3.0, dock_extent(geom)))
    W["segs"].extend(geom["segs"])

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
    # 회피 구간 = 구역 이름에 '회피' 가 든 mode 5 두 점(WP6→WP7). 도킹 접근점도 mode 5 라 이름으로 고른다.
    i5 = [i for i, m in enumerate(by_mode) if m == 5 and "회피" in world_wps[i]["label"]]
    if len(i5) < 2:
        i5 = [i for i, m in enumerate(by_mode) if m == 5 and "도킹" not in world_wps[i]["label"]]
    W["i5"] = i5
    a, b = P[i5[0]], P[i5[1]]; L = np.linalg.norm(b - a)
    rand_obst = world is None or world.get("random_obstacles", True)
    if rand_obst:
        scatter(a, b, 0.1 * L, 0.9 * L, "obst_5")
    i8 = by_mode.index(8)
    if rand_obst and by_mode[i8 - 1] == 7:   # 옛 순서(도킹 → WP10 토너먼트 회피)만 두 번째 장애물 구간이 있다. 2026 순서는 회피 → 종료.
        a, b = P[i7], P[i8]; L = np.linalg.norm(b - a)
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
    near = math.hypot(x - W["dock"]["O"][0], y - W["dock"]["O"][1]) < LIDAR_MAX + W["dock"].get("reach", 3.0)   # 도크 크기(tournament_sim 등 reach 없는 W = 옛 3.0)
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


AVOID_ZONE_END_M = 4.0    # 장애물 구간 직사각형이 WP6·WP7 너머로 뻗는 길이(회피 WP 가 경계에서 3 m 이상 안쪽)


def avoid_rect(W, half_w):
    """WP6→WP7(회피 mode 5 두 점, W["i5"]) 따라 ±half_w, 양 끝 AVOID_ZONE_END_M 연장한 직사각형 꼭짓점(로컬 x, y)."""
    a, b = W["P"][W["i5"][0]], W["P"][W["i5"][1]]
    u = (b - a) / np.linalg.norm(b - a); n = np.array([-u[1], u[0]])
    s0, s1 = a - u * AVOID_ZONE_END_M, b + u * AVOID_ZONE_END_M
    return [tuple(map(float, p)) for p in (s0 - n * half_w, s1 - n * half_w, s1 + n * half_w, s0 + n * half_w)]


def _poly_edge_dist(p, pts):
    best = math.inf
    for i in range(len(pts)):
        a = np.array(pts[i]); b = np.array(pts[(i + 1) % len(pts)]); e = b - a
        s_ = max(0.0, min(1.0, float((p - a) @ e) / float(e @ e)))
        best = min(best, float(np.linalg.norm(p - (a + s_ * e))))
    return best


def _in_poly(p, pts):
    inside = False
    for i in range(len(pts)):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % len(pts)]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


# ───────────────────────── 한 런 ─────────────────────────
SEARCH_BEHAVIOR_PARAMS = {   # launch_files.launch.py search_behavior_params 와 같은 값
    "hold": {"ship_back": {"active_wp_modes": [2, 3]}, "ship_turn": {"enabled": False}},
    "orbit": {"ship_back": {"active_wp_modes": [2]}, "ship_turn": {"enabled": True}},
}
ORBIT_GEO_MIN_DEG = 350.0     # 기하 판정: 부호 있는 누적이 이 이상
ORBIT_GEO_NEAR_M = 8.0        # 이 거리 안일 때만 누적


def node_params(wp_file, overrides=None, search_behavior="orbit"):
    rosstub.PARAMS.clear()
    rosstub.PARAMS["north_goal_angle"] = dict(P_NGA, waypoints_file=wp_file)
    rosstub.PARAMS["ship_goal_angle"] = dict(P_SGA)
    for n in ("ship_direction", "motor_control", "ship_gate", "ship_dock"):
        rosstub.PARAMS[n] = {}
    for node, kv in SEARCH_BEHAVIOR_PARAMS[search_behavior].items():
        rosstub.PARAMS[node] = dict(kv)
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
        node_overrides=None, lidar_noise=False, search_behavior="orbit", avoid_zone=0.0, avoid_measure_only=False,
        world=None, dock_cfg=None, dock_align="wp"):
    """world = 세계 파일 경로(--world, None = 옛 규칙), dock_cfg = {bay_w, gap, depth, target}(None 값은 기본·세계 파일),
    dock_align = "wp"(웨이포인트 파일 그대로) | "target"(정렬·WP8·WP9 를 목표 칸 축으로 옮긴 사본)."""
    world_d = load_world(world) if isinstance(world, str) else world
    if world_d is not None and world_wp_file in (None, WP_DEFAULT):   # 세계 파일이 있으면 그 위경도 사본
        world_wp_file = world_d["_wp_path"]
    if world_wp_file is None:
        world_wp_file = WP_DEFAULT
    dcfg = resolve_dock_cfg(world_d, dock_cfg)
    dock_custom = world_d is not None or any(v is not None for v in (dock_cfg or {}).values()) or dock_align != "wp"
    world_wps = load_wps(world_wp_file)
    nrng = np.random.default_rng(20_000 + seed) if lidar_noise else None
    W = build_world(world_wps, seed, dcfg, world_d)
    geo = W["geo"]; dock = W["dock"]
    if wp_file is None:
        wp_file = variant_b_file(W) if variant == "B" else world_wp_file
    if dock_align == "target":
        wp_file = align_target_file(wp_file, W)
    wp_file = apply_dwell(wp_file, dwell_override)
    wps = load_wps(wp_file)
    WPXY = [np.array(geo.xy(w["lat"], w["lon"])) for w in wps]
    avoid_pts = None
    if avoid_zone > 0.0:   # 장애물 구간 경계(규정 종합임무 (4)) — WP6→WP7 직사각형, 양 끝 AVOID_ZONE_END_M 여유
        avoid_pts = avoid_rect(W, avoid_zone)
        poly = [c for p_ in avoid_pts for c in geo.ll(*p_)]
        node_overrides = {k: dict(v) for k, v in (node_overrides or {}).items()}
        if not avoid_measure_only:          # 기준선: 경계는 재기만 하고 노드엔 안 넘긴다
            node_overrides.setdefault("north_goal_angle", {})["geofence_avoid_polygon"] = poly
    node_params(wp_file, node_overrides, search_behavior)
    Clock.t = 0.0
    vtag = variant + ("" if not dwell_override else "-dwell(" + ",".join(f"{k}={v:g}" for k, v in dwell_override.items()) + ")")
    out = dict(seed=seed, variant=vtag, wp_file=wp_file, bay=dock["bay"], exception="", search_behavior=search_behavior)
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
    blacks = [(b["x"], b["y"]) for b in W["buoys"] if b["color"] == "black"]   # 탐색 검정(11-12)
    by_color = {"red": reds, "green": greens, "white": whites, "black": blacks}

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
    gate_cross = [0] * len(W["gates"]); gate_wrong = [0] * len(W["gates"])
    hold_run = 0.0; hold_best = {i: 0.0 for i in W["hold_by_wp"]}; hold_wp = None; srch_run = {}; srch_best = {}
    sb_events = []; sk_events = []; prev_sb = sb.state; prev_sk = sk.state
    st_events = []; prev_st = st.state            # ship_turn 상태 전이 (t, mode, 이전, 다음, 진행각)
    c3 = (TURN_COLOR_MAP.get(3) or [None])[0]       # 공지 탐색 색(검출기 mode 3 대상)
    orb_buoys = [b for b in W["buoys"] if b["kind"].startswith("search") and b["color"] == c3]
    orb_prev = {}; orb_sum = {}                     # 부표 index → 직전 방위, 부호 있는 누적(시계 +)
    orb_min_after = math.inf                        # DONE 뒤 공지 색 부표 표면까지 최소 거리
    sb_log_idx = 0
    contacts_sb_state = {}
    _hd = HULL_DIMS.get(hull or variant[:1], HULL_DIMS["A"]); rect_hl, rect_hb = _hd[0] / 2.0, _hd[1] / 2.0
    contacts_rect = {"gate": 0, "hold": 0, "search": 0, "obst_5": 0, "obst_8": 0}; rect_on = {}
    dock_contact = None; dock_mode7_t0 = None; dock_dist_at = {}
    dock_r = max(4.0, dock["half"] + dock["depth"] + HULL_HALF_L + 1.0)   # 도크 접촉 검사 반경(기본 형상 = 옛 4.0 m)
    t_last_reach = None; img_pub_conflict = 0
    rec = dict(t=[], x=[], y=[], psi=[], v=[], mode=[], wp=[], sk=[], sb=[], cand=[], des=[], dd=[]) if record else None
    n_wps = len(wps)
    i_last = n_wps - 1
    first_mode = {}
    av_flags = ([w["mode"] == 5 and _in_poly(p_, avoid_pts) for w, p_ in zip(wps, WPXY)]
                if avoid_pts is not None else [])     # 노드 avoid_fence_wps 와 같은 규칙(재기만 할 때도 같은 구간)
    av_in = False; av_out_max = 0.0; av_out_s = 0.0; av_on_s = 0.0   # 장애물 구간 경계(켜는 WP 동안)
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
            if math.hypot(x - dock["O"][0], y - dock["O"][1]) < dock_r:
                h = np.array([math.sin(math.radians(psi)), math.cos(math.radians(psi))])
                bow, stern = pos + h * HULL_HALF_L, pos - h * HULL_HALF_L
                for j, (a_, b_, kind) in enumerate(W["segs"]):
                    dd = seg_seg_dist(stern, bow, a_, b_)
                    on = dd < (0.02 if kind == "front" else HALF_W)   # div·cap(칸막이 끝면) = 선체 반폭
                    if on and not contact_on.get(("s", j)):
                        contacts["dock"] += 1; contacts_by_mode[cur_mode] = contacts_by_mode.get(cur_mode, 0) + 1
                    contact_on[("s", j)] = on
            # 장애물 구간 경계 — 노드가 '켜는 WP'(mode 5 + 구역 안)일 때만, 한 번 든 뒤 밖이면 이탈
            if avoid_pts is not None:
                if av_flags[nga.wp_idx] if nga.wp_idx < len(av_flags) else False:
                    av_on_s += DT
                    if _in_poly(pos, avoid_pts):
                        av_in = True
                    elif av_in:
                        av_out_s += DT; av_out_max = max(av_out_max, _poly_edge_dist(pos, avoid_pts))
                else:
                    av_in = False
            # 위치유지·탐색 기하
            if cur_mode == 2:
                hw = nga.wp_idx if nga.wp_idx in W["hold_by_wp"] else hold_wp   # 전진 직후 틱은 직전 위치유지
                if hw is not None:
                    if hw != hold_wp:
                        hold_run = 0.0; hold_wp = hw
                    hb = W["buoys"][W["hold_by_wp"][hw]]
                    hold_run = hold_run + DT if math.hypot(x - hb["x"], y - hb["y"]) <= 5.0 else 0.0
                    hold_best[hw] = max(hold_best[hw], hold_run)
            if cur_mode == 3:
                for b in W["buoys"]:
                    if b["kind"].startswith("search"):
                        kk = b["kind"]
                        srch_run[kk] = srch_run.get(kk, 0.0) + DT if math.hypot(x - b["x"], y - b["y"]) <= 5.0 else 0.0
                        srch_best[kk] = max(srch_best.get(kk, 0.0), srch_run[kk])
            if cur_mode == 3 and st.state == "DONE":
                for b in orb_buoys:   # DONE(20000) 뒤 GPS 가 WP 좌표(=부표 위치)로 갈 때 최소 거리
                    orb_min_after = min(orb_min_after, math.hypot(x - b["x"], y - b["y"]) - BUOY_R)
            if cur_mode == 3 and st.state != "DONE":   # 기하 누적은 DONE 전까지만(뒤는 GPS 경로)
                for bi, b in enumerate(orb_buoys):
                    if math.hypot(x - b["x"], y - b["y"]) <= ORBIT_GEO_NEAR_M:
                        brg = math.degrees(math.atan2(x - b["x"], y - b["y"]))   # 부표→배 나침반 방위
                        if bi in orb_prev:
                            orb_sum[bi] = orb_sum.get(bi, 0.0) + wrap(brg - orb_prev[bi])
                        orb_prev[bi] = brg
                    else:
                        orb_prev.pop(bi, None)
            if st.state != prev_st:
                st_events.append((round(t, 1), cur_mode, prev_st, st.state, round(st.progress.progress_deg(), 1)))
                prev_st = st.state
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
                    # 접안 순간: 배 중심 → 뒷벽 거리(깊이), ship_dock 과 같은 전방 섹터 거리와 그 광선이 맞은 물체
                    dock_contact["depth"] = float((pos - dock["O"]) @ dock["n"])
                    fd, fk = front_sector_hit(W, x, y, psi, sk.front_sector_half_deg, sk.front_median_kernel)
                    dock_contact["front_d"], dock_contact["hit"] = fd, fk
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
    # 위치유지 규정(5 m·5 s)은 **모든** 위치유지 WP 에서(2026: 2회). 최솟값으로 판정.
    out["hold_geo"] = bool(hold_best) and min(hold_best.values()) >= 5.0
    out["hold_best_s"] = round(min(hold_best.values()), 1) if hold_best else 0.0
    out["hold_n"] = len(hold_best)
    m2 = [e for e in sb_events if e[1] == 2]
    done2 = [e for e in m2 if e[3] == "DONE"]
    if len(done2) < len(hold_best):
        out["hold_node"] = "미완"
    elif any(e[6] == "포기" for e in done2):
        out["hold_node"] = "포기"
    else:
        out["hold_node"] = "HOLD→DONE" if all(e[2] == "HOLD" for e in done2) else "SEARCH 타임아웃"
    m3 = [e for e in sb_events if e[1] == 3]
    done3 = [e for e in m3 if e[3] == "DONE"]
    if done3:
        e = done3[0]
        out["search_node"] = ("포기" if e[6] == "포기" else ("HOLD→DONE" if e[2] == "HOLD" else "SEARCH 타임아웃"))
        out["search_at"] = e[4] if e[2] == "HOLD" else ""
    else:
        out["search_node"] = "미완"; out["search_at"] = ""
    out["search_ok"] = out["search_node"] == "HOLD→DONE" and out["search_at"] == "search_red"
    # orbit 판정 (C4)
    target = st.orbit_target_deg
    done_t = [e for e in st_events if e[3] == "DONE" and e[2] == "ORBIT"]
    if done_t:
        out["orbit_node"] = "완주" if done_t[0][4] >= target - 1e-6 else "안전 타임아웃"
    else:
        out["orbit_node"] = "미완"
    want_cw = {"red": True, "green": True, "yellow": False, "white": False, "black": False}.get(c3)
    geo = max(orb_sum.values(), key=abs) if orb_sum else 0.0
    out["orbit_geo_deg"] = round(geo, 1)
    out["orbit_geo_ok"] = want_cw is not None and abs(geo) >= ORBIT_GEO_MIN_DEG and (geo > 0) == want_cw
    out["orbit_ok"] = out["orbit_node"] == "완주" and out["orbit_geo_ok"]
    out["orbit_tries"] = sum(1 for e in st_events if e[3] == "APPROACH")
    out["orbit_min_d_after_done"] = None if not math.isfinite(orb_min_after) else round(orb_min_after, 2)
    out["st_events"] = st_events
    if search_behavior == "orbit":
        out["search_ok"] = out["orbit_ok"]
    out["search_geo_red"] = srch_best.get("search_red", 0.0) >= 5.0
    out["search_best"] = {kk: round(vv, 1) for kk, vv in srch_best.items()}
    out["dock_contact"] = dock_contact is not None
    out["dock_inbay"] = dock_contact is not None and abs(dock_contact["lat"]) <= dock["bay_w"] / 2.0
    out["dock_lat"] = None if dock_contact is None else round(dock_contact["lat"], 2)
    out["dock_backout"] = _backoff_by_dist(sk_events)
    out["dock_search_timeout"] = any(e[2] == "SEARCH" and e[3] == "DONE" for e in sk_events)
    out["dock_dist_mode7"] = None if "mode7" not in dock_dist_at else round(dock_dist_at["mode7"], 1)
    out["n_gates"] = len(W["gates"])
    out["dock_custom"] = dock_custom
    out["dock_geom"] = dict(bay_w=dock["bay_w"], gap=dcfg["gap"], depth=dock["depth"], align=dock_align)
    out["dock_depth_at"] = None if dock_contact is None else round(dock_contact["depth"], 2)
    out["dock_hit"] = None if dock_contact is None else dock_contact["hit"]
    out["dock_front_d"] = None if dock_contact is None else dock_contact["front_d"]
    out["dock_class"] = dock_outcome(out, sk_events, dock_contact, dock["bay_w"], dock["depth"])
    out["sk_events"] = sk_events; out["sb_events"] = sb_events
    out["contacts"] = contacts; out["contacts_total"] = sum(contacts.values())
    out["contacts_avoid"] = contacts["obst_5"] + contacts["obst_8"]
    out["avoid_zone"] = avoid_zone; out["avoid_measure_only"] = avoid_measure_only
    if avoid_pts is not None:
        out["avoid_on_s"] = round(av_on_s, 1); out["avoid_out_s"] = round(av_out_s, 1); out["avoid_out_max"] = round(av_out_max, 2)
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


def front_sector_hit(W, x, y, psi, half_deg, kernel):
    """ship_dock.front_min_dist 와 같은 정의(정면 ±half_deg, 공간 median kernel)로 전방 거리와 그 광선이 맞은 물체.
    → (거리 또는 None, 'front'|'div'|'cap'|부표 kind|None). 접안 순간 1회만 부른다(진단용)."""
    from ship_dock.dock_fsm_logic import median_min, MEDIAN_MIN_VALID
    r = lidar(W, x, y, psi)
    inc = K.SCAN_INC_DEG; a0 = K.SCAN_ANGLE_MIN_DEG
    lo = max(0, int((80.0 - half_deg - a0) / inc)); hi = min(len(r) - 1, int((80.0 + half_deg - a0) / inc))
    d, idx = median_min(list(r), lo, hi, kernel, MEDIAN_MIN_VALID)
    if d is None:
        return None, None
    return round(float(d), 2), ray_hit_kind(W, x, y, psi, idx)


def ray_hit_kind(W, x, y, psi, i):
    """광선 i 가 가장 먼저 맞는 물체 종류(lidar() 와 같은 기하)."""
    th = math.radians(psi - SCAN_REL[i]); dx, dy = math.sin(th), math.cos(th)
    best = (math.inf, None)
    for b in W["buoys"]:
        ox, oy = b["x"] - x, b["y"] - y; tc = dx * ox + dy * oy; d2 = ox * ox + oy * oy - tc * tc
        if tc > 0 and d2 <= BUOY_R ** 2:
            tt = tc - math.sqrt(max(BUOY_R ** 2 - d2, 0.0))
            if tt < best[0]: best = (tt, b["kind"])
    for a_, b_, kind in W["segs"]:
        ex, ey = b_[0] - a_[0], b_[1] - a_[1]; den = dx * ey - dy * ex
        if abs(den) <= 1e-9: continue
        tt = ((a_[0] - x) * ey - (a_[1] - y) * ex) / den; uu = ((a_[0] - x) * dy - (a_[1] - y) * dx) / den
        if tt > 0 and 0.0 <= uu <= 1.0 and tt < best[0]: best = (tt, kind)
    return best[1]


DOCK_CLASSES = ("칸 안 접안(안쪽 벽)", "칸 안 접안(입구)", "거짓 CONTACT@칸막이 끝", "CONTACT 칸 밖",
                "SEARCH 타임아웃", "ALIGN 진동", "APPROACH 중단", "mode 7 미진입·기타")


def dock_outcome(out, sk_events, dc, bay_w, depth):
    """도킹 결과 한 줄 분류 (시뮬 정의, DOCK_CLASSES).
    CONTACT 가 있으면: 목표 칸 |lat| ≤ 칸폭/2 이고 전방 섹터가 뒷벽(front)을 봤으면 '안쪽 벽', 아니면 '입구'.
      목표 칸 밖인데 전방 섹터가 칸막이(div)·끝면(cap)을 봤으면 '거짓 CONTACT@칸막이 끝', 그 밖(다른 칸 뒷벽 등)은 'CONTACT 칸 밖'.
    CONTACT 가 없으면: SEARCH→DONE 이 있으면 SEARCH 타임아웃, APPROACH→BACKOFF(접근 타임아웃) 이면 APPROACH 중단,
      ALIGN 진입 3회 이상이거나 ALIGN→SEARCH(정렬 타임아웃) 이면 ALIGN 진동, 나머지 기타."""
    if dc is not None:
        if abs(dc["lat"]) <= bay_w / 2.0:
            return DOCK_CLASSES[0] if dc.get("hit") == "front" else DOCK_CLASSES[1]
        return DOCK_CLASSES[2] if dc.get("hit") in ("div", "cap") else DOCK_CLASSES[3]
    tr = [(e[2], e[3]) for e in sk_events]
    if ("SEARCH", "DONE") in tr:
        return DOCK_CLASSES[4]
    if ("APPROACH", "BACKOFF") in tr:
        return DOCK_CLASSES[6]
    if sum(1 for a, b in tr if b == "ALIGN") >= 3 or ("ALIGN", "SEARCH") in tr:
        return DOCK_CLASSES[5]
    return DOCK_CLASSES[7]


def align_target_file(path, W):
    """정렬(첫 mode 7 직전 mode 5)·첫 mode 7(WP8)·두 번째 mode 7(WP9) 을 목표 칸 축으로 옮긴 사본(가로 bay·pitch)."""
    d = W["dock"]; geo = W["geo"]; off = d["t"] * (d["bay"] * d["pitch"])
    with open(path, encoding="utf-8") as f:
        src = f.read().splitlines()
    items = [k for k, ln in enumerate(src) if ln.strip().startswith("- {")]
    modes = [int(yaml.safe_load(src[k].strip())[0]["mode"]) for k in items]
    j7 = modes.index(7)
    move = [items[j7 - 1], items[j7]] + ([items[j7 + 1]] if j7 + 1 < len(modes) and modes[j7 + 1] == 7 else [])
    for k in move:
        e = yaml.safe_load(src[k].strip())[0]
        x0, y0 = geo.xy(float(e["lat"]), float(e["lon"]))
        lat, lon = geo.ll(x0 + off[0], y0 + off[1])
        src[k] = re.sub(r"lat:\s*[-0-9.]+", f"lat: {lat:.9f}", src[k])
        src[k] = re.sub(r"lon:\s*[-0-9.]+", f"lon: {lon:.9f}", src[k])
    os.makedirs(SCRATCH_B, exist_ok=True)
    tag = {-1.0: "m1", 0.0: "0", 1.0: "p1"}.get(d["bay"], f"{d['bay']:+g}")
    out = os.path.join(SCRATCH_B, os.path.basename(path).replace(".yaml", f"_aligntarget_bay{tag}_p{d['pitch']:g}.yaml"))
    tmp = f"{out}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(src) + "\n")
    os.replace(tmp, out)
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
    entr = d["marker"] + d["depth"] * d["n"]               # 칸 입구 중심(칸막이 끝 선 위)
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
            "segments", "sk_events", "node_errors", "mode_pubs",
            "search_behavior", "orbit_node", "orbit_geo_deg", "orbit_geo_ok", "orbit_ok", "orbit_tries",
            "orbit_min_d_after_done"]
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
            seg, ske, str(r["node_errors"]), str({m: sorted(v) for m, v in r["mode_pubs"].items()}),
            r.get("search_behavior"), r.get("orbit_node"), r.get("orbit_geo_deg"), int(bool(r.get("orbit_geo_ok"))),
            int(bool(r.get("orbit_ok"))), r.get("orbit_tries"), r.get("orbit_min_d_after_done")] + \
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
    ng = R[0].get("n_gates", 4)
    g4 = sum(1 for r in R if r["gates_geo"] == ng); gm = sum(1 for r in R if r["gates_node"] == r["gates_geo"])
    lines.append(f"  게이트 {ng}/{ng} 기하 통과 {ci(g4)} | 기하 통과 합 {sum(r['gates_geo'] for r in R)}/{ng * n} | /gates_passed = 기하 {gm}/{n} | 역방향 가로지름 {sum(r['gates_wrongdir'] for r in R)}")
    lines.append(f"  위치유지 규정(5 m·5 s, {R[0].get('hold_n', 1)}회 모두) {ci(S('hold_geo'))} | 노드 HOLD→DONE {sum(1 for r in R if r['hold_node'] == 'HOLD→DONE')}, SEARCH 타임아웃 {sum(1 for r in R if r['hold_node'] == 'SEARCH 타임아웃')}, 포기 {sum(1 for r in R if r['hold_node'] == '포기')}")
    at = {}
    for r in R:
        at[r["search_at"] or r["search_node"]] = at.get(r["search_at"] or r["search_node"], 0) + 1
    if R and all(r.get("search_behavior") == "orbit" for r in R):
        on = {}
        for r in R:
            on[r["orbit_node"]] = on.get(r["orbit_node"], 0) + 1
        geos = sorted(r["orbit_geo_deg"] for r in R)
        lines.append(f"  탐색(orbit: 노드 완주 + 기하 ≥{ORBIT_GEO_MIN_DEG:.0f}° 규정 방향) {ci(S('orbit_ok'))} | 노드 {on} | "
                     f"기하 OK {S('orbit_geo_ok')}/{n} (부호 있는 누적 중앙값 {med(geos):.0f}°, 시계 +) | 시도(APPROACH) 합 {sum(r['orbit_tries'] for r in R)}")
        dd = sorted(r["orbit_min_d_after_done"] for r in R if r.get("orbit_min_d_after_done") is not None)
        if dd:
            lines.append(f"  orbit DONE 뒤(20000 → GPS 가 WP 좌표로) 공지 색 부표 표면 최소 거리: 최소 {dd[0]:.2f} m, 중앙값 {med(dd):.2f} m "
                         f"(선체 반폭 0.29 m 미만 {sum(1 for v in dd if v < 0.29)} 런)")
    else:
        lines.append(f"  탐색(빨강 앞 HOLD→DONE) {ci(S('search_ok'))} | 빨강 기하 5 m·5 s {S('search_geo_red')}/{n} | mode 3 종료 대상 {at}")
    lines.append(f"  도킹 칸 안 접안 {ci(S('dock_inbay'))} | CONTACT {S('dock_contact')}/{n} | 후진 이탈 {S('dock_backout')}/{n} | SEARCH 타임아웃 {S('dock_search_timeout')}/{n} | "
                 f"mode 7 진입 시 도크 거리 중앙값 {med(sorted(r['dock_dist_mode7'] for r in R if r['dock_dist_mode7'] is not None)):.1f} m")
    if any(r.get("dock_custom") for r in R):
        g_ = R[0]["dock_geom"]; cls = {}
        for r in R:
            cls[r["dock_class"]] = cls.get(r["dock_class"], 0) + 1
        dep = sorted(r["dock_depth_at"] for r in R if r["dock_depth_at"] is not None)
        lines.append(f"  도크 형상 칸폭 {g_['bay_w']:g} m·칸막이 {g_['gap']:g} m·깊이 {g_['depth']:g} m, 정렬 {g_['align']} | 분류 "
                     + ", ".join(f"{k} {cls[k]}" for k in DOCK_CLASSES if k in cls)
                     + (f" | 접안 순간 뒷벽까지(배 중심) 중앙값 {med(dep):.2f} m" if dep else ""))
    lines.append(f"  코스 시간(WP10 2.5 m 도달) 중앙값 {med(tc):.0f} s (도달 {len(tc)}/{n}, ≤900 s {sum(1 for v in tc if v <= 900)}/{n}) | 런 종료 중앙값 {med(tend):.0f} s | 전 WP 완료 {S('finished_all')}/{n}")
    lines.append(f"  WP 시간 초과 합 {sum(r['timeouts'] for r in R)} (런당 중앙값 {med(sorted(r['timeouts'] for r in R))}) | 접촉 합 {sum(r['contacts_total'] for r in R)} "
                 f"(회피 장애물 {sum(r['contacts_avoid'] for r in R)}, 게이트 {sum(r['contacts']['gate'] for r in R)}, 위치유지 {sum(r['contacts']['hold'] for r in R)}, "
                 f"탐색 부표 {sum(r['contacts']['search'] for r in R)}, 도크 {sum(r['contacts']['dock'] for r in R)})")
    if all("avoid_out_s" in r for r in R):
        om = sorted(r["avoid_out_max"] for r in R)
        lines.append(f"  장애물 구간 경계(±{R[0]['avoid_zone']:g} m, 끝 {AVOID_ZONE_END_M:g} m{', 노드엔 안 넘김 — 기준선' if R[0]['avoid_measure_only'] else ''}) 이탈 런 {sum(1 for r in R if r['avoid_out_s'] > 0)}/{n} "
                     f"(최대 {om[-1]:.2f} m, 이탈 시간 합 {sum(r['avoid_out_s'] for r in R):.1f} s) | 켜진 시간 중앙값 {med(sorted(r['avoid_on_s'] for r in R)):.0f} s")
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
    colmap = {"red": "crimson", "green": "seagreen", "white": "white", "orange": "#ff7f0e", "black": "#222"}
    for b in W["buoys"]:
        ax.add_patch(plt.Circle((b["x"], b["y"]), 0.45 if b["kind"] in ("hold",) or b["kind"].startswith("search") else 0.3,
                                fc=colmap[b["color"]], ec="k", lw=0.7, zorder=4))
    for g in W["gates"]:
        ax.plot([g["red"][0], g["green"][0]], [g["red"][1], g["green"][1]], color="k", ls=":", lw=0.7)
    for a_, b_, kind in W["segs"]:
        ax.plot([a_[0], b_[0]], [a_[1], b_[1]], color="#222", lw=3 if kind == "front" else (2.5 if kind == "cap" else 2))
    if W.get("arena"):
        ar = W["arena"] + [W["arena"][0]]
        ax.plot([q[0] for q in ar], [q[1] for q in ar], color="#888", lw=0.8, ls="--")
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
    ax.set_title(f"변형 {r['variant']} seed {r['seed']} — 게이트 기하 {r['gates_geo']}/{r.get('n_gates', 4)} (/gates_passed {r['gates_node']}), 위치유지 {'달성' if r['hold_geo'] else '미달'}, "
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
    ap.add_argument("--world-wp-file", default=None, help="세계 배치용 웨이포인트(기본 src/north_goal_angle/config/waypoints.yaml, --world 면 그 waypoints_file)")
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
                    help="LiDAR 잡음(σ3cm·결측5%%·정면±30° 고립 물보라1%%) — 합성, add_lidar_noise 참조")
    # %% — Python 3.13+ argparse 가 도움말의 단독 % 를 형식 오류로 막는다(11-11 Mac 3.15 에서 시작부터 죽음).
    ap.add_argument("--search-behavior", default="orbit", choices=["hold", "orbit"],   # = launch 기본(C3)
                    help="탐색(mode 3) 행동 — launch search_behavior 와 같은 값을 ship_back·ship_turn 에 넘긴다")
    ap.add_argument("--param", action="append", default=[], metavar="노드.키=값",
                    help="노드 파라미터 덮어쓰기(반복 가능), 예 --param ship_direction.dwa_enabled=true")
    ap.add_argument("--avoid-zone", type=float, default=0.0, metavar="반폭m",
                    help="장애물 구간 경계 켬: WP6→WP7 ±반폭 m 직사각형(양 끝 4 m 연장)을 geofence_avoid_polygon 으로 넘기고 이탈을 잰다")
    ap.add_argument("--avoid-measure-only", action="store_true",
                    help="--avoid-zone 경계를 노드엔 넘기지 않고 이탈만 잰다(경계 효과 비교 기준선)")
    # 세계·도크 (기본값이면 옛 동작과 같다 — 기본 형상 = 칸 1 m·칸막이 두께 0·깊이 0.5 m, 목표 칸 무작위)
    ap.add_argument("--world", default=None, metavar="yaml",
                    help="명시 배치 세계 파일(예 tools/sim/worlds/arena_2025.yaml). 세계·웨이포인트를 그 파일에서")
    ap.add_argument("--write-world-wp", action="store_true",
                    help="--world 의 waypoints(로컬 xy)를 위경도 웨이포인트 파일(waypoints_file)로 쓰고 끝낸다")
    ap.add_argument("--dock-bay-w", type=float, default=None, help="칸 폭 m (기본 1.0, 세계 파일 값이 있으면 그 값)")
    ap.add_argument("--dock-gap", type=float, default=None, help="칸막이(칸 사이·양 끝 덩어리) 두께 m (기본 0)")
    ap.add_argument("--dock-depth", type=float, default=None, help="칸 깊이 m (기본 0.5)")
    ap.add_argument("--dock-target", default="random", choices=["-1", "0", "1", "random"],
                    help="목표 칸(가로 t 방향 번호, t = 물 쪽 n 을 시계로 90°). random = 시드 무작위(기본)")
    ap.add_argument("--dock-align", default="wp", choices=["wp", "target"],
                    help="wp = 웨이포인트 그대로(정렬점 = 파일 값) / target = 정렬·WP8·WP9 를 목표 칸 축으로 옮긴 사본")
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    if a.write_world_wp:
        if not a.world:
            raise SystemExit("--write-world-wp 는 --world 와 같이")
        wd = load_world(a.world)
        with open(wd["_wp_path"], "w", encoding="utf-8") as f:
            f.write(world_wp_text(wd))
        print("저장:", wd["_wp_path"])
        return
    dock_cfg = dict(bay_w=a.dock_bay_w, gap=a.dock_gap, depth=a.dock_depth,
                    target=None if a.dock_target == "random" else float(a.dock_target))
    dov = {kv.split("=")[0].strip(): float(kv.split("=")[1]) for kv in a.dwell_label.split(",") if "=" in kv} or None
    kw = dict(wp_file=a.wp_file, world_wp_file=a.world_wp_file, gps_sigma=a.gps_sigma, current=tuple(a.current), t_max=a.t_max, tail=a.tail, dyn=a.dyn, hull=a.hull,
              dwell_override=dov, node_overrides=parse_param_overrides(a.param), lidar_noise=a.lidar_noise,
              search_behavior=a.search_behavior, avoid_zone=a.avoid_zone,
              avoid_measure_only=a.avoid_measure_only, world=a.world, dock_cfg=dock_cfg, dock_align=a.dock_align)
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
            print(summarize([r for r in R if r["variant"].split("-")[0] == vv], f"변형 {vv}{'' if not dov else ' dwell ' + a.dwell_label}, search_behavior {a.search_behavior}, seed {a.seed0}~{a.seed0 + a.seeds - 1}"))
        bad = [r for r in R if not r["ok"]]
        for r in bad[:3]:
            print(f"예외 seed {r['seed']} {r['variant']}: {r.get('traceback', r['exception'])}")


if __name__ == "__main__":
    main()

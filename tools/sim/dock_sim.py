#!/usr/bin/env python3
"""도킹 접안 FSM 2D 시뮬레이션 — 실제 ship_dock 노드 코드를 그대로 돌린다.

  · src/ship_dock/ship_dock/ship_dock.py 의 ShipDock 클래스를 rclpy 스텁으로 import 해서
    timer_cb() 를 틱마다 호출한다(로직 이식이 아니라 실제 FSM 코드 실행). 파라미터는
    src/ship_dock/config/ship_dock.yaml 에서 읽는다.
  · 2D 세계: 도크 정면 벽(폭 3 m, 1 m 간격 3칸, 칸막이 0.5 m) + 가운데 칸 표식.
    카메라: 표식 상대 방위(HFOV 63.9° = vision.yaml, 확정 지연 0.3 s = 3프레임, 0.7~12 m 에서 보임).
    LiDAR: 벽 레이캐스팅(0.5° 간격, 80° = 정면 규약), ship_dock 이 전방 ±15° 최소거리를 스스로 고른다.
  · 조향·추진: ship_direction 의 candidate 매핑(desired = 80 + rel)과 motor_control 분기
    (조향 차동, SPIN 제자리 선회, 후진 1590, 1.2 m 이내 감속 0.7)를 재현. 운동 상수는 tools/sim/ssf_sim.py
    와 같다(k_v 0.010 m/s/PWM, k_w 0.22 °/s/PWM → 순항 1.4 m/s, 제자리 선회 44 °/s). 1차 지연 τ 0.5 s.
  · 몬테카를로 500 런: 거리 3~10 m(표식까지 11 m 이내), 좌우 −5~5 m, 출발 헤딩 0~360° 전방향, 목표 칸 좌/중/우 무작위.
  · 모르는 것(도크 실제 규격·실제 추력 응답)은 가정이다 — 그림·보고서에 그렇게 적는다.

재현: python3 tools/sim/dock_sim.py   → docs/전달용/배치도/dock_sim.png, tools/sim/out/dock_sim.csv
"""
import csv
import importlib.util
import math
import os
import random
import sys
import types

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT_PNG = os.path.join(ROOT, "docs", "전달용", "배치도", "dock_sim.png")
OUT_CSV = os.path.join(ROOT, "tools", "sim", "out", "dock_sim.csv")
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False

# ───────────────────────── 1. rclpy 스텁 + 실제 ship_dock import ─────────────────────────
class _Param:
    def __init__(self, v): self.value = v

class _Logger:
    def __init__(self): self.lines = []
    def _log(self, lvl, msg, **kw): self.lines.append((lvl, msg))
    def info(self, m, **kw): self._log("INFO", m)
    def warn(self, m, **kw): self._log("WARN", m)
    def error(self, m, **kw): self._log("ERROR", m)

class _Pub:
    def __init__(self): self.last = None
    def publish(self, msg): self.last = float(msg.data)

class _Node:
    def __init__(self, name):
        self._params = dict(_PARAMS); self._logger = _Logger(); self.pubs = []
    def declare_parameter(self, name, default): return _Param(self._params.get(name, default))
    def create_publisher(self, typ, topic, qos):
        p = _Pub(); self.pubs.append(p); return p
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_logger(self): return self._logger

class _Msg:
    def __init__(self, data=0.0): self.data = data

def _stub_modules():
    rclpy = types.ModuleType("rclpy"); rclpy.init = lambda *a, **k: None; rclpy.ok = lambda: True; rclpy.shutdown = lambda: None
    node = types.ModuleType("rclpy.node"); node.Node = _Node
    ex = types.ModuleType("rclpy.executors"); ex.MultiThreadedExecutor = object
    qos = types.ModuleType("rclpy.qos")
    for n in ("QoSProfile", "ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(qos, n, type(n, (), {"__init__": lambda self, **k: None, "BEST_EFFORT": 1, "RELIABLE": 2, "KEEP_LAST": 1, "VOLATILE": 1}))
    std = types.ModuleType("std_msgs"); stdm = types.ModuleType("std_msgs.msg"); stdm.Float32 = _Msg; stdm.Int32 = _Msg
    sen = types.ModuleType("sensor_msgs"); senm = types.ModuleType("sensor_msgs.msg"); senm.LaserScan = object
    for k, v in {"rclpy": rclpy, "rclpy.node": node, "rclpy.executors": ex, "rclpy.qos": qos,
                 "std_msgs": std, "std_msgs.msg": stdm, "sensor_msgs": sen, "sensor_msgs.msg": senm}.items():
        sys.modules.setdefault(k, v)

def _load_params():
    import yaml
    with open(os.path.join(ROOT, "src", "ship_dock", "config", "ship_dock.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)["/ship_dock"]["ros__parameters"]

_PARAMS = _load_params()
_stub_modules()
sys.path.insert(0, os.path.join(ROOT, "src", "ship_dock"))   # ship_dock.py 의 `from ship_dock.dock_fsm_logic import …` 용
_spec = importlib.util.spec_from_file_location("ship_dock_real", os.path.join(ROOT, "src", "ship_dock", "ship_dock", "ship_dock.py"))
SD = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(SD)

class _Clock:
    t = 0.0
SD.time = types.SimpleNamespace(monotonic=lambda: _Clock.t)   # 노드의 time.monotonic() 을 시뮬 시계로

# ───────────────────────── 2. 세계 ─────────────────────────
DOCK_W, BAY, DIV = 3.0, 1.0, 0.5         # 정면 벽 폭, 칸 폭, 칸막이 길이 (⚠️ 실제 규격 공지 전 가정)
MARKER = (0.0, 0.0)                      # 가운데 칸 표식 (벽 y=0, 배는 y>0 쪽에서 접근)
SEGS = [((-DOCK_W / 2, 0.0), (DOCK_W / 2, 0.0))] + \
       [((x, 0.0), (x, DIV)) for x in (-1.5, -0.5, 0.5, 1.5)]
HFOV = 63.9                              # vision.yaml hfov_deg
CAM_CONFIRM, CAM_MIN, CAM_MAX = 0.3, 0.7, 12.0
SCAN_MIN_DEG, SCAN_INC = -180.0, 0.5
LIDAR_FWD = SD.LIDAR_FORWARD_DEG         # 80
# motor_control / 운동 상수 (tools/sim/ssf_sim.py 와 동일)
BASE_PWM, MAX_ANGLE, MAX_DIFF, NEUTRAL, REVERSE_PWM = 1360, 81, 100, 1500, 1590
SPIN_CENTER, SPIN_DIFF = 1500, 100
SLOW_START, MIN_RATIO = 1.2, 0.7
K_V, K_W, TAU, DT = 0.010, 0.22, 0.5, 0.1
HULL_HALF_L = 0.83                       # 1번배 LOA 1.66 / 2 — 선수가 벽에 닿았는지 보는 데만 쓴다

def wrap(a): return (a + 180.0) % 360.0 - 180.0

def ray_hit(px, py, bearing_deg, segs, max_r=25.0):
    dx, dy = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
    best = math.inf
    for (ax, ay), (bx, by) in segs:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9: continue
        t = ((ax - px) * ey - (ay - py) * ex) / den
        u = ((ax - px) * dy - (ay - py) * dx) / den
        if t > 0 and 0.0 <= u <= 1.0 and t < best: best = t
    return best if best < max_r else math.inf

class Scan:                              # sensor_msgs/LaserScan 흉내 (ship_dock 이 읽는 필드만)
    def __init__(self, ranges):
        self.angle_min = math.radians(SCAN_MIN_DEG); self.angle_increment = math.radians(SCAN_INC); self.ranges = ranges

_N_SCAN = int(360 / SCAN_INC)
_A_LIDAR = SCAN_MIN_DEG + np.arange(_N_SCAN) * SCAN_INC          # LiDAR 프레임 각 (80 = 정면, 커질수록 좌)
_FRONT = np.abs(_A_LIDAR - LIDAR_FWD) <= 60.0                      # 전방 ±60° 만 캐스팅 (속도)
_SEG = np.array([[a[0], a[1], b[0], b[1]] for a, b in SEGS])

def lidar(x, y, psi):
    """벽 레이캐스팅(numpy). 반환 ranges 는 ship_dock.front_min_dist 가 읽는 리스트."""
    b = np.radians(psi - (_A_LIDAR[_FRONT] - LIDAR_FWD))
    dx, dy = np.sin(b), np.cos(b)
    best = np.full(dx.shape, np.inf)
    for ax, ay, bx, by in _SEG:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        ok = np.abs(den) > 1e-9
        t = np.where(ok, ((ax - x) * ey - (ay - y) * ex) / np.where(ok, den, 1.0), np.inf)
        u = np.where(ok, ((ax - x) * dy - (ay - y) * dx) / np.where(ok, den, 1.0), -1.0)
        hit = ok & (t > 0) & (u >= 0.0) & (u <= 1.0)
        best = np.where(hit & (t < best), t, best)
    ranges = np.full(_N_SCAN, np.inf); ranges[_FRONT] = np.where(best < 25.0, best, np.inf)
    return Scan(ranges.tolist())

def motor(desired, front_d):
    """motor_control 분기 → (pwm_r, pwm_l)."""
    if desired >= 50000: return NEUTRAL, NEUTRAL
    if 20000 <= desired < 50000: r = l = BASE_PWM
    elif 5000 <= desired < 6000: return SPIN_CENTER + SPIN_DIFF, SPIN_CENTER - SPIN_DIFF   # 우선회
    elif 6000 <= desired < 20000: return SPIN_CENTER - SPIN_DIFF, SPIN_CENTER + SPIN_DIFF  # 좌선회
    elif -1.0 <= desired <= 161.0:
        off = max(-MAX_ANGLE, min(MAX_ANGLE, desired - 80.0)); d = abs(off) / MAX_ANGLE * MAX_DIFF
        r, l = (BASE_PWM - d, BASE_PWM + d) if off > 0 else (BASE_PWM + d, BASE_PWM - d)
    elif 161.0 < desired < 5000.0: return REVERSE_PWM, REVERSE_PWM
    else: r = l = BASE_PWM
    if front_d is not None and front_d < SLOW_START:            # 전진 명령만 감속
        k = max(MIN_RATIO, front_d / SLOW_START)
        r, l = NEUTRAL - (NEUTRAL - r) * k, NEUTRAL - (NEUTRAL - l) * k
    return r, l

def ship_direction_map(cand):
    if cand == SD.STOP_HOLD or cand == SD.CANDIDATE_INVALID: return 50000.0 if cand == SD.STOP_HOLD else 20000.0
    if cand in (SD.SPIN_RIGHT, SD.SPIN_LEFT): return cand
    c = cand % 360.0
    m = 80.0 + c if c <= 180.0 else 80.0 - (360.0 - c)
    return m if 0.0 <= m <= 160.0 else 50000.0

# ───────────────────────── 3. 한 런 ─────────────────────────
STATES = ["INIT", "SEARCH", "ALIGN", "APPROACH", "CONTACT", "BACKOFF", "DONE"]
COL = {"INIT": "#9e9e9e", "SEARCH": "#8e44ad", "ALIGN": "#e67e22", "APPROACH": "#1f77b4",
       "CONTACT": "#d62728", "BACKOFF": "#2ca02c", "DONE": "#555555"}

def run(x, y, psi, seed=0, t_max=120.0, marker=(0.0, 0.0)):
    MARKER = marker
    rng = random.Random(seed)
    _Clock.t = 0.0
    node = SD.ShipDock(); pub = node.pubs[0]
    node.wp_mode = node.active_wp_mode
    v = w = 0.0; vis_since = None
    log = dict(t=[], x=[], y=[], psi=[], state=[], img=[], dist=[])
    out = dict(success=False, t_contact=None, head_err=None, lat_err=None, done=False, min_wall=math.inf, bumped=False, reached_approach=False, lost_dist=None)
    steps = int(t_max / DT)
    for k in range(steps):
        t = _Clock.t
        # 카메라: 표식 상대 방위 (양수 = 좌, image_positive_is_left=true)
        d_m = math.hypot(MARKER[0] - x, MARKER[1] - y)
        bearing = math.degrees(math.atan2(MARKER[0] - x, MARKER[1] - y))
        rel = wrap(bearing - psi)                       # 양수 = 우(CW)
        visible = abs(rel) <= HFOV / 2 and CAM_MIN <= d_m <= CAM_MAX and y > 0.05
        vis_since = (vis_since if vis_since is not None else t) if visible else None
        if visible and (t - vis_since) >= CAM_CONFIRM:
            node.image_angle = -rel + rng.gauss(0.0, 0.5); node.last_image_t = t   # 방위 잡음 0.5°
        # LiDAR
        node.latest_scan = lidar(x, y, psi); node.last_scan_t = t
        # 실제 FSM 한 틱
        node.timer_cb()
        cand = pub.last if pub.last is not None else SD.STOP_HOLD
        front = node.front_min_dist(t)
        pr, pl = motor(ship_direction_map(cand), front)
        TR, TL = NEUTRAL - pr, NEUTRAL - pl
        v_cmd, w_cmd = K_V * (TR + TL) / 2.0, K_W * (TL - TR)
        v += (v_cmd - v) * DT / TAU; w += (w_cmd - w) * DT / TAU
        psi = (psi + w * DT) % 360.0
        x += v * math.sin(math.radians(psi)) * DT; y += v * math.cos(math.radians(psi)) * DT
        bow_y = y - HULL_HALF_L * math.cos(math.radians(psi)) if abs(wrap(psi - 180)) < 90 else y
        out["min_wall"] = min(out["min_wall"], bow_y)
        st = node.state
        if st == "APPROACH": out["reached_approach"] = True
        if log["state"] and log["state"][-1] == "APPROACH" and st in ("ALIGN", "SEARCH") and front is not None: out["lost_dist"] = front
        if st == "CONTACT" and out["t_contact"] is None:
            out.update(success=True, t_contact=t, head_err=abs(wrap(psi - 180.0)), lat_err=x - MARKER[0])
        if st == "DONE":
            out["done"] = True
        for key, val in zip(log, (t, x, y, psi, st, node.image_angle if node.marker_ok(t) else float("nan"), front if front is not None else float("nan"))):
            log[key].append(val)
        if st == "DONE" and k > 5: break
        _Clock.t += DT
    out["bumped"] = out["min_wall"] < 0.0
    out["t_end"] = _Clock.t; out["final_state"] = node.state
    return log, out

# ───────────────────────── 4. 시나리오 ─────────────────────────
REPR = [  # (x, y, psi) — psi 180 = 도크 정면 향함
    (0.0, 8.0, 180.0, "정면 8 m"),
    (3.0, 7.0, 180.0, "우측 3 m 비켜섬"),
    (-3.0, 7.0, 120.0, "좌측 3 m, 60° 틀어짐"),
    (2.0, 6.0, 300.0, "표식 등지고 시작(탐색)"),
    (-1.5, 9.0, 220.0, "좌 1.5 m, 40° 틀어짐"),
    (4.0, 4.0, 180.0, "우측 4 m 가까이"),
]

def main():
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    reps = [(run(x, y, p, seed=i), lab) for i, (x, y, p, lab) in enumerate(REPR)]
    # 몬테카를로: 거리 5~10 m, 좌우 −4~4 m, 헤딩 오차 ±90°
    rng = random.Random(2026); mc = []; N_MC = 500
    i = 0
    while len(mc) < N_MC:
        x0 = rng.uniform(-5, 5); y0 = rng.uniform(3, 10); p0 = rng.uniform(0, 360); bay = rng.choice((-1.0, 0.0, 1.0))
        if math.hypot(x0 - bay, y0) > 11.0: continue       # 표식 인식 가정 거리(12 m) 안에서만 출발
        i += 1
        _, o = run(x0, y0, p0, seed=100 + i, marker=(bay, 0.0)); o.update(x0=x0, y0=y0, psi0=p0, bay=bay, herr0=abs(wrap(p0 - 180.0)), lat0=x0 - bay); mc.append(o)
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        wri = csv.writer(f); wri.writerow(["x0", "y0", "psi0", "bay", "success", "t_contact", "head_err_deg", "lat_err_m", "done", "bumped", "final_state", "reached_approach", "lost_dist"])
        for o in mc:
            wri.writerow([f"{o['x0']:.2f}", f"{o['y0']:.2f}", f"{o['psi0']:.1f}", f"{o['bay']:.0f}", int(o["success"]),
                          "" if o["t_contact"] is None else f"{o['t_contact']:.1f}",
                          "" if o["head_err"] is None else f"{o['head_err']:.1f}",
                          "" if o["lat_err"] is None else f"{o['lat_err']:.2f}", int(o["done"]), int(o["bumped"]), o["final_state"], int(o["reached_approach"]), "" if o["lost_dist"] is None else f"{o['lost_dist']:.2f}"])
    ok = [o for o in mc if o["success"]]
    print(f"몬테카를로 {N_MC} 런: 접안 {len(ok)}/{N_MC}, 이탈 완료(DONE) {sum(o['done'] for o in mc)}/{N_MC}, 선수 벽 접촉 {sum(o['bumped'] for o in mc)}/{N_MC}")
    HB = [(0, 30), (30, 60), (60, 90), (90, 135), (135, 180.01)]; LB = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 10.01)]
    def rate(sel): return (sum(o["success"] for o in sel), len(sel))
    print("  출발 헤딩 오차별 접안:", ", ".join(f"{a:.0f}~{b:.0f}° {rate([o for o in mc if a <= o['herr0'] < b])[0]}/{rate([o for o in mc if a <= o['herr0'] < b])[1]}" for a, b in HB))
    print("  표식 기준 좌우 거리별 접안:", ", ".join(f"{a}~{b:.0f} m {rate([o for o in mc if a <= abs(o['lat0']) < b])[0]}/{rate([o for o in mc if a <= abs(o['lat0']) < b])[1]}" for a, b in LB))
    fails = [o for o in mc if not o["success"]]
    print(f"  실패 {len(fails)}: 표식 못 찾음(탐색 타임아웃) {sum(1 for o in fails if not o['reached_approach'])}, 접근 중 유실 {sum(1 for o in fails if o['reached_approach'])}"
          + (f" (유실 거리 중앙값 {sorted(o['lost_dist'] for o in fails if o['lost_dist'] is not None)[len([o for o in fails if o['lost_dist'] is not None])//2]:.2f} m)" if any(o['lost_dist'] is not None for o in fails) else ""))
    byh = [rate([o for o in ok if a <= o['herr0'] < b]) for a, b in HB]
    medh = [sorted(o["head_err"] for o in ok if a <= abs(o['lat0']) < b) for a, b in LB]
    print("  좌우 거리별 접안 헤딩 오차 중앙값:", ", ".join(f"{a}~{b:.0f} m {m[len(m)//2]:.0f}°" for (a, b), m in zip(LB, medh) if m))
    if ok:
        he = sorted(o["head_err"] for o in ok); le = sorted(abs(o["lat_err"]) for o in ok); tc = sorted(o["t_contact"] for o in ok)
        print(f"  접안 시 헤딩 오차: 중앙값 {he[len(he)//2]:.1f}°, 최대 {he[-1]:.1f}° | 좌우 벗어남: 중앙값 {le[len(le)//2]:.2f} m, 최대 {le[-1]:.2f} m | 접안까지 중앙값 {tc[len(tc)//2]:.0f} s, 최대 {tc[-1]:.0f} s")
    for (log, o), lab in reps:
        print(f"  [{lab}] 접안 {o['success']} t={o['t_contact']} 헤딩오차={None if o['head_err'] is None else round(o['head_err'],1)} 좌우={None if o['lat_err'] is None else round(o['lat_err'],2)} 종료={o['final_state']}")

    # ── 그림 ──
    fig = plt.figure(figsize=(13.2, 9.6))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.0, 1.0], height_ratios=[1.0, 1.0], hspace=0.34, wspace=0.3)
    ax = fig.add_subplot(gs[0, 0])
    for (a, b) in SEGS: ax.plot([a[0], b[0]], [a[1], b[1]], color="#333", lw=3)
    ax.plot(*MARKER, marker="s", ms=10, color="#d62728", zorder=5); ax.text(0.0, -0.85, "목표 표식 (가운데 칸)", ha="center", va="top", fontsize=8)
    for (log, o), lab in reps:
        xs, ys, ss = log["x"], log["y"], log["state"]
        for i in range(1, len(xs)):
            ax.plot(xs[i - 1:i + 1], ys[i - 1:i + 1], color=COL[ss[i]], lw=1.6, solid_capstyle="round")
        for st in ("SEARCH", "ALIGN"):                     # 제자리 선회 구간은 길이가 없어 점으로
            pts = [(xs[i], ys[i]) for i in range(len(xs)) if ss[i] == st]
            if pts: ax.plot(*zip(*pts), ls="none", marker="o", ms=3.2, color=COL[st], alpha=0.8)
        ax.plot(xs[0], ys[0], marker="o", ms=5, color="k"); ax.annotate(lab, (xs[0], ys[0]), xytext=(4, 4), textcoords="offset points", fontsize=7.5)
    ax.set_aspect("equal"); ax.set_xlim(-6.5, 6.5); ax.set_ylim(-1.8, 11.0); ax.grid(alpha=0.25)
    ax.tick_params(labelsize=8)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]  (도크 벽 = 0)")
    ax.set_title("(a) 대표 출발 6곳 — 궤적 색 = FSM 상태", fontsize=10.5)
    ax.legend(handles=[Line2D([0], [0], color=COL[s], lw=3, label=s) for s in STATES], loc="lower left", fontsize=7, ncol=1, framealpha=0.9)
    # (b) 한 런의 시간선
    (log, o), lab = reps[2]
    ax2 = fig.add_subplot(gs[0, 1:]); t = log["t"]
    ax2.plot(t, log["dist"], color="#1f77b4", lw=1.4, label="LiDAR 전방 ±15° 최소거리 [m]")
    ax2.axhline(_PARAMS["standoff_dist_m"], color="#1f77b4", ls=":", lw=1); ax2.axhline(_PARAMS["contact_dist_m"], color="#d62728", ls=":", lw=1); ax2.axhline(_PARAMS["backoff_dist_m"], color="#2ca02c", ls=":", lw=1)
    ax2.text(t[-1], _PARAMS["standoff_dist_m"], " standoff", va="bottom", ha="right", fontsize=7.5, color="#1f77b4")
    ax2.text(t[-1], _PARAMS["contact_dist_m"], " contact", va="bottom", ha="right", fontsize=7.5, color="#d62728")
    ax2.set_ylabel("거리 [m]"); ax2.set_ylim(0, 11)
    ax2b = ax2.twinx(); ax2b.plot(t, log["img"], color="#e67e22", lw=1.2, label="/image_angle [°]"); ax2b.set_ylim(-45, 45); ax2b.set_ylabel("표식 방위 [°] (양수 = 좌)")
    ax2b.axhspan(-_PARAMS["align_tol_deg"], _PARAMS["align_tol_deg"], color="#e67e22", alpha=0.08)
    prev = None
    for i, s in enumerate(log["state"]):
        if s != prev:
            ax2.axvspan(t[i], t[i], color=COL[s]); prev = s
    # 상태 띠
    y0 = 10.3
    i0 = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or log["state"][i] != log["state"][i0]:
            ax2.axvspan(t[i0], t[min(i, len(t) - 1)], ymin=0.93, ymax=1.0, color=COL[log["state"][i0]], alpha=0.9)
            if t[min(i, len(t) - 1)] - t[i0] > 2.5: ax2.text((t[i0] + t[min(i, len(t) - 1)]) / 2, y0 + 0.1, log["state"][i0], ha="center", va="bottom", fontsize=6.5, color="white", weight="bold")
            i0 = i
    ax2.set_xlabel("시간 [s]"); ax2.set_title(f"(b) 출발 '{lab}' — 거리·표식 방위·상태", fontsize=10.5)
    h1, l1 = ax2.get_legend_handles_labels(); h2, l2 = ax2b.get_legend_handles_labels(); ax2.legend(h1 + h2, l1 + l2, fontsize=7.5, loc="center right")
    # (c) 몬테카를로
    ax3 = fig.add_subplot(gs[1, 0:2])
    sc = ax3.scatter([o["lat_err"] for o in ok], [o["head_err"] for o in ok], c=[o["t_contact"] for o in ok], cmap="viridis", s=28, edgecolor="k", lw=0.4)
    cb = fig.colorbar(sc, ax=ax3, pad=0.02); cb.set_label("접안까지 시간 [s]", fontsize=8.5)
    ax3.axvspan(-0.5, 0.5, color="#2ca02c", alpha=0.08); ax3.axhline(_PARAMS["align_tol_deg"], color="#e67e22", ls=":", lw=1)
    ax3.text(0.5, _PARAMS["align_tol_deg"], f" align_tol {_PARAMS['align_tol_deg']:.0f}°", fontsize=7.5, color="#e67e22", va="bottom")
    ax3.set_xlabel("접안 시 목표 칸 중심 기준 좌우 벗어남 [m]  (칸 폭 ±0.5 m 음영)"); ax3.set_ylabel("접안 시 헤딩 오차 [°]")
    ax3.set_xlim(-1.6, 1.6); ax3.set_ylim(0, max(20, max([o["head_err"] for o in ok] + [0]) * 1.15))
    ax3.set_title(f"(c) 무작위 출발 {N_MC} 런(거리 3~10 m · 좌우 ±5 m · 헤딩 전방향 · 칸 3개): 접안 {len(ok)}, 이탈 완료 {sum(o['done'] for o in mc)}, 벽 접촉 {sum(o['bumped'] for o in mc)}", fontsize=10)
    ax3.grid(alpha=0.25)
    # (d) 출발 조건별 접안 성공률
    ax4 = fig.add_subplot(gs[1, 2])
    M = np.zeros((len(LB), len(HB))); Ncnt = np.zeros_like(M, dtype=int)
    for i, (la, lb) in enumerate(LB):
        for j, (ha, hb) in enumerate(HB):
            sel = [o for o in mc if la <= abs(o["lat0"]) < lb and ha <= o["herr0"] < hb]
            Ncnt[i, j] = len(sel); M[i, j] = (sum(o["success"] for o in sel) / len(sel)) if sel else np.nan
    im = ax4.imshow(M, origin="lower", cmap="RdYlGn", vmin=0.5, vmax=1.0, aspect="auto")
    for i in range(len(LB)):
        for j in range(len(HB)):
            if Ncnt[i, j]: ax4.text(j, i, f"{M[i, j] * 100:.0f}%\n n={Ncnt[i, j]}", ha="center", va="center", fontsize=7.5)
    ax4.set_xticks(range(len(HB))); ax4.set_xticklabels([f"{a:.0f}~{min(b, 180):.0f}°" for a, b in HB], fontsize=7.5)
    ax4.set_yticks(range(len(LB))); ax4.set_yticklabels([f"{a}~{min(b, 5):.0f} m" for a, b in LB], fontsize=7.5)
    ax4.set_xlabel("출발 헤딩 오차 (도크 정면 기준)"); ax4.set_ylabel("출발 시 표식 기준 좌우 거리")
    ax4.set_title("(d) 출발 조건별 접안 성공률", fontsize=10.5)
    fig.suptitle("도킹 접안 FSM 2D 시뮬레이션 — 실제 ship_dock 코드 실행, 도크 규격·추력 응답은 가정 (tools/sim/dock_sim.py)", fontsize=11, y=0.995)
    fig.savefig(OUT_PNG, dpi=160, bbox_inches="tight"); print("저장:", OUT_PNG)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""항로추종(빨강-초록 게이트) 2D 시뮬레이션 — 실제 ship_gate·ship_direction·motor_control 노드 코드를 그대로 돌린다.

  · rosstub.py 로 세 노드 파일을 import 해 틱마다 콜백·타이머를 호출(파라미터는 각 노드 yaml).
  · 카메라 모델: 색별로 HFOV 80.32°(vision.yaml) 안·0.5~12 m 의 가장 가까운 부표(검출기의 '가장 큰 컨투어 1개'와 같음)
    의 상대 방위를 /red_angle·/green_angle 로 발행. 부호는 vision_geom.angle_from_pixel 과 같이 **왼쪽 = 양수**.
    confirm_frames 1(vision.yaml 기본). 안 보이면 10000. 방위 잡음 0.5°.
  · LiDAR: FastWorld 부표 레이캐스팅(ship_gate 의 arming 거리, ship_direction 의 갭 회피가 쓴다).
  · 코스: 게이트 4개(x 15·24·33·42 m, 스윕 코스와 같은 자리), 시드마다 폭 3~6 m·중심 ±1 m 흔들림,
    빨강은 진행 방향(동쪽) 기준 **좌현**(north), 초록 우현. 출발 (5, 3±1), 헤딩 90°±30°.
    GPS 웨이포인트(= /yaw_error 의 목표)는 다음 게이트 중점, 마지막은 (48, 3).
  · 변형: 수정 후(red_on_port true) / red_on_port false(= 수정 전 단일 부표 부호와 동일한 동작) / 비전 없이 GPS 웨이포인트만.
  · 간격 스트레스: 게이트 사이 4·6·8 m × 폭 4~8 m 로 /gates_passed 일치율만 추가 집계(카메라만으로는 ≤ 6 m 에서 누락 가능).
  · 운동 모델은 스윕과 같다(순항 1.2 m/s, k_w 0.22, 지연 없음). 접촉 = 배 반경 half_width 원 vs 부표 반경 0.2 m.

재현: python3 tools/sim/gate_sim.py → docs/전달용/배치도/gate_sim.png, tools/sim/out/gate_sim.csv
"""
import csv
import math
import os
import random
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import rosstub                                   # noqa: E402
rosstub.install()
from rosstub import Clock                        # noqa: E402
import kaboat_full_course_sim as K               # noqa: E402
from fastworld import FastWorld                  # noqa: E402

P_SG = rosstub.load_params(os.path.join(ROOT, "src/ship_gate/config/ship_gate.yaml"), "/ship_gate")
P_SD = rosstub.load_params(os.path.join(ROOT, "src/ship_direction/config/ship_direction.yaml"), "/ship_direction")
P_MC = rosstub.load_params(os.path.join(ROOT, "src/motor_control/config/motor_control.yaml"), "/motor_control")
SG = rosstub.load_node(os.path.join(ROOT, "src/ship_gate/ship_gate/ship_gate.py"), "ship_gate_real", extra_path=os.path.join(ROOT, "src/ship_gate"))
SD = rosstub.load_node(os.path.join(ROOT, "src/ship_direction/ship_direction/ship_direction.py"), "ship_direction_real", extra_path=os.path.join(ROOT, "src/ship_direction"))
MC = rosstub.load_node(os.path.join(ROOT, "src/motor_control/motor_control/motor_control.py"), "motor_control_real")
std = sys.modules["std_msgs.msg"]
Float32, Int32 = std.Float32, std.Int32

OUT_PNG = os.path.join(ROOT, "docs", "전달용", "배치도", "gate_sim.png")
OUT_CSV = os.path.join(HERE, "out", "gate_sim.csv")
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False

GATE_MODE = 1                    # 표 Ⅲ-7: 0·1 = 게이트 (ship_gate 담당)
GATE_X = [15.0, 24.0, 33.0, 42.0]
HFOV, CAM_MIN, CAM_MAX, CAM_NOISE = 80.32, 0.5, 12.0, 0.5
DT, MAX_T, X_END = K.DT, 150.0, 46.0
INVALID = 10000.0


class Scan:
    def __init__(self, ranges):
        self.angle_min = math.radians(K.SCAN_ANGLE_MIN_DEG)
        self.angle_increment = math.radians(K.SCAN_INC_DEG)
        self.ranges = ranges


def wrap(a): return (a + 180.0) % 360.0 - 180.0


def make_course(seed):
    rng = random.Random(seed)
    gates = []
    for gx in GATE_X:
        w = rng.uniform(3.0, 6.0); yc = 3.0 + rng.uniform(-1.0, 1.0)
        gates.append(dict(x=gx, yc=yc, w=w, red=(gx, yc + w / 2), green=(gx, yc - w / 2)))   # 동진 기준 좌현 = +y = 빨강
    start = (5.0, 3.0 + rng.uniform(-1.0, 1.0)); psi0 = 90.0 + rng.uniform(-30.0, 30.0)
    return gates, start, psi0


def cam_angle(x, y, psi, buoys, rng):
    """색별 가장 가까운 가시 부표의 상대 방위(왼쪽 양수). 없으면 10000."""
    best = None
    for (bx, by) in buoys:
        d = math.hypot(bx - x, by - y)
        rel_cw = wrap(math.degrees(math.atan2(bx - x, by - y)) - psi)     # 양수 = 우(CW)
        if CAM_MIN <= d <= CAM_MAX and abs(rel_cw) <= HFOV / 2 and (best is None or d < best[0]):
            best = (d, -rel_cw)
    return INVALID if best is None else best[1] + rng.gauss(0.0, CAM_NOISE)


def run(seed, variant, tau=0.0):
    gates, (x, y), psi = make_course(seed)
    reds = [g["red"] for g in gates]; greens = [g["green"] for g in gates]
    buoys = [(bx, by, K.BUOY_R) for (bx, by) in reds + greens]
    world = FastWorld(buoys); rng = random.Random(1000 + seed)
    use_vision = variant != "gps"
    if variant == "flip":
        rosstub.PARAMS["ship_gate"] = dict(P_SG, red_on_port=not P_SG["red_on_port"])
    else:
        rosstub.PARAMS["ship_gate"] = dict(P_SG)
    Clock.t = 0.0
    sg = SG.ShipGate() if use_vision else None
    sd = SD.ShipDirection(); mc = MC.MotorController()
    sd.wp_mode_cb(Int32(GATE_MODE))
    if sg: sg.wp_mode_cb(Int32(GATE_MODE))
    v = w = 0.0; t = 0.0; traj = [(x, y)]
    k_goal = 0; hits = 0; hit_flag = {}
    crossed = [None] * len(gates)          # 게이트 x 선을 넘을 때의 y (기하 판정)
    prev_x = x; n_cand = 0; n_fallback = 0
    while t < MAX_T and x < X_END:
        scan = Scan(world.lidar_scan(x, y, psi, max_range=20.0).tolist())
        sd.scan_callback(scan)
        goal = (gates[k_goal]["x"], gates[k_goal]["yc"]) if k_goal < len(gates) else (48.0, 3.0)
        sd.yaw_error_callback(Float32((K.bearing_to(x, y, goal[0], goal[1]) - psi) % 360.0))
        if sg:
            sg.scan_cb(scan)
            sg.red_angle_cb(Float32(cam_angle(x, y, psi, reds, rng)))
            sg.green_angle_cb(Float32(cam_angle(x, y, psi, greens, rng)))
            sg.timer_cb()
            cand = sg.pubs["/candidate_angle"].last.data if sg.pubs["/candidate_angle"].last is not None else SD.CANDIDATE_INVALID
        else:
            cand = SD.CANDIDATE_INVALID
        if cand == SD.CANDIDATE_INVALID: n_fallback += 1
        else: n_cand += 1
        sd.candidate_callback(Float32(cand))
        sd.watchdog_cb(); sd.control_cb()
        desired = sd.pubs["/desired_angle"].last.data
        mc.angle_callback(Float32(desired))
        if sd.pubs["/obstacle_distance_array"].last is not None: mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
        mc.failsafe_callback(sd.pubs["/failsafe_level"].last); mc.timer_callback()
        code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
        TR, TL = 1500 - pr, 1500 - pl
        v_cmd, w_cmd = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
        if tau > 0: v += (v_cmd - v) * DT / tau; w += (w_cmd - w) * DT / tau
        else: v, w = v_cmd, w_cmd
        psi = (psi + w * DT) % 360.0
        prev_x = x
        x += v * math.sin(math.radians(psi)) * DT; y += v * math.cos(math.radians(psi)) * DT
        t += DT; Clock.t = t; traj.append((x, y))
        for i, g in enumerate(gates):
            if crossed[i] is None and prev_x < g["x"] <= x:
                crossed[i] = y
            if k_goal == i and x > g["x"]: k_goal = i + 1
        for i, (bx, by, br) in enumerate(buoys):
            if math.hypot(x - bx, y - by) - br < K.BOAT_HALF_W and not hit_flag.get(i):
                hits += 1; hit_flag[i] = True
    passed = [c is not None and g["green"][1] < c < g["red"][1] for c, g in zip(crossed, gates)]
    lat = [abs(c - g["yc"]) / (g["w"] / 2) for c, g in zip(crossed, gates) if c is not None]
    node_count = sg.pubs["/gates_passed"].last.data if sg and sg.pubs["/gates_passed"].last is not None else -1
    return dict(seed=seed, variant=variant, traj=np.array(traj), t=t, hits=hits, passed=sum(passed), reached=x >= X_END,
                lat=lat, node_count=node_count, n_cand=n_cand, n_fallback=n_fallback, gates=gates)


VARIANTS = [("vision", "비전 게이트 — 부호·카운트 수정 후 (red_on_port=true)"), ("flip", "비전 게이트 — 수정 전 부호와 동일 (red_on_port=false)"), ("gps", "GPS 웨이포인트만 (비전 없음, 중점 좌표를 안다고 가정)")]


def stress(spacing, wmin=4.0, wmax=8.0, N=40):
    """게이트 사이 거리 spacing 으로 코스를 바꿔 /gates_passed 일치율만 센다 (전역 코스 함수 임시 교체)."""
    global GATE_X, X_END, make_course
    keep = (GATE_X, X_END, make_course)
    GATE_X = [15.0 + spacing * i for i in range(4)]; X_END = GATE_X[-1] + 4.0
    def mc(seed):
        rng = random.Random(seed); gates = []
        for gx in GATE_X:
            w = rng.uniform(wmin, wmax); yc = 3.0 + rng.uniform(-1.0, 1.0)
            gates.append(dict(x=gx, yc=yc, w=w, red=(gx, yc + w / 2), green=(gx, yc - w / 2)))
        return gates, (5.0, 3.0 + rng.uniform(-1.0, 1.0)), 90.0 + rng.uniform(-30.0, 30.0)
    make_course = mc
    try:
        R = [run(s, "vision") for s in range(1, N + 1)]
    finally:
        GATE_X, X_END, make_course = keep
    return dict(spacing=spacing, ok=sum(1 for r in R if r["node_count"] == r["passed"]), over=sum(1 for r in R if r["node_count"] > r["passed"]),
                under=sum(1 for r in R if r["node_count"] < r["passed"]), passed=sum(r["passed"] for r in R), hits=sum(r["hits"] for r in R), N=N)


def main():
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    N = 40; res = {k: [run(s, k) for s in range(1, N + 1)] for k, _ in VARIANTS}
    res["vision_tau"] = [run(s, "vision", tau=0.5) for s in range(1, N + 1)]
    names = dict(VARIANTS); names["vision_tau"] = "수정 후, 추력 응답 지연 τ 0.5 s"
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(["variant", "seed", "passed_of_4", "node_count", "hits", "reached", "t", "lat_norm_mean", "cand_ticks", "fallback_ticks"])
        for k in res:
            for r in res[k]:
                wr.writerow([k, r["seed"], r["passed"], r["node_count"], r["hits"], int(r["reached"]), f"{r['t']:.1f}",
                             f"{(sum(r['lat']) / len(r['lat'])) if r['lat'] else float('nan'):.3f}", r["n_cand"], r["n_fallback"]])
    summ = {}
    print(f"항로추종 {N} 시드 × 변형 {len(res)} (게이트 4개, 폭 3~6 m, 중심 ±1 m, 출발 헤딩 90°±30°, 카메라 {CAM_MAX:.0f} m·HFOV {HFOV}°)")
    for k in res:
        R = res[k]; tot = sum(r["passed"] for r in R); full = sum(1 for r in R if r["passed"] == 4)
        lat_all = sorted(v for r in R for v in r["lat"])
        summ[k] = dict(tot=tot, full=full, hits=sum(r["hits"] for r in R), reach=sum(r["reached"] for r in R),
                       lat_med=lat_all[len(lat_all) // 2] if lat_all else float("nan"), lat_max=lat_all[-1] if lat_all else float("nan"),
                       t_med=sorted(r["t"] for r in R)[N // 2], cand=sum(r["n_cand"] for r in R) / max(1, sum(r["n_cand"] + r["n_fallback"] for r in R)),
                       node_ok=sum(1 for r in R if r["node_count"] == r["passed"]))
        s = summ[k]
        print(f"  [{names[k]}] 게이트 통과 {tot}/{4 * N} (4개 전부 {full}/{N}), 접촉 {s['hits']}, 종점 도달 {s['reach']}/{N}, 통과 시 중심 벗어남 중앙값 {s['lat_med']:.2f}·최대 {s['lat_max']:.2f} (반폭=1), "
              f"비전 조향 틱 비율 {s['cand']:.0%}, /gates_passed = 기하 통과 수 일치 {s['node_ok']}/{N}, 시간 중앙값 {s['t_med']:.0f} s")

    st = [stress(sp) for sp in (4.0, 6.0, 8.0)]
    with open(OUT_CSV, "a", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        for x in st:
            wr.writerow([f"stress_spacing_{x['spacing']:.0f}m_w4-8", "all", x["passed"], x["ok"], x["hits"], x["N"], "", "", "", f"over={x['over']} under={x['under']}"])
    print("  간격 스트레스(폭 4~8 m, 수정 후 코드): " + ", ".join(f"사이 {x['spacing']:.0f} m → /gates_passed 일치 {x['ok']}/{x['N']} (이중 {x['over']}, 누락 {x['under']}, 기하 통과 {x['passed']}/{4 * x['N']}, 접촉 {x['hits']})" for x in st))

    # ── 그림 ──
    fig = plt.figure(figsize=(13.4, 8.6))
    gs = fig.add_gridspec(2, 3, height_ratios=[0.9, 1.0], hspace=0.32, wspace=0.28)
    ax = fig.add_subplot(gs[0, :])
    seed_show = 1
    cols = {"vision": "#1f77b4", "flip": "#ff7f0e", "gps": "#7f7f7f"}
    g0 = res["vision"][seed_show - 1]["gates"]
    for g in g0:
        ax.add_patch(plt.Circle(g["red"], 0.35, color="crimson")); ax.add_patch(plt.Circle(g["green"], 0.35, color="seagreen"))
        ax.plot([g["x"], g["x"]], [g["green"][1], g["red"][1]], color="k", lw=0.8, ls=":")
    for k in ("vision", "flip", "gps"):
        r = res[k][seed_show - 1]
        ax.plot(r["traj"][:, 0], r["traj"][:, 1], color=cols[k], lw=1.7, label=f"{names[k]} — 통과 {r['passed']}/4, 접촉 {r['hits']}, {r['t']:.0f} s")
    ax.plot(*res["vision"][seed_show - 1]["traj"][0], "o", color="k", ms=7); ax.text(5.0, 0.4, "출발", ha="center", fontsize=8.5)
    ax.set_xlim(2, 48); ax.set_ylim(-1.5, 8.5); ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]  (빨강 = 진행 방향 좌현)")
    ax.legend(fontsize=8, loc="upper left", ncol=1, framealpha=0.9)
    ax.set_title(f"(a) 시드 {seed_show} — 게이트 4개 통과 궤적, 변형 3종", fontsize=10.5)
    # (b) 통과 게이트 수 분포
    ax2 = fig.add_subplot(gs[1, 0])
    xs = np.arange(5); wbar = 0.2
    for i, k in enumerate(("vision", "flip", "gps", "vision_tau")):
        cnt = [sum(1 for r in res[k] if r["passed"] == n) for n in range(5)]
        ax2.bar(xs + (i - 1.5) * wbar, cnt, width=wbar, color=cols.get(k, "#1f77b4"), alpha=0.55 if k == "vision_tau" else 1.0, hatch="//" if k == "vision_tau" else None, label=names[k])
    ax2.set_xticks(xs); ax2.set_xlabel("통과한 게이트 수 (4개 중)"); ax2.set_ylabel("런 수"); ax2.set_title("(b) 시드별 통과 게이트 수 분포", fontsize=10.5); ax2.legend(fontsize=6.8); ax2.grid(alpha=0.2, axis="y")
    # (c) 통과 시 중심 벗어남
    ax3 = fig.add_subplot(gs[1, 1])
    bins = np.linspace(0, 1.2, 25)
    for k in ("vision", "flip", "gps"):
        ax3.hist([min(v, 1.2) for r in res[k] for v in r["lat"]], bins=bins, histtype="step", lw=1.6, color=cols[k], label=names[k])
    ax3.axvline(1.0, color="r", ls=":", lw=1); ax3.text(1.0, ax3.get_ylim()[1] * 0.95, " 부표 위치", color="r", fontsize=7.5, va="top")
    ax3.set_xlabel("게이트 선 통과 시 중심에서 벗어남 / 반폭"); ax3.set_ylabel("통과 횟수"); ax3.set_title("(c) 통과 위치 (0 = 정중앙, 1 = 부표)", fontsize=10.5); ax3.legend(fontsize=6.8); ax3.grid(alpha=0.2)
    # (d) 표
    ax4 = fig.add_subplot(gs[1, 2]); ax4.axis("off")
    rows = [[names[k], f"{summ[k]['tot']}/{4 * N}", f"{summ[k]['full']}", f"{summ[k]['hits']}", f"{summ[k]['lat_med']:.2f}", f"{summ[k]['t_med']:.0f}"] for k in ("vision", "flip", "gps", "vision_tau")]
    tb = ax4.table(cellText=rows, colLabels=["변형", "통과", "4/4\n런", "접촉", "벗어남\n중앙값", "시간\n[s]"], loc="center", cellLoc="center", colWidths=[0.56, 0.13, 0.08, 0.08, 0.1, 0.09])
    tb.auto_set_font_size(False); tb.set_fontsize(7.0); tb.scale(1.0, 1.8)
    for (ri, ci), cell in tb.get_celld().items():
        if ri == 0: cell.set_facecolor("#e8eef7"); cell.set_text_props(weight="bold")
        if ci == 0: cell.set_text_props(ha="left"); cell.PAD = 0.04
    ax4.set_title(f"(d) {N} 시드 요약", fontsize=10.5)
    fig.suptitle("항로추종 2D 시뮬레이션 — 실제 ship_gate·ship_direction·motor_control 코드 실행, 카메라·LiDAR·추력 응답은 모델 (tools/sim/gate_sim.py)", fontsize=11, y=0.995)
    fig.savefig(OUT_PNG, dpi=160, bbox_inches="tight"); print("저장:", OUT_PNG)


if __name__ == "__main__":
    main()

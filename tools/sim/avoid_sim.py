#!/usr/bin/env python3
"""장애물 회피 2D 시뮬레이션 — 실제 ship_direction·motor_control 노드 코드를 그대로 돌린다.

  · 보고서 Ⅲ-6.1 스윕(tools/sim/ssf_sim.py)은 두 노드의 로직을 **다시 구현**한 것이다. 여기서는
    src/ship_direction/ship_direction/ship_direction.py 와 src/motor_control/motor_control/motor_control.py 를
    rclpy 스텁(rosstub.py)으로 import 해 scan_callback → watchdog_cb → control_cb → motor timer_callback 을
    틱마다 호출한다. 파라미터는 두 노드의 config yaml 에서 읽는다(대회 launch 와 같은 값).
  · 세계·코스·접촉 판정은 스윕과 동일(kaboat_full_course_sim.build_course 의 장애물 구간, FastWorld 부표
    레이캐스팅, 배 = 반경 half_width 원, 부표마다 접촉 1회). 같은 시드·같은 출발로 재구현(K.leg)과 나란히 비교.
  · 실코드에만 있는 것: /geofence_state 병합(north_goal_angle 의 가짜 LiDAR 규약 — 상대방위 좌 양수, LiDAR a−80 과
    동일 — 그대로 경계 사각형에서 생성; 계약서_geofence부호_20261104 수정 후 규약),
    페일세이프 워치독, 변화율 제한, 후진/재탐색 상태기계, motor_control 감속·명령 워치독.
  · 운동 모델은 스윕과 같다(순항 1.2 m/s, k_w 0.22, 지연 없음). 지연 τ 0.5 s 변형도 함께 돌린다.

재현: python3 tools/sim/avoid_sim.py → docs/전달용/배치도/avoid_sim.png, tools/sim/out/avoid_sim.csv
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
import kaboat_full_course_sim as K               # noqa: E402  (코스·접촉 모델·운동 상수)
from fastworld import FastWorld                  # noqa: E402

P_SD = rosstub.load_params(os.path.join(ROOT, "src/ship_direction/config/ship_direction.yaml"), "/ship_direction")
P_MC = rosstub.load_params(os.path.join(ROOT, "src/motor_control/config/motor_control.yaml"), "/motor_control")
SD = rosstub.load_node(os.path.join(ROOT, "src/ship_direction/ship_direction/ship_direction.py"), "ship_direction_real",
                       extra_path=os.path.join(ROOT, "src/ship_direction"))
MC = rosstub.load_node(os.path.join(ROOT, "src/motor_control/motor_control/motor_control.py"), "motor_control_real")
std = sys.modules["std_msgs.msg"]
Float32, Int32, F32MA = std.Float32, std.Int32, std.Float32MultiArray

OUT_PNG = os.path.join(ROOT, "docs", "전달용", "배치도", "avoid_sim.png")
OUT_CSV = os.path.join(HERE, "out", "avoid_sim.csv")
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False

AVOID_MODE = 5                      # 표 Ⅲ-7: 5 = 장애물 회피 (ship_direction 순수 회피)
DT, MAX_T, STOP_R = K.DT, 300.0, 1.5
GF_MIN, GF_MAX, GF_INC, GF_RANGE = -80.0, 80.0, 1.0, 30.0   # north_goal_angle 기본값


class Scan:
    def __init__(self, ranges):
        self.angle_min = math.radians(K.SCAN_ANGLE_MIN_DEG)
        self.angle_increment = math.radians(K.SCAN_INC_DEG)
        self.ranges = ranges


def ray_rect(x, y, bearing_deg, rect):
    """안에서 쏜 광선이 사각형 경계를 만나는 거리(없으면 inf)."""
    x0, y0, x1, y1 = rect
    dx, dy = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
    ts = []
    if dx > 1e-9: ts.append((x1 - x) / dx)
    if dx < -1e-9: ts.append((x0 - x) / dx)
    if dy > 1e-9: ts.append((y1 - y) / dy)
    if dy < -1e-9: ts.append((y0 - y) / dy)
    ts = [t for t in ts if t > 0]
    return min(ts) if ts else math.inf


def geofence_msg(x, y, psi, rect):
    x0, y0, x1, y1 = rect
    if not (x0 <= x <= x1 and y0 <= y <= y1):
        return F32MA([])                               # north: 이미 밖이면 침묵
    n = int(round((GF_MAX - GF_MIN) / GF_INC)) + 1
    r = []
    for k in range(n):
        t = ray_rect(x, y, psi - (GF_MIN + k * GF_INC), rect)      # rel 양수 = 좌현 → 절대 방위 = yaw − rel
        r.append(t if t <= GF_RANGE else math.inf)
    return F32MA([GF_MIN, GF_INC] + r)


def course(seed):
    c = K.build_course(gate_w=3.0, seed=seed)
    buoys = []
    for (a, b) in c["gates"]:
        buoys += [(a[0], a[1], K.BUOY_R), (b[0], b[1], K.BUOY_R)]
    buoys += [(c["wp1"][0], c["wp1"][1], K.BUOY_R), (c["search"][0], c["search"][1], K.BUOY_R)]
    buoys += [(p[0], p[1], K.BUOY_R) for p in c["search_others"]] + c["obst"]
    return c, buoys


def run_real(seed, psi0, geofence=True, tau=0.0):
    c, buoys = course(seed); world = FastWorld(buoys)
    (x, y), goal, fence = c["wp2"], c["goal"], c["fence"]
    psi = psi0; v = w = 0.0
    Clock.t = 0.0
    sd = SD.ShipDirection(); mc = MC.MotorController()
    sd.wp_mode_cb(Int32(AVOID_MODE)); sd.candidate_callback(Float32(SD.CANDIDATE_INVALID))
    traj = [(x, y)]; hits = 0; hit_flag = {}; oob = False; entered = False; ok = False
    min_clear = math.inf; n_rev = 0; prev_rev = False; stop_ticks = 0; fs_max = 0; gf_merge = 0
    t = 0.0
    while t < MAX_T:
        if math.hypot(goal[0] - x, goal[1] - y) < STOP_R:
            ok = True; break
        sd.scan_callback(Scan(world.lidar_scan(x, y, psi, max_range=20.0).tolist()))
        sd.yaw_error_callback(Float32((K.bearing_to(x, y, goal[0], goal[1]) - psi) % 360.0))   # ship_goal_angle 출력
        if geofence:
            sd.geofence_cb(geofence_msg(x, y, psi, fence))
        sd.watchdog_cb(); sd.control_cb()
        desired = sd.pubs["/desired_angle"].last.data
        mc.angle_callback(Float32(desired))
        if sd.pubs["/obstacle_distance_array"].last is not None:
            mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
        mc.failsafe_callback(sd.pubs["/failsafe_level"].last); fs_max = max(fs_max, sd.failsafe_level)
        mc.timer_callback()
        code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
        TR, TL = 1500 - pr, 1500 - pl
        v_cmd, w_cmd = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
        if tau > 0:
            v += (v_cmd - v) * DT / tau; w += (w_cmd - w) * DT / tau
        else:
            v, w = v_cmd, w_cmd
        psi = (psi + w * DT) % 360.0
        x += v * math.sin(math.radians(psi)) * DT; y += v * math.cos(math.radians(psi)) * DT
        t += DT; Clock.t = t; traj.append((x, y))
        is_rev = 161.0 < desired < 5000.0
        if is_rev and not prev_rev: n_rev += 1
        prev_rev = is_rev
        if desired >= 50000: stop_ticks += 1
        for i, (bx, by, br) in enumerate(world.buoys):
            d = math.hypot(x - bx, y - by) - br
            if i >= len(buoys) - len(c["obst"]): min_clear = min(min_clear, d)
            if d < K.BOAT_HALF_W and not hit_flag.get(i):
                hits += 1; hit_flag[i] = True
        x0, y0, x1, y1 = fence
        inside = (x0 <= x <= x1 and y0 <= y <= y1)
        if inside: entered = True
        elif entered and not (x0 - 2 <= x <= x1 + 2 and y0 - 2 <= y <= y1 + 2): oob = True
    for lvl, m in sd.get_logger().lines:
        if "경계가 스캔에 병합" in m: gf_merge += 1
    return dict(traj=np.array(traj), t=t, hits=hits, oob=oob, ok=ok, min_clear=min_clear, n_rev=n_rev,
                stop_ticks=stop_ticks, fs_max=fs_max, gf_merge=gf_merge)


def run_reimpl(seed, psi0):
    c, buoys = course(seed); world = FastWorld(buoys)
    tr, t, h, o, ok, _ = K.leg(world, c["wp2"], psi0, c["goal"], stop_r=STOP_R, dd=P_SD["detection_distance_default"],
                               max_t=MAX_T, fence=c["fence"])
    n_obst = len(c["obst"]); ob = buoys[-n_obst:]
    mc = min((math.hypot(px - bx, py - by) - br) for (px, py) in tr for (bx, by, br) in ob) if len(tr) else math.inf
    return dict(traj=tr, t=t, hits=h, oob=o, ok=ok, min_clear=mc, n_rev=-1, stop_ticks=-1, fs_max=0, gf_merge=0)


VARIANTS = [("실코드 + 경계", dict(geofence=True)), ("실코드 (경계 없음)", dict(geofence=False)),
            ("실코드 + 경계, τ 0.5 s", dict(geofence=True, tau=0.5)), ("재구현(스윕 모델)", None)]


def main():
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    rng = random.Random(2026); N = 40
    starts = []
    for s in range(1, N + 1):
        c, _ = course(s); b = K.bearing_to(*c["wp2"], *c["goal"])
        starts.append((s, (b + rng.uniform(-45, 45)) % 360.0))
    res = {name: [] for name, _ in VARIANTS}
    for name, kw in VARIANTS:
        for s, p0 in starts:
            r = run_real(s, p0, **kw) if kw is not None else run_reimpl(s, p0)
            r.update(seed=s, psi0=p0, variant=name); res[name].append(r)
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(["variant", "seed", "psi0", "ok", "t", "hits", "oob", "min_clear", "n_rev", "stop_ticks", "fs_max", "gf_merge"])
        for name in res:
            for r in res[name]:
                wr.writerow([name, r["seed"], f"{r['psi0']:.1f}", int(r["ok"]), f"{r['t']:.1f}", r["hits"], int(r["oob"]),
                             f"{r['min_clear']:.2f}", r["n_rev"], r["stop_ticks"], r["fs_max"], r["gf_merge"]])
    summ = {}
    print(f"장애물 회피 구간 {N} 시드 × 변형 {len(VARIANTS)} (출발 위치유지2 → 도착, 부표 16개, dd {P_SD['detection_distance_default']} m, half_width {P_SD['half_width']} · clearance {P_SD['clearance']})")
    for name in res:
        R = res[name]; okr = [r for r in R if r["ok"]]
        ts = sorted(r["t"] for r in okr); hits = [r["hits"] for r in R]
        summ[name] = dict(reach=len(okr), hits_mean=sum(hits) / N, hits_zero=sum(1 for h in hits if h == 0), oob=sum(r["oob"] for r in R),
                          t_med=ts[len(ts) // 2] if ts else float("nan"), clear_med=sorted(r["min_clear"] for r in R)[N // 2],
                          rev=sum(max(r["n_rev"], 0) for r in R), fs=max(r["fs_max"] for r in R))
        s = summ[name]
        print(f"  [{name}] 도달 {s['reach']}/{N}, 접촉 평균 {s['hits_mean']:.2f} (무접촉 {s['hits_zero']}), 경계 이탈 {s['oob']}, "
              f"시간 중앙값 {s['t_med']:.0f} s, 최소 여유 중앙값 {s['clear_med']:.2f} m, 후진 {s['rev']}회, 페일세이프 최대 L{s['fs']}")
    a, b = res["실코드 + 경계"], res["재구현(스윕 모델)"]
    same = sum(1 for p, q in zip(a, b) if p["hits"] == q["hits"]); print(f"  실코드(경계)·재구현 접촉 수 일치 시드 {same}/{N}")

    # ── 그림 ──
    fig = plt.figure(figsize=(13.4, 8.6))
    gs = fig.add_gridspec(2, 3, height_ratios=[0.9, 1.0], hspace=0.3, wspace=0.28)
    ax = fig.add_subplot(gs[0, :])
    show = [r for r in res["실코드 + 경계"] if r["seed"] in (1, 2, 3)]
    cols = {1: "#1f77b4", 2: "#2ca02c", 3: "#9467bd"}
    for r in show:
        c, _ = course(r["seed"])
        for (bx, by, br) in c["obst"]:
            ax.add_patch(plt.Circle((bx, by), br, color=cols[r["seed"]], alpha=0.55))
            ax.add_patch(plt.Circle((bx, by), br + P_SD["half_width"] + P_SD["clearance"], color=cols[r["seed"]], alpha=0.06))
        ax.plot(r["traj"][:, 0], r["traj"][:, 1], color=cols[r["seed"]], lw=1.6, label=f"시드 {r['seed']} — 접촉 {r['hits']}, {r['t']:.0f} s")
        q = [x for x in res["재구현(스윕 모델)"] if x["seed"] == r["seed"]][0]
        ax.plot(q["traj"][:, 0], q["traj"][:, 1], color=cols[r["seed"]], lw=1.0, ls="--", alpha=0.8)
    c, _ = course(1); x0, y0, x1, y1 = c["fence"]
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="olive", ls="--", lw=1.2))
    ax.text(x0 + 0.4, y0 + 0.3, "장애물 구간 경계 (geofence)", color="olive", fontsize=8.5)
    ax.plot(*c["wp2"], "o", color="#333", ms=7); ax.text(c["wp2"][0] + 0.5, c["wp2"][1] - 1.2, "출발 (위치유지 2)", fontsize=8.5)
    ax.plot(*c["goal"], "*", color="k", ms=13); ax.text(c["goal"][0], c["goal"][1] + 1.0, "도착", ha="center", fontsize=8.5)
    ax.plot([], [], color="gray", lw=1.6, label="실선 = 실제 노드 코드"); ax.plot([], [], color="gray", lw=1.0, ls="--", label="점선 = 재구현(스윕 모델)")
    ax.set_xlim(1, 49); ax.set_ylim(9.3, 21.2); ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(fontsize=8, loc="lower right", ncol=3, framealpha=0.9)
    ax.set_title("(a) 장애물 회피 구간 (70 × 20 m 코스 중 y 10.5~20, 부표 16개) — 시드 1·2·3, 실제 코드 vs 재구현", fontsize=10.5)
    # (b) 시드별 접촉: 실코드 vs 재구현
    ax2 = fig.add_subplot(gs[1, 0])
    xs = np.arange(1, N + 1)
    ax2.bar(xs - 0.2, [r["hits"] for r in res["실코드 + 경계"]], width=0.4, color="#1f77b4", label="실제 코드 + 경계")
    ax2.bar(xs + 0.2, [r["hits"] for r in res["재구현(스윕 모델)"]], width=0.4, color="#bbbbbb", label="재구현(스윕 모델)")
    ax2.set_xlim(0, N + 1); ax2.set_yticks(range(0, 5))
    for r in res["실코드 + 경계"]:
        if not r["ok"]: ax2.plot(r["seed"] - 0.2, max(r["hits"], 0) + 0.15, "rx", ms=6)
    ax2.set_xlabel("시드"); ax2.set_ylabel("접촉 횟수"); ax2.set_title("(b) 시드별 접촉 횟수 (× = 미도달)", fontsize=10.5); ax2.legend(fontsize=7.5); ax2.grid(alpha=0.2, axis="y")
    # (c) 최소 여유 분포
    ax3 = fig.add_subplot(gs[1, 1])
    bins = np.linspace(-0.3, 2.5, 29)
    for name, col in (("실코드 + 경계", "#1f77b4"), ("재구현(스윕 모델)", "#999999"), ("실코드 + 경계, τ 0.5 s", "#ff7f0e")):
        ax3.hist([min(max(r["min_clear"], -0.3), 2.5) for r in res[name]], bins=bins, histtype="step", lw=1.6, color=col, label=name)
    ax3.axvline(P_SD["half_width"], color="r", ls=":", lw=1); ax3.text(P_SD["half_width"], ax3.get_ylim()[1] * 0.95, " 접촉 기준 (half_width)", color="r", fontsize=7.5, va="top")
    ax3.axvline(P_SD["half_width"] + P_SD["clearance"], color="k", ls=":", lw=1); ax3.text(P_SD["half_width"] + P_SD["clearance"], ax3.get_ylim()[1] * 0.8, " 설계 여유", fontsize=7.5, va="top")
    ax3.set_xlabel("런 중 부표 표면까지 최소 거리 [m]"); ax3.set_ylabel("런 수"); ax3.set_title("(c) 부표 최소 여유 분포", fontsize=10.5); ax3.legend(fontsize=7.5); ax3.grid(alpha=0.2)
    # (d) 요약 표
    ax4 = fig.add_subplot(gs[1, 2]); ax4.axis("off")
    rows = [[n, f"{summ[n]['reach']}/{N}", f"{summ[n]['hits_mean']:.2f}", f"{summ[n]['hits_zero']}", f"{summ[n]['oob']}", f"{summ[n]['t_med']:.0f}"] for n, _ in VARIANTS]
    tb = ax4.table(cellText=rows, colLabels=["변형", "도달", "접촉\n평균", "무접촉\n런", "경계\n이탈", "시간\n중앙값 [s]"], loc="center", cellLoc="center", colWidths=[0.42, 0.12, 0.12, 0.12, 0.11, 0.14])
    tb.auto_set_font_size(False); tb.set_fontsize(7.8); tb.scale(1.0, 1.75)
    for (ri, ci), cell in tb.get_celld().items():
        if ri == 0: cell.set_facecolor("#e8eef7"); cell.set_text_props(weight="bold")
        if ci == 0: cell.set_text_props(ha="left"); cell.PAD = 0.04
    ax4.set_title(f"(d) {N} 시드 요약 (출발 헤딩 = 도착 방위 ±45° 무작위)", fontsize=10.5)
    fig.suptitle("장애물 회피 2D 시뮬레이션 — 실제 ship_direction·motor_control 코드 실행, 코스·접촉 모델은 Ⅲ-6.1 스윕과 동일 (tools/sim/avoid_sim.py)", fontsize=11, y=0.995)
    fig.savefig(OUT_PNG, dpi=160, bbox_inches="tight"); print("저장:", OUT_PNG)


if __name__ == "__main__":
    main()

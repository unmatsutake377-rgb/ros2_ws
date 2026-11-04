#!/usr/bin/env python3
"""미션 노드 5개 — 보고서용 한 판짜리 궤적 그림 (실제 노드 코드 실행).

  python3 tools/sim/mission_figs.py            # 5장 전부 (노드별 별도 프로세스 — 스텁 충돌 방지)
  python3 tools/sim/mission_figs.py turn       # 하나만

  gate  → node_gate.png   ship_gate + ship_direction + motor_control (gate_sim.run)
  back  → node_back.png   ship_back + ship_direction + motor_control, 부표 1개, 조류 0.3 m/s 가정
  search→ node_search.png ship_back(모드 3, hold) + ship_direction + motor_control, 빨강 부표 앞 위치유지 (2026 안내문 기준 탐색)
  turn  → node_turn.png   ship_turn + ship_direction + motor_control, 빨강 부표 1개(시계 선회 — 2025 규정, orbit 옵션)
  avoid → node_avoid.png  ship_direction + motor_control, 장애물 구간 (avoid_sim.run_real)
  dock  → node_dock.png   ship_dock, 도크 벽 + 표식 (dock_sim.run)
운동 모델·카메라·LiDAR 모델은 각 *_sim.py 와 같다(순항 1.2 m/s, HFOV 63.9°, 좌 양수). 전부 가정값 — 경향용.
"""
import math
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "docs", "전달용", "배치도")
sys.path.insert(0, HERE)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False

HFOV, CAM_MIN, CAM_MAX = 63.9, 0.5, 12.0


def wrap(a): return (a + 180.0) % 360.0 - 180.0


def cam_rel(x, y, psi, bx, by):
    """부표 하나의 상대 방위(왼쪽 양수) 또는 None."""
    d = math.hypot(bx - x, by - y)
    rel_cw = wrap(math.degrees(math.atan2(bx - x, by - y)) - psi)
    if CAM_MIN <= d <= CAM_MAX and abs(rel_cw) <= HFOV / 2:
        return -rel_cw
    return None


def finish(fig, ax, name, title):
    ax.set_aspect("equal"); ax.grid(alpha=0.25); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(title, fontsize=11)
    path = os.path.join(OUT, f"node_{name}.png")
    fig.savefig(path, dpi=160, bbox_inches="tight"); print("저장:", path)


# ───────────────────────── gate ─────────────────────────
def fig_gate():
    import gate_sim as G
    r = G.run(1, "vision")
    fig, ax = plt.subplots(figsize=(11, 3.8))
    for g in r["gates"]:
        ax.add_patch(plt.Circle(g["red"], 0.35, color="crimson")); ax.add_patch(plt.Circle(g["green"], 0.35, color="seagreen"))
        ax.plot([g["x"], g["x"]], [g["green"][1], g["red"][1]], color="k", lw=0.8, ls=":")
    ax.plot(r["traj"][:, 0], r["traj"][:, 1], color="#1f77b4", lw=2.0)
    ax.plot(*r["traj"][0], "o", color="k", ms=7); ax.text(r["traj"][0][0], r["traj"][0][1] - 1.3, "출발", ha="center", fontsize=9)
    ax.text(r["gates"][0]["red"][0], r["gates"][0]["red"][1] + 0.8, "빨강 = 좌현", ha="center", fontsize=8.5, color="crimson")
    ax.text(r["gates"][0]["green"][0], r["gates"][0]["green"][1] - 1.0, "초록 = 우현", ha="center", fontsize=8.5, color="seagreen")
    ax.set_xlim(2, 48); ax.set_ylim(-1.5, 8.5)
    finish(fig, ax, "gate", f"항로추종 (ship_gate) — 게이트 4개 중 {r['passed']}개 통과, 부표 접촉 {r['hits']}회, {r['t']:.0f} s, /gates_passed = {r['node_count']}")


# ───────────────────────── avoid ─────────────────────────
def fig_avoid():
    import avoid_sim as A
    c, buoys = A.course(1)
    b = A.K.bearing_to(*c["wp2"], *c["goal"])
    r = A.run_real(1, b, geofence=True)
    fig, ax = plt.subplots(figsize=(11, 3.8))
    for (bx, by, br) in c["obst"]:
        ax.add_patch(plt.Circle((bx, by), br, color="#ff7f0e", alpha=0.9))
        ax.add_patch(plt.Circle((bx, by), br + A.P_SD["half_width"] + A.P_SD["clearance"], color="#ff7f0e", alpha=0.08))
    x0, y0, x1, y1 = c["fence"]
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="olive", ls="--", lw=1.2))
    ax.text(x0 + 0.4, y0 + 0.3, "장애물 구간 경계", color="olive", fontsize=8.5)
    ax.plot(r["traj"][:, 0], r["traj"][:, 1], color="#1f77b4", lw=2.0)
    ax.plot(*c["wp2"], "o", color="k", ms=7); ax.text(c["wp2"][0] + 0.5, c["wp2"][1] - 1.3, "출발", fontsize=9)
    ax.plot(*c["goal"], "*", color="k", ms=13); ax.text(c["goal"][0], c["goal"][1] + 1.0, "도착", ha="center", fontsize=9)
    ax.set_xlim(1, 49); ax.set_ylim(9.3, 21.2)
    finish(fig, ax, "avoid", f"장애물 회피 (ship_direction) — 부표 16개, 접촉 {r['hits']}회, 경계 이탈 {'없음' if not r['oob'] else '있음'}, {r['t']:.0f} s")


# ───────────────────────── dock ─────────────────────────
def fig_dock():
    import dock_sim as D
    log, o = D.run(2.0, 6.0, 300.0, seed=3)
    fig, ax = plt.subplots(figsize=(7.5, 6.2))
    for (a, b) in D.SEGS: ax.plot([a[0], b[0]], [a[1], b[1]], color="#333", lw=3)
    ax.plot(*D.MARKER, marker="s", ms=11, color="#d62728"); ax.text(0, -0.75, "목표 표식 (가운데 칸)", ha="center", fontsize=9)
    xs, ys, ss = log["x"], log["y"], log["state"]
    for i in range(1, len(xs)):
        ax.plot(xs[i - 1:i + 1], ys[i - 1:i + 1], color=D.COL[ss[i]], lw=2.2, solid_capstyle="round")
    for st in ("SEARCH", "ALIGN"):
        pts = [(xs[i], ys[i]) for i in range(len(xs)) if ss[i] == st]
        if pts: ax.plot(*zip(*pts), ls="none", marker="o", ms=9 if st == "SEARCH" else 4, color=D.COL[st], alpha=0.9, zorder=5)
    ax.plot(xs[0], ys[0], "o", color="k", ms=7, zorder=4); ax.text(xs[0] + 0.3, ys[0] + 0.3, "출발 (표식을 등지고 → 제자리 탐색 선회)", fontsize=9)
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([0], [0], color=D.COL[s], lw=3, label={"INIT": "INIT 정지", "SEARCH": "SEARCH 탐색 선회", "ALIGN": "ALIGN 정면 정렬", "APPROACH": "APPROACH 접근", "CONTACT": "CONTACT 접안", "BACKOFF": "BACKOFF 후진", "DONE": "DONE"}[s]) for s in D.STATES], fontsize=8, loc="lower left")
    ax.set_xlim(-5.5, 5.5); ax.set_ylim(-1.6, 8.5)
    finish(fig, ax, "dock", f"도킹 (ship_dock) — 접안 {'성공' if o['success'] else '실패'} {o['t_contact']:.0f} s, 접안 시 헤딩 오차 {o['head_err']:.0f}°, 후진 이탈 완료" if o["success"] else "도킹 (ship_dock) — 접안 실패")


# ───────────────────────── back / turn (공용 루프) ─────────────────────────
def _single_buoy_run(node_kind, t_max=60.0, current=(0.0, 0.0), color="red", mode_override=None, param_override=None):
    import avoid_sim as A                      # rosstub 설치 + 실제 ship_direction·motor_control 로드
    from rosstub import Clock
    import rosstub
    from fastworld import FastWorld
    std = sys.modules["std_msgs.msg"]
    Float32, Float64, Int32, String = std.Float32, std.Float64, std.Int32, std.String
    K = A.K
    buoy = (0.0, 0.0); world = FastWorld([(0.0, 0.0, K.BUOY_R)])
    if node_kind == "back":
        P = rosstub.load_params(os.path.join(ROOT, "src/ship_back/config/ship_back.yaml"), "/ship_back")
        if param_override: rosstub.PARAMS["ship_back"] = dict(P, **param_override)
        M = rosstub.load_node(os.path.join(ROOT, "src/ship_back/ship_back/ship_back.py"), "ship_back_real", extra_path=os.path.join(ROOT, "src/ship_back"))
        node = M.ShipBack(); mode = mode_override if mode_override is not None else P["active_wp_modes"][0]
    else:
        P = rosstub.load_params(os.path.join(ROOT, "src/ship_turn/config/ship_turn.yaml"), "/ship_turn")
        if param_override: rosstub.PARAMS["ship_turn"] = dict(P, **param_override)
        M = rosstub.load_node(os.path.join(ROOT, "src/ship_turn/ship_turn/ship_turn.py"), "ship_turn_real", extra_path=os.path.join(ROOT, "src/ship_turn"))
        node = M.ShipTurn(); mode = P["active_wp_mode"]
    sd = A.SD.ShipDirection(); mc = A.MC.MotorController()
    Clock.t = 0.0
    sd.wp_mode_cb(Int32(mode)); node.wp_cb(Int32(mode))
    x, y, psi = 0.0, -8.0, 0.0            # 남쪽 8 m 에서 북쪽(부표) 을 향해
    traj = [(x, y)]; states = [node.state]; t = 0.0; v = w = 0.0; dist_log = []
    done_t = None; rng = random.Random(1)
    while t < t_max:
        scan = A.Scan(world.lidar_scan(x, y, psi, max_range=20.0).tolist())
        sd.scan_callback(scan); node.scan_cb(scan)
        rel = cam_rel(x, y, psi, *buoy)
        node.angle_cb(Float32(10000.0 if rel is None else rel + rng.gauss(0, 0.5)))
        if node_kind == "turn":
            node.color_cb(String(color)); node.yaw_cb(Float64(psi))
        sd.yaw_error_callback(Float32((K.bearing_to(x, y, *buoy) - psi) % 360.0))   # GPS 목표 = 부표
        node.timer_cb()
        cand = node.pubs["/candidate_angle"].last.data if node.pubs["/candidate_angle"].last is not None else A.SD.CANDIDATE_INVALID
        sd.candidate_callback(Float32(cand)); sd.watchdog_cb(); sd.control_cb()
        desired = sd.pubs["/desired_angle"].last.data
        mc.angle_callback(Float32(desired))
        if sd.pubs["/obstacle_distance_array"].last is not None: mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
        mc.failsafe_callback(sd.pubs["/failsafe_level"].last); mc.timer_callback()
        code = mc.pubs["Motor_run"].last.data; pr, pl = code // 10000, code % 10000
        TR, TL = 1500 - pr, 1500 - pl
        v, w = K.K_V * (TR + TL) / 2.0, K.K_W * (TL - TR)
        psi = (psi + w * K.DT) % 360.0
        x += (v * math.sin(math.radians(psi)) + current[0]) * K.DT
        y += (v * math.cos(math.radians(psi)) + current[1]) * K.DT
        t += K.DT; Clock.t = t
        traj.append((x, y)); states.append(node.state); dist_log.append((t, math.hypot(x - buoy[0], y - buoy[1])))
        if node.state == "DONE" and done_t is None: done_t = t
        if node_kind == "turn" and node.state == "COOLDOWN": done_t = done_t or t; break
        if node_kind == "back" and node.state == "DONE": break
    return dict(traj=traj, states=states, dist=dist_log, done_t=done_t, node=node, P=P, t=t)


def fig_back():
    import numpy as np
    r = _single_buoy_run("back", t_max=60.0, current=(0.3, 0.0), color="white")   # 조류 0.3 m/s 동쪽으로
    tr = np.array(r["traj"]); P = r["P"]
    fig, ax = plt.subplots(figsize=(7.5, 6.2))
    ax.add_patch(plt.Circle((0, 0), P["keep_radius_m"], color="gold", alpha=0.15)); ax.text(0, P["keep_radius_m"] + 0.3, "규정: 5 m 이내 5 s", ha="center", fontsize=8.5, color="#8a6d00")
    ax.add_patch(plt.Circle((0, 0), P["hold_dist_m"] + P["dist_tol_m"], fill=False, ec="#1f77b4", ls=":", lw=1))
    ax.add_patch(plt.Circle((0, 0), max(0.05, P["hold_dist_m"] - P["dist_tol_m"]), fill=False, ec="#1f77b4", ls=":", lw=1))
    ax.text(0, -(P["hold_dist_m"] + P["dist_tol_m"]) - 0.5, f"유지 목표 {P['hold_dist_m']} ± {P['dist_tol_m']} m", ha="center", fontsize=8.5, color="#1f77b4")
    ax.add_patch(plt.Circle((0, 0), 0.3, color="white", ec="k", lw=1.5)); ax.text(0.5, 0.2, "흰 부표", fontsize=9)
    ax.plot(tr[:, 0], tr[:, 1], color="#1f77b4", lw=2.0)
    ax.plot(tr[0, 0], tr[0, 1], "o", color="k", ms=7); ax.text(tr[0, 0] + 0.3, tr[0, 1], "출발 (8 m 남쪽)", fontsize=9)
    ax.plot(tr[-1, 0], tr[-1, 1], "s", color="#d62728", ms=7); ax.text(tr[-1, 0] + 0.3, tr[-1, 1] - 0.4, f"완료 t = {r['done_t']:.0f} s" if r["done_t"] else "미완료", fontsize=9, color="#d62728")
    ax.annotate("", xy=(4.5, -7.0), xytext=(2.5, -7.0), arrowprops=dict(arrowstyle="->", lw=1.5, color="#555")); ax.text(3.5, -6.6, "조류 0.3 m/s (가정)", ha="center", fontsize=8.5, color="#555")
    ax.set_xlim(-6, 6); ax.set_ylim(-9, 6.5)
    held = [d for (t, d) in r["dist"] if r["done_t"] and r["done_t"] - 5.0 <= t <= r["done_t"]]
    rng = f"{min(held):.1f}~{max(held):.1f} m" if held else "—"
    finish(fig, ax, "back", f"위치유지 (ship_back) — LiDAR 거리로 전·후진 반복, 마지막 5 s 거리 {rng}, {'규정 달성' if r['done_t'] else '미달성'} {r['done_t'] or r['t']:.0f} s")
    return r


def fig_turn():
    import numpy as np
    r = _single_buoy_run("turn", t_max=90.0, color="red", param_override={"enabled": True})   # orbit 옵션(launch search_behavior:=orbit 이 True 를 넘김)
    tr = np.array(r["traj"]); P = r["P"]
    # 실제 회전 방향: 부표 기준 방위각의 부호 있는 누적
    ang = np.degrees(np.arctan2(tr[:, 0], tr[:, 1])); d = ((np.diff(ang) + 180) % 360) - 180
    turned = float(np.sum(d[np.hypot(tr[1:, 0], tr[1:, 1]) < P["approach_dist_m"] + 1.0]))
    dirn = "시계" if turned > 0 else "반시계"
    rad = np.hypot(tr[:, 0], tr[:, 1]); orbit_r = rad[(rad < P["approach_dist_m"] + 1.0)]
    fig, ax = plt.subplots(figsize=(7.5, 6.6))
    ax.add_patch(plt.Circle((0, 0), P["orbit_radius_m"], fill=False, ec="#2ca02c", ls=":", lw=1.2)); ax.text(0, P["orbit_radius_m"] + 0.3, f"목표 반경 {P['orbit_radius_m']} m", ha="center", fontsize=8.5, color="#2ca02c")
    ax.add_patch(plt.Circle((0, 0), 0.3, color="crimson")); ax.text(0.5, 0.2, "빨강 부표 (규정: 시계방향)", fontsize=9)
    ax.plot(tr[:, 0], tr[:, 1], color="#2ca02c", lw=2.0)
    n = len(tr); 
    for k in range(n // 6, n - 2, max(1, n // 6)):
        ax.annotate("", xy=tr[k + 1], xytext=tr[k], arrowprops=dict(arrowstyle="-|>", lw=1.2, color="#2ca02c", mutation_scale=14))
    ax.plot(tr[0, 0], tr[0, 1], "o", color="k", ms=7); ax.text(tr[0, 0] + 0.3, tr[0, 1], "출발 (8 m 남쪽)", fontsize=9)
    ax.set_xlim(-6.5, 6.5); ax.set_ylim(-9, 6.5)
    finish(fig, ax, "turn", f"탐색 선회 옵션 (ship_turn, search_behavior:=orbit) — 실제 {dirn}방향 {abs(turned):.0f}° 선회, 반경 {orbit_r.mean():.1f} m (목표 {P['orbit_radius_m']}), {r['done_t'] or r['t']:.0f} s")
    print(f"  turn: 노드 dir_cw={r['node'].dir_cw}, 궤적 실제 회전 {dirn} {abs(turned):.0f}°, 상태 {r['node'].state}, 진행각(노드) {r['node'].progress.progress_deg():.0f}°")
    return r


def fig_search():
    """탐색 = 2026 안내문: 특정 색 부표 탐지 후 그 앞 위치유지. launch 기본 search_behavior=hold → ship_back 이 모드 3 담당([2,3])."""
    import numpy as np
    r = _single_buoy_run("back", t_max=60.0, current=(0.2, 0.0), color="red", mode_override=3)   # yaml 기본 [2,3] (hold)
    tr = np.array(r["traj"]); P = r["P"]
    fig, ax = plt.subplots(figsize=(7.5, 6.2))
    ax.add_patch(plt.Circle((0, 0), P["keep_radius_m"], color="gold", alpha=0.15)); ax.text(0, P["keep_radius_m"] + 0.3, "5 m 이내 5 s 유지", ha="center", fontsize=8.5, color="#8a6d00")
    ax.add_patch(plt.Circle((0, 0), P["hold_dist_m"] + P["dist_tol_m"], fill=False, ec="#1f77b4", ls=":", lw=1))
    ax.add_patch(plt.Circle((0, 0), max(0.05, P["hold_dist_m"] - P["dist_tol_m"]), fill=False, ec="#1f77b4", ls=":", lw=1))
    ax.text(0, -(P["hold_dist_m"] + P["dist_tol_m"]) - 0.5, f"유지 목표 {P['hold_dist_m']} ± {P['dist_tol_m']} m (LiDAR 거리)", ha="center", fontsize=8.5, color="#1f77b4")
    ax.add_patch(plt.Circle((0, 0), 0.3, color="crimson")); ax.text(0.5, 0.2, "탐색 대상 빨강 부표", fontsize=9)
    ax.plot(tr[:, 0], tr[:, 1], color="#1f77b4", lw=2.0)
    ax.plot(tr[0, 0], tr[0, 1], "o", color="k", ms=7); ax.text(tr[0, 0] + 0.3, tr[0, 1], "출발 (8 m 남쪽, 모드 3)", fontsize=9)
    ax.plot(tr[-1, 0], tr[-1, 1], "s", color="#d62728", ms=7); ax.text(tr[-1, 0] + 0.3, tr[-1, 1] - 0.4, f"완료 t = {r['done_t']:.0f} s" if r["done_t"] else "미완료", fontsize=9, color="#d62728")
    ax.annotate("", xy=(4.5, -7.0), xytext=(2.5, -7.0), arrowprops=dict(arrowstyle="->", lw=1.5, color="#555")); ax.text(3.5, -6.6, "조류 0.2 m/s (가정)", ha="center", fontsize=8.5, color="#555")
    ax.set_xlim(-6, 6); ax.set_ylim(-9, 6.5)
    finish(fig, ax, "search", f"탐색 (ship_back, search_behavior=hold) — 빨강 부표 탐지 후 그 앞 위치유지, {'규정 달성' if r['done_t'] else '미달성'} {r['done_t'] or r['t']:.0f} s, 상태 {r['node'].state}")
    return r


FIGS = {"gate": fig_gate, "back": fig_back, "search": fig_search, "turn": fig_turn, "avoid": fig_avoid, "dock": fig_dock}

if __name__ == "__main__":
    names = sys.argv[1:] or list(FIGS)
    if len(names) > 1:
        for n in names:
            subprocess.run([sys.executable, __file__, n], check=False)
    else:
        FIGS[names[0]]()

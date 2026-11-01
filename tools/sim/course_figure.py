#!/usr/bin/env python3
"""전 코스 시뮬레이션 궤적 — 보고서 Ⅲ-6.1 그림. kaboat_full_course_sim.py 의 코스·구간 주행 함수를 그대로 쓰고
임무 구간별 색으로 그린다. 노드 로직(ship_direction·motor_control) 재현 + 가정 동역학(순항 1.2 m/s) 이라
물 위 결과가 아니다.

설정: 게이트 간격 4.0 m(규정 최소), 탐지거리 항로추종 2.0 m · 그 외 3.0 m (ship_direction.yaml
      detection_distance_gate / _default), half_width 0.29 · clearance 0.25.
재현: python3 tools/sim/course_figure.py  → docs/전달용/배치도/sim_course.png
"""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import kaboat_full_course_sim as K   # noqa: E402

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(HERE, "..", "..", "docs", "전달용", "배치도", "sim_course.png")
GATE_W, DD_GATE, DD = 4.0, 2.0, 3.0

c = K.build_course(gate_w=GATE_W)
buoys = []
for (a, b) in c["gates"]:
    buoys += [(a[0], a[1], K.BUOY_R), (b[0], b[1], K.BUOY_R)]
buoys += [(c["wp1"][0], c["wp1"][1], K.BUOY_R), (c["search"][0], c["search"][1], K.BUOY_R)]
buoys += [(p[0], p[1], K.BUOY_R) for p in c["search_others"]] + c["obst"]
w = K.World(buoys)

segs = []   # (임무, 궤적, 시간, 접촉)
pos, psi = c["start"], 90.0
def run(name, goal, **kw):
    global pos, psi
    tr, t, h, o, ok, psi = K.leg(w, pos, psi, goal, **kw)
    segs.append((name, tr, t, h, ok))
    if len(tr): pos = tuple(tr[-1])
for (a, b) in c["gates"]:
    run("항로추종", ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), stop_r=1.0, dd=DD_GATE, max_t=120)
run("위치유지", c["wp1"], stop_r=K.HOLD_RADIUS - 1, dd=DD, max_t=120)
run("도킹 접근", c["docks"][1], stop_r=1.5, dd=DD, max_t=180)
run("탐색", c["search"], stop_r=3.0, dd=DD, max_t=180)
run("위치유지", c["wp2"], stop_r=K.HOLD_RADIUS - 1, dd=DD, max_t=180)
run("장애물 회피", c["goal"], stop_r=1.5, dd=DD, max_t=300, fence=c["fence"])

COL = {"항로추종": "#d62728", "위치유지": "#b8860b", "도킹 접근": "#1f3a93", "탐색": "#2ca02c", "장애물 회피": "#ff7f0e"}
fig, ax = plt.subplots(figsize=(15, 5.4))
ax.add_patch(plt.Rectangle((0, 0), K.FIELD_L, K.FIELD_W, fill=False, ec="k", lw=1.5))
for (a, b) in c["gates"]:
    ax.add_patch(plt.Circle(a, 0.5, color="red")); ax.add_patch(plt.Circle(b, 0.5, color="green"))
for wp, lbl in ((c["wp1"], "위치유지 1"), (c["wp2"], "위치유지 2")):
    ax.add_patch(plt.Circle(wp, K.HOLD_RADIUS, color="gold", alpha=0.15)); ax.add_patch(plt.Circle(wp, 0.5, color="gold"))
    ax.text(wp[0] + 1.0, wp[1] - 1.6, lbl, ha="left", fontsize=9)
for d in c["docks"]:
    ax.add_patch(plt.Rectangle((d[0] - .8, d[1] - .8), 1.6, 1.6, color="navy", alpha=.6))
ax.text(c["docks"][1][0] + 2.0, c["docks"][1][1] + 3.2, "도크 3개", ha="center", fontsize=9)
ax.add_patch(plt.Circle(c["search"], 0.5, color="limegreen")); ax.add_patch(plt.Circle(c["search"], 3.0, fill=False, ls="--", ec="navy"))
ax.text(c["search"][0], c["search"][1] + 3.6, "탐색 부표", ha="center", fontsize=9)
for p in c["search_others"]:
    ax.add_patch(plt.Circle(p, 0.5, color="white", ec="gray"))
for (bx, by, br) in c["obst"]:
    ax.add_patch(plt.Circle((bx, by), 0.5, color="orange", alpha=0.85))
x0, y0, x1, y1 = c["fence"]
ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="olive", ls="--", lw=1.2))
ax.text(x0 + 0.5, y0 + 0.5, "장애물 구간 경계", ha="left", color="olive", fontsize=8.5)
seen = set()
for name, tr, t, h, ok in segs:
    if len(tr):
        ax.plot(tr[:, 0], tr[:, 1], color=COL[name], lw=2.0, label=None if name in seen else name); seen.add(name)
ax.plot(*c["start"], "o", color="#333", ms=9); ax.text(c["start"][0], c["start"][1] - 1.7, "출발", ha="center", fontsize=9)
ax.plot(*c["goal"], "*", color="k", ms=15); ax.text(c["goal"][0], c["goal"][1] + 1.4, "도착", ha="center", fontsize=9)
tot = {n: (sum(s[2] for s in segs if s[0] == n), sum(s[3] for s in segs if s[0] == n)) for n in COL}
T = sum(v[0] for v in tot.values()); Hh = sum(v[1] for v in tot.values())
ax.set_xlim(-2, K.FIELD_L + 2); ax.set_ylim(-3, K.FIELD_W + 2.5); ax.set_aspect("equal"); ax.grid(alpha=.2)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.legend(loc="lower right", fontsize=9, ncol=5)
ax.set_title(f"전 코스 시뮬레이션 (70 × 20 m) — 구간 이동 {T:.0f} s (위치유지 대기·탐색 선회·접안 제외), 부표 접촉 {Hh}회  ·  게이트 간격 {GATE_W:.0f} m, 탐지거리 {DD_GATE:.0f} / {DD:.0f} m", fontsize=11)
fig.tight_layout(); fig.savefig(OUT, dpi=130)
print(OUT); [print(f"  {n}: {v[0]:.1f}s, 접촉 {v[1]}") for n, v in tot.items()]
print("  전 구간 도달:", all(s[4] for s in segs))

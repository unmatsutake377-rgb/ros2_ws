#!/usr/bin/env python3
"""장애물 회피 장면 — 보고서 Ⅲ-4.4 그림. 전 코스 시뮬레이션의 장애물 구간을 한 스텝씩 다시 돌리며
LiDAR 광선·막힌 구간·통과 가능한 틈·고른 방향을 기록하고, 회피가 일어난 순간 3개를 확대한다.

판단은 ssf_sim.ship_direction_step(노드 ship_direction 재현)을 그대로 호출하고, 그림용으로 같은 단계
(이진화 → 스파이크 제거 → 가장자리 정리 → 팽창 → 폭 판정)를 한 번 더 풀어 틈 구간을 얻는다.
두 결과(고른 각도)가 같은지 매 스텝 확인한다. 동역학은 가정값(순항 1.2 m/s).
재현: python3 tools/sim/avoid_figure.py  → docs/전달용/배치도/sim_avoid.png
"""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Polygon

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import kaboat_full_course_sim as K   # noqa: E402
import ssf_sim as S                  # noqa: E402

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(HERE, "..", "..", "docs", "전달용", "배치도", "sim_avoid.png")
GATE_W, DD_GATE, DD = 4.0, 2.0, 3.0


def zones(ranges, dd):
    """ship_direction_step 과 같은 단계로 통과 가능한 틈(스캔각 구간)과 막힌 셀을 낸다."""
    lo = int((0 - S.SCAN_ANGLE_MIN_DEG) / S.SCAN_INC_DEG); hi = int((160 - S.SCAN_ANGLE_MIN_DEG) / S.SCAN_INC_DEG)
    dist = list(ranges[lo:hi + 1]); ang = [S.SCAN_ANGLE_MIN_DEG + (lo + i) * S.SCAN_INC_DEG for i in range(len(dist))]
    b = [0 if (math.isinf(r) or math.isnan(r)) else int(r < dd) for r in dist]
    b = S.dilate_obstacles(S.suppress_spike_edges(S.smooth_spikes(b)), dist, S.SCAN_INC_DEG, dd)
    zs, st = [], None
    for i, v in enumerate(b):
        if v == 0 and st is None: st = i
        elif v == 1 and st is not None: zs.append((st, i - 1)); st = None
    if st is not None: zs.append((st, len(b) - 1))
    ok = []
    for s, e in zs:
        rs = dist[s] if np.isfinite(dist[s]) else dd; re_ = dist[e] if np.isfinite(dist[e]) else dd
        if math.radians(S.SCAN_INC_DEG) * min(rs, re_) * (e - s) >= S.HALF_WIDTH * 2 + S.CLEARANCE:
            ok.append((ang[s], ang[e]))
    return ok, [ang[i] for i, v in enumerate(b) if v]


# 앞 구간은 course_figure 와 같은 순서로 지나가 장애물 구간 시작 자세를 얻는다
c = K.build_course(gate_w=GATE_W)
buoys = []
for (a, b_) in c["gates"]:
    buoys += [(a[0], a[1], K.BUOY_R), (b_[0], b_[1], K.BUOY_R)]
buoys += [(c["wp1"][0], c["wp1"][1], K.BUOY_R), (c["search"][0], c["search"][1], K.BUOY_R)]
buoys += [(p[0], p[1], K.BUOY_R) for p in c["search_others"]] + c["obst"]
w = K.World(buoys)
pos, psi = c["start"], 90.0
for goal, kw in [(((a[0] + b_[0]) / 2, (a[1] + b_[1]) / 2), dict(stop_r=1.0, dd=DD_GATE, max_t=120)) for (a, b_) in c["gates"]] + [
        (c["wp1"], dict(stop_r=K.HOLD_RADIUS - 1, dd=DD, max_t=120)), (c["docks"][1], dict(stop_r=1.5, dd=DD, max_t=180)),
        (c["search"], dict(stop_r=3.0, dd=DD, max_t=180)), (c["wp2"], dict(stop_r=K.HOLD_RADIUS - 1, dd=DD, max_t=180))]:
    tr, t, h, o, ok, psi = K.leg(w, pos, psi, goal, **kw); pos = tuple(tr[-1])

# 장애물 구간: K.leg 와 같은 갱신식, 매 스텝 상태 기록
x, y = pos; goal = c["goal"]; rec = []; t = 0.0
while t < 300 and math.hypot(goal[0] - x, goal[1] - y) >= 1.5:
    rng = w.lidar_scan(x, y, psi, max_range=20.0)
    e = S.yaw_error_chain(S.bearing_to(x, y, *goal), psi)
    des, ym = S.ship_direction_step(list(rng), S.SCAN_ANGLE_MIN_DEG, S.SCAN_INC_DEG, e, detection_distance=DD)
    zs, blocked = zones(rng, DD)
    if -1 <= des <= 161:   # 노드 재현 함수가 고른 방향이 그림용으로 다시 푼 틈 안에 있어야 한다
        assert any(s0 - 0.6 <= des <= s1 + 0.6 for s0, s1 in zs), (t, des, zs)
    rec.append(dict(x=x, y=y, psi=psi, rng=rng, des=des, ym=ym, zs=zs, blocked=blocked))
    pr, pl, _ = S.motor_control_step(des)
    TR, TL = S.PWM_NEUTRAL - pr, S.PWM_NEUTRAL - pl
    v = K.K_V * (TR + TL) / 2.0; psi = (psi + K.K_W * (TL - TR) * K.DT) % 360.0
    x += v * math.sin(math.radians(psi)) * K.DT; y += v * math.cos(math.radians(psi)) * K.DT; t += K.DT
traj = np.array([(r["x"], r["y"]) for r in rec])

# 회피가 일어난 순간: 막힌 셀이 있고 고른 방향이 목표 방향과 5° 이상 다른 스텝 중 x 간격 4 m 이상으로 3개
cand = [i for i, r in enumerate(rec) if r["blocked"] and abs(r["des"] - r["ym"]) >= 5 and -1 <= r["des"] <= 161]
pick = []
for i in cand:
    if all(abs(rec[i]["x"] - rec[j]["x"]) >= 4 for j in pick): pick.append(i)
    if len(pick) == 2: break
# 비교용: 부표가 탐지거리 안에 보이지만 목표 방향이 틈 안이라 그대로 가는 순간
keep = [i for i, r in enumerate(rec) if r["blocked"] and abs(r["des"] - r["ym"]) < 1.0 and all(abs(r["x"] - rec[j]["x"]) >= 4 for j in pick)]
if keep: pick.append(keep[len(keep) // 2])
pick.sort(key=lambda i: -rec[i]["x"])

def world_bearing(r, scan_deg):   # 스캔각(80 = 정면) → 나침반 방위
    return (r["psi"] - (scan_deg - 80.0)) % 360.0

def ray_xy(r, scan_deg, d):
    b = math.radians(world_bearing(r, scan_deg)); return r["x"] + d * math.sin(b), r["y"] + d * math.cos(b)

fig = plt.figure(figsize=(15, 8.6))
gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.25])
ax0 = fig.add_subplot(gs[0, :])
for (bx, by, br) in c["obst"]:
    ax0.add_patch(plt.Circle((bx, by), 0.45, color="orange", alpha=0.9))
x0, y0, x1, y1 = c["fence"]
ax0.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="olive", ls="--", lw=1.2))
ax0.plot(traj[:, 0], traj[:, 1], color="#ff7f0e", lw=2)
for k, i in enumerate(pick):
    ax0.plot(rec[i]["x"], rec[i]["y"], "o", color="#1f3a93", ms=8); ax0.text(rec[i]["x"], rec[i]["y"] + 0.9, "(" + "abc"[k] + ")", ha="center", fontsize=10, color="#1f3a93")
ax0.plot(*goal, "*", color="k", ms=14); ax0.text(goal[0], goal[1] + 1.0, "도착", ha="center", fontsize=9)
ax0.set_xlim(x0 - 1, x1 + 1); ax0.set_ylim(y0 - 0.5, y1 + 0.5); ax0.set_aspect("equal"); ax0.grid(alpha=.2)
ax0.set_title("장애물 구간 전체 궤적 (오른쪽 → 왼쪽으로 진행) — 파란 점 = 아래 확대 장면", fontsize=11)

for k, i in enumerate(pick):
    r = rec[i]; ax = fig.add_subplot(gs[1, k])
    for (bx, by, br) in c["obst"]:
        ax.add_patch(plt.Circle((bx, by), br, color="orange"))
    ax.add_patch(Wedge((r["x"], r["y"]), DD, 90 - world_bearing(r, 0), 90 - world_bearing(r, 160), fc="#cccccc", alpha=0.25, ec="none"))   # 스캔각은 반시계(+)라 0→160 이 반시계 순서
    for s0, s1 in r["zs"]:   # 통과 가능한 틈
        ax.add_patch(Wedge((r["x"], r["y"]), DD, 90 - world_bearing(r, s0), 90 - world_bearing(r, s1), fc="#2ca02c", alpha=0.28, ec="none"))
    for a in r["blocked"][::2]:   # 막힌 셀(팽창 포함)
        ax.plot(*zip((r["x"], r["y"]), ray_xy(r, a, DD)), color="#d62728", lw=0.6, alpha=0.35)
    hits = [(a, d) for a, d in zip(S.SCAN_ANGLE_MIN_DEG + np.arange(S.SCAN_N) * S.SCAN_INC_DEG, r["rng"]) if 0 <= a <= 160 and np.isfinite(d) and d < 8]
    if hits:
        hp = np.array([ray_xy(r, a, d) for a, d in hits]); ax.plot(hp[:, 0], hp[:, 1], ".", color="#d62728", ms=3)
    ax.annotate("", xy=ray_xy(r, r["ym"], 2.6), xytext=(r["x"], r["y"]), arrowprops=dict(arrowstyle="->", color="#555", lw=1.5, ls="--"))
    ax.annotate("", xy=ray_xy(r, r["des"], 2.6), xytext=(r["x"], r["y"]), arrowprops=dict(arrowstyle="-|>", color="#1f7a1f", lw=2.2))
    hd = math.radians(r["psi"]); L = 0.5
    ax.add_patch(Polygon([(r["x"] + L * math.sin(hd), r["y"] + L * math.cos(hd)),
                          (r["x"] + 0.29 * math.cos(hd) - 0.4 * math.sin(hd), r["y"] - 0.29 * math.sin(hd) - 0.4 * math.cos(hd)),
                          (r["x"] - 0.29 * math.cos(hd) - 0.4 * math.sin(hd), r["y"] + 0.29 * math.sin(hd) - 0.4 * math.cos(hd))], fc="#1f3a93"))
    seg = traj[max(0, i - 120):i + 1]; ax.plot(seg[:, 0], seg[:, 1], color="#ff7f0e", lw=1.2, alpha=0.7)
    ax.set_xlim(r["x"] - 4, r["x"] + 4); ax.set_ylim(r["y"] - 3.2, r["y"] + 3.2); ax.set_aspect("equal"); ax.grid(alpha=.2)
    d = r['des'] - r['ym']
    ax.set_title(f"({'abc'[k]}) " + (f"목표 방향 대비 {d:+.0f}° 틀어 틈으로" if abs(d) >= 1 else "부표가 보여도 목표 방향이 틈 안 → 그대로"), fontsize=10.5)
handles = [plt.Line2D([], [], color="#555", ls="--", label="목표(다음 지점) 방향"), plt.Line2D([], [], color="#1f7a1f", lw=2.2, label="고른 진행 방향"),
           plt.Rectangle((0, 0), 1, 1, fc="#2ca02c", alpha=0.3, label=f"통과 가능한 틈 (탐지거리 {DD:.0f} m 안)"),
           plt.Line2D([], [], color="#d62728", alpha=0.5, label="막힌 방향(부표 + 배 폭 팽창)"), plt.Line2D([], [], color="#d62728", marker=".", ls="", label="LiDAR 반사점")]
fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=9.5, frameon=False)
fig.tight_layout(rect=(0, 0.04, 1, 1)); fig.savefig(OUT, dpi=125)
print(OUT, "스텝", len(rec), "선택", pick, [round(rec[i]["des"] - rec[i]["ym"], 1) for i in pick])

#!/usr/bin/env python3
"""그림 Ⅲ-3 갭 선택 개념도 — ship_direction._compute 의 회피 파이프라인을 같은 순서·같은 파라미터로 재현.

    python3 tools/gap_selection_figure.py                          → 가상 장면 → docs/전달용/배치도/gap_selection.png
    python3 tools/gap_selection_figure.py --bag <rosbag 폴더> --t 12.3  → 실제 /scan 한 프레임 (Ubuntu, rosbag2_py 필요)
        → docs/전달용/배치도/gap_selection_bag.png. 같은 시각의 /desired_angle 기록과 계산값을 나란히 적는다.
        녹화 토픽: /scan /wp_mode /yaw_error /candidate_angle /desired_angle /geofence_state (tank_test_plan 1단계)
        ⚠️ 그림은 지오펜스 병합·변화율 제한·후진 상태기계를 그리지 않는다 — 그게 걸린 프레임은 기록값과 다를 수 있다.

가상 장면(설명용, 실측 아님): 부표 2개 + 물보라 한 점. 파라미터는 ship_direction.yaml 대회값
(detection 3.0 m, half_width 0.29, clearance 0.25, max_spike_ratio 0.01, border_margin 2, min_obstacle_cells 1),
각분해능 0.25°(1번배 개체 80BF). 단계 함수는 ship_direction.py 의 smooth_spikes / suppress_spike_edges /
dilate_obstacles / select_valid_zones / clamp 를 그대로 옮겼다(노드는 rclpy 를 import 해 Mac 에서 불러올 수 없음).
"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "배치도", "gap_selection.png")

# ---- 대회 파라미터 (ship_direction.yaml) ----
DET, HW, CL = 3.0, 0.29, 0.25
SPIKE_RATIO, BORDER, MIN_CELLS = 0.01, 2, 1
INC = 0.25                      # deg, 80BF
ANG = [i * INC for i in range(int(160 / INC) + 1)]   # LiDAR 각 0~160 (80 = 정면)
N = len(ANG)
MIN_W = HW * 2 + CL             # 0.83 m


def scene():
    """부표(중심 각·거리·지름) 두 개 + 물보라 한 점. 나머지는 탐지거리 밖(inf)."""
    r = [math.inf] * N
    buoys = [(60.0, 2.5, 0.40), (100.0, 2.0, 0.40)]
    for ca, d, dia in buoys:
        half = math.degrees(math.atan((dia / 2) / d))
        for i, a in enumerate(ANG):
            if abs(a - ca) <= half:
                r[i] = d
    r[ANG.index(30.0)] = 0.45   # 물보라 반사 한 점
    return r, buoys


def mask(r):
    return [1 if (not math.isinf(v) and not math.isnan(v) and v < DET) else 0 for v in r]


def smooth_spikes(b):
    mx = int(len(b) * SPIKE_RATIO); out = b[:]; cnt = 0; st = None
    for i, v in enumerate(b):
        if v == 1:
            st = i if st is None else st; cnt += 1
        else:
            if st is not None and cnt <= mx:
                for j in range(st, i):
                    out[j] = 0
            st = None; cnt = 0
    if st is not None and cnt <= mx:
        for j in range(st, len(b)):
            out[j] = 0
    return out


def suppress_edges(b):
    out = b[:]
    for i, v in enumerate(b):
        if v == 1:
            for off in range(-BORDER, BORDER + 1):
                j = i + off
                if 0 <= j < len(b) and b[j] == 0:
                    out[i] = 0
                    break
    return out


def dilate(b, r, inc=INC):
    n = len(b); out = b[:]; i = 0
    while i < n:
        if b[i] == 1:
            s = i
            while i + 1 < n and b[i + 1] == 1:
                i += 1
            e = i
            if e - s + 1 >= MIN_CELLS:
                re = min([DET] + [r[k] for k in range(s, e + 1) if not math.isinf(r[k])])
                m = max(1, int(round(math.degrees(math.atan((HW + CL) / max(re, 0.01))) / inc)))
                for j in range(max(0, s - m), min(n - 1, e + m) + 1):
                    out[j] = 1
        i += 1
    return out


def zones(b):
    z = []; st = None
    for i, v in enumerate(b):
        if v == 0 and st is None:
            st = i
        elif v == 1 and st is not None:
            z.append((st, i - 1)); st = None
    if st is not None:
        z.append((st, len(b) - 1))
    return z


def edge(r, i):
    return DET if math.isinf(r[i]) else r[i]


def pipeline(ang, r, inc, target):
    """ship_direction._compute 와 같은 순서: 마스크 → 스파이크 → 가장자리 → 팽창 → 구간 → 폭 판정 → clamp."""
    b0 = mask(r); b1 = smooth_spikes(b0); b2 = suppress_edges(b1); b3 = dilate(b2, r, inc)
    inc_rad = math.radians(inc); info = []
    for s_, e_ in zones(b3):
        re = min(edge(r, s_), edge(r, e_)); arc = inc_rad * re * (e_ - s_)
        info.append((s_, e_, re, arc, arc >= MIN_W))
    cands = []
    for s_, e_, re, arc, ok in info:
        if ok:
            best = min(max(target, ang[s_]), ang[e_])
            cands.append((abs(best - target), -arc, best))
    cands.sort()
    return (b0, b1, b2, b3), info, (cands[0][2] if cands else None)


def yaw_error_to_lidar(yaw_error):
    """ship_direction: yaw_mapped = 80 + yaw (≤180) / 80 − (360 − yaw)."""
    y = yaw_error % 360.0
    return 80 + y if y <= 180 else 80 - (360 - y)


def read_bag(path, t_rel):
    """rosbag2 에서 t_rel 초에 가장 가까운 /scan 과 그 시각의 /yaw_error·/candidate_angle·/desired_angle. Ubuntu 전용."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    want = {"/scan", "/yaw_error", "/candidate_angle", "/desired_angle"}
    msgs = {k: [] for k in want}; t0 = None
    while rd.has_next():
        topic, data, ts = rd.read_next()
        t0 = ts if t0 is None else t0
        if topic in want:
            msgs[topic].append(((ts - t0) * 1e-9, deserialize_message(data, get_message(types[topic]))))
    if not msgs["/scan"]:
        raise SystemExit("bag 에 /scan 이 없다")
    ts_, scan = min(msgs["/scan"], key=lambda x: abs(x[0] - t_rel))
    def last_before(topic):
        prev = [m for (tt, m) in msgs[topic] if tt <= ts_]
        return prev[-1].data if prev else None
    return ts_, scan, last_before("/yaw_error"), last_before("/candidate_angle"), last_before("/desired_angle")


def from_scan(scan):
    """ship_direction 과 같은 방식으로 0~160° 구간을 자른다(min_index/max_index)."""
    amin = math.degrees(scan.angle_min); inc = math.degrees(scan.angle_increment)
    lo = int((0 - amin) / inc); hi = int((160 - amin) / inc)
    rr = list(scan.ranges)[lo:hi + 1]
    ang = [amin + (lo + i) * inc for i in range(len(rr))]
    rr = [math.inf if (math.isnan(v) or v <= 0) else v for v in rr]
    return ang, rr, inc


def draw(ang, r, inc, target, stages, info, chosen, out, note):
    b0, b1, b2, b3 = stages
    fig = plt.figure(figsize=(13, 9.2))
    gs = fig.add_gridspec(5, 2, width_ratios=[1.05, 1.6], hspace=0.55, wspace=0.18)
    # 제목은 보고서 캡션이 맡는다(그림 Ⅲ-3) — 그림 안에 넣지 않는다
    ax = fig.add_subplot(gs[:, 0])
    def xy(a, d):
        t = math.radians(a - 80 + 90)
        return d * math.cos(t), d * math.sin(t)
    for a in (0, 80, 160):
        x, y = xy(a, 3.3); ax.plot([0, x], [0, y], color="#bbb", lw=0.8, ls=":")
        ax.text(*xy(a, 3.45), f"{a:.0f}°", ha="center", va="center", fontsize=8, color="#777")
    th = [xy(a, DET) for a in ang]; ax.plot([p[0] for p in th], [p[1] for p in th], color="#999", lw=1, ls="--")
    ax.text(*xy(150, DET + 0.12), "탐지 3.0 m", fontsize=8, color="#777")
    pts = [xy(a, d) for a, d in zip(ang, r) if not math.isinf(d) and d < 3.6]
    ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=4, color="#c0392b", zorder=3)
    for s_, e_, re, arc, ok in info:
        col = "#27ae60" if ok else "#e67e22"
        arcp = [xy(ang[k], re) for k in range(s_, e_ + 1)]
        ax.plot([p[0] for p in arcp], [p[1] for p in arcp], color=col, lw=4, alpha=0.6)
    if chosen is not None:
        x, y = xy(chosen, 2.9); ax.annotate("", xy=(x, y), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color="#2471a3", lw=2.4))
        ax.text(x, y + 0.1, f"조향 {chosen:.1f}°", color="#2471a3", fontsize=9, ha="center", weight="bold")
    xt, yt = xy(target, 2.4); ax.plot([0, xt], [0, yt], color="#2471a3", lw=1, ls="--")
    ax.text(xt, yt - 0.18, f"목표 {target:.0f}°", color="#2471a3", fontsize=8, ha="center")
    ax.add_patch(plt.Rectangle((-HW, -0.5), HW * 2, 0.5, color="#555"))
    ax.text(0, -0.75, "배 (폭 2×half_width)", ha="center", fontsize=8)
    if note:
        ax.text(0, -1.05, note, ha="center", fontsize=8, color="#555")
    ax.set_aspect("equal"); ax.set_xlim(-3.6, 3.6); ax.set_ylim(-1.2, 3.7); ax.axis("off")
    ax.set_title("(a) 위에서 본 장면 · 초록 = 통과 가능 구간, 주황 = 폭 부족", fontsize=10, loc="left")
    rows = [(b0, "(b) 이진 마스크: r < 3.0 m → 장애물"),
            (b1, "(c) 스파이크 제거: 길이 ≤ int(n×0.01) 셀 구간 삭제"),
            (b2, "(d) 가장자리 정리: 경계 ±2 셀 안의 장애물 셀 제거"),
            (b3, "(e) 팽창: m = round(atan((0.29+0.25)/r_min)/Δθ) 셀씩 양옆")]
    for k, (bb, title) in enumerate(rows):
        a2 = fig.add_subplot(gs[k, 1])
        a2.fill_between(ang, 0, bb, step="mid", color="#c0392b", alpha=0.75, lw=0)
        a2.set_xlim(160, 0); a2.set_ylim(0, 1.2); a2.set_yticks([]); a2.axvline(80, color="#888", lw=0.8, ls=":")
        a2.set_title(title, fontsize=9.5, loc="left"); a2.tick_params(labelsize=8)
    a3 = fig.add_subplot(gs[4, 1])
    a3.set_xlim(160, 0); a3.set_ylim(0, 1); a3.set_yticks([]); a3.axvline(80, color="#888", lw=0.8, ls=":")
    for s_, e_, re, arc, ok in info:
        col = "#27ae60" if ok else "#e67e22"
        a3.axvspan(ang[s_], ang[e_], color=col, alpha=0.35)
        a3.text((ang[s_] + ang[e_]) / 2, 0.55, f"호 {arc:.2f} m\n{'≥' if ok else '<'} 0.83", ha="center", va="center", fontsize=8)
    a3.axvline(target, color="#2471a3", lw=1, ls="--")
    if chosen is not None:
        a3.axvline(chosen, color="#2471a3", lw=2.4)
    a3.set_title("(f) 폭 판정 arc = Δθ·r_edge·(e-s) ≥ half_width×2 + clearance(0.83 m) → 목표를 구간 안으로 clamp, 가장 가까운 각", fontsize=9.5, loc="left")
    a3.set_xlabel("LiDAR 각 [°]  (80 = 정면)", fontsize=9); a3.tick_params(labelsize=8)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(os.path.abspath(out))
    for s_, e_, re, arc, ok in info:
        print(f"구간 {ang[s_]:6.2f}~{ang[e_]:6.2f}°  r_edge {re:.2f}  호 {arc:.2f} m  {'통과' if ok else '부족'}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", help="rosbag2 폴더 (Ubuntu)")
    ap.add_argument("--t", type=float, default=0.0, help="bag 시작부터 초")
    a = ap.parse_args()
    if not a.bag:
        r, _ = scene(); target = 96.0           # 목표 방위(LiDAR 각) — 일부러 부표 쪽
        stages, info, chosen = pipeline(ANG, r, INC, target)
        draw(ANG, r, INC, target, stages, info, chosen, OUT, "")
        print("목표", target, "→ 조향", chosen)
        return
    ts, scan, yaw_err, cand, desired = read_bag(a.bag, a.t)
    ang, r, inc = from_scan(scan)
    # 갭 선택은 /candidate_angle == 20000(자율 회피, 모드 5·8) 일 때만 돈다(ship_direction._compute (C)).
    #   임무 노드가 후보각·SPIN 을 내는 프레임은 갭 선택을 거치지 않으므로 그림의 계산값과 비교할 수 없다.
    if cand is not None and cand != 20000.0:
        raise SystemExit(f"t={ts:.2f} s 는 /candidate_angle={cand:.1f} — 갭 선택을 쓰지 않는 프레임이다. "
                         "모드 5·8(자율 회피) 구간의 시각을 고를 것")
    if yaw_err is None:
        raise SystemExit("목표를 정할 /yaw_error 가 없다")
    target = yaw_error_to_lidar(yaw_err); src = f"/yaw_error {yaw_err:.1f} → LiDAR {target:.1f}"
    stages, info, chosen = pipeline(ang, r, inc, target)
    rec = f"{desired:.1f}" if desired is not None else "없음"
    note = f"bag t={ts:.2f} s · 목표 {src} · 계산 {chosen if chosen is None else round(chosen, 2)} / 기록 /desired_angle {rec}"
    draw(ang, r, inc, target, stages, info, chosen, OUT.replace(".png", "_bag.png"), note)
    print(note)


if __name__ == "__main__":
    main()

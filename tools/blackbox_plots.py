#!/usr/bin/env python3
"""블랙박스 CSV → 보고서용 그림 2장 (주행 궤적 · 시간 그래프).

    python3 tools/blackbox_plots.py ~/ssf_logs/blackbox_20261105_143000.csv [--name 수조1차] [--waypoints]

출력: docs/전달용/배치도/주행_<name>_궤적.png, 주행_<name>_시간.png  (name 기본 = CSV 파일 이름)
- 궤적: 첫 유효 fix 기준 동·북 [m], 임무 모드(wp_mode)별 색. --waypoints 면 config/waypoints.yaml 의 점을 같은 기준으로 겹친다.
- 시간: wp_mode · 헤딩(imu_yaw)과 조향(desired_angle, LiDAR 각 80 = 정면) · 최근접 장애물 · 페일세이프 레벨 · PWM 좌우.
블랙박스는 /scan 원본·영상을 저장하지 않는다(그건 rosbag) — 이 스크립트는 CSV 의 결과값만 그린다.
빈 칸·없는 열은 건너뛴다(구버전 CSV 호환). ROS 불필요 — Mac 에서도 돈다.
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from matplotlib import font_manager
_have = {f.name for f in font_manager.fontManager.ttflist}
matplotlib.rcParams["font.family"] = next((f for f in ("Apple SD Gothic Neo", "NanumGothic", "Noto Sans CJK KR")
                                           if f in _have), "DejaVu Sans")   # Mac / Ubuntu 한글 글꼴
matplotlib.rcParams["axes.unicode_minus"] = False
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIGDIR = os.path.join(ROOT, "docs", "전달용", "배치도")
MODE_NAME = {0: "게이트 시작", 1: "게이트 끝", 2: "위치유지", 3: "부표 선회", 5: "장애물 회피", 7: "도킹", 8: "토너먼트"}
MODE_COLOR = {0: "#1f77b4", 1: "#17becf", 2: "#2ca02c", 3: "#ff7f0e", 5: "#d62728", 7: "#9467bd", 8: "#8c564b"}


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_local(lat, lon, lat0, lon0):
    """동·북 [m] — 수백 m 범위라 등거리 근사로 충분(north_goal_angle 지오펜스와 같은 방식)."""
    return (lon - lon0) * 111320.0 * math.cos(math.radians(lat0)), (lat - lat0) * 110540.0


def col(rows, k):
    return [num(r.get(k)) for r in rows]


def plot_track(rows, out, waypoints):
    pts = [(num(r.get("gps_lat")), num(r.get("gps_lon")), num(r.get("wp_mode")), num(r.get("gps_status")))
           for r in rows]
    pts = [p for p in pts if p[0] and p[1] and abs(p[0]) > 1e-6 and (p[3] is None or p[3] >= 0)]
    if not pts:
        print("궤적: 유효 GPS 없음 — 건너뜀")
        return False
    lat0, lon0 = pts[0][0], pts[0][1]
    fig, ax = plt.subplots(figsize=(8, 7))
    seg = []; cur = None
    for la, lo, m, _ in pts + [(None, None, "end", None)]:
        if m != cur and seg:
            xs, ys = zip(*seg)
            c = MODE_COLOR.get(int(cur), "#555") if isinstance(cur, float) else "#555"
            ax.plot(xs, ys, color=c, lw=2)
            seg = seg[-1:]
        if la is None:
            break
        cur = m; seg.append(to_local(la, lo, lat0, lon0))
    for m, c in MODE_COLOR.items():
        if any(p[2] == m for p in pts):
            ax.plot([], [], color=c, lw=3, label=f"{m} {MODE_NAME[m]}")
    x0, y0 = to_local(pts[0][0], pts[0][1], lat0, lon0)
    ax.plot(x0, y0, "k^", ms=9, label="출발")
    if waypoints:
        far = max(math.hypot(*to_local(la, lo, lat0, lon0)) for la, lo, _, _ in waypoints)
        if far > 2000.0:     # 다른 장소의 좌표(작년 대회장 등)면 축이 터진다 — 겹치지 않는다
            print(f"웨이포인트가 주행 기록에서 {far / 1000:.1f} km 떨어져 있다 — 다른 장소 좌표라 겹치지 않음")
            waypoints = None
    if waypoints:
        for i, (la, lo, m, _) in enumerate(waypoints):
            x, y = to_local(la, lo, lat0, lon0)
            ax.plot(x, y, "o", mfc="none", mec="k", ms=9)
            ax.text(x + 0.6, y + 0.6, f"WP{i}", fontsize=8)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("동 [m]"); ax.set_ylabel("북 [m]")
    ax.legend(fontsize=8, loc="best")
    fig.savefig(out, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(out)
    return True


def plot_time(rows, out):
    t = col(rows, "t_mono")
    if not any(v is not None for v in t):
        print("시간: t_mono 없음 — 건너뜀")
        return False
    t0 = next(v for v in t if v is not None)
    tt = [None if v is None else v - t0 for v in t]
    panels = [
        ("wp_mode", ["wp_mode"], "임무 모드"),
        ("heading", ["imu_yaw"], "헤딩 /imu/yaw [°]"),
        ("steer", ["desired_angle"], "조향 /desired_angle [LiDAR °, 80 = 정면]"),
        ("obst", ["obstacle_min_dist"], "최근접 장애물 [m]"),
        ("fs", ["failsafe_level"], "페일세이프 레벨"),
        ("pwm", ["pwm_l", "pwm_r"], "PWM [us] (ROS 기준 <1500 = 전진)"),
    ]
    panels = [p for p in panels if any(any(num(r.get(k)) is not None for r in rows) for k in p[1])]
    fig, axs = plt.subplots(len(panels), 1, figsize=(11, 1.9 * len(panels)), sharex=True)
    axs = axs if len(panels) > 1 else [axs]
    for ax, (key, keys, title) in zip(axs, panels):
        for k in keys:
            ys = col(rows, k)
            if key == "steer":       # 특수 명령(5000·20000·50000 등)은 0~160 밖 — 따로 표시
                xs2 = [x for x, y in zip(tt, ys) if x is not None and y is not None and y > 200]
                ys = [y if (y is not None and y <= 200) else None for y in ys]
                if xs2:
                    ax.plot(xs2, [165] * len(xs2), "|", color="#999", ms=6, label="특수 명령")
            pairs = [(x, y) for x, y in zip(tt, ys) if x is not None and y is not None]
            if pairs:
                xs, ys2 = zip(*pairs)
                st = "steps-post" if key in ("wp_mode", "fs") else "default"
                ax.plot(xs, ys2, lw=1.2, drawstyle=st, label=k if len(keys) > 1 else None)
        if key == "steer":
            ax.axhline(80, color="#888", lw=0.8, ls=":")
        ax.set_title(title, fontsize=9, loc="left"); ax.grid(alpha=0.3); ax.tick_params(labelsize=8)
        if len(keys) > 1 or key == "steer":
            ax.legend(fontsize=7, loc="upper right")
    axs[-1].set_xlabel("시간 [s]")
    fig.tight_layout(); fig.savefig(out, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(out)
    return True


def load_waypoints():
    import yaml
    from importlib.machinery import SourceFileLoader
    wl = SourceFileLoader("waypoint_loader", os.path.join(ROOT, "src/north_goal_angle/north_goal_angle/waypoint_loader.py")).load_module()
    raw = yaml.safe_load(open(os.path.join(ROOT, "src/north_goal_angle/config/waypoints.yaml"), encoding="utf-8"))
    return [(w[0], w[1], w[2], None) for w in wl.parse_waypoints(raw)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--name")
    ap.add_argument("--waypoints", action="store_true", help="config/waypoints.yaml 점을 겹친다")
    a = ap.parse_args()
    rows = load(a.csv)
    name = a.name or os.path.splitext(os.path.basename(a.csv))[0]
    os.makedirs(FIGDIR, exist_ok=True)
    wps = load_waypoints() if a.waypoints else None
    plot_track(rows, os.path.join(FIGDIR, f"주행_{name}_궤적.png"), wps)
    plot_time(rows, os.path.join(FIGDIR, f"주행_{name}_시간.png"))


if __name__ == "__main__":
    main()

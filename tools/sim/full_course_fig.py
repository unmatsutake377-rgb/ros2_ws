#!/usr/bin/env python3
"""보고서용 전체 코스 궤적 그림 — full_course_sim 의 한 런(실제 노드 코드)을 mode 색으로 그린다.

  python3 tools/sim/full_course_fig.py --seed 1000 --world-wp-file <5faf86b waypoints 사본> --png out.png

full_course_sim.plot 과 달리 진단 문구(접촉 수 등)는 넣지 않고, 웨이포인트를 구역 이름으로 적으며
도킹 구역을 확대해 옆에 둔다. 판정 수치는 full_course_sim 의 200 시드 요약을 쓴다.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import full_course_sim as F  # noqa: E402

HIDE_DOCK_WP = False
MODE_NAME = {0: "게이트(항로추종)", 1: "게이트(항로추종)", 2: "위치유지", 3: "탐색",
             5: "장애물 회피", 7: "도킹", 8: "토너먼트 회피"}


def short(label):
    """'WP3 부표(초록)' → 'WP3', '도킹 접근 10m' → '접근 10 m'."""
    s = label.split("(")[0].strip()
    if s.startswith("도킹 접근"):
        return "접근 " + s.replace("도킹 접근", "").replace("m", " m").strip()
    return s.split()[0] if s.startswith("WP") else s


def draw(ax, r, zoom=None):
    import matplotlib.pyplot as plt
    W, rec, wps = r["W"], r["rec"], r["wps"]
    X, Y, M = np.array(rec["x"]), np.array(rec["y"]), rec["mode"]
    i0 = 0
    for i in range(1, len(X) + 1):
        if i == len(X) or M[i] != M[i0]:
            ax.plot(X[i0:i + 1], Y[i0:i + 1], color=F.MODE_COL.get(0 if M[i0] == 1 else M[i0], "k"), lw=1.8)
            i0 = i
    colmap = {"red": "crimson", "green": "seagreen", "white": "white", "orange": "#ff7f0e"}
    for b in W["buoys"]:
        big = b["kind"] == "hold" or b["kind"].startswith("search")
        ax.add_patch(plt.Circle((b["x"], b["y"]), 0.45 if big else 0.3,
                                fc=colmap[b["color"]], ec="k", lw=0.7, zorder=4))
    for g in W["gates"]:
        ax.plot([g["red"][0], g["green"][0]], [g["red"][1], g["green"][1]], color="k", ls=":", lw=0.7)
    for a_, b_, kind in W["segs"]:
        ax.plot([a_[0], b_[0]], [a_[1], b_[1]], color="#222", lw=3 if kind == "front" else 2)
    mk = W["dock"]["marker"]
    ax.plot(mk[0], mk[1], "s", color="crimson", ms=7, zorder=5)
    seen = {}
    for w in wps:
        wx, wy = W["geo"].xy(w["lat"], w["lon"])
        key = (round(wx, 1), round(wy, 1))
        k = seen.get(key, 0)
        seen[key] = k + 1
        if HIDE_DOCK_WP and w["mode"] in (5, 7) and short(w["label"]).startswith(("접근", "WP8", "WP9")):
            continue            # --plain: 도킹 구역 웨이포인트는 그리지 않음(규정 확인 전 보고서용, 11-04)
        ax.plot(wx, wy, "+", color="k", ms=9, mew=1.4, zorder=6)
        if zoom is None and w["mode"] in (5, 7) and short(w["label"]).startswith(("접근", "WP8", "WP9")):
            continue            # 전체 그림에서는 도킹 네 줄 이름을 생략(확대 그림에 적음)
        off = (-30, 6) if short(w["label"]) == "WP8" else (5, 4 + 9 * k)
        ax.annotate(short(w["label"]), (wx, wy), xytext=off, textcoords="offset points",
                    fontsize=8 if zoom else 8.5)
    ax.plot(X[0], Y[0], "o", color="k", ms=6)
    ax.annotate("출발", (X[0], Y[0]), xytext=(5, -11), textcoords="offset points", fontsize=8.5)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    if zoom:
        ax.set_xlim(*zoom[0])
        ax.set_ylim(*zoom[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--variant", default="B")
    ap.add_argument("--world-wp-file", required=True)
    ap.add_argument("--png", required=True)
    ap.add_argument("--plain", action="store_true", help="도킹 확대 칸·도킹 웨이포인트 표시 없이 (a) 만")
    a = ap.parse_args()
    global HIDE_DOCK_WP
    HIDE_DOCK_WP = a.plain
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
    matplotlib.rcParams["axes.unicode_minus"] = False
    r = F.run(a.seed, a.variant, world_wp_file=a.world_wp_file, record=True)
    assert r["ok"], r.get("traceback")
    W = r["W"]
    if a.plain:
        fig = plt.figure(figsize=(7.6, 8.2))
        ax = fig.add_subplot(111)
    else:
        fig = plt.figure(figsize=(12, 8.2))
        gs = fig.add_gridspec(1, 2, width_ratios=[1.55, 1.0], wspace=0.12)
        ax = fig.add_subplot(gs[0])
    draw(ax, r)
    ax.set_xlabel("x [m] (동)")
    ax.set_ylabel("y [m] (북)")
    ax.set_title(("" if a.plain else "(a) ") + "전체 코스 궤적 (색 = 임무 mode)", fontsize=11)
    used = []
    for m in [0, 2, 3, 5, 7, 8]:
        if m in r["rec"]["mode"]:
            used.append(Line2D([], [], color=F.MODE_COL[m], lw=2.5, label=MODE_NAME[m]))
    used += [Line2D([], [], marker="+", color="k", ls="", ms=9, mew=1.4, label="웨이포인트"),
             Line2D([], [], marker="s", color="crimson", ls="", ms=7, label="도킹 목표 표식")]
    ax.legend(handles=used, fontsize=8.5, loc="upper left", framealpha=0.9)
    if a.plain:
        fig.savefig(a.png, dpi=160, bbox_inches="tight")
        print(a.png, "코스", r["t_course"], "s")
        return
    dx, dy = W["dock"]["marker"]
    ax2 = fig.add_subplot(gs[1])
    draw(ax2, r, zoom=((dx - 9, dx + 9), (dy - 3, dy + 14)))
    ax2.set_xlabel("x [m]")
    ax2.set_title("(b) 도킹 구역 확대", fontsize=11)
    fig.savefig(a.png, dpi=160, bbox_inches="tight")
    print(a.png, "코스", r["t_course"], "s")


if __name__ == "__main__":
    main()

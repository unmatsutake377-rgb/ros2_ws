#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S1 스윕 — half_width × clearance 전 코스 (시드 N개)

  python3 tools/sim/sweep_s1.py                       # 기본: 폭 5단계 × clr 6단계 × 시드 20
  python3 tools/sim/sweep_s1.py --widths 0.31 --seeds 30

- 제어기(ssf_sim.HALF_WIDTH/CLEARANCE)와 접촉 판정(kaboat_full_course_sim.BOAT_HALF_W)을 런마다 같이 패치
- 구간 순서는 kaboat_full_course_sim.main() 과 같다 (게이트4 → WP1 → 도킹 → 탐색 → WP2 → 장애물 → GOAL)
- 결과: tools/sim/out/sweep_s1.csv + 콘솔 표
- 한계: 접촉 모델은 반경 half_width 원. 선회 중 측면(길이 방향) 접촉은 못 잡는다 → swept_width() 표로 보완
"""
import argparse, csv, itertools, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def run_one(args):
    half_w, phys_w, clr, dd, gate_w, seed = args
    import ssf_sim, kaboat_full_course_sim as K
    from fastworld import FastWorld as World
    ssf_sim.HALF_WIDTH = half_w
    ssf_sim.CLEARANCE = clr
    K.BOAT_HALF_W = phys_w
    c = K.build_course(gate_w=gate_w, seed=seed)
    buoys = []
    for (a, b) in c["gates"]:
        buoys += [(a[0], a[1], K.BUOY_R), (b[0], b[1], K.BUOY_R)]
    buoys += [(c["wp1"][0], c["wp1"][1], K.BUOY_R), (c["search"][0], c["search"][1], K.BUOY_R)]
    buoys += [(p[0], p[1], K.BUOY_R) for p in c["search_others"]] + c["obst"]
    w = World(buoys)
    st = dict(pos=c["start"], psi=90.0, t=0.0, hits=0, fails=0, h_gate=0, h_obst=0)

    def go(goal, stop_r, max_t, fence=None, tag=None):
        tr, t, h, o, ok, st["psi"] = K.leg(w, st["pos"], st["psi"], goal, stop_r=stop_r,
                                          dd=dd, max_t=max_t, fence=fence)
        st["t"] += t; st["hits"] += h; st["fails"] += 0 if ok else 1
        if tag: st[tag] += h
        if len(tr): st["pos"] = tuple(tr[-1])

    for (a, b) in c["gates"]:
        go(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), 1.0, 120, tag="h_gate")
    go(c["wp1"], K.HOLD_RADIUS - 1, 120); st["t"] += K.HOLD_TIME
    go(c["docks"][1], 1.5, 180);          st["t"] += 10.0
    go(c["search"], 3.0, 180);            st["t"] += 2 * math.pi * 3.0 / K.CRUISE
    go(c["wp2"], K.HOLD_RADIUS - 1, 180); st["t"] += K.HOLD_TIME
    go(c["goal"], 1.5, 300, fence=c["fence"], tag="h_obst")
    return dict(half_w=half_w, phys_w=phys_w, clr=clr, dd=dd, gate_w=gate_w, seed=seed,
                t=round(st["t"], 1), hits=st["hits"], h_gate=st["h_gate"], h_obst=st["h_obst"],
                fails=st["fails"], score=round(st["t"] + st["hits"] * K.PENALTY_PER_HIT, 1))


def swept_width(W, L, deg):
    th = math.radians(deg)
    return W * math.cos(th) + L * math.sin(th)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--widths", type=float, nargs="+", default=[0.215, 0.25, 0.29, 0.32, 0.35])
    ap.add_argument("--clears", type=float, nargs="+", default=[0.15, 0.20, 0.25, 0.30, 0.35, 0.45])
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--dds", type=float, nargs="+", default=[2.0, 3.0])
    ap.add_argument("--gates", type=float, nargs="+", default=[2.0, 3.0])
    a = ap.parse_args()

    grid = [(hw, hw, clr, dd, gw, s) for hw, clr, dd, gw, s in
            itertools.product(a.widths, a.clears, a.dds, a.gates, range(1, a.seeds + 1))]
    t0 = time.time()
    with ProcessPoolExecutor() as ex:
        rows = list(ex.map(run_one, grid, chunksize=8))
    print(f"{len(rows)} runs, {time.time() - t0:.0f}s wall, seeds={a.seeds}")
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    with open(os.path.join(HERE, "out", "sweep_s1.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)

    n_per = len(a.dds) * len(a.gates) * a.seeds
    print(f"\n평균 = dd {a.dds} × 게이트 {a.gates} × 시드 {a.seeds} = {n_per}런")
    print(f"{'half_w':>6} {'clr':>5} {'갭요구':>6} | {'접촉':>5} {'게이트':>6} {'장애물':>6} {'정체런%':>7} {'무접촉%':>7} {'기록':>6}")
    best = {}
    for hw in a.widths:
        for clr in a.clears:
            sub = [r for r in rows if r["half_w"] == hw and r["clr"] == clr]
            m = lambda k: sum(r[k] for r in sub) / len(sub)
            stall = sum(1 for r in sub if r["fails"]) / len(sub) * 100
            clean = sum(1 for r in sub if r["hits"] == 0 and r["fails"] == 0) / len(sub) * 100
            print(f"{hw:6.3f} {clr:5.2f} {hw*2+clr:6.2f} | {m('hits'):5.2f} {m('h_gate'):6.2f} {m('h_obst'):6.2f} "
                  f"{stall:7.0f} {clean:7.0f} {m('score'):6.1f}")
            key = (stall, m("hits"))
            if hw not in best or key < best[hw][0]:
                best[hw] = (key, clr)
        print()
    print("폭별 최적 clearance (정체 최소 → 접촉 최소):")
    for hw, (_, clr) in best.items():
        print(f"  half_w {hw:.3f} (폭 {hw*2:.2f}m) → clearance {clr:.2f}")

    print("\n선회 유효 폭 W·cosθ + L·sinθ  (정지 폭 대비)")
    for name, W, L in [("1번배", 0.58, 1.66), ("2번배", 0.43, 1.70)]:
        cells = "  ".join(f"{d:2d}°={swept_width(W, L, d):.2f}m" for d in (0, 5, 10, 15, 20))
        print(f"  {name} W={W} L={L}: {cells}")


if __name__ == "__main__":
    main()

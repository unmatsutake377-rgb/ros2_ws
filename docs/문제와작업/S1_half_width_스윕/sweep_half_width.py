#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S1 스윕: half_width(실측 폭) × clearance × 탐지거리 × 게이트폭
 - ssf_sim.HALF_WIDTH / CLEARANCE 와 kaboat_full_course_sim.BOAT_HALF_W 를 런마다 패치
 - main() 의 구간 순서를 그대로 재현 (게이트4 → WP1 → 도킹 → 탐색 → WP2 → 장애물 → GOAL)
 - 결과: sweep_half_width.csv + 콘솔 요약
"""
import sys, math, csv, itertools, time
from concurrent.futures import ProcessPoolExecutor

HALF_WS = [0.45, 0.29, 0.215]                 # 코드값 / 1번배 58cm / 2번배 43cm
CLEARS  = [0.15, 0.20, 0.25, 0.30, 0.35, 0.45]
DDS     = [2.0, 3.0]                          # yaml gate 2.0 / default 3.0
GATE_WS = [2.0, 3.0, 4.0]


def run_one(args):
    half_w, clr, dd, gate_w = args
    import ssf_sim, kaboat_full_course_sim as K
    ssf_sim.HALF_WIDTH = half_w
    ssf_sim.CLEARANCE = clr
    K.BOAT_HALF_W = half_w
    from fastworld import FastWorld as World

    c = K.build_course(gate_w=gate_w)
    buoys = []
    for (a, b) in c["gates"]:
        buoys += [(a[0], a[1], K.BUOY_R), (b[0], b[1], K.BUOY_R)]
    buoys += [(c["wp1"][0], c["wp1"][1], K.BUOY_R)]
    buoys += [(c["search"][0], c["search"][1], K.BUOY_R)]
    buoys += [(p[0], p[1], K.BUOY_R) for p in c["search_others"]]
    buoys += c["obst"]
    w = World(buoys)

    total_t, total_hits, oob = 0.0, 0, False
    fails = 0
    pos, psi = c["start"], 90.0

    def go(goal, stop_r, max_t, fence=None):
        nonlocal pos, psi, total_t, total_hits, oob, fails
        tr, t, h, o, ok, psi = K.leg(w, pos, psi, goal, stop_r=stop_r, dd=dd, max_t=max_t, fence=fence)
        total_t += t; total_hits += h; oob |= o
        if not ok: fails += 1
        if len(tr): pos = tuple(tr[-1])
        return h

    h_gate = 0
    for (a, b) in c["gates"]:
        h_gate += go(((a[0]+b[0])/2, (a[1]+b[1])/2), 1.0, 120)
    go(c["wp1"], K.HOLD_RADIUS - 1, 120); total_t += K.HOLD_TIME
    go(c["docks"][1], 1.5, 180);          total_t += 10.0
    go(c["search"], 3.0, 180);            total_t += 2*math.pi*3.0/K.CRUISE
    go(c["wp2"], K.HOLD_RADIUS - 1, 180); total_t += K.HOLD_TIME
    h_obst = go(c["goal"], 1.5, 300, fence=c["fence"])

    score = total_t + total_hits * K.PENALTY_PER_HIT
    return dict(half_w=half_w, clr=clr, dd=dd, gate_w=gate_w,
                t=round(total_t,1), hits=total_hits, h_gate=h_gate, h_obst=h_obst,
                fails=fails, oob=int(oob), score=round(score,1))


def main():
    grid = list(itertools.product(HALF_WS, CLEARS, DDS, GATE_WS))
    t0 = time.time()
    with ProcessPoolExecutor() as ex:
        rows = list(ex.map(run_one, grid))
    print(f"{len(rows)} runs, {time.time()-t0:.0f}s wall")

    with open("sweep_half_width.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader(); wr.writerows(rows)

    # 요약: (half_w, clr) 별 게이트폭/탐지거리 평균
    print(f"\n{'half_w':>6} {'clr':>5} {'minW':>5} | {'hits':>5} {'gate':>5} {'obst':>5} {'fail':>5} {'oob':>4} | {'t':>6} {'score':>6}")
    for hw in HALF_WS:
        for clr in CLEARS:
            sub = [r for r in rows if r["half_w"]==hw and r["clr"]==clr]
            n = len(sub)
            avg = lambda k: sum(r[k] for r in sub)/n
            print(f"{hw:6.3f} {clr:5.2f} {hw*2+clr:5.2f} | {avg('hits'):5.2f} {avg('h_gate'):5.2f} {avg('h_obst'):5.2f} "
                  f"{avg('fails'):5.2f} {sum(r['oob'] for r in sub):4d} | {avg('t'):6.1f} {avg('score'):6.1f}")
        print()

    # 게이트폭 2.0 (가장 빡빡) 단독
    print("=== gate_w=2.0 만 (가장 빡빡한 조건) ===")
    print(f"{'half_w':>6} {'clr':>5} | dd=2.0 hits/fail/score | dd=3.0 hits/fail/score")
    for hw in HALF_WS:
        for clr in CLEARS:
            cells = []
            for dd in DDS:
                r = next(x for x in rows if x["half_w"]==hw and x["clr"]==clr and x["dd"]==dd and x["gate_w"]==2.0)
                cells.append(f"{r['hits']:2d}/{r['fails']}/{r['score']:6.1f}")
            print(f"{hw:6.3f} {clr:5.2f} | {'   |   '.join(cells)}")
        print()

if __name__ == "__main__":
    main()

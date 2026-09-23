#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S1 2차: 실측 폭 2종 × clearance × dd × gate_w × 장애물 배치 seed 6개 — 단일 seed 노이즈 제거"""
import itertools, time, csv, math
from concurrent.futures import ProcessPoolExecutor
import sweep_half_width as S

HALF_WS = [0.29, 0.215]
CLEARS  = [0.20, 0.25, 0.30, 0.35, 0.40]
DDS     = [2.0, 3.0]
GATE_WS = [2.0, 3.0]
SEEDS   = [1, 2, 3, 4, 5, 6]

def run_one(args):
    half_w, clr, dd, gate_w, seed = args
    import kaboat_full_course_sim as K
    orig = K.build_course
    K.build_course = lambda gate_w=3.0, seed=5, _o=orig, _s=seed: _o(gate_w=gate_w, seed=_s)
    r = S.run_one((half_w, clr, dd, gate_w))
    r["seed"] = seed
    return r

def main():
    grid = list(itertools.product(HALF_WS, CLEARS, DDS, GATE_WS, SEEDS))
    t0 = time.time()
    with ProcessPoolExecutor() as ex:
        rows = list(ex.map(run_one, grid))
    print(f"{len(rows)} runs, {time.time()-t0:.0f}s wall")
    with open("sweep_half_width_seeds.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)

    def summ(sub):
        n = len(sub)
        return (sum(r["hits"] for r in sub)/n, sum(r["h_gate"] for r in sub)/n, sum(r["h_obst"] for r in sub)/n,
                sum(r["fails"] for r in sub), sum(1 for r in sub if r["hits"]==0 and r["fails"]==0)/n*100,
                sum(r["score"] for r in sub)/n)
    for hw in HALF_WS:
        print(f"\n### half_w={hw} (폭 {hw*2:.2f}m)")
        print(f"{'clr':>5} {'lat':>5} | {'dd':>3} {'gw':>3} | {'hits':>5} {'gate':>5} {'obst':>5} {'fail':>4} {'clean%':>6} {'score':>6}")
        for clr in CLEARS:
            for dd in DDS:
                for gw in GATE_WS:
                    sub = [r for r in rows if r["half_w"]==hw and r["clr"]==clr and r["dd"]==dd and r["gate_w"]==gw]
                    h, hg, ho, fl, cl, sc = summ(sub)
                    print(f"{clr:5.2f} {hw+clr:5.2f} | {dd:3.1f} {gw:3.1f} | {h:5.2f} {hg:5.2f} {ho:5.2f} {fl:4d} {cl:6.0f} {sc:6.1f}")
            sub = [r for r in rows if r["half_w"]==hw and r["clr"]==clr]
            h, hg, ho, fl, cl, sc = summ(sub)
            print(f"{clr:5.2f} {'ALL':>5} |         | {h:5.2f} {hg:5.2f} {ho:5.2f} {fl:4d} {cl:6.0f} {sc:6.1f}")
            print()

if __name__ == "__main__":
    main()

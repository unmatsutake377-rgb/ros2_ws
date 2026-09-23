#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""단일 yaml(half_w 0.29 / clr 0.25)을 2번배(실폭 0.43)에 그대로 쓸 때: 제어기 0.29, 접촉반경 0.215"""
import math
from concurrent.futures import ProcessPoolExecutor

def run_one(a):
    ctrl_hw, clr, dd, gw, seed, phys_hw = a
    import ssf_sim, kaboat_full_course_sim as K
    from fastworld import FastWorld as World
    ssf_sim.HALF_WIDTH = ctrl_hw; ssf_sim.CLEARANCE = clr; K.BOAT_HALF_W = phys_hw
    c = K.build_course(gate_w=gw, seed=seed)
    buoys = []
    for (p, q) in c["gates"]: buoys += [(p[0], p[1], K.BUOY_R), (q[0], q[1], K.BUOY_R)]
    buoys += [(c["wp1"][0], c["wp1"][1], K.BUOY_R), (c["search"][0], c["search"][1], K.BUOY_R)]
    buoys += [(p[0], p[1], K.BUOY_R) for p in c["search_others"]] + c["obst"]
    w = World(buoys); pos, psi = c["start"], 90.0; T = 0.0; H = 0; F = 0
    def go(goal, sr, mt, fence=None):
        nonlocal pos, psi, T, H, F
        tr, t, h, o, ok, psi = K.leg(w, pos, psi, goal, stop_r=sr, dd=dd, max_t=mt, fence=fence)
        T += t; H += h; F += (0 if ok else 1)
        if len(tr): pos = tuple(tr[-1])
    for (p, q) in c["gates"]: go(((p[0]+q[0])/2, (p[1]+q[1])/2), 1.0, 120)
    go(c["wp1"], 4, 120); go(c["docks"][1], 1.5, 180); go(c["search"], 3.0, 180); go(c["wp2"], 4, 180); go(c["goal"], 1.5, 300, c["fence"])
    return dict(dd=dd, gw=gw, seed=seed, hits=H, fails=F, t=round(T, 1))

if __name__ == "__main__":
    grid = [(0.29, 0.25, dd, gw, s, 0.215) for dd in [2.0, 3.0] for gw in [2.0, 3.0] for s in range(1, 7)]
    with ProcessPoolExecutor() as ex: rows = list(ex.map(run_one, grid))
    for dd in [2.0, 3.0]:
        for gw in [2.0, 3.0]:
            sub = [r for r in rows if r["dd"] == dd and r["gw"] == gw]
            print(f"dd={dd} gw={gw}: hits avg {sum(r['hits'] for r in sub)/6:.2f}  clean {sum(1 for r in sub if r['hits']==0 and r['fails']==0)}/6  fails {sum(r['fails'] for r in sub)}")
    print("ALL hits avg", round(sum(r['hits'] for r in rows)/len(rows), 2), "clean", sum(1 for r in rows if r['hits']==0 and r['fails']==0), "/", len(rows))

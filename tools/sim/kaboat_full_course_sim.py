#!/usr/bin/env python3
"""
KABOAT 2025 종합임무 — 실제 경기규정 기반 전 코스 시뮬레이션
==============================================================
경기규정(경기규정-202511-수정.docx)의 확정 수치를 반영:
  · 1경기장(종합임무) 70m × 20m
  · 제한시간 15분(900초)
  · 장애물 접촉 패널티 1회당 30초
  · 미션 순서: 항로추종 → 위치유지 → 도킹 → 탐색 → 장애물회피
  · 위치유지 2회, 부표 5m 이내 5초
  · 배점: 항로추종15% 위치유지10% 도킹15% 탐색15% 장애물회피20%
  · 보트 길이 L ≥ 1.0m, 높이 ≤ 1m
  · ★ 구성물의 GPS 좌표는 공개되지 않음 (센서로 직접 인지해야 함)
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fastworld import FastWorld as World
from ssf_sim import (
    ship_direction_step, motor_control_step, yaw_error_chain, bearing_to,
    SCAN_ANGLE_MIN_DEG, SCAN_INC_DEG,
    HALF_WIDTH, CLEARANCE, BASE_PWM, PWM_NEUTRAL,
)

# ── 규정 수치 ────────────────────────────────────────────────
FIELD_L, FIELD_W = 70.0, 20.0     # 1경기장
TIME_LIMIT = 900.0                # 15분
PENALTY_PER_HIT = 30.0            # 접촉 1회당 30초
HOLD_TIME = 5.0                   # 위치유지 5초
HOLD_RADIUS = 5.0                 # 부표 5m 이내
WEIGHTS = {"항로추종": 15, "위치유지": 10, "도킹": 15, "탐색": 15, "장애물회피": 20}

# ── 배/센서 (가정 — 실측 필요) ────────────────────────────────
BOAT_HALF_W = HALF_WIDTH          # 0.29m (1번배 설계값) → 폭 0.58m. 2번배는 0.215
CRUISE = 1.2                      # m/s
BUOY_R = 0.20
K_V = CRUISE / (PWM_NEUTRAL - BASE_PWM)
K_W = 0.22
DT = 0.1


def build_course(gate_w=3.0, seed=5):
    """그림2(종합임무) 배치를 70×20m 경기장에 재현."""
    rng = np.random.default_rng(seed)
    c = {}
    c["start"] = (5.0, 3.0)
    # 1) 항로추종: 빨강(좌)-초록(우) 게이트 4개
    c["gates"] = []
    for gx in [15.0, 24.0, 33.0, 42.0]:
        c["gates"].append(((gx, 3.0 - gate_w / 2), (gx, 3.0 + gate_w / 2)))
    # 2) 위치유지 WP1
    c["wp1"] = (52.0, 3.5)
    # 3) 도킹 (3개 도크)
    c["docks"] = [(64.0, 2.0), (64.0, 5.0), (64.0, 8.0)]
    # 4) 탐색 (목표 부표 + 방해 부표)
    c["search"] = (62.0, 15.0)
    c["search_others"] = [(58.0, 17.5), (57.0, 12.5)]
    # 5) 위치유지 WP2
    c["wp2"] = (46.0, 16.0)
    # 6) 장애물 회피 구간 (가상경계선 내부)
    c["obst"] = []
    for gx in np.arange(12, 42, 4.0):
        for k in range(2):
            c["obst"].append((gx + rng.uniform(-1.2, 1.2),
                              16.5 + rng.uniform(-3.5, 3.5), BUOY_R))
    c["fence"] = (3.0, 10.5, 47.0, 20.0)   # 장애물 구간 가상경계선(진입~출구게이트 포함)
    c["goal"] = (5.0, 16.5)
    return c


def leg(world, start, psi, goal, stop_r=1.2, dd=1.8, max_t=300.0, fence=None):
    """한 구간 주행. 반환: 궤적, 소요시간, 접촉수, 경계이탈, 도달여부"""
    x, y = start
    traj = [(x, y)]
    t = 0.0
    hits, hit_flag = 0, {}
    oob = False
    entered = False
    while t < max_t:
        if math.hypot(goal[0] - x, goal[1] - y) < stop_r:
            return np.array(traj), t, hits, oob, True, psi
        ranges = world.lidar_scan(x, y, psi, max_range=20.0)
        e = yaw_error_chain(bearing_to(x, y, goal[0], goal[1]), psi)
        desired, _ = ship_direction_step(list(ranges), SCAN_ANGLE_MIN_DEG,
                                         SCAN_INC_DEG, e, detection_distance=dd)
        pr, pl, _ = motor_control_step(desired)
        TR, TL = PWM_NEUTRAL - pr, PWM_NEUTRAL - pl
        v = K_V * (TR + TL) / 2.0
        psi = (psi + K_W * (TL - TR) * DT) % 360.0
        th = math.radians(psi)
        x += v * math.sin(th) * DT
        y += v * math.cos(th) * DT
        t += DT
        traj.append((x, y))
        # 접촉 판정 (부표별 1회만 카운트)
        for i, (bx, by, br) in enumerate(world.buoys):
            if math.hypot(x - bx, y - by) - br < BOAT_HALF_W:
                if not hit_flag.get(i):
                    hits += 1
                    hit_flag[i] = True
        if fence:
            x0, y0, x1, y1 = fence
            inside = (x0 <= x <= x1 and y0 <= y <= y1)
            if inside:
                entered = True
            elif entered and not (x0 - 2 <= x <= x1 + 2 and y0 - 2 <= y <= y1 + 2):
                oob = True   # 진입 후 '확연히' 벗어난 경우만 실격
    return np.array(traj), t, hits, oob, False, psi


def main():
    R = []
    P = R.append
    P("=" * 78)
    P("KABOAT 2025 종합임무 — 실제 경기규정 기반 전 코스 시뮬레이션")
    P("=" * 78)
    P(f"[규정] 경기장 {FIELD_L}m×{FIELD_W}m / 제한시간 {TIME_LIMIT/60:.0f}분 / "
      f"접촉 패널티 {PENALTY_PER_HIT:.0f}초·회")
    P(f"[규정] 위치유지 부표 {HOLD_RADIUS}m 이내 {HOLD_TIME:.0f}초, 2회 수행")
    P(f"[가정] 배 폭 {BOAT_HALF_W*2:.1f}m / 순항 {CRUISE}m/s (실측 필요)")

    # ==================================================================
    # ★ 규정상 치명적 발견 — GPS 좌표 비공개
    # ==================================================================
    P("\n" + "★" * 39)
    P("[규정 5.2] \"각 구성물의 정확한 위치(GPS 좌표)는 공개되지 않으며,")
    P("            각 팀은 탑재 센서를 활용하여 상황 인지를 직접 수행해야 한다\"")
    P("")
    P("  → 현재 코드 north_goal_angle 은 waypoints[] 에 GPS 좌표를 하드코딩하고")
    P("     그 좌표로 항법한다. 좌표가 공개되지 않으므로 이 방식은 원천적으로 불가.")
    P("  → GPS는 '대략적 구간 이동'에만 쓰고, 미션 수행은 LiDAR/카메라 기반으로")
    P("     전면 전환해야 함. (아키텍처 수준의 변경)")
    P("★" * 39)

    # ==================================================================
    # 전 코스 주행 (게이트폭·탐지거리 조합)
    # ==================================================================
    P("\n" + "─" * 78)
    P("[전 코스 주행] 항로추종 → WP1 → 도킹 → 탐색 → WP2 → 장애물회피 → GOAL")
    P("─" * 78)

    results = {}
    for dd in [1.8, 3.0]:
        for gate_w in [2.0, 3.0, 4.0]:
            c = build_course(gate_w=gate_w)
            # 월드: 게이트 + 탐색 부표 + 장애물 (도크는 별도)
            buoys = []
            for (a, b) in c["gates"]:
                buoys += [(a[0], a[1], BUOY_R), (b[0], b[1], BUOY_R)]
            buoys += [(c["wp1"][0], c["wp1"][1], BUOY_R)]
            buoys += [(c["search"][0], c["search"][1], BUOY_R)]
            buoys += [(p[0], p[1], BUOY_R) for p in c["search_others"]]
            buoys += c["obst"]
            w = World(buoys)

            total_t, total_hits, oob_any = 0.0, 0, False
            legs = []
            pos, psi = c["start"], 90.0   # 동쪽(+x) 향해 출발

            # 1구간: 항로추종 (게이트 4개 통과)
            for (a, b) in c["gates"]:
                mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                tr, t, h, o, ok, psi = leg(w, pos, psi, mid, stop_r=1.0, dd=dd, max_t=120)
                total_t += t; total_hits += h; oob_any |= o
                legs.append(tr)
                if len(tr): pos = tuple(tr[-1])
            t_gate = total_t
            h_gate = total_hits

            # WP1 위치유지 (5초)
            tr, t, h, o, ok, psi = leg(w, pos, psi, c["wp1"], stop_r=HOLD_RADIUS - 1, dd=dd, max_t=120)
            total_t += t + HOLD_TIME; total_hits += h
            legs.append(tr)
            if len(tr): pos = tuple(tr[-1])

            # 2구간: 도킹 (접근만 모사)
            tr, t, h, o, ok, psi = leg(w, pos, psi, c["docks"][1], stop_r=1.5, dd=dd, max_t=180)
            total_t += t + 10.0; total_hits += h     # 접안/이탈 10초 가정
            legs.append(tr)
            if len(tr): pos = tuple(tr[-1])

            # 3구간: 탐색 (부표까지 접근 + 선회 1바퀴 = 2πr/v)
            tr, t, h, o, ok, psi = leg(w, pos, psi, c["search"], stop_r=3.0, dd=dd, max_t=180)
            orbit_t = 2 * math.pi * 3.0 / CRUISE
            total_t += t + orbit_t; total_hits += h
            legs.append(tr)
            if len(tr): pos = tuple(tr[-1])

            # WP2 위치유지
            tr, t, h, o, ok, psi = leg(w, pos, psi, c["wp2"], stop_r=HOLD_RADIUS - 1, dd=dd, max_t=180)
            total_t += t + HOLD_TIME; total_hits += h
            legs.append(tr)
            if len(tr): pos = tuple(tr[-1])

            # 4구간: 장애물 회피 → GOAL
            tr, t, h, o, ok, psi = leg(w, pos, psi, c["goal"], stop_r=1.5, dd=dd,
                                       max_t=300, fence=c["fence"])
            total_t += t; total_hits += h; oob_any |= o
            legs.append(tr)
            h_obst = h

            penalty = total_hits * PENALTY_PER_HIT
            score_t = total_t + penalty
            fin = total_t < TIME_LIMIT
            results[(dd, gate_w)] = dict(t=total_t, hits=total_hits, pen=penalty,
                                         score=score_t, fin=fin, oob=oob_any,
                                         legs=legs, course=c,
                                         t_gate=t_gate, h_gate=h_gate, h_obst=h_obst)
            P(f"  탐지 {dd:.1f}m / 게이트 {gate_w:.1f}m → "
              f"주행 {total_t:5.1f}s + 패널티 {penalty:5.1f}s({total_hits}회) "
              f"= 기록 {score_t:6.1f}s | "
              f"{'시간내완주' if fin else '★시간초과'}"
              f"{' ★경계이탈(실격)' if oob_any else ''}")

    P("")
    P("  ※ 제한시간 15분(900초)은 여유. 실질 승부는 '접촉 패널티 30초/회'.")
    P("     접촉 1회 = 30초 손실 = 전체 주행시간의 상당 부분을 날림.")

    # ==================================================================
    # 접촉 패널티 민감도
    # ==================================================================
    P("\n" + "─" * 78)
    P("[패널티 영향] 접촉 1회 = 30초. 기록 대비 얼마나 치명적인가")
    P("─" * 78)
    best = min(results.values(), key=lambda r: r["score"])
    P(f"  최선 조합 주행시간 ≈ {best['t']:.0f}초")
    for n in [0, 1, 2, 3, 5]:
        P(f"    접촉 {n}회 → 기록 {best['t'] + n*PENALTY_PER_HIT:6.1f}초 "
          f"(순수주행 대비 +{n*PENALTY_PER_HIT/best['t']*100:5.1f}%)")
    P("  → 접촉 3회면 기록이 거의 2배. 속도보다 '안 부딪히는 것'이 압도적으로 중요.")

    # ==================================================================
    # 그림
    # ==================================================================
    key = (3.0, 3.0) if (3.0, 3.0) in results else list(results)[0]
    r = results[key]
    c = r["course"]

    fig, ax = plt.subplots(figsize=(15, 5.2))
    ax.add_patch(plt.Rectangle((0, 0), FIELD_L, FIELD_W, fill=False, ec='k', lw=2))
    # 게이트
    for (a, b) in c["gates"]:
        ax.add_patch(plt.Circle(a, BUOY_R * 3, color='red'))
        ax.add_patch(plt.Circle(b, BUOY_R * 3, color='green'))
    # WP
    for wp, lbl in [(c["wp1"], "WP1"), (c["wp2"], "WP2")]:
        ax.add_patch(plt.Circle(wp, HOLD_RADIUS, color='yellow', alpha=0.18))
        ax.add_patch(plt.Circle(wp, BUOY_R * 3, color='gold'))
        ax.text(wp[0], wp[1] + 1.2, lbl, ha='center', fontsize=9)
    # 도크
    for d in c["docks"]:
        ax.add_patch(plt.Rectangle((d[0] - .8, d[1] - .8), 1.6, 1.6, color='navy', alpha=.7))
    ax.text(c["docks"][1][0], c["docks"][1][1] + 2.2, "DOCK", ha='center', fontsize=9)
    # 탐색
    ax.add_patch(plt.Circle(c["search"], BUOY_R * 3, color='limegreen'))
    ax.add_patch(plt.Circle(c["search"], 3.0, fill=False, ls='--', ec='navy'))
    ax.text(c["search"][0], c["search"][1] + 3.5, "SEARCH(orbit)", ha='center', fontsize=9)
    for p in c["search_others"]:
        ax.add_patch(plt.Circle(p, BUOY_R * 3, color='white', ec='gray'))
    # 장애물 + 경계선
    for (bx, by, br) in c["obst"]:
        ax.add_patch(plt.Circle((bx, by), br * 3, color='orange'))
    x0, y0, x1, y1 = c["fence"]
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                               ec='y', ls='--', lw=2))
    ax.text((x0 + x1) / 2, y1 - 0.8, "virtual fence", ha='center', color='olive', fontsize=8)
    # 궤적
    for tr in r["legs"]:
        if len(tr):
            ax.plot(tr[:, 0], tr[:, 1], 'b-', lw=1.8)
    ax.plot(*c["start"], 'go', ms=10)
    ax.text(c["start"][0], c["start"][1] - 1.6, "START", ha='center', fontsize=9)
    ax.plot(*c["goal"], 'k*', ms=16)
    ax.text(c["goal"][0], c["goal"][1] + 1.4, "GOAL", ha='center', fontsize=9)
    ax.set_xlim(-2, FIELD_L + 2); ax.set_ylim(-3, FIELD_W + 2)
    ax.set_aspect('equal')
    ax.set_title(f"KABOAT 2025 Course 70x20m — det {key[0]}m, gate {key[1]}m | "
                 f"time {r['t']:.0f}s + penalty {r['pen']:.0f}s ({r['hits']} hits) "
                 f"= {r['score']:.0f}s")
    ax.grid(alpha=.25)
    plt.tight_layout()
    plt.savefig("kaboat_course_result.png", dpi=130)
    P("\n[그림] kaboat_course_result.png 저장")
    P("=" * 78)

    txt = "\n".join(R)
    print(txt)
    open("kaboat_course_report.txt", "w").write(txt + "\n")


if __name__ == "__main__":
    main()

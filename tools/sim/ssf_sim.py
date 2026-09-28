#!/usr/bin/env python3
"""
SSF 자율운항선박 - 로직 시뮬레이터 (ROS2 불필요)
=================================================
실제 최종본(ros2_ws) 노드의 알고리즘을 그대로 이식해서
가상 부표 / 가상 LiDAR / 가상 IMU 로 구동하고, 궤적을 그려 검증한다.

이식한 실제 로직:
  - ship_direction.scan_callback  (LiDAR 갭 팔로잉 + 장애물 팽창)
  - ship_goal_angle               (yaw_error = (goal - yaw) % 360)
  - ship_direction.yaw_error_cb   ((360 - raw) % 360  ← 방향 뒤집기)
  - motor_control.timer_callback  (각도 → PWM 테이블)
  - Arduino 펌웨어 매핑            (입력 1500 = 중립)

좌표/각도 규약 (실제 코드와 동일):
  - 배 정면 = 스캔각 80도.  스캔각은 반시계(CCW=좌현) 방향으로 증가.
  - 방위(heading, psi) = 나침반 각 (0=북, 시계방향 CW 증가)
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ============================================================
# 1. 실제 노드 파라미터 (final version 그대로)
# ============================================================
CANDIDATE_INVALID = 20000.0
STOP_HOLD = 50000.0

# --- ship_direction ---
BASE_DETECTION_DISTANCE = 1.8
HALF_WIDTH = 0.29           # 1번배 설계 폭 0.58m ÷ 2 (yaml 과 동일, 줄자 실측 전)
CLEARANCE = 0.25            # yaml clearance 와 동일 (2026-09-23 동기화)
BORDER_MARGIN = 2
MAX_SPIKE_RATIO = 0.01
REAR_OBSTACLE_IGNORE_MARGIN = None   # 노드에서 제거됨(증명된 결함, ship_direction.py [C]). None = 비활성. 재현 실험용으로만 숫자 지정
MIN_OBSTACLE_CELLS = 1     # yaml min_obstacle_cells 와 동일 (2026-09-23 동기화)

# --- motor_control ---
BASE_PWM = 1360
MAX_ANGLE = 81
MAX_DIFF = 100
TURN_OFFSET = -60
PWM_NEUTRAL = 1500
PWM_REVERSE = 1590

# --- LiDAR 스캔 규격 (RPLIDAR A3 유사) ---
SCAN_ANGLE_MIN_DEG = -180.0
SCAN_INC_DEG = 0.5
SCAN_N = int(360.0 / SCAN_INC_DEG)

# ============================================================
# 2. ship_direction 로직 (실제 코드 이식)
# ============================================================
def smooth_spikes(binary):
    total_len = len(binary)
    max_spike_length = int(total_len * MAX_SPIKE_RATIO)
    smoothed = binary[:]
    count, start = 0, None
    for i, v in enumerate(binary):
        if v == 1:
            if start is None:
                start = i
            count += 1
        else:
            if start is not None and count <= max_spike_length:
                for j in range(start, i):
                    smoothed[j] = 0
            start, count = None, 0
    if start is not None and count <= max_spike_length:
        for j in range(start, len(binary)):
            smoothed[j] = 0
    return smoothed


def suppress_spike_edges(binary):
    suppressed = binary[:]
    for i, v in enumerate(binary):
        if v == 1:
            for off in range(-BORDER_MARGIN, BORDER_MARGIN + 1):
                j = i + off
                if 0 <= j < len(binary) and binary[j] == 0:
                    suppressed[i] = 0
                    break
    return suppressed


def dilate_obstacles(binary, distance_array, angle_increment_deg, detection_distance):
    n = len(binary)
    out = binary[:]
    i = 0
    while i < n:
        if binary[i] == 1:
            start = i
            while i + 1 < n and binary[i + 1] == 1:
                i += 1
            end = i
            length = end - start + 1
            if length >= MIN_OBSTACLE_CELLS:
                r_edge = detection_distance
                for k in range(start, end + 1):
                    r = distance_array[k]
                    if not math.isinf(r) and not math.isnan(r):
                        r_edge = min(r_edge, r)
                lateral = HALF_WIDTH + CLEARANCE
                r_use = max(r_edge, 0.01)
                ang_margin = math.degrees(math.atan(lateral / r_use))
                cells_margin = max(1, int(round(ang_margin / max(angle_increment_deg, 1e-6))))
                new_start = max(0, start - cells_margin)
                new_end = min(n - 1, end + cells_margin)
                for j in range(new_start, new_end + 1):
                    out[j] = 1
            i += 1
        else:
            i += 1
    return out


def ship_direction_step(ranges, angle_min_deg, inc_deg, yaw_error_sd,
                        detection_distance=BASE_DETECTION_DISTANCE,
                        debug=None):
    """실제 scan_callback의 (C) 경로 = candidate 20000(자율회피) 상태를 재현."""
    inc_rad = math.radians(inc_deg)
    min_index = int((0 - angle_min_deg) / inc_deg)
    max_index = int((160 - angle_min_deg) / inc_deg)
    sub_ranges = ranges[min_index:max_index + 1]

    angle_array, distance_array = [], []
    front_min_distance = float('inf')
    for i, r in enumerate(sub_ranges):
        ang = angle_min_deg + (min_index + i) * inc_deg
        angle_array.append(ang)
        distance_array.append(r)
        if not math.isinf(r) and not math.isnan(r):
            front_min_distance = min(front_min_distance, r)

    # 이진 장애물 마스크
    binary = []
    for r in distance_array:
        if math.isinf(r) or math.isnan(r):
            binary.append(0)
        elif r < detection_distance:
            if (REAR_OBSTACLE_IGNORE_MARGIN is not None
                    and abs(r - front_min_distance) > REAR_OBSTACLE_IGNORE_MARGIN):
                binary.append(0)   # 작년 로직 재현용. 노드는 이 분기 없음
            else:
                binary.append(1)
        else:
            binary.append(0)

    raw_hits = sum(binary)
    binary = smooth_spikes(binary)
    after_smooth = sum(binary)
    binary = suppress_spike_edges(binary)
    after_suppress = sum(binary)
    binary = dilate_obstacles(binary, distance_array, inc_deg, detection_distance)
    after_dilate = sum(binary)

    if debug is not None:
        debug.update(dict(raw_hits=raw_hits, after_smooth=after_smooth,
                          after_suppress=after_suppress, after_dilate=after_dilate))

    # 안전구역
    safe_zones, start = [], None
    for i, v in enumerate(binary):
        if v == 0 and start is None:
            start = i
        elif v == 1 and start is not None:
            safe_zones.append((start, i - 1))
            start = None
    if start is not None:
        safe_zones.append((start, len(binary) - 1))

    valid_safe_zones = []
    min_required_width = HALF_WIDTH * 2 + CLEARANCE
    for s, e in safe_zones:
        r_s = distance_array[s] if np.isfinite(distance_array[s]) else detection_distance
        r_e = distance_array[e] if np.isfinite(distance_array[e]) else detection_distance
        r_edge = min(r_s, r_e)
        arc_len = inc_rad * r_edge * (e - s)
        if arc_len >= min_required_width:
            valid_safe_zones.append((s, e, r_edge))

    # 목표 방위 → 스캔 프레임 매핑
    yaw_raw = yaw_error_sd % 360.0
    yaw_mapped = 80 + yaw_raw if yaw_raw <= 180 else 80 - (360 - yaw_raw)

    candidates = []
    for s, e, r_edge in valid_safe_zones:
        zone_min, zone_max = angle_array[s], angle_array[e]
        if yaw_mapped < zone_min:
            best_angle = zone_min
        elif yaw_mapped > zone_max:
            best_angle = zone_max
        else:
            best_angle = yaw_mapped
        angle_diff = abs(best_angle - yaw_mapped)
        arc_length = inc_rad * r_edge * (e - s)
        candidates.append((angle_diff, -arc_length, best_angle, s, e))

    if not candidates:
        return 260.0, yaw_mapped   # 회피 경로 없음 → 후진
    candidates.sort(key=lambda x: (x[0], x[1]))
    return candidates[0][2], yaw_mapped


# ============================================================
# 3. motor_control 로직 (실제 코드 이식)
# ============================================================
def linear_diff(offset):
    off = max(-MAX_ANGLE, min(MAX_ANGLE, offset))
    return int((abs(off) / MAX_ANGLE) * MAX_DIFF)


def motor_control_step(angle):
    if angle >= 50000:                       # (1) STOP
        return PWM_NEUTRAL, PWM_NEUTRAL, "STOP"
    elif 20000 <= angle < 50000:             # (2) fallback 전진
        return BASE_PWM, BASE_PWM, "FALLBACK_FWD"
    elif 5000 <= angle < 20000:              # (3) 느린 좌선회
        d = linear_diff(TURN_OFFSET)
        return BASE_PWM - d, BASE_PWM + d, "SPIN_LEFT"
    elif -1.0 <= angle <= 161.0:             # (4) 정상 조향
        offset = angle - 80.0
        d = linear_diff(offset)
        if offset > 0:
            return BASE_PWM - d, BASE_PWM + d, "STEER_LEFT"
        else:
            return BASE_PWM + d, BASE_PWM - d, "STEER_RIGHT"
    elif 161.0 < angle < 5000.0:             # (5) 후진 (260 포함)
        return PWM_REVERSE, PWM_REVERSE, "REVERSE"
    else:                                    # (6) 직진
        return BASE_PWM, BASE_PWM, "STRAIGHT"


# ============================================================
# 4. IMU / 방위 오차 (ship_goal_angle + ship_direction 뒤집기)
# ============================================================
def yaw_error_chain(goal_bearing, current_yaw):
    e_sga = (goal_bearing - current_yaw) % 360.0     # ship_goal_angle
    e_sd = (360.0 - e_sga) % 360.0                   # ship_direction 뒤집기
    return e_sd


# ============================================================
# 5. 가상 세계 (부표 + LiDAR 광선투사 + 배 동역학)
# ============================================================
class World:
    def __init__(self, buoys):
        self.buoys = buoys   # [(x, y, radius), ...]

    def lidar_scan(self, x, y, psi, noise_spike=None, max_range=12.0):
        """psi = 나침반 방위(0=북, CW+). 스캔각 80=정면, CCW 증가."""
        ranges = np.full(SCAN_N, np.inf)
        for i in range(SCAN_N):
            a_scan = SCAN_ANGLE_MIN_DEG + i * SCAN_INC_DEG   # 스캔 프레임 각
            # 스캔각 → 배 기준 상대각(CCW+) → 세계 나침반 방위
            rel_ccw = a_scan - 80.0
            world_bearing = psi - rel_ccw
            th = math.radians(world_bearing)
            dx, dy = math.sin(th), math.cos(th)   # 나침반: 0=북(+y)
            best = np.inf
            for (bx, by, br) in self.buoys:
                ox, oy = bx - x, by - y
                t_ca = ox * dx + oy * dy
                if t_ca < 0:
                    continue
                d2 = (ox * ox + oy * oy) - t_ca * t_ca
                if d2 > br * br:
                    continue
                t_hc = math.sqrt(br * br - d2)
                t = t_ca - t_hc
                if 0 < t < best and t < max_range:
                    best = t
            ranges[i] = best
        if noise_spike is not None:
            idx, val = noise_spike
            ranges[idx] = val
        return ranges


def bearing_to(x, y, gx, gy):
    return math.degrees(math.atan2(gx - x, gy - y)) % 360.0


def simulate(world, start=(0.0, 0.0), start_psi=0.0, goal=(0.0, 20.0),
             dt=0.1, steps=700, k_v=0.010, k_w=0.22, stop_radius=1.0,
             noise_spike=None):
    x, y, psi = start[0], start[1], start_psi
    traj, pwms, angles, modes = [], [], [], []
    for _ in range(steps):
        if math.hypot(goal[0] - x, goal[1] - y) < stop_radius:
            break
        ranges = world.lidar_scan(x, y, psi, noise_spike=noise_spike)
        gb = bearing_to(x, y, goal[0], goal[1])
        e_sd = yaw_error_chain(gb, psi)
        desired, _ = ship_direction_step(list(ranges), SCAN_ANGLE_MIN_DEG,
                                         SCAN_INC_DEG, e_sd)
        pwm_r, pwm_l, mode = motor_control_step(desired)

        # 펌웨어: 입력 1500 = 중립. 추력 ∝ (1500 - pwm)
        TR = PWM_NEUTRAL - pwm_r
        TL = PWM_NEUTRAL - pwm_l
        v = k_v * (TR + TL) / 2.0
        dpsi = k_w * (TL - TR) * dt      # TL>TR → 우선회(CW, psi 증가)

        psi = (psi + dpsi) % 360.0
        th = math.radians(psi)
        x += v * math.sin(th) * dt
        y += v * math.cos(th) * dt

        traj.append((x, y))
        pwms.append((pwm_r, pwm_l))
        angles.append(desired)
        modes.append(mode)
    return np.array(traj), np.array(pwms), np.array(angles), modes


# ============================================================
# 6. 시나리오 & 검증
# ============================================================
def make_scan_with_buoy(dist, half_angle_deg=3.0, center_scan_deg=80.0):
    """정면(80도) 근처에 폭 half_angle*2 인 장애물을 놓은 가짜 스캔."""
    ranges = np.full(SCAN_N, np.inf)
    for i in range(SCAN_N):
        a = SCAN_ANGLE_MIN_DEG + i * SCAN_INC_DEG
        if abs(a - center_scan_deg) <= half_angle_deg:
            ranges[i] = dist
    return ranges


def main():
    report = []
    R = report.append
    R("=" * 66)
    R("SSF 자율운항 로직 시뮬레이터 — 검증 리포트")
    R("=" * 66)

    # ---------- 검증 1: 조향 부호 ----------
    R("\n[검증 1] 조향 부호 (장애물 없음, 목표만 좌/우에 있을 때)")
    for label, goal_b, yaw in [("목표가 우측(+30도)", 30.0, 0.0),
                               ("목표가 좌측(-30도)", 330.0, 0.0)]:
        e = yaw_error_chain(goal_b, yaw)
        clear = np.full(SCAN_N, np.inf)
        ang, ymap = ship_direction_step(list(clear), SCAN_ANGLE_MIN_DEG, SCAN_INC_DEG, e)
        pr, pl, mode = motor_control_step(ang)
        turn = "좌회전" if mode == "STEER_LEFT" else ("우회전" if mode == "STEER_RIGHT" else mode)
        R(f"  {label}: yaw_err={e:6.1f} → desired={ang:6.1f} → {mode:12s} ({turn})  R={pr} L={pl}")
    R("  → 목표가 우측이면 우회전, 좌측이면 좌회전이어야 정상.")

    # ---------- 검증 2: 특수값(정지/후진/회전) ----------
    R("\n[검증 2] 특수 신호 → PWM (펌웨어 중립=1500)")
    for a, name in [(50000.0, "STOP_HOLD(50000)"), (260.0, "후진(260)"),
                    (5000.0, "회전(5000)"), (20000.0, "폴백(20000)"),
                    (80.0, "정면(80)")]:
        pr, pl, mode = motor_control_step(a)
        R(f"  {name:20s} → R={pr} L={pl}  [{mode}]")
    R("  → STOP은 1500/1500(중립), 후진은 1590/1590 이어야 정상.")

    # ---------- 검증 3: 작은 부표가 침식으로 지워지는가 ----------
    R("\n[검증 3] 부표 크기별 탐지 여부 (침식 suppress_spike_edges 영향)")
    for half in [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]:
        ranges = make_scan_with_buoy(1.2, half_angle_deg=half)
        dbg = {}
        ship_direction_step(list(ranges), SCAN_ANGLE_MIN_DEG, SCAN_INC_DEG, 0.0, debug=dbg)
        n_cells = int(half * 2 / SCAN_INC_DEG) + 1
        survived = "탐지 O" if dbg['after_dilate'] > 0 else "★ 탐지 실패(지워짐)"
        R(f"  부표 각폭 ±{half:3.1f}도({n_cells:2d}셀): raw={dbg['raw_hits']:3d} "
          f"→smooth={dbg['after_smooth']:3d} →침식={dbg['after_suppress']:3d} "
          f"→팽창={dbg['after_dilate']:3d}  {survived}")
    R("  → 침식(suppress) 단계에서 작은 부표가 0이 되면 그대로 사라진다.")

    # ---------- 검증 4: 노이즈 스파이크가 진짜 장애물을 가리는가 ----------
    R("\n[검증 4] 근거리 노이즈 1점이 먼 장애물을 무시하게 만드는가")
    ranges = make_scan_with_buoy(1.6, half_angle_deg=4.0)   # 1.6m 앞 실제 부표
    dbg0 = {}
    ship_direction_step(list(ranges), SCAN_ANGLE_MIN_DEG, SCAN_INC_DEG, 0.0, debug=dbg0)
    R(f"  노이즈 없음 : 장애물 셀 raw={dbg0['raw_hits']}, 최종={dbg0['after_dilate']}")
    noisy = ranges.copy()
    noisy[int((10 - SCAN_ANGLE_MIN_DEG) / SCAN_INC_DEG)] = 0.3   # 0.3m 물보라
    dbg1 = {}
    ship_direction_step(list(noisy), SCAN_ANGLE_MIN_DEG, SCAN_INC_DEG, 0.0, debug=dbg1)
    R(f"  노이즈 0.3m 1점 추가: 장애물 셀 raw={dbg1['raw_hits']}, 최종={dbg1['after_dilate']}")
    if dbg1['raw_hits'] < dbg0['raw_hits']:
        R("  ★ 확인됨: 근거리 노이즈 하나 때문에 진짜 부표가 마스크에서 사라짐")
        R("     (원인: rear_obstacle_ignore_margin — 최근접과 1m 이상 차이나면 무시)")

    # ---------- 시나리오 A: 부표 회피 주행 ----------
    buoys = [(-0.6, 7.0, 0.25), (1.2, 11.0, 0.25), (-1.0, 15.0, 0.25)]
    w = World(buoys)
    traj, pwms, angles, modes = simulate(w, start=(0, 0), start_psi=0.0, goal=(0, 20))

    # ---------- 시나리오 B: 장애물 없음 (부호 검증 주행) ----------
    w2 = World([])
    traj2, _, _, _ = simulate(w2, start=(0, 0), start_psi=90.0, goal=(0, 20))

    R("\n[시나리오 A] 부표 3개 회피 주행")
    if len(traj):
        R(f"  스텝 {len(traj)}, 최종위치=({traj[-1][0]:.2f}, {traj[-1][1]:.2f}), 목표=(0.00, 20.00)")
        d_min = min(math.hypot(px - bx, py - by) - br
                    for (px, py) in traj for (bx, by, br) in buoys)
        R(f"  부표 표면까지 최소 접근거리 = {d_min:.2f} m  (배 반폭 0.45m)")
        R(f"  {'✔ 충돌 없이 회피' if d_min > 0.45 else '★ 배 폭 안으로 접근(충돌 위험)'}")
        from collections import Counter
        R(f"  모드 분포: {dict(Counter(modes))}")
    R("\n[시나리오 B] 장애물 없음, 초기 방위 90도(동) → 목표는 북쪽")
    if len(traj2):
        R(f"  최종위치=({traj2[-1][0]:.2f}, {traj2[-1][1]:.2f})  → 목표(0,20)로 수렴하면 부호 정상")

    # ---------- 그림 ----------
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    ax = axes[0]
    for (bx, by, br) in buoys:
        ax.add_patch(plt.Circle((bx, by), br, color='crimson', alpha=0.8))
        ax.add_patch(plt.Circle((bx, by), br + HALF_WIDTH + CLEARANCE,
                                color='crimson', alpha=0.12))
    if len(traj):
        ax.plot(traj[:, 0], traj[:, 1], 'b-', lw=2, label='Boat path')
        ax.plot(traj[0, 0], traj[0, 1], 'go', ms=9, label='Start')
    ax.plot(0, 20, 'k*', ms=16, label='Goal')
    ax.set_title("Scenario A: Buoy Avoidance")
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(); ax.set_xlim(-5, 5)

    ax = axes[1]
    if len(traj2):
        ax.plot(traj2[:, 0], traj2[:, 1], 'b-', lw=2)
        ax.plot(traj2[0, 0], traj2[0, 1], 'go', ms=9, label='Start (heading 90=E)')
    ax.plot(0, 20, 'k*', ms=16, label='Goal (North)')
    ax.set_title("Scenario B: Steering Sign Check")
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend()

    ax = axes[2]
    if len(pwms):
        ax.plot(pwms[:, 0], label='PWM R')
        ax.plot(pwms[:, 1], label='PWM L')
        ax.axhline(1500, color='k', ls='--', lw=1, label='Neutral 1500')
        ax.axhline(BASE_PWM, color='g', ls=':', lw=1, label='Cruise 1360')
    ax.set_title("Scenario A: Motor PWM"); ax.set_xlabel("step")
    ax.grid(alpha=.3); ax.legend()

    plt.tight_layout()
    plt.savefig("ssf_sim_result.png", dpi=130)
    R("\n[그림] ssf_sim_result.png 저장 완료")
    R("=" * 66)

    text = "\n".join(report)
    print(text)
    with open("ssf_sim_report.txt", "w") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    main()

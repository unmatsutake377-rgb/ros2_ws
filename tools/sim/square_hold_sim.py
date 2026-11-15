#!/usr/bin/env python3
"""위치유지 사각형 시뮬 — 남색 부표 4개(5×5 m) + 실제 노드 코드(north·ship_goal_angle·ship_direction·motor_control·ship_back).

계약서: study_square_sim/CONTRACT_v2_ship_back_square.md 변경 단위 (b)(시뮬 = (d) 에 커밋).
  · full_course_sim.py 의 스텁·노드 로드·운동 모델(K_V·K_W·τ 0.5 s)·LiDAR 규약(SCAN_REL)을 그대로 쓴다.
  · ship_back 은 hold_source=square(코드 기본). /buoy_bearings 는 새 검출기 모형(OAK 63.9°, 0.5~12 m, 방위 잡음,
    color_shape_detector.stop_detect_logic.pack_bearings 로 패킹, 10 Hz), 거리는 LiDAR(/scan) 레이캐스트.
  · /ublox_gps_node/fix 1 Hz(σ, RTK status 2) → north·ship_back. /imu/yaw_raw = 참 헤딩 + 임의 기준 + 드리프트·잡음.
  · /imu/yaw: --heading true(참값) | gps_cog(실제 ssf_heading.heading_logic.HeadingMux(gps_cog) — 0.5 m/s 미만 HOLD,
    선회·후진 IMU Δ 보조 10 s 상한, COG 잡음 --cog-noise). full_course_sim 의 gps_cog_real 과 같은 배선.
  · 웨이포인트: WP0 = 사각형 중심(mode 2, dwell 0.5 — 대회 WP2 와 같은 값), WP1 = 15 m 뒤(mode 5).
    출발 = 중심에서 --start-dist m(무작위 방향 — 꼭짓점 쪽 진입 포함), 헤딩 = 중심 ±20°, 속도 순항.
  · 부표: 사각형 회전 무작위, --drift m 이면 부표마다 반경 drift 원 안 무작위 이동(고정 표류).

지표 (시뮬 정의, 실측 아님)
  · 참 성공 = mode 2 동안 선체(1.77 × 0.57 직사각형) 네 모서리가 **참 부표 사각형** 안(부표 반지름만큼 안쪽)에 연속 5 s.
  · 노드 DONE = ship_back SquareFSM 이 DONE(성공) — 그 순간 참 내부 연속 시간도 적는다(거짓 성공 = DONE 인데 참 밖).
  · 포기 = ship_back ERROR '포기'. 접촉 = 직사각형 선체 ↔ 부표 원(반지름 --buoy-r) 겹침 시작 횟수.
  · 조기 넘김 = north 가 ship_back DONE 전에 WP0(사각형) 을 넘김. 20000 위반 = INIT·DONE 밖에서 20000 을 낸 틱 수.
  · 출력 위반 = square_logic.output_allowed 밖 값을 낸 틱 수.

재현:  python3 tools/sim/square_hold_sim.py --seeds 20
       python3 tools/sim/square_hold_sim.py --seeds 20 --heading gps_cog --drift 1.0 --cog-noise 0.05
       python3 tools/sim/square_hold_sim.py --seeds 20 --heading gps_cog --param ship_back.heading_use_raw=false
의존: numpy, pyyaml. ROS 불필요. ⚠️ 운동·센서·부표 치수는 가정값.
"""
import argparse
import math
import os
import random
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:                                              # 그림은 안 그린다 — matplotlib 이 없는 환경에서도 돌게 빈 모듈
    import matplotlib                             # noqa: F401
except ImportError:
    import types as _types
    _mpl = _types.ModuleType("matplotlib"); _mpl.use = lambda *a, **k: None
    _mpl.pyplot = _types.ModuleType("matplotlib.pyplot")
    sys.modules["matplotlib"] = _mpl; sys.modules["matplotlib.pyplot"] = _mpl.pyplot
import full_course_sim as F                       # noqa: E402  (스텁·노드 로드·상수)
import rosstub                                    # noqa: E402
from rosstub import Clock                         # noqa: E402

SRC = F.SRC
sys.path.insert(0, os.path.join(SRC, "ssf_heading"))
sys.path.insert(0, os.path.join(SRC, "color_shape_detector"))
sys.path.insert(0, os.path.join(SRC, "ship_back"))
from ssf_heading import heading_logic as HL                       # noqa: E402
from color_shape_detector.stop_detect_logic import pack_bearings  # noqa: E402
from ship_back.square_logic import output_allowed                 # noqa: E402

std = sys.modules["std_msgs.msg"]
Float32, Float64, Int32, Float32MultiArray = std.Float32, std.Float64, std.Int32, std.Float32MultiArray

DT = F.DT
HFOV = 63.9                 # vision.yaml hfov_deg (OAK-1)
CAM_MIN, CAM_MAX = 0.5, 12.0
LIDAR_MAX = 20.0
HULL_L, HULL_B = 1.77, 0.57  # 공용 보수값(ship_back 기본과 같음)
SIDE = 5.0
LAT0, LON0 = 35.18688, 128.56604


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def lidar_ranges(buoys, br, x, y, psi):
    th = np.radians(psi - F.SCAN_REL)
    dx, dy = np.sin(th), np.cos(th)
    bx = np.array([b[0] for b in buoys]) - x
    by = np.array([b[1] for b in buoys]) - y
    t_ca = dx[:, None] * bx[None, :] + dy[:, None] * by[None, :]
    d2 = (bx ** 2 + by ** 2)[None, :] - t_ca ** 2
    hit = (d2 <= br * br) & (t_ca > 0)
    t = np.where(hit, t_ca - np.sqrt(np.maximum(br * br - d2, 0.0)), np.inf)
    r = t.min(axis=1)
    r[r > LIDAR_MAX] = np.inf
    return r


def hull_corners(x, y, psi):
    h = math.radians(psi)
    f = (math.sin(h), math.cos(h)); rt = (math.cos(h), -math.sin(h))
    return [(x + sf * HULL_L / 2 * f[0] + sr * HULL_B / 2 * rt[0], y + sf * HULL_L / 2 * f[1] + sr * HULL_B / 2 * rt[1])
            for sf, sr in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def hull_buoy_clear(x, y, psi, bx, by, br):
    s, c = math.sin(math.radians(psi)), math.cos(math.radians(psi))
    dx, dy = bx - x, by - y
    fwd = dx * s + dy * c
    lat = dx * c - dy * s
    qx = min(max(fwd, -HULL_L / 2), HULL_L / 2)
    qy = min(max(lat, -HULL_B / 2), HULL_B / 2)
    return math.hypot(fwd - qx, lat - qy) - br


def inside_truth(quad, x, y, psi, inset):
    q = sorted(quad, key=lambda p: math.atan2(p[1] - sum(a[1] for a in quad) / 4, p[0] - sum(a[0] for a in quad) / 4))
    for cx, cy in hull_corners(x, y, psi):
        for i in range(4):
            p, r = q[i], q[(i + 1) % 4]
            ex, ey = r[0] - p[0], r[1] - p[1]
            if (ex * (cy - p[1]) - ey * (cx - p[0])) / math.hypot(ex, ey) < inset:
                return False
    return True


def write_wps(geo, center, exit_xy):
    lat_c, lon_c = geo.ll(*center)
    lat_e, lon_e = geo.ll(*exit_xy)
    fd, path = tempfile.mkstemp(prefix="square_wp_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write("waypoints:\n")
        f.write(f'  - {{구역: "WP2 위치유지(사각형 중심)", lat: {lat_c:.9f}, lon: {lon_c:.9f}, mode: 2, dwell: 0.5}}\n')
        f.write(f'  - {{구역: "다음", lat: {lat_e:.9f}, lon: {lon_e:.9f}, mode: 5, dwell: 1.0}}\n')
    return path


def run(seed, heading="gps_cog", drift=0.0, cog_noise=0.05, gps_sigma=0.03, buoy_r=0.25, start_dist=(18.0, 28.0),
        current=(0.0, 0.0), t_max=200.0, overrides=None, imu_drift_dps=0.3, cam_noise=0.5, verbose=False, trace=None,
        random_heading=False):
    """한 런. trace = 리스트면 mode 2 틱마다 (t, 상태, 출력, x, y, 참헤딩, 추정헤딩, 확정 부표 수, 최근접) 을 붙인다."""
    rng = random.Random(seed)
    geo = F.Geo(LAT0, LON0)
    rot = rng.uniform(0.0, 90.0)
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    buoys = []
    for ux, uy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        bx, by = ux * SIDE / 2, uy * SIDE / 2
        x0, y0 = bx * c - by * s, bx * s + by * c
        if drift > 0.0:
            a, rr = rng.uniform(0, 2 * math.pi), drift * math.sqrt(rng.random())
            x0, y0 = x0 + rr * math.cos(a), y0 + rr * math.sin(a)
        buoys.append((x0, y0))
    ang = rng.uniform(0.0, 360.0)
    d0 = rng.uniform(*start_dist)
    x, y = d0 * math.sin(math.radians(ang)), d0 * math.cos(math.radians(ang))
    psi = (math.degrees(math.atan2(-x, -y)) + rng.uniform(-20, 20)) % 360.0
    v0 = None
    if random_heading:                              # 정지·아무 방향(사각형 가까이에서 시작 — 도리도리 경로)
        psi = rng.uniform(0.0, 360.0); v0 = 0.0
    exit_xy = (-x * 15.0 / d0, -y * 15.0 / d0)
    wp = write_wps(geo, (0.0, 0.0), exit_xy)

    rosstub.PARAMS.clear()
    rosstub.PARAMS["north_goal_angle"] = dict(F.P_NGA, waypoints_file=wp)
    rosstub.PARAMS["ship_goal_angle"] = dict(F.P_SGA)
    for n in ("ship_direction", "motor_control"):
        rosstub.PARAMS[n] = {}
    rosstub.PARAMS["ship_back"] = {"active_wp_modes": [2]}
    for node, kv in (overrides or {}).items():
        rosstub.PARAMS.setdefault(node, {}).update(kv)
    Clock.t = 0.0
    nga = F.NGA.NorthGoalAngle(); sga = F.SGA.ShipGoalAngle(); sd = F.SD.ShipDirection()
    mc = F.MC.MotorController(); sb = F.SB.ShipBack()
    nga.boat_mode_cb(Int32(2))
    hmux = None
    if heading == "gps_cog":
        hmux = HL.HeadingMux(HL.SRC_GPS_COG, gps_cog=HL.GPSCOGTracker(min_speed_mps=0.5))
        hmux.update_boat_mode(2)
    irng = random.Random(50_000 + seed)
    imu_base = irng.uniform(0, 360)       # IMU 상대 yaw 기준(임의)
    imu_rate = irng.uniform(-imu_drift_dps, imu_drift_dps)
    imu_drift = 0.0
    v = (F.K.CRUISE if hasattr(F.K, "CRUISE") else 1.2) if v0 is None else v0
    w = 0.0
    t = 0.0; k = 0
    pub_mode = -1
    res = dict(seed=seed, gt_ok=False, gt_t=None, done=False, done_t=None, done_inside=None, giveup=False,
               contacts=0, early=False, inv20=0, bad_out=0, min_clear=math.inf, rev_ticks=0, spin_ticks=0,
               states={}, max_center_err=None, center_err=None, head_err=[], wp_adv_t=None, reason="")
    contact_on = [False] * 4
    gt_run = 0.0
    done_seen = False
    try:
        while t < t_max:
            # ---- 센서 ----
            scan = F.Scan(lidar_ranges(buoys, buoy_r, x, y, psi).tolist())
            sd.scan_callback(scan); sb.scan_cb(scan)
            imu_drift += (imu_rate if abs(v) > 0.3 else 0.07) * DT
            raw = (psi - imu_base + imu_drift + irng.gauss(0, 0.2)) % 360.0
            sb.yaw_raw_cb(Float64(raw))
            yaw_v = psi
            if hmux is not None:
                hmux.update_imu(raw, t)
                mr = mc.pubs["/motor_reverse"].last
                if mr is not None:
                    hmux.update_reverse(bool(mr.data), t)
                da = sd.pubs["/desired_angle"].last
                if da is not None:
                    hmux.update_desired_angle(float(da.data), t)
                tp = nga.pubs["/north_goal_angle_tp"].last
                if tp is not None:
                    hmux.update_goal_bearing(float(tp.data), t)
                if k % 10 == 3:
                    vx = v * math.sin(math.radians(psi)) + current[0] + rng.gauss(0, cog_noise)
                    vy = v * math.cos(math.radians(psi)) + current[1] + rng.gauss(0, cog_noise)
                    cog, spd = HL.cog_from_velocity(vx, vy, "enu")
                    hmux.update_cog(cog, spd, t)
                hy, hs = hmux.heading(t)
                yaw_v = hy if (hy is not None and hs in HL.PUBLISH_STATES) else None
            if yaw_v is not None:
                ym = Float64(yaw_v)
                nga.yaw_cb(ym); sga.yaw_callback(ym); sb.yaw_cb(ym)
            if k % 10 == 3:
                lat, lon = geo.ll(x + rng.gauss(0, gps_sigma), y + rng.gauss(0, gps_sigma))
                fx = F.NavSatFix(lat, lon, cov=max(gps_sigma, 1e-3) ** 2)
                nga.gps_cb(fx); sb.fix_cb(fx)
            # ---- north 0.5 s ----
            if k % 5 == 0:
                c0 = {p: nga.pubs[p].count for p in ("/wp_mode", "/north_goal_angle_tp", "/geofence_state",
                                                     "/goal_distance", "/candidate_angle")}
                wp_before = nga.wp_idx
                nga.timer_cb()
                if nga.pubs["/goal_distance"].count > c0["/goal_distance"]:
                    gd = nga.pubs["/goal_distance"].last
                    mc.goal_distance_cb(gd); sb.goal_distance_cb(gd)
                if nga.pubs["/wp_mode"].count > c0["/wp_mode"]:
                    m = nga.pubs["/wp_mode"].last
                    sd.wp_mode_cb(m); sb.wp_cb(m); pub_mode = m.data
                if nga.pubs["/north_goal_angle_tp"].count > c0["/north_goal_angle_tp"]:
                    sga.goal_angle_callback(nga.pubs["/north_goal_angle_tp"].last)
                if nga.pubs["/geofence_state"].count > c0["/geofence_state"]:
                    sd.geofence_cb(nga.pubs["/geofence_state"].last)
                if nga.pubs["/candidate_angle"].count > c0["/candidate_angle"]:
                    nc = nga.pubs["/candidate_angle"].last
                    sd.candidate_callback(nc); nga.candidate_cb(nc)
                if nga.wp_idx != wp_before and wp_before == 0:
                    res["wp_adv_t"] = t
                    if not done_seen:
                        res["early"] = True
            # ---- 검출기(stop) 10 Hz, mode 2 만 ----
            if pub_mode == 2:
                rows = []
                for bx, by in buoys:
                    d = math.hypot(bx - x, by - y)
                    rel = wrap(psi - math.degrees(math.atan2(bx - x, by - y)))
                    if CAM_MIN <= d <= CAM_MAX and abs(rel) <= HFOV / 2:
                        area = 400.0 * (5.0 / d) ** 2
                        rows.append((rel + rng.gauss(0, cam_noise), area, min(1.0, area / 400.0) * 0.95, 0.05))
                data, _ = pack_bearings(rows)
                sb.buoy_bearings_cb(Float32MultiArray(data))
            # ---- ship_goal_angle → ship_back → ship_direction → motor_control ----
            cy = sga.pubs["/yaw_error"].count
            sga.publish_yaw_error()
            if sga.pubs["/yaw_error"].count > cy:
                sd.yaw_error_callback(sga.pubs["/yaw_error"].last)
            cn = sb.pubs["/candidate_angle"].count
            sb.timer_cb()
            if sb.pubs["/candidate_angle"].count > cn:
                cm = sb.pubs["/candidate_angle"].last
                sd.candidate_callback(cm); nga.candidate_cb(cm)
                if pub_mode == 2 and nga.wp_idx == 0:
                    st = sb.sq_fsm.state
                    if trace is not None:
                        trace.append((round(t, 1), st, float(cm.data), round(x, 2), round(y, 2), round(psi, 1),
                                      sb.fuser.heading(t), len(sb.sq_fsm.map.confirmed()), sb.sq_fsm.fit.kind,
                                      sb._nearest_any(t)))
                    res["states"][st] = res["states"].get(st, 0) + 1
                    if not output_allowed(float(cm.data), st):
                        res["bad_out"] += 1
                    if cm.data == 20000.0 and st not in ("INIT", "DONE"):
                        res["inv20"] += 1
                    if cm.data == 180.0:
                        res["rev_ticks"] += 1
                    if cm.data in (5000.0, 6000.0):
                        res["spin_ticks"] += 1
                if sb.sq_fsm.state == "DONE" and not done_seen:
                    done_seen = True
                    res["done"] = bool(sb.sq_fsm.success)
                    res["giveup"] = not sb.sq_fsm.success
                    res["done_t"] = t
                    res["done_inside"] = gt_run
                    res["reason"] = sb.sq_fsm.done_reason[:80]
                    if sb.sq_fsm.center is not None and sb.sq_origin is not None:
                        cx, cy_ = F.Geo(*sb.sq_origin).ll(*sb.sq_fsm.center)
                        tx, ty = geo.xy(cx, cy_)
                        gx = sum(b[0] for b in buoys) / 4; gy = sum(b[1] for b in buoys) / 4
                        res["center_err"] = math.hypot(tx - gx, ty - gy)
            sd.watchdog_cb(); sd.control_cb()
            if sd.pubs["/desired_angle"].last is not None:
                mc.angle_callback(sd.pubs["/desired_angle"].last)
            if sd.pubs["/obstacle_distance_array"].last is not None:
                mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
            if sd.pubs["/failsafe_level"].last is not None:
                mc.failsafe_callback(sd.pubs["/failsafe_level"].last)
            mc.timer_callback()
            code = mc.pubs["Motor_run"].last.data
            pr, pl = code // 10000, code % 10000
            TR, TL = 1500 - pr, 1500 - pl
            v_cmd, w_cmd = F.K.K_V * (TR + TL) / 2.0, F.K.K_W * (TL - TR)
            v += (v_cmd - v) * DT / F.TAU; w += (w_cmd - w) * DT / F.TAU
            psi = (psi + w * DT) % 360.0
            x += (v * math.sin(math.radians(psi)) + current[0]) * DT
            y += (v * math.cos(math.radians(psi)) + current[1]) * DT
            t += DT; k += 1; Clock.t = t
            # ---- 판정 ----
            if pub_mode == 2 and nga.wp_idx == 0:
                h = sb.fuser.heading(t)
                if h is not None:
                    res["head_err"].append(abs(wrap(h - psi)))
                ins = inside_truth(buoys, x, y, psi, buoy_r)
                gt_run = gt_run + DT if ins else 0.0
                if gt_run >= 5.0 - 1e-9 and not res["gt_ok"]:
                    res["gt_ok"] = True; res["gt_t"] = t
            for i, (bx, by) in enumerate(buoys):
                cl = hull_buoy_clear(x, y, psi, bx, by, buoy_r)
                res["min_clear"] = min(res["min_clear"], cl)
                on = cl < 0.0
                if on and not contact_on[i]:
                    res["contacts"] += 1
                contact_on[i] = on
            if res["wp_adv_t"] is not None and t > res["wp_adv_t"] + 2.0:
                break
    finally:
        os.remove(wp)
    he = res.pop("head_err")
    res["head_err_p95"] = float(np.percentile(he, 95)) if he else float("nan")
    res["sb_errors"] = [m for lv, m in sb.get_logger().lines if lv == "ERROR"][:3]
    res["buoys"] = buoys
    if verbose:
        for lv, m in sb.get_logger().lines:
            if lv != "DEBUG":
                print(f"   [{lv}] {m}")
    return res


def summarize(R, label):
    n = len(R)
    ok = [r for r in R if r["done"]]
    gt = sum(r["gt_ok"] for r in R)
    true_done = sum(1 for r in ok if (r["done_inside"] or 0) >= 5.0 - 1e-6)
    dt = sorted(r["done_t"] for r in ok)
    med = dt[len(dt) // 2] if dt else float("nan")
    print(f"\n== {label} ({n} 시드)")
    print(f"  노드 DONE(성공) {len(ok)}/{n} · 그때 참 내부 연속 ≥5 s {true_done}/{len(ok)} · 참 성공(5 s) {gt}/{n} · "
          f"포기 {sum(r['giveup'] for r in R)}/{n}")
    print(f"  접촉 {sum(r['contacts'] for r in R)}회({sum(1 for r in R if r['contacts'])} 시드) · 최소 여유 "
          f"{min(r['min_clear'] for r in R):.2f} m · 조기 넘김 {sum(r['early'] for r in R)} · "
          f"INIT·DONE 밖 20000 {sum(r['inv20'] for r in R)} 틱 · 출력 위반 {sum(r['bad_out'] for r in R)} 틱")
    print(f"  DONE 시각(출발 기준) 중앙 {med:.1f} s, 최대 {max(dt) if dt else float('nan'):.1f} s · "
          f"중심 오차 최대 {max((r['center_err'] or 0) for r in ok) if ok else float('nan'):.2f} m · "
          f"헤딩 오차 p95 최대 {max(r['head_err_p95'] for r in R):.1f}° · 후진 {sum(r['rev_ticks'] for r in R)} 틱")
    bad = [r for r in R if not r["done"] or r["contacts"]]
    for r in bad[:6]:
        print(f"   seed {r['seed']}: done={r['done']} giveup={r['giveup']} contacts={r['contacts']} "
              f"states={r['states']} reason={r['reason']!r} err={r['sb_errors'][:1]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=1)
    ap.add_argument("--heading", choices=("true", "gps_cog"), default="gps_cog")
    ap.add_argument("--drift", type=float, default=0.0, help="부표 고정 표류 반경 m (예 1.0)")
    ap.add_argument("--cog-noise", type=float, default=0.05, help="GPS 속도 잡음 σ m/s(gps_cog COG)")
    ap.add_argument("--gps-sigma", type=float, default=0.03)
    ap.add_argument("--buoy-r", type=float, default=0.25)
    ap.add_argument("--current", type=float, nargs=2, default=(0.0, 0.0))
    ap.add_argument("--imu-drift", type=float, default=0.3, help="IMU 상대 yaw 드리프트 상한 °/s(이동 중, 런마다 ±)")
    ap.add_argument("--start-dist", type=float, nargs=2, default=(18.0, 28.0), help="출발 거리 범위 m(중심 기준)")
    ap.add_argument("--random-heading", action="store_true", help="정지·무작위 방향 출발(가까이 시작해 도리도리 경로 시험)")
    ap.add_argument("--param", action="append", default=[], help="노드.키=값 (예 ship_back.heading_use_raw=false)")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    ov = F.parse_param_overrides(a.param)
    R = []
    for sd_ in range(a.seed0, a.seed0 + a.seeds):
        r = run(sd_, heading=a.heading, drift=a.drift, cog_noise=a.cog_noise, gps_sigma=a.gps_sigma, buoy_r=a.buoy_r,
                current=tuple(a.current), overrides=ov, imu_drift_dps=a.imu_drift, verbose=a.verbose,
                start_dist=tuple(a.start_dist), random_heading=a.random_heading)
        R.append(r)
    label = (f"heading={a.heading} drift={a.drift:g} cog_noise={a.cog_noise:g} current={tuple(a.current)} "
             + (f"start={a.start_dist[0]:g}~{a.start_dist[1]:g}m" + (" 무작위방향 " if a.random_heading else " "))
             + " ".join(a.param))
    summarize(R, label)


if __name__ == "__main__":
    main()

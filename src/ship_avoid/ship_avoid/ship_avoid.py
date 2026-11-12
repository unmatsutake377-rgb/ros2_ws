"""도리도리회피 — ship_avoid: mode 5 장애물 회피 미션 노드 (낮은 장애물 + 도리도리).

계약서: docs/문제와작업/계약서_도리도리회피_20261112.md (초안, 미승인)
        docs/문제와작업/계약서_도리도리회피_launch_20261113.md C1 (avoid_mode·검출 침묵 폴백·/wp_mode stale)
avoid_mode (yaml, 기본 'lidar'): lidar = 담당 모드에서 매 틱 20000 만(GPS·yaw 무관, 지금 north 폴백과 같음).
  doridori = 도리도리 코어. 단 /avoid_detections 침묵이면 20000 으로 넘기고 도리도리를 시작하지 않는다.
  판정은 avoid_mode_logic.plan_tick (순수 함수).
🚨 아직 대회 launch 에 넣지 않는다. 넣을 땐 같은 커밋으로(원자 묶음, 계약서 §2 ③):
   north_goal_angle FALLBACK_MODES 에서 5 제거 + ssf_tools.yaml 매핑 5→ship_avoid + test_mode_gate.
   (안 하면 mode 5 에서 /candidate_angle 발행자가 2개가 된다.)

입력
  /wp_mode (Int32)                 담당 모드(active_wp_mode, 기본 5)일 때만 발행.
                                   wp_mode_stale_sec 넘게 묵으면 발행 중지(north 주기 0.5 s)
  /imu/yaw (Float64)               나침반 yaw(deg)
  /imu/data (sensor_msgs/Imu)      피치(수면 기준 거리 공식)
  /ublox_gps_node/fix (NavSatFix)  위치 → 첫 fix 기준 로컬 m
  /scan (LaserScan)                노란부표 추출(lidar_buoys) — BEST_EFFORT depth 1
  /avoid_detections (Float32MultiArray)  [cls, bearing_left_deg, v_norm, range_m, conf] × N
       cls 0 = 빨강(낮은 장애물), 2 = 노랑. v_norm = (v_bottom − cy)/fy. range_m 모르면 NaN.
       ⚠️ 발행 검출기 미구현 — doridori 여도 침묵이면 20000 폴백(진입 후 quiet 초 넘으면 ERROR).
출력
  /candidate_angle (Float32)       lidar·검출 침묵 = 20000, 주행 = (B) 각도, 도리도리 = 5000/6000/50000
  /virtual_obstacle_state (Float32MultiArray)  /geofence_state 형식 (ship_direction 병합은 계약 단위 ①)
                                   도리도리 코어가 돌 때만 낸다(lidar·20000 폴백 땐 지도가 없으니 침묵)
  /avoid_state (String)            관측용 — 'LIDAR' / 'DORIDORI_NO_DET …' / 코어 상태
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, Float64, Int32, String
from sensor_msgs.msg import Imu, LaserScan, NavSatFix

from ship_avoid import avoid_mode_logic as ml
from ship_avoid.core import AvoidCore, AvoidParams, virtual_ranges
from ship_avoid.lidar_buoys import cluster_buoys

EARTH_R = 6371000.0
SCAN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class ShipAvoid(Node):
    def __init__(self):
        super().__init__('ship_avoid')
        # 🚨 선언은 한 줄에 하나, 키·기본값 리터럴로 — test_launch_params._declared·test_node_params 가 ast 로 읽는다.
        #    기본값 = core.AvoidParams 기본값 = config/ship_avoid.yaml (소수점 타입까지).
        self.active_wp_mode = int(self.declare_parameter('active_wp_mode', 5).value)
        self.period = float(self.declare_parameter('period_sec', 0.1).value)
        self.gps_stale = float(self.declare_parameter('gps_stale_sec', 2.5).value)
        self.yaw_stale = float(self.declare_parameter('yaw_stale_sec', 0.5).value)
        det_quiet = self.declare_parameter('detections_quiet_warn_sec', 5.0).value
        self.lidar_forward = float(self.declare_parameter('lidar_forward_deg', 80.0).value)
        self.pitch_sign = float(self.declare_parameter('pitch_sign', 1.0).value)    # ⚠️ 실측: 선수 숙임 = 양수가 되게
        self.virtual_extra = float(self.declare_parameter('virtual_extra_m', 0.2).value)
        avoid_mode = self.declare_parameter('avoid_mode', 'lidar').value              # 'lidar' | 'doridori'
        wp_stale = self.declare_parameter('wp_mode_stale_sec', 1.0).value             # north /wp_mode 주기 0.5 s 의 2배
        P = AvoidParams()
        P.half_l = float(self.declare_parameter('half_l', 0.83).value)
        P.half_w = float(self.declare_parameter('half_w', 0.285).value)
        P.low_r = float(self.declare_parameter('low_r', 0.2).value)
        P.buoy_r = float(self.declare_parameter('buoy_r', 0.12).value)
        P.default_half_width_m = float(self.declare_parameter('default_half_width_m', 1.5).value)
        P.clear_low = float(self.declare_parameter('clear_low', 0.45).value)
        P.clear_wall = float(self.declare_parameter('clear_wall', 0.35).value)
        P.guard_m = float(self.declare_parameter('guard_m', 0.15).value)
        P.stop_lead_s = float(self.declare_parameter('stop_lead_s', 0.6).value)
        P.dori_angle_deg = float(self.declare_parameter('dori_angle_deg', 60.0).value)
        P.dori_tol_deg = float(self.declare_parameter('dori_tol_deg', 4.0).value)
        P.dori_lead_s = float(self.declare_parameter('dori_lead_s', 0.3).value)
        P.dori_settle_s = float(self.declare_parameter('dori_settle_s', 0.3).value)
        P.dori_hold_s = float(self.declare_parameter('dori_hold_s', 1.2).value)
        P.dori_sweep_margin_m = float(self.declare_parameter('dori_sweep_margin_m', 0.25).value)
        P.dori_defer_max_m = float(self.declare_parameter('dori_defer_max_m', 1.5).value)
        P.dori_explore_m = float(self.declare_parameter('dori_explore_m', 3.0).value)
        P.dori_timeout_s = float(self.declare_parameter('dori_timeout_s', 25.0).value)
        P.stop_speed = float(self.declare_parameter('stop_speed', 0.05).value)
        P.look_m = float(self.declare_parameter('look_m', 1.5).value)
        P.turn_in_m = float(self.declare_parameter('turn_in_m', 2.5).value)
        P.max_steer_deg = float(self.declare_parameter('max_steer_deg', 60.0).value)
        P.step_deg = float(self.declare_parameter('step_deg', 10.0).value)
        P.cam_h = float(self.declare_parameter('cam_h', 0.6).value)                    # ⚠️ 실측 필요
        P.cam_pitch_deg = float(self.declare_parameter('cam_pitch_deg', 0.0).value)    # ⚠️ 실측 필요
        P.bearing_sigma_deg = float(self.declare_parameter('bearing_sigma_deg', 0.7).value)
        P.pitch_sigma_hold = float(self.declare_parameter('pitch_sigma_hold', 0.5).value)
        P.pitch_sigma_move = float(self.declare_parameter('pitch_sigma_move', 2.0).value)
        P.pos_sigma = float(self.declare_parameter('pos_sigma', 0.03).value)
        P.range_sigma_rel = float(self.declare_parameter('range_sigma_rel', 0.03).value)
        P.max_range = float(self.declare_parameter('max_range', 8.0).value)
        P.confirm_n = int(self.declare_parameter('confirm_n', 3).value)
        P.red_ratio = float(self.declare_parameter('red_ratio', 0.6).value)
        P.wall_fill_step = float(self.declare_parameter('wall_fill_step', 0.15).value)
        P.buoy_match_m = float(self.declare_parameter('buoy_match_m', 0.35).value)

        self.avoid_mode, err = ml.check_avoid_mode(avoid_mode)
        if err:
            self._err_param(err)
        self.wp_stale, err = ml.check_stale_sec('wp_mode_stale_sec', wp_stale, ml.DEFAULT_WP_MODE_STALE_SEC)
        if err:
            self._err_param(err)
        self.det_quiet, err = ml.check_stale_sec('detections_quiet_warn_sec', det_quiet, ml.DEFAULT_DET_QUIET_SEC)
        if err:
            self._err_param(err)
        self.core = AvoidCore(P)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cand = self.create_publisher(Float32, '/candidate_angle', qos)
        self.pub_virtual = self.create_publisher(Float32MultiArray, '/virtual_obstacle_state', qos)
        self.pub_state = self.create_publisher(String, '/avoid_state', qos)
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, qos)
        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, qos)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, SCAN_QOS)
        self.create_subscription(Float32MultiArray, '/avoid_detections', self.det_cb, qos)

        self.wp_mode = None
        self.wp_mode_t = None
        self.active_since = None          # 이번 담당 구간 진입 시각(검출 침묵 유예 기준)
        self.yaw = None
        self.yaw_t = None
        self.lat0 = self.lon0 = None
        self.x = self.y = None
        self.gps_t = None
        self.last_det_t = None
        self.was_active = False
        self.create_timer(self.period, self.timer_cb)
        self.get_logger().info(
            f"도리도리회피(ship_avoid) 시작: avoid_mode={self.avoid_mode}, mode {self.active_wp_mode} 담당 "
            f"(lidar = 매 틱 20000), /wp_mode {self.wp_stale:g} s 넘게 묵으면 발행 중지, "
            f"도리도리 ±{P.dori_angle_deg:.0f}°")

    # ── 입력 ──
    def wp_mode_cb(self, msg):
        self.wp_mode = int(msg.data)
        self.wp_mode_t = time.monotonic()

    def yaw_cb(self, msg):
        if math.isfinite(msg.data):
            self.yaw = float(msg.data) % 360.0
            self.yaw_t = time.monotonic()

    def imu_cb(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        self.core.set_pitch(self.pitch_sign * pitch)

    def gps_cb(self, msg):
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return                         # fix 없음 — 위치 갱신 안 함(stale 로 드러난다)
        if self.lat0 is None:
            self.lat0, self.lon0 = msg.latitude, msg.longitude
        k = math.pi / 180.0 * EARTH_R
        self.x = (msg.longitude - self.lon0) * k * math.cos(math.radians(self.lat0))
        self.y = (msg.latitude - self.lat0) * k
        self.gps_t = time.monotonic()

    def scan_cb(self, msg):
        if self.x is None or self.yaw is None:
            return
        bl = cluster_buoys(list(msg.ranges), math.degrees(msg.angle_min), math.degrees(msg.angle_increment),
                           lidar_forward_deg=self.lidar_forward, buoy_r=self.core.p.buoy_r)
        pts = []
        for rel, d in bl:
            wb = math.radians(self.yaw - rel)
            pts.append((self.x + d * math.sin(wb), self.y + d * math.cos(wb)))
        self.core.set_buoys(pts)

    def det_cb(self, msg):
        self.last_det_t = time.monotonic()
        if self.x is None or self.yaw is None:
            return
        d = list(msg.data)
        dets = []
        for i in range(0, len(d) - 4, 5):
            dets.append((int(round(d[i])), d[i + 1], d[i + 2], d[i + 3], d[i + 4]))
        self.core.add_detections(time.monotonic(), self.x, self.y, self.yaw, dets)

    # ── 로그 (호출 지점 분리 — CLAUDE.md §1-7) ──
    def _err_param(self, msg):
        self.get_logger().error(f"ship_avoid: {msg}")

    def _err_stale(self, what):
        self.get_logger().error(f"ship_avoid: {what} → 발행 중지(모르면 입을 다문다)", throttle_duration_sec=5.0)

    def _err_wp_stale(self, age):
        self.get_logger().error(
            f"ship_avoid: /wp_mode {age:.1f} s 묵음(> {self.wp_stale:g} s, 마지막 mode {self.wp_mode}) "
            "→ /candidate_angle 발행 중지(north 확인)", throttle_duration_sec=5.0)

    def _err_det_dead(self):
        self.get_logger().error(
            f"ship_avoid: doridori 인데 /avoid_detections 가 {self.det_quiet:g} s 넘게 침묵 → 20000 폴백, "
            "도리도리 시작 안 함(검출기 확인)", throttle_duration_sec=5.0)

    def _info_state(self, s):
        self.get_logger().info(f"ship_avoid: {s}")

    # ── 주기 ──
    def _leave(self, why):
        if self.was_active:
            self.core.deactivate()
            self._info_state(f"{why} → 도리도리 비활성")
        self.was_active = False

    def timer_cb(self):
        now = time.monotonic()
        state = ml.plan_tick(self.avoid_mode, self.wp_mode, self.wp_mode_t, self.active_wp_mode, now,
                             self.wp_stale, self.last_det_t, self.active_since, self.det_quiet)
        if not ml.is_active(state):
            self.active_since = None
            self._leave("mode 이탈" if state == ml.IDLE else "/wp_mode stale")
            if state == ml.WP_STALE:
                self._err_wp_stale(now - self.wp_mode_t)
            return
        if self.active_since is None:
            self.active_since = now
        fixed = ml.fixed_candidate(state)
        if fixed is not None:                                  # LIDAR · DET_WAIT · DET_DEAD → 20000
            if state != ml.LIDAR:
                self._leave("검출 침묵")
            if state == ml.DET_DEAD:
                self._err_det_dead()
            self.pub_cand.publish(Float32(data=fixed))
            self.pub_state.publish(String(data='LIDAR' if state == ml.LIDAR else f'DORIDORI_NO_DET {state}'))
            return
        # DORI — 도리도리 코어 경로
        if self.gps_t is None or now - self.gps_t > self.gps_stale:
            self._err_stale("GPS 위치 없음/stale")
            return
        if self.yaw_t is None or now - self.yaw_t > self.yaw_stale:
            self._err_stale("/imu/yaw 없음/stale")
            return
        self.core.set_pose(now, self.x, self.y, self.yaw)
        if not self.was_active:
            if not self.core.activate(now):
                return
            self.was_active = True
            self._info_state(f"활성 — 코스 축 {self.core.psi0:.1f}° 잠금, 시작 도리도리")
        cand = self.core.step(now)
        if cand is None:
            return
        self.pub_cand.publish(Float32(data=float(cand)))
        vo = self.core.virtual_obstacles(self.virtual_extra)
        arr = Float32MultiArray()
        arr.data = [float(v) for v in virtual_ranges(self.x, self.y, self.yaw, vo)] if vo else []
        self.pub_virtual.publish(arr)
        self.pub_state.publish(String(data=self.core.status()))


def main(args=None):
    rclpy.init(args=args)
    node = ShipAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

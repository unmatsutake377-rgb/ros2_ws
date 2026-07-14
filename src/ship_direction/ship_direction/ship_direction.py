import math
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan

# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
SPIN_RIGHT = 5000.0          # 우선회
SPIN_LEFT = 6000.0           # 좌선회
CANDIDATE_INVALID = 20000.0  # 미션 없음 → yaw_error 기반 자율 회피
STOP_HOLD = 50000.0          # 정지/대기

REVERSE_ANGLE = 260.0        # 회피 경로 없음 → 후진


class ShipDirection(Node):
    """LiDAR 회피 + 페일세이프. /desired_angle · /obstacle_distance_array · /failsafe_level 발행.

    3단계에서 고친 것:

    [A] 제어 루프를 고정 주기 타이머로 분리했다.
        작년: 모든 계산·발행이 scan_callback 안에 있었다 → **LiDAR 가 끊기면 콜백이 아예 안 와서
              아무도 /desired_angle 을 발행하지 않았다.** (측정: LiDAR 1~3초 끊김 → 발행 0회, 완전 침묵.
              페일세이프 레벨은 올라가는데 출력이 없으니 L1/L2 가 '존재'하지 않았다.)
        지금: scan_cb 는 **스캔 저장만**. control_cb 가 고정 주기로 **항상** 계산·발행한다.
              → ship_direction 이 살아있는 한 /desired_angle 은 흐른다.
                침묵 = ship_direction 사망 → motor_control 이 cmd_timeout 으로 잡는다(0.5s 로 조일 수 있게 됨).

    [B] 페일세이프(독립 watchdog 타이머). scan 이 묵은 정도로 레벨을 매긴다.
        L0 : 계산한 각도        / 속도 100%
        L1 : 계산한 각도 그대로 / 속도 70%    ← 각도 안 건드림
        L2 : 마지막 각도 freeze / 속도 50%
        L3 : STOP_HOLD override / 정지        ← 각도를 덮어쓰는 건 여기뿐

        ★ L1 에서 '직진 강제' 같은 걸 하면 회피 기동 중간에 핸들을 놓는 셈이다.
          부표를 피하던 중 직진하면 그대로 박는다. 0.7초 묵었다고 조향을 포기하는 건 과잉반응이다.
          묵은 데이터라도 없는 것보다 낫다 — 대신 속도를 줄여 오차를 줄인다.

        속도는 /desired_angle(각도)에 담을 수 없다 → **motor_control 이 /failsafe_level 을 구독해
        속도 상한을 건다.** (L3 는 STOP_HOLD 가 각도로 오므로 motor_control 분기 (1) 이 이미 처리)

        오탐 방지(CLAUDE.md §5): ARMED(스캔 한 번은 받아야 평가 시작) + 올릴 땐 연속 N회 확인 +
        내릴 땐 즉시(자동 복구) + time.monotonic().

    [D] time.time() → time.monotonic() 전부 교체.
    """

    def __init__(self):
        super().__init__('ship_direction')

        self.lock = Lock()

        # ----------------------
        #   파라미터 (config yaml, CLAUDE.md 1-4)
        # ----------------------
        self.control_period = float(self.declare_parameter('control_period_sec', 0.1).value)    # 10Hz
        self.watchdog_period = float(self.declare_parameter('watchdog_period_sec', 0.1).value)

        # 페일세이프 임계(스캔이 묵은 시간). CLAUDE.md §5: warn 0.7 / stop 3.0
        self.fs_l1_sec = float(self.declare_parameter('failsafe_l1_sec', 0.7).value)
        self.fs_l2_sec = float(self.declare_parameter('failsafe_l2_sec', 1.5).value)
        self.fs_l3_sec = float(self.declare_parameter('failsafe_l3_sec', 3.0).value)
        self.fs_confirm_n = int(self.declare_parameter('failsafe_confirm_n', 3).value)

        # 회피 튜닝 — 값은 작년 그대로 유지(§6 시뮬값 반영은 별도 커밋에서 측정하며 진행)
        self.base_detection_distance = float(self.declare_parameter('base_detection_distance', 1.8).value)
        self.mode2_detection_distance = float(self.declare_parameter('mode2_detection_distance', 1.8).value)
        self.half_width = float(self.declare_parameter('half_width', 0.45).value)
        self.clearance = float(self.declare_parameter('clearance', 0.15).value)
        self.border_margin = int(self.declare_parameter('border_margin', 2).value)
        self.max_spike_ratio = float(self.declare_parameter('max_spike_ratio', 0.01).value)
        self.rear_obstacle_ignore_margin = float(
            self.declare_parameter('rear_obstacle_ignore_margin', 1.0).value)
        self.min_obstacle_cells = int(self.declare_parameter('min_obstacle_cells', 3).value)
        self.reverse_cooldown = float(self.declare_parameter('reverse_cooldown_sec', 3.0).value)

        self.detection_distance = self.base_detection_distance

        # ----------------------
        #      Subscriptions
        # ----------------------
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Float32, '/yaw_error', self.yaw_error_callback, 10)
        self.create_subscription(Float32, '/candidate_angle', self.candidate_callback, 10)
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, 10)

        # ----------------------
        #       Publishers  (토픽/타입 불변 + /failsafe_level 신규, CLAUDE.md 3-9)
        # ----------------------
        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)
        self.pub_failsafe = self.create_publisher(Int32, '/failsafe_level', 10)

        # ----------------------
        #       State
        # ----------------------
        self.wp_mode = -1
        self.wp4_enter_time = None
        self.yaw_error = 0.0
        self.candidate_angle = CANDIDATE_INVALID

        self.latest_scan = None       # scan_cb 는 저장만 한다
        self.last_scan_t = None       # monotonic. None = ARMED 전(스캔 한 번도 못 받음)
        self.last_reverse_time = time.monotonic()

        self.failsafe_level = 0
        self._raise_count = 0
        self.last_desired_angle = None   # L2 freeze 용

        # ----------------------
        #  타이머: 제어 / 워치독 (서로 독립)
        # ----------------------
        self.control_timer = self.create_timer(self.control_period, self.control_cb)
        self.watchdog_timer = self.create_timer(self.watchdog_period, self.watchdog_cb)

        self.get_logger().info(
            f"ship_direction 시작: 제어 {1.0 / self.control_period:.0f}Hz (스캔 유무 무관 항상 발행), "
            f"워치독 {1.0 / self.watchdog_period:.0f}Hz.\n"
            f"   페일세이프 L1={self.fs_l1_sec}s L2={self.fs_l2_sec}s L3={self.fs_l3_sec}s, "
            f"확인 {self.fs_confirm_n}회. 각도 override 는 L3 뿐."
        )

    # =====================================================
    # CALLBACKS — 저장만 한다
    # =====================================================
    def scan_callback(self, msg: LaserScan):
        """저장만. 계산·발행은 control_cb 가 고정 주기로 한다.
        (작년엔 여기서 다 했다 → LiDAR 가 죽으면 콜백이 안 와서 아무도 발행하지 않았다)"""
        with self.lock:
            self.latest_scan = msg
            self.last_scan_t = time.monotonic()

    def yaw_error_callback(self, msg):
        with self.lock:
            self.yaw_error = (360.0 - msg.data) % 360.0

    def candidate_callback(self, msg):
        val = msg.data
        with self.lock:
            if val in (CANDIDATE_INVALID, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT):
                self.candidate_angle = val
            else:
                self.candidate_angle = (val % 360.0)

    def wp_mode_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = msg.data

            new_dd = self.mode2_detection_distance if self.wp_mode == 2 else self.base_detection_distance
            if abs(new_dd - self.detection_distance) > 1e-6:
                self.detection_distance = new_dd

            if prev != 4 and self.wp_mode == 4:
                self.wp4_enter_time = time.monotonic()
            elif prev == 4 and self.wp_mode != 4:
                self.wp4_enter_time = None

    # =====================================================
    # WATCHDOG — 독립 타이머 (CLAUDE.md §5)
    # =====================================================
    def watchdog_cb(self):
        """페일세이프를 독립 타이머로 평가한다.
        scan_cb 안에서 평가하면 LiDAR 가 완전히 죽었을 때 콜백이 아예 안 와서
        페일세이프가 영원히 평가되지 않는다 (§5 에 기록된 실제 버그)."""
        now = time.monotonic()
        with self.lock:
            last = self.last_scan_t

        if last is None:
            # ARMED 전: 스캔을 한 번도 못 받음(부팅 중) → 발동 안 함 (부팅 오발동 방지)
            level = 0
            self._raise_count = 0
        else:
            age = now - last
            if age > self.fs_l3_sec:
                raw = 3
            elif age > self.fs_l2_sec:
                raw = 2
            elif age > self.fs_l1_sec:
                raw = 1
            else:
                raw = 0

            level = self.failsafe_level
            if raw > level:
                # 올릴 땐 연속 N회 확인 — 순간 지터로 레벨 안 올림
                self._raise_count += 1
                if self._raise_count >= self.fs_confirm_n:
                    level = raw
                    self._raise_count = 0
            elif raw < level:
                # 자동 복구: 신선한 스캔이 왔다는 뜻 → 즉시 해제 (히스테리시스: 올릴 땐 느리게, 풀 땐 빠르게)
                level = raw
                self._raise_count = 0
            else:
                self._raise_count = 0

        if level != self.failsafe_level:
            log = self.get_logger().warn if level > self.failsafe_level else self.get_logger().info
            log(f"페일세이프 L{self.failsafe_level} → L{level}")

        self.failsafe_level = level
        self.pub_failsafe.publish(Int32(data=int(level)))

    # =====================================================
    # CONTROL — 고정 주기, 스캔 유무와 무관하게 항상 발행
    # =====================================================
    def control_cb(self):
        level = self.failsafe_level

        with self.lock:
            scan = self.latest_scan
            wp4_enter_time = self.wp4_enter_time
            candidate_angle = self.candidate_angle
            yaw_error = self.yaw_error
            detection_distance = self.detection_distance

        # ---- L3: 각도를 STOP_HOLD 로 override. 각도를 덮어쓰는 건 여기뿐. ----
        if level >= 3:
            self.pub_obstacle_distance.publish(self._closest_obstacle(scan))
            self.pub_desired_angle.publish(Float32(data=STOP_HOLD))
            return

        # 스캔을 한 번도 못 받음(부팅 중) → 발행할 각도가 없다.
        # motor_control 은 '첫 명령 전 중립'이라 안전하다.
        if scan is None:
            return

        angle, obst = self._compute(scan, wp4_enter_time, candidate_angle,
                                    yaw_error, detection_distance)

        # ---- L2: 마지막 각도 freeze (묵은 데이터로 새 조향을 만들지 않음) ----
        if level >= 2 and self.last_desired_angle is not None:
            angle = self.last_desired_angle
        else:
            self.last_desired_angle = angle

        # ---- L1: 각도 그대로. 속도만 줄인다 → motor_control 이 /failsafe_level 보고 상한을 건다. ----

        self.pub_obstacle_distance.publish(obst)
        self.pub_desired_angle.publish(Float32(data=angle))

    # =====================================================
    # 계산 (회피 알고리즘 — 작년 로직 보존)
    # =====================================================
    def _closest_obstacle(self, msg):
        """전방 0~160° 최근접 → Float32MultiArray([거리(m), 각도(deg)]). 없으면 [inf, nan]."""
        obst = Float32MultiArray()
        if msg is None:
            obst.data = [float('inf'), float('nan')]
            return obst

        angle_min = math.degrees(msg.angle_min)
        angle_increment_deg = math.degrees(msg.angle_increment)
        ranges = msg.ranges

        min_index = int((0 - angle_min) / angle_increment_deg)
        max_index = int((160 - angle_min) / angle_increment_deg)

        closest_distance = float('inf')
        closest_angle = float('nan')

        if 0 <= min_index < len(ranges) and 0 <= max_index < len(ranges):
            for i in range(min_index, max_index + 1):
                r = ranges[i]
                if not math.isinf(r) and not math.isnan(r) and r < closest_distance:
                    closest_distance = r
                    closest_angle = angle_min + i * angle_increment_deg

        obst.data = ([closest_distance, closest_angle]
                     if not math.isinf(closest_distance) else [float('inf'), float('nan')])
        return obst

    def _compute(self, msg, wp4_enter_time, candidate_angle, yaw_error, detection_distance):
        """(desired_angle, obstacle_array) 를 만든다. 작년 scan_callback 의 로직 그대로."""
        now = time.monotonic()

        obst = self._closest_obstacle(msg)

        # ---- (0) WP4 초기 5초 & STOP_HOLD 우선 처리 ----
        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
           (candidate_angle == STOP_HOLD):
            return STOP_HOLD, obst

        # ---- (A) PASS-THROUGH: 미션 노드가 선회를 직접 지시 ----
        if candidate_angle in (SPIN_RIGHT, SPIN_LEFT):
            return candidate_angle, obst

        # ---- (B) 일반 candidate_angle 매핑 (360° → 전방중심 80°) ----
        if candidate_angle != CANDIDATE_INVALID:
            c_raw = candidate_angle % 360.0
            c_mapped = 80 + c_raw if c_raw <= 180 else 80 - (360 - c_raw)
            return c_mapped, obst

        # ---- (C) candidate == 20000 → yaw_error 기반 자율 회피 ----
        angle_min = math.degrees(msg.angle_min)
        angle_increment_deg = math.degrees(msg.angle_increment)
        angle_increment_rad = msg.angle_increment
        ranges = list(msg.ranges)

        min_index = int((0 - angle_min) / angle_increment_deg)
        max_index = int((160 - angle_min) / angle_increment_deg)
        sub_ranges = ranges[min_index:max_index + 1]

        angle_array = []
        distance_array = []
        front_min_distance = float('inf')

        for i, r in enumerate(sub_ranges):
            angle_array.append(angle_min + (min_index + i) * angle_increment_deg)
            distance_array.append(r)
            if not math.isinf(r) and not math.isnan(r):
                front_min_distance = min(front_min_distance, r)

        # ---- Binary obstacle mask ----
        binary = []
        for r in distance_array:
            if math.isinf(r) or math.isnan(r):
                binary.append(0)
            elif r < detection_distance:
                if abs(r - front_min_distance) > self.rear_obstacle_ignore_margin:
                    binary.append(0)
                else:
                    binary.append(1)
            else:
                binary.append(0)

        binary = self.smooth_spikes(binary)
        binary = self.suppress_spike_edges(binary)
        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg)

        # ---- safe zone 탐색 ----
        safe_zones = []
        start = None
        for i, v in enumerate(binary):
            if v == 0 and start is None:
                start = i
            elif v == 1 and start is not None:
                safe_zones.append((start, i - 1))
                start = None
        if start is not None:
            safe_zones.append((start, len(binary) - 1))

        # ---- 통과 가능한 zone 선별 ----
        valid_safe_zones = []
        min_required_width = self.half_width * 2 + self.clearance

        for s, e in safe_zones:
            r_s = distance_array[s] if not math.isinf(distance_array[s]) and not math.isnan(distance_array[s]) else detection_distance
            r_e = distance_array[e] if not math.isinf(distance_array[e]) and not math.isnan(distance_array[e]) else detection_distance
            r_edge = min(r_s, r_e)
            arc_len = angle_increment_rad * r_edge * (e - s)
            if arc_len >= min_required_width:
                valid_safe_zones.append((s, e, r_edge))

        # ---- yaw_error 를 safe zone 안으로 clamp → 최적 후보 ----
        yaw_raw = yaw_error % 360.0
        yaw_mapped = 80 + yaw_raw if yaw_raw <= 180 else 80 - (360 - yaw_raw)

        candidates = []
        for s, e, r_edge in valid_safe_zones:
            zone_min = angle_array[s]
            zone_max = angle_array[e]

            if yaw_mapped < zone_min:
                best_angle = zone_min
            elif yaw_mapped > zone_max:
                best_angle = zone_max
            else:
                best_angle = yaw_mapped

            angle_diff = abs(best_angle - yaw_mapped)
            arc_length = angle_increment_rad * r_edge * (e - s)
            candidates.append((angle_diff, -arc_length, best_angle, s, e))

        # ---- 회피 경로가 없으면 후진 ----
        if not candidates:
            final_angle = REVERSE_ANGLE
            if now - self.last_reverse_time >= self.reverse_cooldown:
                self.last_reverse_time = now
        else:
            candidates.sort(key=lambda x: (x[0], x[1]))
            final_angle = candidates[0][2]

        return final_angle, obst

    # =====================================================
    # 스파이크 억제 / 팽창 (작년 그대로)
    # =====================================================
    def smooth_spikes(self, binary):
        total_len = len(binary)
        max_spike_length = int(total_len * self.max_spike_ratio)
        smoothed = binary[:]
        count = 0
        start = None
        for i, v in enumerate(binary):
            if v == 1:
                if start is None:
                    start = i
                count += 1
            else:
                if start is not None and count <= max_spike_length:
                    for j in range(start, i):
                        smoothed[j] = 0
                start = None
                count = 0
        if start is not None and count <= max_spike_length:
            for j in range(start, len(binary)):
                smoothed[j] = 0
        return smoothed

    def suppress_spike_edges(self, binary):
        suppressed = binary[:]
        for i, v in enumerate(binary):
            if v == 1:
                for off in range(-self.border_margin, self.border_margin + 1):
                    j = i + off
                    if 0 <= j < len(binary) and binary[j] == 0:
                        suppressed[i] = 0
                        break
        return suppressed

    def dilate_obstacles(self, binary, distance_array, angle_increment_deg):
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
                if length >= self.min_obstacle_cells:
                    r_edge = self.detection_distance
                    for k in range(start, end + 1):
                        r = distance_array[k]
                        if not math.isinf(r) and not math.isnan(r):
                            r_edge = min(r_edge, r)

                    lateral = self.half_width + self.clearance
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


def main(args=None):
    rclpy.init(args=args)
    node = ShipDirection()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

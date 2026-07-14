import rclpy
from std_msgs.msg import Int32
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
import time
from threading import Lock   # 🔥 Lock 추가

CANDIDATE_INVALID = 20000.0  # 미션 없음/실패 → yaw_error로 폴백
STOP_HOLD = 50000.0          # 정지/대기 특수 신호


class ShipDirection(Node):
    def __init__(self):
        super().__init__('ship_direction')

        # 🔥 공유 변수 보호용 Lock
        self.lock = Lock()

        # ----------------------
        #      Subscriptions
        # ----------------------
        self.subscription_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.subscription_yaw_error = self.create_subscription(
            Float32, '/yaw_error', self.yaw_error_callback, 10)

        self.subscription_candidate = self.create_subscription(
            Float32, '/candidate_angle', self.candidate_callback, 10)

        self.subscription_wp_mode = self.create_subscription(
            Int32, '/wp_mode', self.wp_mode_cb, 10)

        # ----------------------
        #       Publishers
        # ----------------------
        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)

        # ----------------------
        #       Parameters
        # ----------------------
        self.base_detection_distance = 1.8
        self.mode2_detection_distance = 1.8
        self.detection_distance = self.base_detection_distance

        self.half_width = 0.45
        self.clearance = 0.15
        self.border_margin = 2
        self.max_spike_ratio = 0.01
        self.log_interval = 1.0
        self.debug_interval = 1.0
        self.rear_obstacle_ignore_margin = 1.0
        self.min_obstacle_cells = 3

        # ----------------------
        #       State Vars
        # ----------------------
        self.wp_mode = -1
        self.wp4_enter_time = None
        self.hold_log_ts = 0.0

        self.yaw_error = 0.0
        self.candidate_angle = CANDIDATE_INVALID

        self.last_log_time = time.time()
        self.last_debug_time = time.time()
        self.last_reverse_time = time.time()
        self.reverse_time = 3.0


    # =====================================================
    # CALLBACKS
    # =====================================================

    def yaw_error_callback(self, msg):
        raw = msg.data
        with self.lock:
            self.yaw_error = (360.0 - raw) % 360.0


    def candidate_callback(self, msg):
        val = msg.data
        with self.lock:
            if val in (CANDIDATE_INVALID, STOP_HOLD, 5000.0, 6000.0):
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
                self.wp4_enter_time = time.time()

            elif prev == 4 and self.wp_mode != 4:
                self.wp4_enter_time = None
    # =====================================================
    #               LiDAR MAIN CALLBACK
    # =====================================================
    def scan_callback(self, msg: LaserScan):
        now = time.time()

        # ----------------------------
        # 🔥 먼저 필요한 값들을 Lock으로 안전하게 복사
        # ----------------------------
        with self.lock:
            wp_mode = self.wp_mode
            wp4_enter_time = self.wp4_enter_time
            candidate_angle = self.candidate_angle
            yaw_error = self.yaw_error
            detection_distance = self.detection_distance

        # =====================================================
        # 0) WP4 초기 5초 & STOP_HOLD 우선 처리
        # =====================================================
        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
           (candidate_angle == STOP_HOLD):

            angle_min = math.degrees(msg.angle_min)
            angle_increment_deg = math.degrees(msg.angle_increment)
            ranges = list(msg.ranges)

            min_index = int((0 - angle_min) / angle_increment_deg)
            max_index = int((160 - angle_min) / angle_increment_deg)

            closest_distance = float('inf')
            closest_angle = float('nan')

            if 0 <= min_index < len(ranges) and 0 <= max_index < len(ranges):
                for i in range(min_index, max_index):
                    r = ranges[i]
                    if not math.isinf(r) and not math.isnan(r) and r < closest_distance:
                        closest_distance = r
                        closest_angle = angle_min + i * angle_increment_deg

            obst = Float32MultiArray()
            obst.data = (
                [closest_distance, closest_angle]
                if closest_distance != float('inf')
                else [float('inf'), float('nan')]
            )

            self.pub_obstacle_distance.publish(obst)
            self.pub_desired_angle.publish(Float32(data=STOP_HOLD))
            return

        # =====================================================
        #               LiDAR 기본 전처리
        # =====================================================
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
            ang = angle_min + (min_index + i) * angle_increment_deg
            angle_array.append(ang)
            distance_array.append(r)

            if not math.isinf(r) and not math.isnan(r):
                front_min_distance = min(front_min_distance, r)

        # =====================================================
        #            Binary obstacle mask 생성
        # =====================================================
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

        # 악성 spike 억제
        binary = self.smooth_spikes(binary)
        binary = self.suppress_spike_edges(binary)
        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg)

        # =====================================================
        # safe zone(장애물 없음 구간) 탐색
        # =====================================================
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

        # safety width 기준 통과 가능한 zone만 선별
        valid_safe_zones = []
        min_required_width = self.half_width * 2 + self.clearance

        for s, e in safe_zones:
            r_s = distance_array[s] if not math.isinf(distance_array[s]) and not math.isnan(distance_array[s]) else detection_distance
            r_e = distance_array[e] if not math.isinf(distance_array[e]) and not math.isnan(distance_array[e]) else detection_distance

            r_edge = min(r_s, r_e)
            arc_len = angle_increment_rad * r_edge * (e - s)

            if arc_len >= min_required_width:
                valid_safe_zones.append((s, e, r_edge))

        # =====================================================
        # 가장 가까운 장애물 잡기
        # =====================================================
        closest_distance = float('inf')
        closest_angle = float('nan')

        for i in range(min_index, max_index + 1):
            r = ranges[i]
            if not math.isinf(r) and not math.isnan(r) and r < closest_distance:
                closest_distance = r
                closest_angle = angle_min + i * angle_increment_deg

        # =====================================================
        # (A) PASS-THROUGH: 이미지가 선회 명령을 직접 내리는 경우
        # =====================================================
        if candidate_angle in (5000.0, 6000.0):
            obst = Float32MultiArray()
            obst.data = (
                [closest_distance, closest_angle]
                if not math.isinf(closest_distance)
                else [float('inf'), float('nan')]
            )
            self.pub_obstacle_distance.publish(obst)
            self.pub_desired_angle.publish(Float32(data=candidate_angle))
            return

        # =====================================================
        # (B) 일반 candidate_angle 처리 (맵핑)
        # =====================================================
        if candidate_angle not in (CANDIDATE_INVALID, STOP_HOLD):
            c_raw = candidate_angle % 360.0

            # 360° → 전방중심 80° 맵핑
            if c_raw <= 180:
                c_mapped = 80 + c_raw
            else:
                c_mapped = 80 - (360 - c_raw)

            final_angle = c_mapped

            obst = Float32MultiArray()
            obst.data = (
                [closest_distance, closest_angle]
                if not math.isinf(closest_distance)
                else [float('inf'), float('nan')]
            )
            self.pub_obstacle_distance.publish(obst)
            self.pub_desired_angle.publish(Float32(data=final_angle))
            return

        # =====================================================
        # (C) candidate == 20000 → yaw_error 기반 자율 회피
        # =====================================================
        yaw_raw = yaw_error % 360.0
        yaw_mapped = 80 + yaw_raw if yaw_raw <= 180 else 80 - (360 - yaw_raw)

        candidates = []

        for s, e, r_edge in valid_safe_zones:
            zone_min = angle_array[s]
            zone_max = angle_array[e]

            # yaw_mapped를 safe zone 안으로 clamp
            if yaw_mapped < zone_min:
                best_angle = zone_min
            elif yaw_mapped > zone_max:
                best_angle = zone_max
            else:
                best_angle = yaw_mapped

            angle_diff = abs(best_angle - yaw_mapped)
            arc_length = angle_increment_rad * r_edge * (e - s)

            candidates.append((angle_diff, -arc_length, best_angle, s, e))

        # =====================================================
        # 회피 경로가 없으면 → 후진 260°
        # =====================================================
        if not candidates:
            final_angle = 260.0

            # 후진 쿨타임 유지
            if now - self.last_reverse_time >= self.reverse_time:
                self.last_reverse_time = now

        else:
            # 최적 후보 선택
            candidates.sort(key=lambda x: (x[0], x[1]))
            final_angle = candidates[0][2]

        # =====================================================
        # publish 결과
        # =====================================================
        obst = Float32MultiArray()
        obst.data = (
            [closest_distance, closest_angle]
            if not math.isinf(closest_distance)
            else [float('inf'), float('nan')]
        )
        self.pub_obstacle_distance.publish(obst)
        self.pub_desired_angle.publish(Float32(data=final_angle))

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

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

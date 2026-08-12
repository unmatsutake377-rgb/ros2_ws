#!/usr/bin/env python3
"""HSV 튜닝 도구 — 마스크를 **보면서** 범위를 잡고, yaml 줄을 그대로 뽑는다.

    ros2 run color_shape_detector basic_image_subscriberhsv

🚨 [2026-08-12 재작성] 예전 판은 **마우스 위치의 HSV 숫자만 찍었다.**
   그걸로는 "내가 정한 범위가 실제로 뭘 잡는지" 를 볼 수 없어서, 값을 넣고 → 검출기를
   재시작하고 → 각도가 나오나 보고 → 다시 고치는 왕복을 해야 했다.
   이제 마스크를 실시간으로 보여주고, 클릭 한 번으로 범위를 넓히고,
   `p` 키로 **vision.yaml 에 그대로 붙여넣을 줄**을 찍는다.

⚠️ 이 노드는 튜닝 전용이다. **대회 launch 에 넣지 마라.**
⚠️ 실내에서 잡은 값은 실외에서 안 맞는다 — 햇빛에서 S·V 가 통째로 달라진다.
   실제 부표로, 실제 조명에서 다시 잡을 것.

조작
    마우스 클릭   그 픽셀이 들어오도록 현재 범위를 넓힌다 (여백 포함)
    트랙바        H/S/V 하한·상한 직접 조정
    c            다음 색 (red→green→white→orange→yellow→blue)
    n            같은 색의 다음 범위 (빨강은 색상환 0을 넘어 범위가 2개다)
    p            현재 색의 yaml 줄 출력  ← 이걸 vision.yaml 에 붙여넣는다
    r            현재 색을 파라미터 원래값으로 되돌림
    q / ESC      종료
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from color_shape_detector import hsv_ranges

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

WIN = "HSV tuner"
WIN_MASK = "mask (white = detected)"
_BARS = (("H lo", 180), ("H hi", 180), ("S lo", 255),
         ("S hi", 255), ("V lo", 255), ("V hi", 255))


class HSVSubscriber(Node):

    def __init__(self):
        super().__init__('hsv_debug_subscriber')

        # ⚠️ 구독보다 먼저 선언해야 한다 — 뒤에 두면 부팅 즉시 AttributeError 다.
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value)

        # 이 노드만 debug_view 기본 true — 창이 없으면 존재 이유가 없다.
        # 그래도 게이트는 둔다: namedWindow 가 __init__ 에 있어 헤드리스면 여기서 터진다.
        self.debug_view = bool(self.declare_parameter('debug_view', True).value)

        # 최소 면적 — 이보다 작은 덩어리는 노이즈로 보고 윤곽을 안 그린다(검출기의 min_area 감각).
        self.min_area = float(self.declare_parameter('tuner_min_area_px', 40.0).value)

        # 현재 표를 파라미터에서 읽는다. 검출기와 **같은 로더**를 써야 눈으로 본 것과
        # 실제 검출이 일치한다 — 여기만 따로 파싱하면 또 두 벌이 된다.
        self.ranges = hsv_ranges.load(
            lambda n, d: self.declare_parameter(n, d).value,
            hsv_ranges.VALID_COLORS,
            on_error=self.get_logger().error)
        self._orig = {c: list(v) for c, v in self.ranges.items()}

        self.colors = list(hsv_ranges.VALID_COLORS)
        self.ci = 0          # 현재 색 인덱스
        self.ri = 0          # 현재 범위 인덱스 (빨강은 2개)
        self._pending = None  # 트랙바 → 범위 반영용
        self.br = CvBridge()

        self.create_subscription(Image, self.image_topic,
                                 self.image_callback, SENSOR_QOS)

        if self.debug_view:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 900, 700)
            cv2.namedWindow(WIN_MASK, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN_MASK, 640, 480)
            cv2.setMouseCallback(WIN, self.mouse_callback)
            for name, hi in _BARS:
                cv2.createTrackbar(name, WIN, 0, hi, lambda _v: None)
            self._push_trackbars()
        else:
            self.get_logger().warn(
                "debug_view=false 로 HSV 튜닝 노드를 띄웠다. 창이 없으면 할 일이 없다.")

        print(__doc__)
        self._announce()

    # ── 현재 선택 ────────────────────────────────────────────────
    @property
    def color(self):
        return self.colors[self.ci]

    def _cur(self):
        rs = self.ranges[self.color]
        self.ri = min(self.ri, len(rs) - 1)
        return rs[self.ri]

    def _set_cur(self, lo, hi):
        self.ranges[self.color][self.ri] = (tuple(lo), tuple(hi))

    def _announce(self):
        n = len(self.ranges[self.color])
        lo, hi = self._cur()
        print(f"\n▶ [{self.color}] 범위 {self.ri + 1}/{n}   lo={lo}  hi={hi}")

    # ── 트랙바 ↔ 범위 ───────────────────────────────────────────
    def _push_trackbars(self):
        lo, hi = self._cur()
        for name, val in zip((b[0] for b in _BARS),
                             (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])):
            cv2.setTrackbarPos(name, WIN, int(val))

    def _pull_trackbars(self):
        g = lambda n: cv2.getTrackbarPos(n, WIN)   # noqa: E731
        lo = (g("H lo"), g("S lo"), g("V lo"))
        hi = (g("H hi"), g("S hi"), g("V hi"))
        # S·V 는 하한>상한이면 마스크가 통째로 비어 "왜 안 잡히지" 가 된다 → 눌러 맞춘다.
        # H 는 색상환이라 lo>hi 를 허용한다(빨강).
        lo = (lo[0], min(lo[1], hi[1]), min(lo[2], hi[2]))
        self._set_cur(lo, hi)

    # ── 입력 ────────────────────────────────────────────────────
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._pending = (x, y)

    def _handle_key(self, k):
        if k in (ord('q'), 27):
            raise KeyboardInterrupt
        if k == ord('c'):
            self.ci = (self.ci + 1) % len(self.colors)
            self.ri = 0
            self._push_trackbars()
            self._announce()
        elif k == ord('n'):
            self.ri = (self.ri + 1) % len(self.ranges[self.color])
            self._push_trackbars()
            self._announce()
        elif k == ord('p'):
            line = hsv_ranges.format_yaml(self.color, self.ranges[self.color])
            print("\n" + "=" * 68)
            print("  config/vision.yaml 에 그대로 붙여넣으세요:")
            print(line)
            print("=" * 68 + "\n")
        elif k == ord('r'):
            self.ranges[self.color] = list(self._orig[self.color])
            self._push_trackbars()
            print(f"↩ [{self.color}] 원래값으로 되돌림")
            self._announce()

    # ── 화면 ────────────────────────────────────────────────────
    def image_callback(self, msg):
        if not self.debug_view:
            return
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 클릭한 픽셀을 품도록 넓힌다 (검출기와 같은 순수 로직을 쓴다)
        if self._pending is not None:
            x, y = self._pending
            self._pending = None
            if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                px = hsv[y, x]
                self.ranges[self.color] = hsv_ranges.widen_to_include(
                    self.ranges[self.color], px)
                self._push_trackbars()
                print(f"🖱 클릭 HSV({px[0]},{px[1]},{px[2]}) → [{self.color}] 범위 확장")
                self._announce()
        else:
            self._pull_trackbars()

        # 현재 색 전체 범위로 마스크 (검출기와 동일: OR 결합 + medianBlur 3)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in self.ranges[self.color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.medianBlur(mask, 3)

        view = frame.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        cv2.drawContours(view, big, -1, (0, 255, 255), 2)
        if big:
            top = max(big, key=cv2.contourArea)
            M = cv2.moments(top)
            if M["m00"] > 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                cv2.circle(view, (cx, cy), 6, (0, 0, 255), -1)

        lo, hi = self._cur()
        pct = 100.0 * int(mask.sum() // 255) / mask.size
        hud = [
            f"[{self.color}] range {self.ri + 1}/{len(self.ranges[self.color])}",
            f"lo {lo}  hi {hi}",
            f"mask {pct:5.1f}%   blobs>{int(self.min_area)}px: {len(big)}",
            "click=widen  c=color  n=range  p=print yaml  r=reset  q=quit",
        ]
        for i, t in enumerate(hud):
            cv2.putText(view, t, (8, 22 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0), 3)
            cv2.putText(view, t, (8, 22 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 1)

        cv2.imshow(WIN, view)
        cv2.imshow(WIN_MASK, mask)
        self._handle_key(cv2.waitKey(1) & 0xFF)


def main(args=None):
    rclpy.init(args=args)
    node = HSVSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 전에 지금까지 잡은 값을 전부 찍는다 — 창을 닫고 나서 "아 저장 안 했다" 를 막는다.
        print("\n" + "=" * 68)
        print("  종료 — 현재 값 (바꾼 것만 vision.yaml 에 반영하세요)")
        for c in hsv_ranges.VALID_COLORS:
            print(hsv_ranges.format_yaml(c, node.ranges[c]))
        print("=" * 68)
        node.destroy_node()
        if node.debug_view:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

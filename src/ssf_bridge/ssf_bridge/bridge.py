#!/usr/bin/env python3
"""ssf_bridge — Motor_run(토픽) ↔ Arduino Mega(시리얼) 브릿지.

    motor_control → Motor_run → [이 노드] → 시리얼 → Mega(ssf_boat.ino)
                    /boat_mode ← [이 노드] ← 시리얼 ←

작년(2025) 발굴 코드 `docs/기준/motor_run_bridge_simple.py` 를 되살린 것이다.
계약(형식·baud·디코드·QoS·L/R)은 리뷰에서 **현재 펌웨어와 일치 확인**됐으므로 그대로 두고,
리뷰 §2 가 지적한 3가지만 고쳤다(docs/기준/2025브리지_리뷰_회로AI전달.md):

  2-1 포트  : 기본값 /dev/ttyACM1 → **udev 심링크 /dev/ttyMEGA** (번호는 꽂는 순서로 바뀐다)
  2-2 패키징: 이 패키지 + entry point + pyserial 의존 + launch
  2-3 견고성: ① 종료 시 중립 발신  ② **상태 줄 읽기**  ③ write_timeout

🚨 ②(상태 읽기)는 화면용 기능이 아니라 **버그 수정**이다.
   펌웨어가 같은 시리얼로 10Hz 로 상태를 되쏘는데 작년 브릿지는 안 읽었다
   → OS 입력 버퍼가 계속 쌓인다. 읽어서 비우는 것 자체가 필요했다.
   읽는 김에 ROS 로 올려서 물가에서 모드를 볼 수 있게 한다(배 LED 는 멀면 안 보인다).

⚠️ **실물 검증 안 됨.** 아두이노가 아직 없다. 계약 일치와 순수 로직만 확인된 상태다.
   이 프로젝트는 "테스트 통과 = 안전" 이 아니라는 걸 8건으로 배웠다 — 벤치에서 반드시 확인할 것.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from ssf_bridge import status_parser as sp

# 상태 발행 QoS = **RELIABLE**(기본), depth 10.
#
# 🚨 처음엔 blackbox 처럼 BEST_EFFORT 로 뒀다가 바꿨다. 이유가 둘이다:
#  ① `ros2 topic echo /boat_mode` 가 기본 RELIABLE 구독이라 **아무것도 안 나온다.**
#     장비를 붙이고 "브릿지가 죽었나?" 를 한참 뒤지게 된다 —
#     이 저장소가 반복해서 당한 '에러 없는 침묵' 과 정확히 같은 함정이다.
#  ② 모드는 센서값이 아니라 **상태 보고**다. CLAUDE.md 3-3(V4)이 같은 이유로
#     `/wp_mode` 를 RELIABLE 로 유지했다. 여기도 같은 성격이다.
# RELIABLE 발행자는 BEST_EFFORT 구독자와도 호환된다(반대가 비호환) →
# 이쪽이 **엄격히 더 넓게 호환**된다. 잃는 게 없다.
STATUS_QOS = 10

_MAX_LINE = 128          # 이보다 긴 줄은 노이즈로 보고 버린다
_READ_PERIOD = 0.02      # 50Hz 로 시리얼을 훑는다 (펌웨어 상태는 10Hz)


class SsfBridge(Node):

    def __init__(self):
        super().__init__('ssf_bridge')

        port = self.declare_parameter("port", "/dev/ttyMEGA").value
        baud = int(self.declare_parameter("baud", 115200).value)
        # 아두이노가 멈췄을 때 write 가 executor 를 붙잡지 않게 한다(리뷰 2-3 ③)
        write_timeout = float(self.declare_parameter("write_timeout_sec", 0.1).value)
        self._publish_status = bool(
            self.declare_parameter("publish_status", True).value)

        import serial  # pyserial. 임포트를 여기 두어 파서 테스트가 의존을 안 탄다
        self._ser = serial.Serial(port, baud, timeout=0.0,
                                  write_timeout=write_timeout)

        self._rx = bytearray()
        self._bad_lines = 0

        # 부팅 즉시 중립을 한 번 박는다. 이전 실행의 마지막 PWM 이 남아 있을 수 있다.
        self._send(1500, 1500)

        self.pub_mode = self.create_publisher(Int32, "/boat_mode", STATUS_QOS)
        self.pub_watchdog = self.create_publisher(Bool, "/boat_cmd_watchdog", STATUS_QOS)
        self.pub_estop = self.create_publisher(Bool, "/boat_estop", STATUS_QOS)
        self.pub_boat_id = self.create_publisher(Int32, "/boat_id", STATUS_QOS)

        # 🚨 Motor_run 은 **RELIABLE**(motor_control 기본 발행)에 맞춘다.
        #    센서가 아니라 명령이다 — 한 장 놓치면 그만큼 늦게 반영된다.
        self.create_subscription(Int32, "Motor_run", self._on_motor_run, 10)

        self.create_timer(_READ_PERIOD, self._poll_serial)

        self.get_logger().info(
            f"ssf_bridge 시작 — {port} @ {baud}  "
            f"Motor_run 구독 / 상태발행={'on' if self._publish_status else 'off'}")

    # ───────────────────────── 내보내기 ─────────────────────────

    def _on_motor_run(self, msg):
        pwm_l, pwm_r = sp.decode_motor_run(msg.data)
        self._send(pwm_l, pwm_r)

    def _send(self, pwm_l, pwm_r):
        try:
            self._ser.write(sp.format_command(pwm_l, pwm_r).encode('ascii'))
        except Exception as e:                       # noqa: BLE001 — 어떤 시리얼 오류든 죽지 않는다
            # 여기서 예외로 노드가 죽으면 펌웨어 워치독(500ms)이 중립으로 잡는다.
            # 죽는 것보다 계속 살아서 재시도하는 편이 낫다 — 케이블이 잠깐 튄 것일 수 있다.
            self.get_logger().error(f"시리얼 쓰기 실패: {e}", throttle_duration_sec=2.0)

    # ───────────────────────── 읽어오기 ─────────────────────────

    def _poll_serial(self):
        """입력 버퍼를 비우고(=리뷰 2-3 ②) 온전한 줄만 골라 처리한다."""
        try:
            chunk = self._ser.read(4096)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(f"시리얼 읽기 실패: {e}", throttle_duration_sec=2.0)
            return
        if not chunk:
            return

        self._rx.extend(chunk)
        # 줄 단위로 잘라 처리. 마지막 조각(개행 없음)은 다음 호출로 넘긴다.
        while b'\n' in self._rx:
            raw, _, rest = self._rx.partition(b'\n')
            self._rx = bytearray(rest)
            self._handle_line(raw)

        # 개행이 영영 안 오는 경우(노이즈)에 버퍼가 무한히 크지 않게 자른다
        if len(self._rx) > _MAX_LINE:
            self._rx = bytearray()

    def _handle_line(self, raw):
        try:
            line = raw.decode('ascii', errors='ignore')
        except Exception:                            # noqa: BLE001
            return

        st = sp.parse_status_line(line)
        if st is None:
            # 상태 줄이 아니거나 깨진 줄. 펌웨어가 디버그 문장을 섞어 보낼 수도 있으니
            # 조용히 버리되, 계속 깨지면 그건 신호다(보드레이트·배선 의심).
            if line.strip():
                self._bad_lines += 1
                self.get_logger().warn(
                    f"해석 못한 시리얼 줄 {self._bad_lines}건 (최근: {line.strip()[:60]!r})",
                    throttle_duration_sec=10.0)
            return

        if not self._publish_status:
            return
        self.pub_mode.publish(Int32(data=st["mode"]))
        self.pub_watchdog.publish(Bool(data=st["watchdog"]))
        self.pub_estop.publish(Bool(data=st["estop"]))
        self.pub_boat_id.publish(Int32(data=st["boat_id"]))

    # ───────────────────────── 종료 ─────────────────────────

    def shutdown_neutral(self):
        """리뷰 2-3 ① — Ctrl+C 때 마지막 PWM 이 남지 않게 중립을 박고 닫는다.

        펌웨어 워치독(500ms)이 어차피 중립으로 잡지만, 그 500ms 동안 배가 움직인다.
        """
        try:
            self._send(1500, 1500)
            self._ser.flush()
        except Exception:                            # noqa: BLE001
            pass
        try:
            self._ser.close()
        except Exception:                            # noqa: BLE001
            pass


def _install_sigterm_handler():
    """🚨 SIGTERM 에서도 종료 중립이 나가게 한다.

    처음엔 KeyboardInterrupt(SIGINT)만 잡았는데, 가짜 Mega 시험에서 **종료 중립이
    안 나가는 것**을 발견했다. 파이썬 기본 SIGTERM 핸들러는 프로세스를 즉시 끝내서
    `finally` 가 아예 안 돈다.
    `ros2 launch` 는 Ctrl+C 때 SIGINT 를 먼저 보내지만, 응답이 늦으면 SIGTERM 으로
    올려친다. `kill <pid>` 도 SIGTERM 이다. 그 경로에서 마지막 PWM 이 남는다.
    (펌웨어 워치독 500ms 가 결국 중립으로 잡지만, 그 500ms 를 없애려고 넣은 기능이다)
    """
    import signal

    def _raise(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _raise)


def main(args=None):
    rclpy.init(args=args)
    _install_sigterm_handler()
    node = SsfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_neutral()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

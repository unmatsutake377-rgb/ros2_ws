# ⚠️ 2025(작년) 자료에서 발굴한 코드 — docs/기준/(작년코드 분류)에 보관. 빌드 대상 아님.
# 리뷰·판단 근거는 같은 폴더 2025브리지_리뷰_회로AI전달.md 참고.
# 원문 그대로 보존한다(수정 금지). 되살릴 경우 별도 패키지로 복사해 손본다.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time
import argparse

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

class MotorRunBridge(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('motor_run_bridge')

        # Mega 시리얼 오픈
        self.ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(1.0)

        # 시작 시 중립 한 번 보내기
        self.send_pwms(1500, 1500)

        # motor_control 노드가 퍼블리시하는 Motor_run 구독
        self.sub = self.create_subscription(
            Int32,
            'Motor_run',     # motor_control.py에서 쓰는 토픽 이름
            self.on_motor_run,
            10
        )

        self.get_logger().info(f"Serial opened on {port} @ {baud}")
        self.get_logger().info("Subscribing to 'Motor_run'")

    def on_motor_run(self, msg: Int32):
        data = int(msg.data)

        # motor_control.py에서:
        #   msg.data = pwm_r * 10000 + pwm_l
        pwm_r = data // 10000
        pwm_l = data - pwm_r * 10000

        pwm_r = clamp(pwm_r, 1000, 2000)
        pwm_l = clamp(pwm_l, 1000, 2000)

        # Mega 스케치에서 "L<왼쪽>,R<오른쪽>\n" 형식으로 읽도록 작성되어 있음
        self.send_pwms(pwm_l, pwm_r)

    def send_pwms(self, pwmL: int, pwmR: int):
        pwmL = clamp(pwmL, 1000, 2000)
        pwmR = clamp(pwmR, 1000, 2000)
        line = f"L{pwmL},R{pwmR}\n"
        self.ser.write(line.encode('ascii'))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--port',
        default='/dev/ttyACM1',
        help='Mega가 연결된 시리얼 포트 (예: COM3, /dev/ttyACM0)'
    )
    parser.add_argument('--baud', default=115200, type=int)
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = MotorRunBridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

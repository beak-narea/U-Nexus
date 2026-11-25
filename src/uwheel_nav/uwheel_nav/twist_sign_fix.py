#!/usr/bin/env python3
"""Twist 회전 부호 변환 릴레이.

이 로봇은 motor_driver 에 angular.z > 0 을 주면 실제로는 오른쪽(시계방향)으로 돈다 (ROS 규칙과 반대).
아이패드 조이스틱은 그에 맞춰 반전 없이 쓰고 있으므로, 회피/충돌 계산만 ROS 규칙으로 하도록
  입력:  /cmd_vel (아이패드)      → angular_sign 곱함 → /cmd_vel_ros
  출력:  /cmd_vel_safe_ros (계산) → angular_sign 곱함 → /cmd_vel_safe (모터)
두 번 사용한다. 로봇 배선/드라이버가 ROS 규칙으로 고쳐지면 angular_sign:=1.0 으로.

2026-09-14 실측: 제자리 회전 150회 중 150회 반대 방향, 배율 -1.05 (라이다 스캔 회전으로 측정)
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TwistSignFix(Node):
    def __init__(self):
        super().__init__('twist_sign_fix')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_ros')
        self.declare_parameter('angular_sign', -1.0)
        p = lambda n: self.get_parameter(n).value
        self.sign = float(p('angular_sign'))
        self.pub = self.create_publisher(Twist, p('output_topic'), 10)
        self.create_subscription(Twist, p('input_topic'), self.on_msg, 10)
        self.get_logger().info(f"{p('input_topic')} -> {p('output_topic')} (angular x {self.sign:+.0f})")

    def on_msg(self, msg):
        msg.angular.z *= self.sign
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TwistSignFix()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

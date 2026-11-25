#!/usr/bin/env python3

import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MotorCSVLogger(Node):

    def __init__(self):
        super().__init__("motor_csv_logger")

        self.latest_rpm = [0.0, 0.0]
        self.latest_position = [0.0, 0.0]
        self.latest_current = [0.0, 0.0]

        # joint_states 구독
        self.joint_sub = self.create_subscription(
            JointState,
            "/mdrobot_motor_driver/joint_states",
            self.joint_callback,
            10
        )

        # current 구독
        self.current_sub = self.create_subscription(
            Float64MultiArray,
            "/mdrobot_motor_driver/current",
            self.current_callback,
            10
        )

        # CSV 파일 생성
        filename = datetime.now().strftime("motor_data_%Y%m%d_%H%M%S.csv")
        self.csv_path = os.path.join(os.getcwd(), filename)

        self.csv_file = open(
            self.csv_path,
            "w",
            newline=""
        )

        self.writer = csv.writer(self.csv_file)

        # CSV header
        self.writer.writerow([
            "timestamp",
            "motor1_rpm",
            "motor1_position",
            "motor1_current",
            "motor2_rpm",
            "motor2_position",
            "motor2_current"
        ])

        self.csv_file.flush()

        # 1초마다 저장
        self.timer = self.create_timer(
            1.0,
            self.save_data
        )

        self.get_logger().info(
            f"CSV logging started: {self.csv_path}"
        )

    def joint_callback(self, msg):

        if len(msg.velocity) >= 2:
            self.latest_rpm[0] = msg.velocity[0]
            self.latest_rpm[1] = msg.velocity[1]

        if len(msg.position) >= 2:
            self.latest_position[0] = msg.position[0]
            self.latest_position[1] = msg.position[1]

    def current_callback(self, msg):

        if len(msg.data) >= 2:
            self.latest_current[0] = msg.data[0]
            self.latest_current[1] = msg.data[1]

    def save_data(self):

        timestamp = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )

        row = [
            timestamp,

            self.latest_rpm[0],
            self.latest_position[0],
            self.latest_current[0],

            self.latest_rpm[1],
            self.latest_position[1],
            self.latest_current[1]
        ]

        self.writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f"saved: "
            f"M1 rpm={self.latest_rpm[0]:.2f}, "
            f"pos={self.latest_position[0]:.2f}, "
            f"current={self.latest_current[0]:.2f} | "
            f"M2 rpm={self.latest_rpm[1]:.2f}, "
            f"pos={self.latest_position[1]:.2f}, "
            f"current={self.latest_current[1]:.2f}"
        )

    def close_csv(self):

        if hasattr(self, "csv_file"):
            self.csv_file.flush()
            self.csv_file.close()

            self.get_logger().info(
                f"CSV saved: {self.csv_path}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = MotorCSVLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close_csv()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
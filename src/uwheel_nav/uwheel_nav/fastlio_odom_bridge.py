#!/usr/bin/env python3
"""FAST-LIO /Odometry (Ouster IMU 자세) → odom -> base_link TF 로 변환.

FAST-LIO 의 body 는 Ouster IMU 위치(base_link 에서 약 x0.26 z0.51)이므로 그대로 쓰면
회전할 때 base_link 위치가 틀어진다. TF(base_link -> imu_frame, static)를 한 번 조회해서

    T_odom_base(t) = T_base_imu * T_fastlio(t) * T_base_imu^-1

로 바꾸면 시작 시점의 base_link 가 odom 원점이 된다. slam_toolbox 는 2D 이므로
기본값으로 x, y, yaw 만 남기고 z / roll / pitch 는 버린다(publish_2d).

주의: FAST-LIO 는 frame 이름을 fastlio_* 로 바꾼 config(fastlio_nav.yaml)로 실행해야
기존 base_link / map TF 와 충돌하지 않는다.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0
    y = math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0
    z = math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(x, r[2, 1] - r[1, 2])
    y = math.copysign(y, r[0, 2] - r[2, 0])
    z = math.copysign(z, r[1, 0] - r[0, 1])
    return x, y, z, w


class FastlioOdomBridge(Node):
    def __init__(self):
        super().__init__('fastlio_odom_bridge')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_data_frame')
        self.declare_parameter('publish_2d', True)
        # Ouster 타임스탬프가 시스템 시간과 다를 수 있어서 기본은 수신 시각 사용
        self.declare_parameter('use_msg_stamp', False)

        p = lambda n: self.get_parameter(n).value
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.imu_frame = p('imu_frame')
        self.publish_2d = bool(p('publish_2d'))
        self.use_msg_stamp = bool(p('use_msg_stamp'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, p('output_topic'), 10)
        self.create_subscription(Odometry, p('input_topic'), self.on_odom, 10)

        self.R_bi = None
        self.t_bi = None
        self.warned = False

    def lookup_extrinsic(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.imu_frame, Time())
        except TransformException as e:
            if not self.warned:
                self.get_logger().warn(f'TF {self.base_frame} -> {self.imu_frame} 대기 중: {e}')
                self.warned = True
            return False
        q = t.transform.rotation
        self.R_bi = quat_to_rot(q.x, q.y, q.z, q.w)
        tr = t.transform.translation
        self.t_bi = np.array([tr.x, tr.y, tr.z])
        self.get_logger().info(f'extrinsic {self.base_frame} -> {self.imu_frame}: t={self.t_bi.round(3)}')
        return True

    def on_odom(self, msg):
        if self.R_bi is None and not self.lookup_extrinsic():
            return
        q = msg.pose.pose.orientation
        pp = msg.pose.pose.position
        R_oi = quat_to_rot(q.x, q.y, q.z, q.w)
        t_oi = np.array([pp.x, pp.y, pp.z])

        R = self.R_bi @ R_oi @ self.R_bi.T
        t = self.R_bi @ (t_oi - R_oi @ self.R_bi.T @ self.t_bi) + self.t_bi

        if self.publish_2d:
            yaw = math.atan2(R[1, 0], R[0, 0])
            qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
            t = np.array([t[0], t[1], 0.0])
        else:
            qx, qy, qz, qw = rot_to_quat(R)

        stamp = msg.header.stamp if self.use_msg_stamp else self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t)
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf_pub.sendTransform(tf)

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, t)
        odom.pose.pose.orientation = tf.transform.rotation
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = FastlioOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

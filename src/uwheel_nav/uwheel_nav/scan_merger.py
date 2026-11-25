#!/usr/bin/env python3
"""2D 라이다 여러 대(LaserScan / PointCloud2)를 base_link 기준 LaserScan 하나로 합친다.

- 입력: scan_topics (LaserScan), cloud_topics (PointCloud2, 예: SOSLAB G3)
- 각 센서 → base_link 변환은 TF(static)로 조회 (all_sensors_launch.py 가 발행)
- 로봇 몸체 박스(self_filter_*) 안의 점과 높이 범위(min_z~max_z) 밖의 점은 제거
  → 자기 몸체를 장애물로 인식해서 Collision Monitor 가 멈춰버리는 것 방지
- 출력: output_topic (LaserScan, frame = base_link) → slam_toolbox 가 사용
        center_output_topic: 같은 점을 로봇 회전 중심(바퀴 축, base_link x=rotation_center_x) 기준으로 옮긴 스캔
        → collision_monitor / teleop_avoidance 가 사용 (회전할 때 뒤쪽이 휘둘리는 것을 올바르게 계산하기 위해)
        frame_id 는 base_link 로 두지만 원점이 회전 중심이다 (TF 는 추가/변경하지 않음)
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def transform_to_matrix(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    tr = t.transform.translation
    return rot, np.array([tr.x, tr.y, tr.z])


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.declare_parameter('scan_topics', ['/scan_g2b', '/scan2'])
        self.declare_parameter('cloud_topics', ['/lidar1/pointcloud', '/lidar0/pointcloud'])  # SOSLAB G3: lidarId 에 따라 둘 중 하나로 발행
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('center_output_topic', '/scan_merged_center')
        # 로봇 회전 중심(구동 바퀴 축) x 위치 (base_link 기준). 2026-09-14 bag ICP 추정: 0.47 (사분위 0.42~0.53)
        self.declare_parameter('rotation_center_x', 0.47)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('min_z', 0.03)
        self.declare_parameter('max_z', 1.2)
        self.declare_parameter('source_timeout', 0.5)
        # 로봇 몸체 박스 (base_link 기준, m)
        self.declare_parameter('self_filter_min_x', -0.03)
        self.declare_parameter('self_filter_max_x', 0.59)
        self.declare_parameter('self_filter_min_y', -0.31)
        self.declare_parameter('self_filter_max_y', 0.31)

        p = lambda n: self.get_parameter(n).value
        self.target_frame = p('target_frame')
        self.range_min = float(p('range_min'))
        self.range_max = float(p('range_max'))
        self.min_z, self.max_z = float(p('min_z')), float(p('max_z'))
        self.center_x = float(p('rotation_center_x'))
        self.source_timeout = float(p('source_timeout'))
        self.box = (float(p('self_filter_min_x')), float(p('self_filter_max_x')),
                    float(p('self_filter_min_y')), float(p('self_filter_max_y')))
        self.angle_inc = math.radians(float(p('angle_increment_deg')))
        self.num_bins = int(round(2 * math.pi / self.angle_inc))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_cache = {}      # frame_id -> (rot, trans)
        self.latest = {}        # topic -> (points Nx2 in target frame, receive time)
        self.warned = set()

        for topic in p('scan_topics'):
            if topic:
                self.create_subscription(
                    LaserScan, topic, lambda m, t=topic: self.on_scan(t, m), qos_profile_sensor_data)
        for topic in p('cloud_topics'):
            if topic:
                self.create_subscription(
                    PointCloud2, topic, lambda m, t=topic: self.on_cloud(t, m), qos_profile_sensor_data)

        self.pub = self.create_publisher(LaserScan, p('output_topic'), 10)
        self.pub_center = self.create_publisher(LaserScan, p('center_output_topic'), 10)
        self.create_timer(1.0 / float(p('publish_rate')), self.publish)
        self.get_logger().info(
            f"scan_merger: scans={p('scan_topics')} clouds={p('cloud_topics')} -> {p('output_topic')}")

    def lookup(self, frame_id):
        if frame_id in self.tf_cache:
            return self.tf_cache[frame_id]
        try:
            t = self.tf_buffer.lookup_transform(self.target_frame, frame_id, Time())
        except TransformException as e:
            if frame_id not in self.warned:
                self.get_logger().warn(f'TF {self.target_frame} -> {frame_id} 없음 (대기 중): {e}')
                self.warned.add(frame_id)
            return None
        self.tf_cache[frame_id] = transform_to_matrix(t)
        self.get_logger().info(f'TF {self.target_frame} -> {frame_id} 확보')
        return self.tf_cache[frame_id]

    def store(self, topic, frame_id, pts_sensor):
        tf = self.lookup(frame_id)
        if tf is None:
            return
        rot, trans = tf
        pts = pts_sensor @ rot.T + trans
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        bx0, bx1, by0, by1 = self.box
        in_body = (x > bx0) & (x < bx1) & (y > by0) & (y < by1)
        keep = (z >= self.min_z) & (z <= self.max_z) & ~in_body
        self.latest[topic] = (pts[keep, :2], self.get_clock().now())

    def on_scan(self, topic, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        valid = np.isfinite(r) & (r >= max(msg.range_min, self.range_min)) & (r <= msg.range_max)
        r, a = r[valid], a[valid]
        pts = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r)))
        self.store(topic, msg.header.frame_id, pts)

    def on_cloud(self, topic, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        d = np.linalg.norm(pts, axis=1)
        pts = pts[np.isfinite(d) & (d >= self.range_min)]
        self.store(topic, msg.header.frame_id, pts)

    def ranges_from(self, offset_x):
        now = self.get_clock().now()
        ranges = np.full(self.num_bins, np.inf)
        for topic, (pts, stamp) in self.latest.items():
            if (now - stamp).nanoseconds * 1e-9 > self.source_timeout or pts.size == 0:
                continue
            x = pts[:, 0] - offset_x
            y = pts[:, 1]
            r = np.hypot(x, y)
            ok = (r >= self.range_min) & (r <= self.range_max)
            r, x, y = r[ok], x[ok], y[ok]
            idx = np.rint((np.arctan2(y, x) + math.pi) / self.angle_inc).astype(np.int64)
            idx %= self.num_bins
            np.minimum.at(ranges, idx, r)
        return now, ranges

    def make_msg(self, now, ranges):
        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.target_frame
        msg.angle_min = -math.pi
        msg.angle_increment = self.angle_inc
        msg.angle_max = -math.pi + self.angle_inc * (self.num_bins - 1)
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges.astype(np.float32).tolist()
        return msg

    def publish(self):
        now, ranges = self.ranges_from(0.0)
        self.pub.publish(self.make_msg(now, ranges))
        now, ranges = self.ranges_from(self.center_x)
        self.pub_center.publish(self.make_msg(now, ranges))


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

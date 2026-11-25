#!/usr/bin/env python3
"""PointCloud2 -> foxglove_msgs/CompressedPointCloud (Draco) republisher.

Follows Foxglove CompressedPointCloud guidance:
https://foxglove.dev/blog/compressed-point-cloud-support-in-foxglove

Default input is Ouster `/ouster/points`. Downstream algorithms should keep
using the raw PointCloud2; this topic is for Foxglove visualization / bandwidth.
"""

from __future__ import annotations

import time

import DracoPy
import numpy as np
import rclpy
from foxglove_msgs.msg import CompressedPointCloud
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# Prefer these names when picking a scalar intensity-like channel for Foxglove.
_INTENSITY_CANDIDATES = (
    "intensity",
    "signal",
    "reflectivity",
    "ambient",
    "near_ir",
)


class CompressPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("compress_pointcloud")

        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("output_topic", "/ouster/points/compressed")
        self.declare_parameter("quantization_bits", 14)
        self.declare_parameter("compression_level", 7)
        # False = better ratio (order irrelevant for LiDAR viz). Blog demo uses True.
        self.declare_parameter("preserve_order", False)
        self.declare_parameter("skip_nans", True)
        # Extra named Draco attributes to carry when present on the cloud.
        self.declare_parameter("extra_fields", ["ring", "reflectivity", "signal"])
        self.declare_parameter("log_stats", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.quantization_bits = int(self.get_parameter("quantization_bits").value)
        self.compression_level = int(self.get_parameter("compression_level").value)
        self.preserve_order = bool(self.get_parameter("preserve_order").value)
        self.skip_nans = bool(self.get_parameter("skip_nans").value)
        self.extra_fields = [str(f) for f in self.get_parameter("extra_fields").value]
        self.log_stats = bool(self.get_parameter("log_stats").value)

        self._identity_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        self._pub = self.create_publisher(
            CompressedPointCloud,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self._sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )

        self._frames = 0
        self._encode_ms_ema = 0.0
        self._ratio_ema = 0.0

        self.get_logger().info(
            f"compressing {self.input_topic} -> {self.output_topic} "
            f"(draco q={self.quantization_bits} level={self.compression_level})"
        )

    def _field_names(self, msg: PointCloud2) -> set[str]:
        return {f.name for f in msg.fields}

    def _pick_intensity(self, names: set[str]) -> str | None:
        for name in _INTENSITY_CANDIDATES:
            if name in names:
                return name
        return None

    def _on_cloud(self, msg: PointCloud2) -> None:
        names = self._field_names(msg)
        if not {"x", "y", "z"}.issubset(names):
            self.get_logger().warn(
                "PointCloud2 missing x/y/z — dropping frame",
                throttle_duration_sec=2.0,
            )
            return

        intensity_name = self._pick_intensity(names)
        wanted = ["x", "y", "z"]
        if intensity_name is not None:
            wanted.append(intensity_name)
        for name in self.extra_fields:
            if name in names and name not in wanted:
                wanted.append(name)

        t0 = time.perf_counter()
        try:
            pts = point_cloud2.read_points(
                msg,
                field_names=wanted,
                skip_nans=self.skip_nans,
            )
        except Exception as exc:  # noqa: BLE001 — keep node alive on bad frames
            self.get_logger().error(
                f"read_points failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        # read_points returns a structured ndarray (or empty)
        pts = np.asarray(pts)
        if pts.size == 0:
            return

        positions = np.column_stack(
            [pts["x"], pts["y"], pts["z"]]
        ).astype(np.float32, copy=False)

        generic: dict[str, np.ndarray] = {}
        for name in wanted[3:]:
            col = np.asarray(pts[name]).reshape(-1, 1)
            generic[name] = self._draco_typed(col, name)

        # Drop non-finite leftovers (organized clouds sometimes keep inf even with skip_nans)
        finite = np.isfinite(positions).all(axis=1)
        if not np.all(finite):
            positions = positions[finite]
            generic = {k: v[finite] for k, v in generic.items()}
        if positions.shape[0] == 0:
            return

        try:
            encoded = DracoPy.encode(
                positions,
                faces=None,
                quantization_bits=self.quantization_bits,
                compression_level=self.compression_level,
                create_metadata=False,
                preserve_order=self.preserve_order,
                generic_attributes=generic or None,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Draco encode failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        out = CompressedPointCloud()
        out.timestamp = msg.header.stamp
        out.frame_id = msg.header.frame_id
        out.pose = self._identity_pose
        out.data = encoded
        out.format = "draco"
        self._pub.publish(out)

        if self.log_stats:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            raw_n = len(msg.data)
            ratio = (raw_n / len(encoded)) if encoded else 0.0
            a = 0.1
            self._encode_ms_ema = (1 - a) * self._encode_ms_ema + a * dt_ms if self._frames else dt_ms
            self._ratio_ema = (1 - a) * self._ratio_ema + a * ratio if self._frames else ratio
            self._frames += 1
            self.get_logger().info(
                f"pts={positions.shape[0]} raw={raw_n}B compressed={len(encoded)}B "
                f"ratio~{self._ratio_ema:.1f}x encode~{self._encode_ms_ema:.1f}ms",
                throttle_duration_sec=2.0,
            )

    @staticmethod
    def _draco_typed(col: np.ndarray, name: str) -> np.ndarray:
        """Cast attribute columns to DracoPy-supported dtypes."""
        if name == "ring":
            return np.rint(col).astype(np.uint16, copy=False)
        if name in ("reflectivity",) and col.dtype == np.uint8:
            return col.astype(np.uint8, copy=False)
        # intensity / signal / ambient / near_ir -> float32
        return col.astype(np.float32, copy=False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CompressPointCloudNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

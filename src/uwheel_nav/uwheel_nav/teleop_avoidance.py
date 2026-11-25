#!/usr/bin/env python3
"""조종 명령 보조 회피 (DWA 방식, 지도/odom 불필요).

아이패드 /cmd_vel 을 받아서, 앞으로 sim_time 초 동안의 경로를 여러 (v, w) 후보로 시뮬레이션한다.
  - 명령 그대로 가도 sim_time 안에 부딪히지 않으면 → 그대로 통과 (조종 우선, 좁은 곳에서도 안 끼어듦)
  - 부딪힐 경로면 → 안 부딪히면서 (명령과 비슷함 + 속도 유지 + 장애물 여유 + heading_time 뒤 앞이 뚫린 정도)
    비용이 가장 낮은 (v, w) 선택 = 열린 쪽으로 방향을 틀어서 피함 (필요하면 제자리 회전)
  - 전진 속도가 min_linear 미만(거의 제자리 회전)이거나 후진이면 → 그대로 통과 (collision_monitor 가 감속/정지)
  - 피할 곳이 없으면 → 전진/후진은 0, 회전 명령만 통과
  - 조종 명령이 0 이면 항상 0 (스스로 움직이지 않음)
출력은 collision_monitor 로 들어가서 최종 감속/정지 검사를 한 번 더 받는다.

장애물: /scan_merged_center (원점 = 로봇 회전 중심 base_link x=0.47, scan_merger 가 로봇 몸체 점 제거한 것)
시뮬레이션의 (v, w) 는 회전 중심(바퀴 축)의 움직임 → 제자리 회전 시 뒤쪽이 크게 휘둘리는 것까지 계산
footprint: scan_merger self_filter / collision_monitor FootprintApproach 와 같은 박스 + safety_margin
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TeleopAvoidance(Node):
    def __init__(self):
        super().__init__('teleop_avoidance')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_avoid')
        self.declare_parameter('scan_topic', '/scan_merged_center')   # 원점 = 회전 중심 (scan_merger)
        self.declare_parameter('rotation_center_x', 0.47)                # scan_merger 와 같은 값
        # footprint (base_link 기준) — collision_monitor.yaml FootprintApproach 와 동일하게 유지
        self.declare_parameter('footprint_min_x', -0.03)
        self.declare_parameter('footprint_max_x', 0.59)
        self.declare_parameter('footprint_min_y', -0.31)
        self.declare_parameter('footprint_max_y', 0.31)
        self.declare_parameter('safety_margin', 0.10)       # 이 거리 안으로 들어가는 경로는 충돌로 간주 (footprint 추정값 오차 고려)
        self.declare_parameter('desired_clearance', 0.30)   # 이보다 가까워지면 여유 있는 쪽으로 틀기 시작
        self.declare_parameter('sim_time', 3.0)             # 앞으로 몇 초 경로를 볼지 (길수록 일찍 틀기 시작)
        self.declare_parameter('sim_dt', 0.15)
        self.declare_parameter('heading_time', 1.0)         # 이 시간 뒤 자세에서 앞이 얼마나 뚫렸는지 평가 (제자리 회전 후보의 검사 시간도 이것)
        self.declare_parameter('free_cap', 2.0)             # 앞이 이만큼(m) 뚫려 있으면 충분
        self.declare_parameter('w_free', 1.5)               # 앞이 막힌 방향에 대한 비용
        self.declare_parameter('w_switch', 0.3)             # 회피 방향(좌/우)을 바꾸는 것에 대한 비용 (흔들림 방지)
        self.declare_parameter('max_angular', 1.0)          # 회피용 최대 회전 속도 rad/s
        self.declare_parameter('angular_samples', 21)
        self.declare_parameter('w_deviation', 0.6)          # 명령 회전값과 달라지는 것에 대한 비용
        self.declare_parameter('w_speed', 1.0)              # 속도가 줄어드는 것에 대한 비용
        self.declare_parameter('w_clearance', 4.0)          # 장애물에 가까워지는 것에 대한 비용
        self.declare_parameter('smoothing', 0.3)            # 회피 회전값 저역통과 (0=없음, 0.9=강함)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('min_linear', 0.08)          # 이보다 느린 전진(거의 제자리 회전)은 개입 안 함
        self.declare_parameter('avoid_reverse', False)      # 후진도 회피할지 (라이다가 앞쪽 위주라 기본 끔)

        p = lambda n: self.get_parameter(n).value
        m = float(p('safety_margin'))
        cx = float(p('rotation_center_x'))   # footprint 는 base_link 기준으로 적고, 회전 중심 기준으로 옮겨서 사용
        self.box = (float(p('footprint_min_x')) - m - cx, float(p('footprint_max_x')) + m - cx,
                    float(p('footprint_min_y')) - m, float(p('footprint_max_y')) + m)
        self.desired_clearance = float(p('desired_clearance'))
        self.sim_time = float(p('sim_time'))
        self.sim_dt = float(p('sim_dt'))
        self.max_w = float(p('max_angular'))
        self.w_samples = int(p('angular_samples'))
        self.k_dev = float(p('w_deviation'))
        self.k_speed = float(p('w_speed'))
        self.k_clear = float(p('w_clearance'))
        self.smoothing = float(p('smoothing'))
        self.heading_time = float(p('heading_time'))
        self.free_cap = float(p('free_cap'))
        self.k_free = float(p('w_free'))
        self.k_switch = float(p('w_switch'))
        self.scan_timeout = float(p('scan_timeout'))
        self.min_linear = float(p('min_linear'))
        self.avoid_reverse = bool(p('avoid_reverse'))

        self.points = None      # Nx2 (base_link)
        self.scan_time = None
        self.prev_w = None
        self.avoiding = False

        self.pub = self.create_publisher(Twist, p('output_topic'), 10)
        self.create_subscription(LaserScan, p('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, p('input_topic'), self.on_cmd, 1)
        self.get_logger().info(f"teleop_avoidance: {p('input_topic')} -> {p('output_topic')}")

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.points = np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))
        self.scan_time = self.get_clock().now()

    def evaluate(self, v, w, pts, horizon=None):
        """(v, w) 후보 K개 평가.
        clear: horizon(기본 sim_time) 동안 footprint(+margin) 와 장애물 최소 거리 (K,). 0 이면 충돌
        ahead: heading_time 뒤 자세에서 로봇 폭만큼의 앞 통로가 뚫린 거리 (K,), 최대 free_cap"""
        v = np.atleast_1d(np.asarray(v, dtype=np.float64))
        w = np.atleast_1d(np.asarray(w, dtype=np.float64))
        hz = np.full(v.shape, self.sim_time) if horizon is None else np.atleast_1d(horizon)
        t1 = np.arange(self.sim_dt, self.sim_time + 1e-9, self.sim_dt)
        t = t1[None, :]                                                               # (1, T)
        th = w[:, None] * t                                                           # (K, T)
        straight = np.abs(w) < 1e-4
        w_safe = np.where(straight, 1.0, w)[:, None]
        x = np.where(straight[:, None], v[:, None] * t, v[:, None] / w_safe * np.sin(th))
        y = np.where(straight[:, None], 0.0, v[:, None] / w_safe * (1.0 - np.cos(th)))
        c, s = np.cos(th)[..., None], np.sin(th)[..., None]                           # (K, T, 1)
        dx = pts[None, None, :, 0] - x[..., None]                                     # (K, T, N)
        dy = pts[None, None, :, 1] - y[..., None]
        px = c * dx + s * dy          # 각 시점의 로봇 좌표계로 변환
        py = -s * dx + c * dy
        x0, x1, y0, y1 = self.box
        ex = np.maximum(np.maximum(x0 - px, px - x1), 0.0)
        ey = np.maximum(np.maximum(y0 - py, py - y1), 0.0)
        d = np.hypot(ex, ey).min(axis=2)                                              # (K, T)
        d[t1[None, :] > hz[:, None] + 1e-9] = np.inf
        clear = d.min(axis=1)
        h = int(np.argmin(np.abs(t1 - self.heading_time)))
        pxh, pyh = px[:, h, :], py[:, h, :]
        corridor = (pyh > y0) & (pyh < y1) & (pxh > x1)
        ahead = np.where(corridor, pxh - x1, np.inf).min(axis=1)
        return clear, np.minimum(ahead, self.free_cap)

    def publish(self, v, w):
        out = Twist()
        out.linear.x = float(v)
        out.angular.z = float(w)
        self.pub.publish(out)

    def on_cmd(self, msg):
        v_cmd, w_cmd = msg.linear.x, msg.angular.z
        if abs(v_cmd) < 1e-3 and abs(w_cmd) < 1e-3:
            self.prev_w = None
            self.publish(0.0, 0.0)
            return
        stale = (self.scan_time is None or
                 (self.get_clock().now() - self.scan_time).nanoseconds * 1e-9 > self.scan_timeout)
        if stale:
            # 라이다가 끊기면 회피 판단 불가 → 그대로 넘기고 collision_monitor(source_timeout)가 정지시킴
            self.publish(v_cmd, w_cmd)
            return
        if abs(v_cmd) < self.min_linear or (v_cmd < 0.0 and not self.avoid_reverse):
            # 제자리 회전에 가까운 명령 / 후진은 조종 그대로 (collision_monitor 가 검사)
            self.finish(v_cmd, w_cmd, v_cmd, w_cmd, False)
            return

        # 관심 거리 안의 점만 사용
        reach = abs(v_cmd) * self.sim_time + abs(self.max_w) * self.sim_time * 0.6 + 1.0
        pts = self.points[np.hypot(self.points[:, 0], self.points[:, 1]) < reach]
        if pts.shape[0] > 360:
            pts = pts[:: int(np.ceil(pts.shape[0] / 360))]
        if pts.shape[0] == 0:
            self.finish(v_cmd, w_cmd, v_cmd, w_cmd, False)
            return

        # 명령 그대로 가도 sim_time 안에 안 부딪히면 그대로 통과 (조종 우선)
        if self.evaluate(v_cmd, w_cmd, pts)[0][0] > 0.0:
            self.finish(v_cmd, w_cmd, v_cmd, w_cmd, False)
            return

        w_grid = np.unique(np.append(np.linspace(-self.max_w, self.max_w, self.w_samples), w_cmd))
        S, W = np.meshgrid(np.array([1.0, 0.75, 0.5, 0.25]), w_grid, indexing='ij')
        S, W = S.ravel(), W.ravel()
        spin = w_grid[np.abs(w_grid) > 0.2]                      # 제자리 회전 후보 (heading_time 동안만 검사)
        S = np.concatenate([S, np.zeros(spin.size)])
        W = np.concatenate([W, spin])
        H = np.where(S > 0.0, self.sim_time, self.heading_time)
        clear, ahead = self.evaluate(v_cmd * S, W, pts, H)
        cost = (self.k_dev * np.abs(W - w_cmd) / max(self.max_w, 1e-3)
                + self.k_speed * (1.0 - S)
                + self.k_clear * np.maximum(0.0, self.desired_clearance - clear) / self.desired_clearance
                + self.k_free * (1.0 - ahead / self.free_cap))
        if self.prev_w is not None and abs(self.prev_w) > 0.05:
            cost += self.k_switch * ((np.sign(W) != np.sign(self.prev_w)) & (np.abs(W) > 0.05))
        cost[clear <= 0.0] = np.inf
        i = int(np.argmin(cost))
        best = None if not np.isfinite(cost[i]) else (cost[i], v_cmd * S[i], W[i])

        if best is None:
            # 어느 방향도 막힘 → 전진/후진 정지, 회전 명령만 허용
            self.finish(v_cmd, w_cmd, 0.0, w_cmd, True, blocked=True)
            return
        _, v, w = best
        changed = abs(w - w_cmd) > 1e-3 or abs(v - v_cmd) > 1e-3
        self.finish(v_cmd, w_cmd, v, w, changed)

    def finish(self, v_cmd, w_cmd, v, w, changed, blocked=False):
        if changed and self.prev_w is not None and self.smoothing > 0.0:
            w = self.smoothing * self.prev_w + (1.0 - self.smoothing) * w
        self.prev_w = w if changed else None
        if changed != self.avoiding:
            self.avoiding = changed
            if blocked:
                self.get_logger().info('회피 경로 없음 → 전진 정지 (회전만 허용)')
            elif changed:
                self.get_logger().info(f'회피 시작: cmd v={v_cmd:.2f} w={w_cmd:.2f} → v={v:.2f} w={w:.2f}')
            else:
                self.get_logger().info('회피 종료: 명령 그대로 통과')
        self.publish(v, w)


def main():
    rclpy.init()
    node = TeleopAvoidance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

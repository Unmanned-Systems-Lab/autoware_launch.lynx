#!/usr/bin/env python3

import argparse
import math
from copy import deepcopy
from typing import Optional

import rclpy
from autoware_planning_msgs.msg import Trajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class TrajectoryKeepaliveRelay(Node):
    def __init__(
        self,
        input_topic: str,
        output_topic: str,
        rate_hz: float,
        refresh_stamp: bool,
        goal_decel_distance_m: float,
        goal_min_speed_mps: float,
        curve_max_lat_acc_mps2: float,
        curve_min_speed_mps: float,
    ) -> None:
        super().__init__("trajectory_keepalive_relay")

        self.refresh_stamp = refresh_stamp
        self.goal_decel_distance_m = max(goal_decel_distance_m, 0.1)
        self.goal_min_speed_mps = max(goal_min_speed_mps, 0.0)
        self.curve_max_lat_acc_mps2 = max(curve_max_lat_acc_mps2, 0.1)
        self.curve_min_speed_mps = max(curve_min_speed_mps, 0.0)
        self.latest_msg: Optional[Trajectory] = None

        in_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        out_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Trajectory, input_topic, self.on_input, in_qos)
        self.pub = self.create_publisher(Trajectory, output_topic, out_qos)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.02), self.on_timer)

        self.get_logger().info(f"input_topic={input_topic}")
        self.get_logger().info(f"output_topic={output_topic}")
        self.get_logger().info(f"rate_hz={rate_hz}")
        self.get_logger().info(f"refresh_stamp={refresh_stamp}")
        self.get_logger().info(f"goal_decel_distance_m={self.goal_decel_distance_m}")
        self.get_logger().info(f"goal_min_speed_mps={self.goal_min_speed_mps}")
        self.get_logger().info(f"curve_max_lat_acc_mps2={self.curve_max_lat_acc_mps2}")
        self.get_logger().info(f"curve_min_speed_mps={self.curve_min_speed_mps}")

    @staticmethod
    def _normalize_angle(rad: float) -> float:
        return math.atan2(math.sin(rad), math.cos(rad))

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _distance2d(p0, p1) -> float:
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        return math.hypot(dx, dy)

    def _apply_speed_profile(self, msg: Trajectory) -> None:
        points = msg.points
        if len(points) < 2:
            return

        n = len(points)
        yaws = [self._yaw_from_quat(p.pose.orientation) for p in points]
        seg_len = [0.0] * (n - 1)
        for i in range(n - 1):
            seg_len[i] = max(
                self._distance2d(points[i].pose.position, points[i + 1].pose.position),
                1e-3,
            )

        dist_to_goal = [0.0] * n
        for i in range(n - 2, -1, -1):
            dist_to_goal[i] = dist_to_goal[i + 1] + seg_len[i]

        # Slow down near goal and on high-curvature segments.
        for i in range(n - 1):
            original_v = points[i].longitudinal_velocity_mps
            sign = -1.0 if original_v < 0.0 else 1.0
            base_speed = abs(original_v)
            if base_speed < 1e-3:
                continue

            # Goal-distance based speed limit.
            goal_ratio = max(0.0, min(dist_to_goal[i] / self.goal_decel_distance_m, 1.0))
            goal_limit = self.goal_min_speed_mps + (
                (base_speed - self.goal_min_speed_mps) * goal_ratio
            )

            # Curvature proxy from heading change over segment length.
            if i == 0:
                dyaw = abs(self._normalize_angle(yaws[1] - yaws[0]))
                ds = seg_len[0]
            else:
                dyaw = abs(self._normalize_angle(yaws[i] - yaws[i - 1]))
                ds = seg_len[i - 1]
            curvature = dyaw / max(ds, 1e-3)

            if curvature < 1e-4:
                curve_limit = base_speed
            else:
                curve_limit = math.sqrt(self.curve_max_lat_acc_mps2 / curvature)

            v_target = min(base_speed, goal_limit, curve_limit)
            min_speed = min(base_speed, self.goal_min_speed_mps, self.curve_min_speed_mps)
            v_target = max(min_speed, v_target)
            points[i].longitudinal_velocity_mps = sign * v_target

        # Add explicit zero at direction switching points.
        for i in range(n - 1):
            if points[i].longitudinal_velocity_mps * points[i + 1].longitudinal_velocity_mps < 0.0:
                points[i].longitudinal_velocity_mps = 0.0

        # Last point must be stop.
        points[-1].longitudinal_velocity_mps = 0.0

    def on_input(self, msg: Trajectory) -> None:
        shaped = deepcopy(msg)
        self._apply_speed_profile(shaped)
        self.latest_msg = shaped
        self.publish_latest()

    def on_timer(self) -> None:
        self.publish_latest()

    def publish_latest(self) -> None:
        if self.latest_msg is None:
            return

        out_msg = deepcopy(self.latest_msg)
        if self.refresh_stamp:
            out_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(out_msg)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--goal-decel-distance-m", type=float, default=8.0)
    parser.add_argument("--goal-min-speed-mps", type=float, default=0.3)
    parser.add_argument("--curve-max-lat-acc-mps2", type=float, default=0.7)
    parser.add_argument("--curve-min-speed-mps", type=float, default=0.5)
    parser.add_argument(
        "--refresh-stamp",
        action="store_true",
        help="replace header stamp with current node time before publish",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TrajectoryKeepaliveRelay(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        rate_hz=args.rate_hz,
        refresh_stamp=args.refresh_stamp,
        goal_decel_distance_m=args.goal_decel_distance_m,
        goal_min_speed_mps=args.goal_min_speed_mps,
        curve_max_lat_acc_mps2=args.curve_max_lat_acc_mps2,
        curve_min_speed_mps=args.curve_min_speed_mps,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import argparse
import uuid
from typing import Optional

import rclpy
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID


class FreespaceGoalRouteBridge(Node):
    def __init__(
        self,
        goal_topic: str,
        odom_topic: str,
        route_topic: str,
        default_frame_id: str,
    ) -> None:
        super().__init__("freespace_goal_route_bridge")

        self.default_frame_id = default_frame_id
        self.latest_goal: Optional[PoseStamped] = None
        self.latest_odom: Optional[Odometry] = None
        self.first_goal_received = False
        self.first_odom_received = False
        self.last_wait_log_ns = 0

        route_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        data_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.route_pub = self.create_publisher(LaneletRoute, route_topic, route_qos)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.on_goal, data_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, data_qos)

        # Republish latest route periodically for robustness.
        self.timer = self.create_timer(0.5, self.on_timer)

        self.get_logger().info(f"goal_topic={goal_topic}")
        self.get_logger().info(f"odom_topic={odom_topic}")
        self.get_logger().info(f"route_topic={route_topic}")

    def on_goal(self, msg: PoseStamped) -> None:
        self.latest_goal = msg
        if not self.first_goal_received:
            self.first_goal_received = True
            self.get_logger().warning("received first goal")
        self.publish_route_if_ready(force_log=True)

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if not self.first_odom_received:
            self.first_odom_received = True
            self.get_logger().warning("received first odometry")
        self.publish_route_if_ready(force_log=False)

    def on_timer(self) -> None:
        self.publish_route_if_ready(force_log=False)

    def publish_route_if_ready(self, force_log: bool) -> None:
        if self.latest_goal is None or self.latest_odom is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_wait_log_ns > 5 * 1_000_000_000:
                waiting = []
                if self.latest_goal is None:
                    waiting.append("goal")
                if self.latest_odom is None:
                    waiting.append("odometry")
                self.get_logger().warning(f"waiting for {', '.join(waiting)}")
                self.last_wait_log_ns = now_ns
            return

        route = LaneletRoute()
        route.header = self.latest_goal.header
        if not route.header.frame_id:
            route.header.frame_id = self.default_frame_id

        route.start_pose = self.latest_odom.pose.pose
        route.goal_pose = self.latest_goal.pose
        route.allow_modification = False

        route_uuid = UUID()
        route_uuid.uuid = list(uuid.uuid4().bytes)
        route.uuid = route_uuid

        self.route_pub.publish(route)
        if force_log:
            self.get_logger().warning("published synthetic LaneletRoute from RViz goal + odometry")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-topic", default="/planning/mission_planning/goal")
    parser.add_argument("--odom-topic", default="/localization/kinematic_state")
    parser.add_argument("--route-topic", default="/planning/mission_planning/route")
    parser.add_argument("--default-frame-id", default="map")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = FreespaceGoalRouteBridge(
        goal_topic=args.goal_topic,
        odom_topic=args.odom_topic,
        route_topic=args.route_topic,
        default_frame_id=args.default_frame_id,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

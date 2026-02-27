#!/usr/bin/env python3

import argparse

import rclpy
from autoware_internal_planning_msgs.msg import Scenario
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class InternalScenarioPublisher(Node):
    def __init__(self, topic: str, interval_sec: float) -> None:
        super().__init__("internal_scenario_publisher")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Scenario, topic, qos)
        self.timer = self.create_timer(interval_sec, self.on_timer)

        self.msg = Scenario()
        self.msg.current_scenario = Scenario.PARKING
        self.msg.activating_scenarios = [Scenario.PARKING]

        self.get_logger().info(
            f"publishing internal parking scenario to {topic} every {interval_sec:.2f}s"
        )
        self.on_timer()

    def on_timer(self) -> None:
        self.pub.publish(self.msg)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/planning/scenario_planning/scenario")
    parser.add_argument("--interval-sec", type=float, default=0.5)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = InternalScenarioPublisher(topic=args.topic, interval_sec=args.interval_sec)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

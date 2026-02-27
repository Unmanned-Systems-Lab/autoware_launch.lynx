#!/usr/bin/env python3

import argparse
import os
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None


@dataclass
class StaticMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: List[int]


def read_yaml(path: str) -> Dict:
    if yaml is not None:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    data: Dict = {}
    pending_list_key = None
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            if pending_list_key and line.startswith("-"):
                item = line[1:].strip()
                try:
                    data[pending_list_key].append(float(item))
                except ValueError:
                    data[pending_list_key].append(item)
                continue

            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()

            if value == "":
                data[key] = []
                pending_list_key = key
                continue

            pending_list_key = None
            if value.startswith("[") and value.endswith("]"):
                value = value[1:-1]
                data[key] = [float(v.strip()) for v in value.split(",")]
            elif value in ("true", "false", "True", "False"):
                data[key] = value.lower() == "true"
            else:
                try:
                    if "." in value:
                        data[key] = float(value)
                    else:
                        data[key] = int(value)
                except ValueError:
                    data[key] = value
    return data


def token_stream(f):
    token = b""
    while True:
        ch = f.read(1)
        if not ch:
            if token:
                yield token.decode("ascii")
            break
        if ch == b"#":
            f.readline()
            continue
        if ch.isspace():
            if token:
                yield token.decode("ascii")
                token = b""
            continue
        token += ch


def load_pgm(path: str):
    with open(path, "rb") as f:
        tokens = token_stream(f)
        magic = next(tokens)
        if magic not in ("P2", "P5"):
            raise ValueError(f"unsupported PGM format: {magic}")

        width = int(next(tokens))
        height = int(next(tokens))
        maxval = int(next(tokens))
        if maxval <= 0 or maxval > 255:
            raise ValueError(f"unsupported PGM maxval: {maxval}")

        if magic == "P5":
            pixels = list(f.read(width * height))
            if len(pixels) != width * height:
                raise ValueError("invalid PGM: unexpected data length")
        else:
            pixels = [int(next(tokens)) for _ in range(width * height)]

    return width, height, pixels


def load_static_map(yaml_path: str) -> StaticMap:
    cfg = read_yaml(yaml_path)
    image_field = cfg.get("image")
    if image_field is None:
        raise ValueError(f"missing 'image' in {yaml_path}")

    image_path = image_field
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    width, height, pixels = load_pgm(image_path)

    resolution = float(cfg.get("resolution", 0.2))
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    if len(origin) < 2:
        raise ValueError(f"invalid origin in {yaml_path}")

    negate = int(cfg.get("negate", 0)) != 0
    occupied_thresh = float(cfg.get("occupied_thresh", 0.65))
    free_thresh = float(cfg.get("free_thresh", 0.196))

    data: List[int] = [-1] * (width * height)
    for row in range(height):
        for col in range(width):
            image_idx = col + row * width
            val = pixels[image_idx]
            occ = (float(val) / 255.0) if negate else ((255.0 - float(val)) / 255.0)
            if occ > occupied_thresh:
                cell = 100
            elif occ < free_thresh:
                cell = 0
            else:
                cell = -1

            # PGM starts from top-left; OccupancyGrid expects bottom-left origin.
            map_row = height - row - 1
            map_idx = col + map_row * width
            data[map_idx] = cell

    return StaticMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        data=data,
    )


class StaticOccupancyMapPublisher(Node):
    def __init__(self, map_yaml: str, output_topic: str, frame_id: str) -> None:
        super().__init__("static_occupancy_map_publisher")

        static_map = load_static_map(map_yaml)
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = frame_id
        self.msg.info.resolution = static_map.resolution
        self.msg.info.width = static_map.width
        self.msg.info.height = static_map.height
        self.msg.info.origin.position.x = static_map.origin_x
        self.msg.info.origin.position.y = static_map.origin_y
        self.msg.info.origin.position.z = 0.0
        self.msg.info.origin.orientation.w = 1.0
        self.msg.data = static_map.data

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(OccupancyGrid, output_topic, qos)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.publish_once()

        self.get_logger().info(
            f"loaded static occupancy map: {map_yaml} "
            f"({static_map.width}x{static_map.height}, res={static_map.resolution})"
        )
        self.get_logger().info(f"output_topic={output_topic}, frame_id={frame_id}")

    def publish_once(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.info.map_load_time = self.msg.header.stamp
        self.pub.publish(self.msg)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument(
        "--output-topic",
        default="/planning/scenario_planning/parking/static_occupancy_grid",
    )
    parser.add_argument("--frame-id", default="map")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = StaticOccupancyMapPublisher(
        map_yaml=args.map_yaml,
        output_topic=args.output_topic,
        frame_id=args.frame_id,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

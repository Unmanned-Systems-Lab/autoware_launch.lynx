#!/usr/bin/env python3

import argparse
import math
import os
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

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


class CostmapStaticOverlayNode(Node):
    def __init__(self, input_topic: str, output_topic: str, static_map_yaml: str):
        super().__init__("costmap_static_overlay")

        self.static_map = load_static_map(static_map_yaml)
        self.get_logger().info(
            f"loaded static occupancy map: {static_map_yaml} "
            f"({self.static_map.width}x{self.static_map.height}, "
            f"res={self.static_map.resolution})"
        )

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, output_topic, qos)
        self.sub = self.create_subscription(OccupancyGrid, input_topic, self.on_costmap, qos)

    def sample_static(self, wx: float, wy: float) -> int:
        sx = int(math.floor((wx - self.static_map.origin_x) / self.static_map.resolution))
        sy = int(math.floor((wy - self.static_map.origin_y) / self.static_map.resolution))
        if sx < 0 or sy < 0 or sx >= self.static_map.width or sy >= self.static_map.height:
            return -1
        return self.static_map.data[sx + sy * self.static_map.width]

    def on_costmap(self, msg: OccupancyGrid):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = list(msg.data)

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        for y in range(height):
            wy = oy + (y + 0.5) * res
            for x in range(width):
                idx = x + y * width
                wx = ox + (x + 0.5) * res

                dyn = out.data[idx]
                sta = self.sample_static(wx, wy)

                if dyn < 0 and sta < 0:
                    merged = -1
                elif dyn < 0:
                    merged = sta
                elif sta < 0:
                    merged = dyn
                else:
                    merged = max(dyn, sta)

                out.data[idx] = int(merged)

        self.pub.publish(out)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--static-map-yaml", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CostmapStaticOverlayNode(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        static_map_yaml=args.static_map_yaml,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Lightweight map server node (replacement for nav2_map_server).
Reads .yaml + .pgm map files and publishes OccupancyGrid to /map topic.
Uses transient_local QoS so late subscribers still receive the map.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import yaml
import os

try:
    from PIL import Image
except ImportError:
    import subprocess
    subprocess.check_call(["sudo", "apt", "install", "-y", "python3-pil"])
    from PIL import Image


class SimpleMapServer(Node):
    def __init__(self):
        super().__init__("map_server")

        # Declare parameters manually for maximum compatibility
        self.declare_parameter('yaml_filename', '')
        self.yaml_filename = self.get_parameter('yaml_filename').value
        if not self.yaml_filename:
            self.get_logger().error("yaml_filename parameter is required!")
            raise RuntimeError("yaml_filename parameter is required")
        self.get_logger().info(f"Loading map: {self.yaml_filename}")

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.map_msg = None
        self.load_map()

        # Publish periodically to ensure late subscribers get it
        self.timer = self.create_timer(1.0, self.publish_map)

    def load_map(self):
        with open(self.yaml_filename, "r") as f:
            map_config = yaml.safe_load(f)
        image_file = map_config["image"]
        resolution = float(map_config["resolution"])
        origin = map_config["origin"]
        negate = int(map_config.get("negate", 0))
        occupied_thresh = float(map_config.get("occupied_thresh", 0.65))
        free_thresh = float(map_config.get("free_thresh", 0.196))
        yaml_dir = os.path.dirname(os.path.abspath(self.yaml_filename))
        image_path = os.path.join(yaml_dir, image_file)
        self.get_logger().info(f"  Image: {image_path}")
        self.get_logger().info(f"  Resolution: {resolution} m/pixel")
        self.get_logger().info(f"  Origin: {origin}")
        img = Image.open(image_path).convert("L")
        img_array = np.array(img, dtype=np.float32)
        img_array = np.flipud(img_array)
        if negate:
            img_array = 255.0 - img_array
        img_array = img_array / 255.0
        height, width = img_array.shape
        data = np.full((height * width,), -1, dtype=np.int8)
        for i in range(height * width):
            val = img_array.flat[i]
            if val < free_thresh:
                data[i] = 100  # dark pixel = obstacle
            elif val > occupied_thresh:
                data[i] = 0    # light pixel = free
        meta = MapMetaData()
        meta.map_load_time = self.get_clock().now().to_msg()
        meta.resolution = resolution
        meta.width = width
        meta.height = height
        meta.origin = Pose()
        meta.origin.position.x = float(origin[0])
        meta.origin.position.y = float(origin[1])
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_msg.info = meta
        self.map_msg.data = data.tolist()
        obs = np.sum(np.array(data) == 100)
        free = np.sum(np.array(data) == 0)
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Map loaded! Size: {width}x{height}, res: {resolution}m")
        self.get_logger().info(f"  Obstacles: {obs}, Free: {free}")
        self.get_logger().info("=" * 50)

    def publish_map(self):
        if self.map_msg:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
动态障碍物模拟节点
功能：
1. 订阅 /planned_path 和 /ship_pose 了解船舶航行情况
2. 每 N 秒在航线上或附近随机生成移动障碍物（模拟其他船只/漂浮物）
3. 发布到 /dynamic_obstacles 话题（PoseArray，orientation.w = 半径）
4. 旧障碍物超时后自动清除
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import random
import math
from typing import List, Tuple, Optional


class ObstacleDetector(Node):
    """动态障碍物模拟器"""

    def __init__(self):
        super().__init__('obstacle_detector')

        # 参数
        self.declare_parameter('spawn_interval', 8.0)      # 产生障碍物的间隔 (秒)
        self.declare_parameter('obstacle_radius', 12.0)     # 障碍物半径 (米)
        self.declare_parameter('obstacle_lifetime', 30.0)   # 障碍物存活时间 (秒)
        self.declare_parameter('max_obstacles', 5)          # 同时存在的最大障碍物数
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('ship_pose_topic', '/ship_pose')
        self.declare_parameter('obstacles_topic', '/dynamic_obstacles')
        self.declare_parameter('enabled', True)             # 是否启用动态障碍物

        self.spawn_interval = self.get_parameter('spawn_interval').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.obstacle_lifetime = self.get_parameter('obstacle_lifetime').value
        self.max_obstacles = self.get_parameter('max_obstacles').value
        self.path_topic = self.get_parameter('path_topic').value
        self.ship_pose_topic = self.get_parameter('ship_pose_topic').value
        self.obstacles_topic = self.get_parameter('obstacles_topic').value
        self.enabled = self.get_parameter('enabled').value

        # 内部状态
        self.path_points: List[Tuple[float, float]] = []
        self.ship_position: Optional[Tuple[float, float]] = None
        self.obstacles: List[dict] = []  # [{'x','y','radius','id','spawn_time'}, ...]
        self.next_obstacle_id = 0

        # 订阅规划路径
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )

        # 订阅船舶位姿（了解当前位置）
        self.ship_sub = self.create_subscription(
            PoseStamped, self.ship_pose_topic, self.ship_callback, 10
        )

        # 发布动态障碍物
        self.obstacles_pub = self.create_publisher(
            PoseArray, self.obstacles_topic, 10
        )

        # 可视化 Marker 发布器
        self.marker_pub = self.create_publisher(
            MarkerArray, '/dynamic_obstacles_markers', 10
        )

        # 定时器：生成新障碍物 + 清理过期障碍物 + 发布
        self.spawn_timer = self.create_timer(self.spawn_interval, self.try_spawn_obstacle)
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_and_publish)

        self.get_logger().info('=' * 50)
        self.get_logger().info('动态障碍物模拟器已启动!')
        self.get_logger().info(f'  生成间隔: {self.spawn_interval}s')
        self.get_logger().info(f'  障碍物半径: {self.obstacle_radius}m')
        self.get_logger().info(f'  存活时间: {self.obstacle_lifetime}s')
        self.get_logger().info(f'  最大数量: {self.max_obstacles}')
        self.get_logger().info(f'  状态: {"启用" if self.enabled else "禁用"}')
        self.get_logger().info('=' * 50)

    def path_callback(self, msg: Path):
        """缓存路径点用于障碍物放置参考"""
        pts = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]
        # 忽略无效路径（到达终点后发布的单点路径）
        if len(pts) < 2:
            return

        self.path_points = pts
        self.get_logger().info(
            f'收到路径: {len(self.path_points)} 个航点，'
            f'将在航线附近生成动态障碍物'
        )

    def ship_callback(self, msg: PoseStamped):
        """跟踪船舶当前位置"""
        self.ship_position = (msg.pose.position.x, msg.pose.position.y)

    def try_spawn_obstacle(self):
        """尝试在航线附近生成一个障碍物"""
        if not self.enabled:
            return
        if not self.path_points or not self.ship_position:
            return
        if len(self.obstacles) >= self.max_obstacles:
            return

        # 找到船舶前方 50~150m 处的一个路径点
        ship_x, ship_y = self.ship_position
        ahead_distance = random.uniform(50.0, 150.0)

        # 沿路径累积距离，找到目标位置
        cumulative = 0.0
        target_point = None
        found_ship_segment = False

        for i in range(len(self.path_points)):
            px, py = self.path_points[i]
            dist_to_ship = math.hypot(px - ship_x, py - ship_y)
            if dist_to_ship < 5.0:
                found_ship_segment = True
                cumulative = 0.0
                continue
            if not found_ship_segment:
                continue

            if i > 0:
                prev = self.path_points[i - 1]
                seg_len = math.hypot(px - prev[0], py - prev[1])
                cumulative += seg_len

            if cumulative >= ahead_distance:
                target_point = (px, py)
                break

        # 如果没找到合适位置，在路径中间随机取一个
        if target_point is None and len(self.path_points) > 2:
            idx = random.randint(len(self.path_points) // 4, 3 * len(self.path_points) // 4)
            target_point = self.path_points[idx]

        if target_point is None:
            return

        # 在目标点附近随机偏移（模拟偏离航线的障碍物）
        offset_angle = random.uniform(0, 2 * math.pi)
        offset_dist = random.uniform(5.0, 25.0)
        obs_x = target_point[0] + offset_dist * math.cos(offset_angle)
        obs_y = target_point[1] + offset_dist * math.sin(offset_angle)

        # 确保障碍物不在船舶当前位置太近（至少 30m）
        if math.hypot(obs_x - ship_x, obs_y - ship_y) < 30.0:
            return

        # 可变半径
        radius = self.obstacle_radius * random.uniform(0.6, 1.4)

        obstacle = {
            'x': obs_x,
            'y': obs_y,
            'radius': radius,
            'id': self.next_obstacle_id,
            'spawn_time': self.get_clock().now().nanoseconds / 1e9,
        }
        self.obstacles.append(obstacle)
        self.next_obstacle_id += 1

        self.get_logger().info(
            f'🟡 新障碍物 #{obstacle["id"]}: '
            f'位置({obs_x:.1f}, {obs_y:.1f}), 半径{radius:.1f}m'
        )

    def cleanup_and_publish(self):
        """清理过期障碍物并发布当前列表"""
        now = self.get_clock().now().nanoseconds / 1e9

        # 清理过期
        before = len(self.obstacles)
        self.obstacles = [
            o for o in self.obstacles
            if (now - o['spawn_time']) < self.obstacle_lifetime
        ]
        if len(self.obstacles) < before:
            self.get_logger().info(
                f'清理了 {before - len(self.obstacles)} 个过期障碍物'
            )

        # 发布
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for obs in self.obstacles:
            pose = Pose()
            pose.position.x = obs['x']
            pose.position.y = obs['y']
            pose.position.z = 0.0
            # 用 orientation.w 传递半径信息
            pose.orientation.w = float(obs['radius'])
            msg.poses.append(pose)

        self.obstacles_pub.publish(msg)

        # 同时发布可视化 Marker（先清空命名空间再重新添加，确保消失的障碍物被移除）
        marker_msg = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        # 1. DELETEALL：清除 ns='dynamic_obstacles' 下所有旧标记
        delete_all = Marker()
        delete_all.header.frame_id = 'map'
        delete_all.header.stamp = now_msg
        delete_all.ns = 'dynamic_obstacles'
        delete_all.action = Marker.DELETEALL
        marker_msg.markers.append(delete_all)

        # 2. ADD：重新添加当前存活的障碍物
        for obs in self.obstacles:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now_msg
            marker.ns = 'dynamic_obstacles'
            marker.id = obs['id']
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = obs['radius'] * 2.0
            marker.scale.y = obs['radius'] * 2.0
            marker.scale.z = 2.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker_msg.markers.append(marker)

        self.marker_pub.publish(marker_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

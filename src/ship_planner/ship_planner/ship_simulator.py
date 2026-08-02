#!/usr/bin/env python3
"""
船舶运动模拟节点
功能：
1. 订阅 /planned_path 获取规划路径
2. 模拟船舶沿路径以恒定速度航行
3. 发布船舶当前位置到 /ship_pose，供 RViz2 可视化
4. 到达终点后循环或停止
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from typing import List, Tuple, Optional


class ShipSimulator(Node):
    """船舶运动模拟器"""

    def __init__(self):
        super().__init__('ship_simulator')

        # 参数声明
        self.declare_parameter('speed', 5.0)          # 航行速度 (m/s)
        self.declare_parameter('update_rate', 20.0)    # 更新频率 (Hz)
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('pose_topic', '/ship_pose')
        self.declare_parameter('loop_mode', False)     # 是否循环航行
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'ship_base')

        # 获取参数
        self.speed = self.get_parameter('speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.loop_mode = self.get_parameter('loop_mode').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # 内部状态
        self.path: Optional[List[Tuple[float, float, float]]] = None
        self.current_segment = 0          # 当前位于第几段路径
        self.progress_in_segment = 0.0    # 在当前段中的进度 (0~1)
        self.simulation_running = False
        self.ship_yaw = 0.0               # 船舶朝向

        # 订阅规划路径
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        # 发布船舶位姿
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10
        )

        # TF 广播器（让 RViz2 能显示船舶模型）
        self.tf_broadcaster = TransformBroadcaster(self)

        # 主循环定时器
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.update_simulation)

        self.get_logger().info('=' * 50)
        self.get_logger().info('船舶运动模拟器已启动!')
        self.get_logger().info(f'  航速: {self.speed} m/s')
        self.get_logger().info(f'  更新频率: {self.update_rate} Hz')
        self.get_logger().info(f'  循环模式: {"开启" if self.loop_mode else "关闭"}')
        self.get_logger().info(f'  等待路径: {self.path_topic}')
        self.get_logger().info('=' * 50)

    def path_callback(self, msg: Path):
        """接收规划路径"""
        if len(msg.poses) < 2:
            # 忽略无效路径（到达终点后的单点路径）
            self.get_logger().debug('忽略点数不足的路径 (已到达终点)')
            return

        # 提取路径点
        self.path = [
            (pose.pose.position.x, pose.pose.position.y, 0.0)
            for pose in msg.poses
        ]

        # 重置模拟状态
        self.current_segment = 0
        self.progress_in_segment = 0.0
        self.simulation_running = True

        # 计算初始朝向
        if len(self.path) >= 2:
            dx = self.path[1][0] - self.path[0][0]
            dy = self.path[1][1] - self.path[0][1]
            self.ship_yaw = math.atan2(dy, dx)

        total_dist = self._compute_path_length()
        eta = total_dist / self.speed
        self.get_logger().info(
            f'收到路径: {len(self.path)} 个航点, '
            f'总距离: {total_dist:.1f}m, '
            f'预计航行时间: {eta:.1f}s'
        )

    def _compute_path_length(self) -> float:
        """计算路径总长度"""
        if not self.path:
            return 0.0
        total = 0.0
        for i in range(len(self.path) - 1):
            dx = self.path[i + 1][0] - self.path[i][0]
            dy = self.path[i + 1][1] - self.path[i][1]
            total += math.sqrt(dx * dx + dy * dy)
        return total

    def _interpolate_position(self) -> Tuple[float, float, float]:
        """在当前路径段上线性插值获取位置"""
        p0 = self.path[self.current_segment]
        p1 = self.path[self.current_segment + 1]
        t = self.progress_in_segment
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        return (x, y, 0.0)

    def update_simulation(self):
        """定时器回调：推进船舶位置"""
        if not self.simulation_running or not self.path:
            return

        num_segments = len(self.path) - 1

        if self.current_segment >= num_segments:
            if self.loop_mode:
                # 循环模式：回到起点重新航行
                self.current_segment = 0
                self.progress_in_segment = 0.0
                self.get_logger().info('🔄 循环航行：返回起点')
            else:
                # 到达终点，停止模拟
                self.simulation_running = False
                self.get_logger().info('🏁 船舶已到达终点!')
                return

        # 计算当前段的长度
        p0 = self.path[self.current_segment]
        p1 = self.path[self.current_segment + 1]
        seg_dx = p1[0] - p0[0]
        seg_dy = p1[1] - p0[1]
        seg_length = math.sqrt(seg_dx * seg_dx + seg_dy * seg_dy)

        if seg_length < 0.001:
            # 路径点重合，跳过
            self.current_segment += 1
            self.progress_in_segment = 0.0
            return

        # 本步前进的距离
        dt = 1.0 / self.update_rate
        step_distance = self.speed * dt

        # 更新进度
        self.progress_in_segment += step_distance / seg_length

        # 检查是否超过当前段
        while self.progress_in_segment >= 1.0:
            self.progress_in_segment -= 1.0
            self.current_segment += 1

            if self.current_segment >= num_segments:
                if self.loop_mode:
                    self.current_segment = 0
                    self.progress_in_segment = 0.0
                    self.get_logger().info('🔄 循环航行：返回起点')
                else:
                    # 到达终点
                    self.simulation_running = False
                    self.get_logger().info('🏁 船舶已到达终点!')
                    # 发布终点位置
                    self._publish_pose(self.path[-1])
                    return

            # 重新计算新段长度
            p0 = self.path[self.current_segment]
            p1 = self.path[self.current_segment + 1]
            seg_dx = p1[0] - p0[0]
            seg_dy = p1[1] - p0[1]
            seg_length = math.sqrt(seg_dx * seg_dx + seg_dy * seg_dy)

            if seg_length < 0.001:
                self.progress_in_segment = 0.0
                continue

            self.progress_in_segment = step_distance / seg_length

        # 获取当前位置
        ship_x, ship_y, ship_z = self._interpolate_position()

        # 更新船舶朝向（面向前方航点方向）
        if self.current_segment < num_segments:
            next_p = self.path[self.current_segment + 1]
            curr_p = self.path[self.current_segment]
            dx = next_p[0] - curr_p[0]
            dy = next_p[1] - curr_p[1]
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                self.ship_yaw = math.atan2(dy, dx)

        # 发布位姿
        self._publish_pose((ship_x, ship_y, ship_z))

    def _publish_pose(self, position: Tuple[float, float, float]):
        """发布船舶位姿和 TF 变换"""
        now = self.get_clock().now().to_msg()

        # 发布 PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]

        # 朝向：使用 yaw 转四元数
        half_yaw = self.ship_yaw * 0.5
        pose_msg.pose.orientation.z = math.sin(half_yaw)
        pose_msg.pose.orientation.w = math.cos(half_yaw)

        self.pose_pub.publish(pose_msg)

        # 发布 TF 变换（使 ship_base 坐标系可用于 RViz2 显示）
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = position[0]
        tf_msg.transform.translation.y = position[1]
        tf_msg.transform.translation.z = position[2]
        tf_msg.transform.rotation.z = pose_msg.pose.orientation.z
        tf_msg.transform.rotation.w = pose_msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShipSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

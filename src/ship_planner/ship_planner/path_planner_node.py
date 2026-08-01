#!/usr/bin/env python3
"""
船舶自动驾驶路径规划节点
功能：
1. 订阅地图 (occupancy_grid)
2. 订阅 RViz2 选择的起点和终点
3. 使用 A* 算法规划路径
4. 发布路径到 /planned_path 话题
5. 保存路径到 CSV 文件
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import heapq
import math
import csv
import os
from typing import List, Tuple, Optional
import time


class AStarPlanner:
    """A* 路径规划算法实现"""
    
    def __init__(self, grid: np.ndarray, resolution: float, origin_x: float, origin_y: float):
        """
        初始化 A* 规划器
        
        Args:
            grid: 占据栅格地图 (0=空闲, 100=障碍物, -1=未知)
            resolution: 地图分辨率 (m/pixel)
            origin_x: 地图原点 X 坐标
            origin_y: 地图原点 Y 坐标
        """
        self.grid = grid
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.height, self.width = grid.shape
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转栅格坐标"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """栅格坐标转世界坐标"""
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return x, y
    
    def is_valid(self, gx: int, gy: int) -> bool:
        """检查栅格坐标是否有效且无障碍"""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy, gx] == 0
        return False
    
    def get_neighbors(self, gx: int, gy: int) -> List[Tuple[int, int]]:
        """获取相邻的可通行栅格 (8方向移动)"""
        neighbors = []
        # 8个方向：上下左右 + 4个对角线
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),  # 上下左右
            (-1, -1), (-1, 1), (1, -1), (1, 1)  # 对角线
        ]
        for dx, dy in directions:
            nx, ny = gx + dx, gy + dy
            if self.is_valid(nx, ny):
                neighbors.append((nx, ny))
        return neighbors
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """
        启发函数：使用欧几里得距离
        对于船舶，考虑到可以在水面自由移动，欧几里得距离比较合适
        """
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        A* 路径规划
        
        Args:
            start: 起点世界坐标 (x, y)
            goal: 终点世界坐标 (x, y)
            
        Returns:
            路径点列表 [(x, y), ...] 或 None (无路径)
        """
        # 转换为栅格坐标
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        # 检查起点和终点是否有效
        if not self.is_valid(start_grid[0], start_grid[1]):
            print(f"[警告] 起点无效: {start} -> 栅格 {start_grid}, 值: {self.grid[start_grid[1], start_grid[0]] if 0 <= start_grid[0] < self.width and 0 <= start_grid[1] < self.height else '超出范围'}")
            # 尝试找最近的有效点
            start_grid = self.find_nearest_valid(start_grid)
            if start_grid is None:
                return None
                
        if not self.is_valid(goal_grid[0], goal_grid[1]):
            print(f"[警告] 终点无效: {goal} -> 栅格 {goal_grid}, 值: {self.grid[goal_grid[1], goal_grid[0]] if 0 <= goal_grid[0] < self.width and 0 <= goal_grid[1] < self.height else '超出范围'}")
            goal_grid = self.find_nearest_valid(goal_grid)
            if goal_grid is None:
                return None
        
        print(f"[A*] 起点栅格: {start_grid}, 终点栅格: {goal_grid}")
        
        # A* 算法
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        
        closed_set = set()
        
        max_iterations = self.width * self.height * 2  # 防止无限循环
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            # 到达终点
            if current == goal_grid:
                return self.reconstruct_path(came_from, current)
            
            closed_set.add(current)
            
            # 遍历邻居
            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor in closed_set:
                    continue
                
                # 计算移动代价
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                if dx != 0 and dy != 0:
                    move_cost = 1.414  # 对角线移动
                else:
                    move_cost = 1.0
                
                tentative_g = g_score[current] + move_cost
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal_grid)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        print(f"[A*] 未找到路径! 迭代次数: {iterations}")
        return None
    
    def find_nearest_valid(self, grid_pos: Tuple[int, int], max_radius: int = 20) -> Optional[Tuple[int, int]]:
        """寻找最近的有效栅格点"""
        for radius in range(1, max_radius):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = grid_pos[0] + dx, grid_pos[1] + dy
                    if self.is_valid(nx, ny):
                        return (nx, ny)
        return None
    
    def reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[float, float]]:
        """重建路径并转换为世界坐标"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        
        # 转换为世界坐标
        world_path = [self.grid_to_world(gx, gy) for gx, gy in path]
        return world_path
    
    def smooth_path(self, path: List[Tuple[float, float]], iterations: int = 50) -> List[Tuple[float, float]]:
        """
        路径平滑处理 - 使路径更适合船舶航行
        使用迭代平均法平滑路径
        """
        if len(path) < 3:
            return path
        
        smoothed = list(path)
        alpha = 0.3  # 平滑系数
        
        for _ in range(iterations):
            new_smoothed = [smoothed[0]]
            for i in range(1, len(smoothed) - 1):
                prev_pt = smoothed[i - 1]
                curr_pt = smoothed[i]
                next_pt = smoothed[i + 1]
                
                # 平均当前位置与前后点
                new_x = curr_pt[0] + alpha * ((prev_pt[0] + next_pt[0]) / 2 - curr_pt[0])
                new_y = curr_pt[1] + alpha * ((prev_pt[1] + next_pt[1]) / 2 - curr_pt[1])
                
                new_smoothed.append((new_x, new_y))
            new_smoothed.append(smoothed[-1])
            smoothed = new_smoothed
        
        return smoothed

    def is_line_clear(self, p1, p2, safety_margin=1.0):
        """Check if straight line between two points is free of obstacles."""
        x1, y1 = self.world_to_grid(p1[0], p1[1])
        x2, y2 = self.world_to_grid(p2[0], p2[1])
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        cx, cy = x1, y1
        margin_cells = int(safety_margin / self.resolution)
        while True:
            for ox in range(-margin_cells, margin_cells + 1):
                for oy in range(-margin_cells, margin_cells + 1):
                    nx, ny = cx + ox, cy + oy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.grid[ny, nx] == 100:
                            return False
            if cx == x2 and cy == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
        return True

    def simplify_path(self, path: List[Tuple[float, float]], tolerance: float = 2.0) -> List[Tuple[float, float]]:
        """
        Path simplification using Douglas-Peucker algorithm.
        Removes redundant waypoints while preserving path shape.
        
        Args:
            path: List of (x, y) waypoints
            tolerance: Maximum allowed deviation (meters)
            
        Returns:
            Simplified path
        """
        if len(path) <= 2:
            return path
        
        # Find the point with maximum distance from the line between first and last
        max_dist = 0
        max_idx = 0
        start = np.array(path[0])
        end = np.array(path[-1])
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        
        for i in range(1, len(path) - 1):
            pt = np.array(path[i])
            if line_len > 0:
                # Perpendicular distance from point to line
                t = max(0, min(1, np.dot(pt - start, line_vec) / (line_len ** 2)))
                proj = start + t * line_vec
                dist = np.linalg.norm(pt - proj)
            else:
                dist = np.linalg.norm(pt - start)
            
            if dist > max_dist:
                max_dist = dist
                max_idx = i
        
        # If max distance exceeds tolerance, recursively simplify
        if max_dist > tolerance:
            left = self.simplify_path(path[:max_idx + 1], tolerance)
            right = self.simplify_path(path[max_idx:], tolerance)
            return left[:-1] + right
        else:
            # Only merge if the straight line is obstacle-free
            if self.is_line_clear(path[0], path[-1]):
                return [path[0], path[-1]]
            else:
                # Find the midpoint and split
                mid = len(path) // 2
                left = self.simplify_path(path[:mid + 1], tolerance)
                right = self.simplify_path(path[mid:], tolerance)
                return left[:-1] + right

    def apply_turning_radius_constraint(self, path: List[Tuple[float, float]], 
                                         min_radius: float = 10.0) -> List[Tuple[float, float]]:
        """
        Apply minimum turning radius constraint for ship dynamics.
        Inserts intermediate waypoints to ensure smooth turns.
        
        Args:
            path: List of (x, y) waypoints
            min_radius: Minimum turning radius in meters
            
        Returns:
            Path with smooth turns respecting minimum radius
        """
        if len(path) <= 2:
            return path
        
        result = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev_pt = np.array(path[i - 1])
            curr_pt = np.array(path[i])
            next_pt = np.array(path[i + 1])
            
            # Calculate vectors
            v1 = curr_pt - prev_pt
            v2 = next_pt - curr_pt
            
            # Calculate angle between segments
            len1 = np.linalg.norm(v1)
            len2 = np.linalg.norm(v2)
            
            if len1 < 0.001 or len2 < 0.001:
                result.append(path[i])
                continue
            
            cos_angle = np.clip(np.dot(v1, v2) / (len1 * len2), -1.0, 1.0)
            angle = np.arccos(cos_angle)
            
            # If turn is sharp (angle > threshold), add intermediate points
            if angle < np.pi * 0.75:  # Turn more than ~41 degrees
                # Calculate arc points for smooth turn
                num_arc_points = max(3, int(angle / (np.pi / 12)))  # ~15 degree increments
                arc_radius = min(min_radius, len1 * 0.4, len2 * 0.4)
                
                # Generate arc waypoints
                for j in range(1, num_arc_points):
                    t = j / num_arc_points
                    # Quadratic bezier interpolation
                    p0 = curr_pt - v1 / len1 * arc_radius
                    p2 = curr_pt + v2 / len2 * arc_radius
                    p1 = curr_pt
                    
                    bx = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
                    by = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
                    result.append((float(bx), float(by)))
            else:
                result.append(path[i])
        
        result.append(path[-1])
        return result


class PathPlannerNode(Node):
    """ROS 2 路径规划节点"""
    
    def __init__(self):
        super().__init__('path_planner_node')
        
        # 参数声明
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('start_topic', '/initialpose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('csv_output_path', '/home/douhouqi/ship_ws/planned_path.csv')
        self.declare_parameter('smooth_path', True)
        
        # 获取参数
        self.map_topic = self.get_parameter('map_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.start_topic = self.get_parameter('start_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.csv_output_path = self.get_parameter('csv_output_path').value
        self.smooth_path_enabled = self.get_parameter('smooth_path').value
        
        # 地图数据
        self.map_data: Optional[OccupancyGrid] = None
        self.planner: Optional[AStarPlanner] = None
        self.map_initialized = False
        
        # 起点和终点
        self.start_pose: Optional[Tuple[float, float]] = None
        self.goal_pose: Optional[Tuple[float, float]] = None
        
        # 订阅者
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        
        # 订阅目标点 (RViz2 的 Nav2 目标点)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )
        
        # 订阅初始位置 (RViz2 的初始位置估计)
        self.start_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.start_topic,
            self.start_callback,
            10
        )
        
        # 发布者
        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10
        )
        
        # 用于可视化起终点的发布器
        self.start_marker_pub = self.create_publisher(
            PoseStamped,
            '/current_start',
            10
        )
        
        self.goal_marker_pub = self.create_publisher(
            PoseStamped,
            '/current_goal', 
            10
        )
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('船舶路径规划节点已启动!')
        self.get_logger().info('=' * 50)
        self.get_logger().info('使用说明:')
        self.get_logger().info('1. 等待地图加载...')
        self.get_logger().info('2. 在 RViz2 中:')
        self.get_logger().info('   - 点击 "2D Pose Estimate" 设置起点')
        self.get_logger().info('   - 点击 "Nav2 Goal" 设置终点')
        self.get_logger().info('3. 路径将自动规划并发布到 /planned_path')
        self.get_logger().info('=' * 50)
    
    def map_callback(self, msg: OccupancyGrid):
        """地图回调函数"""
        if self.map_initialized:
            return
        self.get_logger().info(f'收到地图数据: {msg.info.width}x{msg.info.height}, 分辨率: {msg.info.resolution}')
        
        self.map_data = msg
        
        # 将地图数据转换为 numpy 数组
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 创建规划器
        self.planner = AStarPlanner(
            grid=grid,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y
        )
        self.map_initialized = True
        
        self.get_logger().info('地图已加载，A* 规划器已就绪!')
        
        # 如果已有起点和终点，自动规划
        if self.start_pose and self.goal_pose:
            self.plan_path()
    
    def start_callback(self, msg: PoseWithCovarianceStamped):
        """起点回调函数 (来自 RViz2 的 "2D Pose Estimate")"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.start_pose = (x, y)
        self.get_logger().info(f'起点已设置: ({x:.2f}, {y:.2f})')
        
        # 发布起点标记
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.start_marker_pub.publish(pose_msg)
        
        # 如果地图和终点都已就绪，自动规划
        if self.map_data and self.goal_pose:
            self.plan_path()
    
    def goal_callback(self, msg: PoseStamped):
        """终点回调函数 (来自 RViz2 的 "Nav2 Goal")"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self.goal_pose = (x, y)
        self.get_logger().info(f'终点已设置: ({x:.2f}, {y:.2f})')
        
        # 发布终点标记
        self.goal_marker_pub.publish(msg)
        
        # 如果地图和起点都已就绪，自动规划
        if self.map_data and self.start_pose:
            self.plan_path()
    
    def plan_path(self):
        """执行路径规划"""
        if not self.planner:
            self.get_logger().warn('地图未加载，无法规划路径!')
            return
        
        if not self.start_pose or not self.goal_pose:
            self.get_logger().warn('起点或终点未设置!')
            return
        
        self.get_logger().info('=' * 40)
        self.get_logger().info(f'开始路径规划...')
        self.get_logger().info(f'起点: {self.start_pose}')
        self.get_logger().info(f'终点: {self.goal_pose}')
        
        start_time = time.time()
        
        # A* 路径规划
        path = self.planner.plan(self.start_pose, self.goal_pose)
        
        if path is None:
            self.get_logger().error('无法找到可行路径!')
            return
        
        # 路径平滑
        # Path simplification (remove redundant waypoints)
        self.get_logger().info('Simplifying path...')
        original_count = len(path)
        path = self.planner.simplify_path(path, tolerance=3.0)
        self.get_logger().info(f'  Simplified: {original_count} -> {len(path)} waypoints')
        
        # Apply ship dynamics constraints (minimum turning radius)
        self.get_logger().info('Applying turning radius constraint (min_radius=15m)...')
        path = self.planner.apply_turning_radius_constraint(path, min_radius=15.0)
        self.get_logger().info(f'  After dynamics constraint: {len(path)} waypoints')
        
        # Path smoothing
        if self.smooth_path_enabled:
            self.get_logger().info('Smoothing path...')
            path = self.planner.smooth_path(path, iterations=30)
        
        planning_time = time.time() - start_time
        self.get_logger().info(f'路径规划完成! 耗时: {planning_time:.3f}秒')
        self.get_logger().info(f'路径点数: {len(path)}')
        
        # 发布路径
        self.publish_path(path)
        
        # 保存为 CSV
        self.save_path_csv(path)
    
    def publish_path(self, path: List[Tuple[float, float]]):
        """发布路径到 ROS 话题"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'路径已发布到 {self.path_topic}')
    
    def save_path_csv(self, path: List[Tuple[float, float]]):
        """保存路径到 CSV 文件"""
        try:
            with open(self.csv_output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y'])
                for x, y in path:
                    writer.writerow([f'{x:.4f}', f'{y:.4f}'])
            self.get_logger().info(f'路径已保存到: {self.csv_output_path}')
        except Exception as e:
            self.get_logger().error(f'保存 CSV 失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

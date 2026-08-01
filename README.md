# Ship Planner - 船舶自动驾驶路径规划仿真系统

基于 ROS 2 Humble 的船舶自动驾驶路径规划仿真系统，实现从地图构建、路径规划到可视化验证的完整流程。

![RViz2 仿真效果](src/ship_planner/demo.png)

## 项目概述

本项目面向船舶自动驾驶场景，在二维占据栅格地图上实现 A* 全局路径规划算法，并集成路径简化、转向半径约束和平滑处理，最终输出符合船舶动力学约束的可行航行路径。

### 核心功能

- **多形状障碍物地图生成**：支持圆形、矩形、椭圆、三角形、L形等多种几何形状的障碍物
- **A* 路径规划**：8 方向搜索，欧几里得距离启发函数
- **路径简化**：Douglas-Peucker 算法 + 障碍物碰撞检测（Bresenham 画线）
- **动力学约束**：转向半径约束 + 贝塞尔曲线弧线插值
- **路径平滑**：迭代平均法 + 碰撞安全检测
- **RViz2 可视化**：实时显示地图、路径、起点/终点标记

## 系统架构

```
┌─────────────────┐     /map      ┌──────────────────┐
│  Map Generator  │ ────────────► │ Simple Map Server │
│  (map_generator)│  OccupancyGrid│  (simple_map_     │
─────────────────┘               │   server.py)      │
                                  └─────────────────┘
                                           │ /map
                                  ┌────────▼─────────┐
                                  │  Path Planner     │
                                  │  (path_planner_   │
                                  │   node.py)        │
                                  └─────────────────┘
                                           │ /planned_path
                                  ┌────────▼─────────┐
                                  │      RViz2        │
                                  │  (可视化展示)      │
                                  ──────────────────┘
```

## 项目结构

```
ship_planner/
├── ship_planner/
│   ├── path_planner_node.py    # 核心路径规划节点（A* + 简化 + 平滑）
│   ├── simple_map_server.py    # 轻量级地图服务器
│   ├── map_generator.py        # 多形状障碍物地图生成器
│   └── launch_ship_planner.py  # 系统启动文件
├── config/
│   └── ship_planner.rviz       # RViz2 可视化配置
├── resource/
│   └── ship_planner            # 包资源标识
├── test/                       # 单元测试
├── package.xml                 # 包描述文件
├── setup.py                    # Python 入口配置
├── setup.cfg                   # 构建配置
└── demo.png                    # 仿真效果截图
```

## 环境要求

- **操作系统**: Ubuntu 22.04 (WSL2 兼容)
- **ROS 2 版本**: Humble Hawksbill
- **Python**: 3.10+
- **依赖**:
  - `rclpy` - ROS 2 Python 客户端库
  - `nav_msgs` - 导航消息类型
  - `geometry_msgs` - 几何消息类型
  - `sensor_msgs` - 传感器消息类型
  - `numpy` - 数值计算
  - `matplotlib` - 路径几何计算
  - `Pillow` - 图像处理

## 快速开始

### 1. 生成地图

```bash
cd ~/ship_ws
python3 src/ship_planner/ship_planner/map_generator.py
```

地图文件将保存到 `~/ship_ws/maps/` 目录。

### 2. 构建项目

```bash
cd ~/ship_ws
colcon build --packages-select ship_planner
source install/setup.bash
```

### 3. 启动系统

```bash
ros2 launch ship_planner launch_ship_planner.py
```

### 4. 设置起点和终点

在 RViz2 中：
- 点击 **2D Pose Estimate** 设置船舶起点
- 点击 **2D Goal Pose** 设置目标终点
- 绿色路径将自动显示在地图上

## 算法详解

### A* 路径规划

采用 8 方向（含对角线）网格搜索，启发函数为欧几里得距离：

```
f(n) = g(n) + h(n)
g(n) = 从起点到当前节点的实际代价
h(n) = 当前节点到终点的直线距离（可采纳启发函数）
```

### 路径简化（Douglas-Peucker + 碰撞检测）

1. 找到距离首尾连线最远的点
2. 若距离 > 阈值，递归处理左右两段
3. 若距离 ≤ 阈值，检查连线是否穿过障碍物
4. 若连线安全则简化为直线，否则保留中间点

### 转向半径约束

在急弯处插入贝塞尔曲线弧线点，确保船舶转向角度不超过最大限制（默认 30°）。

### 路径平滑

使用迭代平均法对路径进行平滑，同时保持与障碍物的安全距离。

## 地图障碍物类型

| 形状 | 数量 | 模拟场景 |
|------|------|----------|
| 圆形 | 6 + 30随机 | 岛屿、礁石 |
| 矩形 | 3 | 码头、人工岛 |
| 椭圆 | 2 | 狭长岛屿 |
| 三角形 | 2 | 礁石群 |
| L形 | 1 | 防波堤、半岛 |

## 常见问题

### RViz2 显示 OpenGL 错误
WSL2 图形驱动兼容性问题，可设置环境变量：
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### RViz2 窗口无法点击
按 F11 切换全屏模式可解决焦点问题。

### 路径穿过障碍物
路径简化算法已加入 Bresenham 碰撞检测，确保简化后的路径不穿过障碍物。

## 许可证

Apache-2.0

## 作者

worldmiracle

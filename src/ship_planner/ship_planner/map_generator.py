#!/usr/bin/env python3
"""
水域地图生成器
生成一个 500m x 500m 的占据栅格地图，包含多种形状的障碍物
输出格式: .pgm (图片) + .yaml (配置文件)
"""

import numpy as np
from matplotlib.path import Path
import yaml
import os
import sys

try:
    from PIL import Image
except ImportError:
    print("正在安装 Pillow 图像处理库...")
    os.system('sudo apt install python3-pil -y')
    from PIL import Image


def create_water_map(width=500, height=500, resolution=1.0):
    grid = np.zeros((height, width), dtype=np.int8)
    border_width = 10
    grid[:border_width, :] = 100
    grid[-border_width:, :] = 100
    grid[:, :border_width] = 100
    grid[:, -border_width:] = 100

    # 1. 圆形岛屿
    circles = [(150,150,30),(350,350,25),(100,300,15),(400,200,18),(300,150,8),(80,180,8)]
    for cx, cy, r in circles:
        y, x = np.ogrid[:height, :width]
        mask = (x - cx)**2 + (y - cy)**2 <= r**2
        grid[mask] = 100

    # 2. 矩形障碍物
    rectangles = [(200,400,40,20),(420,420,30,30),(50,50,25,35)]
    for cx, cy, w, h in rectangles:
        x1, x2 = max(0, cx - w//2), min(width, cx + w//2)
        y1, y2 = max(0, cy - h//2), min(height, cy + h//2)
        grid[y1:y2, x1:x2] = 100

    # 3. 椭圆形障碍物
    ellipses = [(250,250,40,15,30),(350,100,30,10,-20)]
    for cx, cy, a, b, angle in ellipses:
        y, x = np.ogrid[:height, :width]
        rad = np.radians(angle)
        xr = (x - cx)*np.cos(rad) + (y - cy)*np.sin(rad)
        yr = -(x - cx)*np.sin(rad) + (y - cy)*np.cos(rad)
        mask = (xr/a)**2 + (yr/b)**2 <= 1.0
        grid[mask] = 100

    # 4. 三角形障碍物
    triangles = [(100,100,20),(300,300,25)]
    for cx, cy, size in triangles:
        pts = np.array([[cx, cy-size],[cx-size*0.866, cy+size*0.5],[cx+size*0.866, cy+size*0.5]])
        tri_path = Path(pts)
        yy, xx = np.mgrid[:height, :width]
        points = np.vstack([xx.ravel(), yy.ravel()]).T
        mask = tri_path.contains_points(points).reshape(height, width)
        grid[mask] = 100

    # 5. L形障碍物
    for cx, cy, aw, al, t in [(430,470,15,40,15)]:
        grid[max(0,cy-t//2):min(height,cy+t//2), max(0,cx-aw//2):min(width,cx+aw//2)] = 100
        grid[max(0,cy-t//2):min(height,cy+al), max(0,cx-t//2):min(width,cx+t//2)] = 100

    # 6. 右下角岛屿群
    for cx, cy, r in [(450,450,20),(430,470,15),(470,430,15)]:
        y, x = np.ogrid[:height, :width]
        mask = (x - cx)**2 + (y - cy)**2 <= r**2
        grid[mask] = 100

    # 7. 随机小礁石
    np.random.seed(42)
    for _ in range(30):
        rx = np.random.randint(30, width-30)
        ry = np.random.randint(30, height-30)
        rr = np.random.randint(3, 8)
        if (rx < 60 and ry < 60) or (rx > 440 and ry > 440):
            continue
        y, x = np.ogrid[:height, :width]
        mask = (x - rx)**2 + (y - ry)**2 <= rr**2
        grid[mask] = 100

    return grid


def save_map(grid, output_dir, map_name='water_map'):
    os.makedirs(output_dir, exist_ok=True)
    height, width = grid.shape
    img_array = np.where(grid == 100, 0, 254).astype(np.uint8)
    img = Image.fromarray(img_array, mode='L')
    img = img.transpose(Image.FLIP_TOP_BOTTOM)
    pgm_path = os.path.join(output_dir, f'{map_name}.pgm')
    img.save(pgm_path)
    print(f'地图图片已保存: {pgm_path}')
    yaml_data = {'image': f'{map_name}.pgm','resolution': 1.0,'origin': [0.0,0.0,0.0],'negate': 0,'occupied_thresh': 0.65,'free_thresh': 0.196}
    yaml_path = os.path.join(output_dir, f'{map_name}.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    print(f'地图配置已保存: {yaml_path}')
    npy_path = os.path.join(output_dir, f'{map_name}.npy')
    np.save(npy_path, grid)
    print(f'栅格数据已保存: {npy_path}')
    total_pixels = width * height
    obstacle_pixels = np.sum(grid == 100)
    free_pixels = np.sum(grid == 0)
    print(f'\n地图统计:')
    print(f'  尺寸: {width} x {height} 像素')
    print(f'  分辨率: 1 m/pixel')
    print(f'  实际大小: {width}m x {height}m')
    print(f'  总像素: {total_pixels}')
    print(f'  障碍物: {obstacle_pixels} ({obstacle_pixels/total_pixels*100:.1f}%)')
    print(f'  自由水域: {free_pixels} ({free_pixels/total_pixels*100:.1f}%)')


def main():
    print('=' * 50)
    print('船舶航行水域地图生成器')
    print('=' * 50)
    width = 500
    height = 500
    resolution = 1.0
    output_dir = os.path.expanduser('~/ship_ws/maps')
    print(f'\n生成地图: {width}m x {height}m, 分辨率 {resolution}m')
    grid = create_water_map(width, height, resolution)
    save_map(grid, output_dir)
    print('\n' + '=' * 50)
    print('地图生成完成!')
    print('=' * 50)


if __name__ == '__main__':
    main()

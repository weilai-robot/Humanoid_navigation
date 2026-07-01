#!/usr/bin/env python3
"""
从 lab_env.xml 的几何体定义直接生成 Nav2 2D 占据栅格地图。
纯 Python 实现，不需要 MuJoCo 绑定。

用法:
  python3 generate_map_from_xml.py
  python3 generate_map_from_xml.py --output navigation/humanoid_sim/maps/lab_env_map

输出:
  <output>.pgm + <output>.yaml
"""

import math
import re
import argparse

# ─── lab_env.xml 中的所有可碰撞几何体 (conaffinity=7) ───
# 格式: (type, cx, cy, size_param1, size_param2, name)

GEOMS = [
    # 外墙 (box: cx, cy, half_x, half_y)
    ('box',  -10,    0,  0.1, 5.1, 'wall_west'),
    ('box',   10,    0,  0.1, 5.1, 'wall_east'),
    ('box',    0, -5.0, 10.1, 0.1, 'wall_south'),
    ('box',    0,  5.0, 10.1, 0.1, 'wall_north'),

    # 中央隔断墙 X=2
    ('box',    2, -4.20,  0.1, 0.80, 'par_s1'),
    ('box',    2,  0.2125, 0.1, 2.8125, 'par_s2'),
    ('box',    2,  4.1875, 0.1, 0.8125, 'par_s3'),

    # 通道A 警示柱
    ('circle', 2, -3.42, 0.04, 0, 'marker_A1'),
    ('circle', 2, -2.58, 0.04, 0, 'marker_A2'),
    # 通道B 警示柱
    ('circle', 2,  3.02, 0.04, 0, 'marker_B1'),
    ('circle', 2,  3.38, 0.04, 0, 'marker_B2'),

    # 玻璃隔断 X=5
    ('box',    5,  2.9,  0.04, 1.9, 'glass1'),
    ('box',    5,  2.9,  0.06, 1.9, 'glass1_beam'),
    ('box',    5, -3.3,  0.04, 1.5, 'glass2'),
    ('box',    5, -3.3,  0.06, 1.5, 'glass2_beam'),
    ('box',    5,  1.05, 0.05, 0.05, 'glass1_frameL'),
    ('box',    5,  4.75, 0.05, 0.05, 'glass1_frameR'),
    ('box',    5, -4.75, 0.05, 0.05, 'glass2_frameL'),
    ('box',    5, -1.85, 0.05, 0.05, 'glass2_frameR'),

    # 西区工作台 (贴南墙)
    ('box',   -7,  -4.47, 1.2, 0.32, 'wbench1'),
    ('box',   -2,  -4.47, 1.0, 0.32, 'wbench2'),

    # 会议桌 + 椅子 (x=-5.5, y=0)
    ('box',  -5.5,  0, 1.6, 0.70, 'conf_table'),
    ('box',  -7.0, -0.95, 0.21, 0.21, 'chair_s1'),
    ('box',  -6.2, -0.95, 0.21, 0.21, 'chair_s2'),
    ('box',  -5.4, -0.95, 0.21, 0.21, 'chair_s3'),
    ('box',  -4.6, -0.95, 0.21, 0.21, 'chair_s4'),
    ('box',  -7.0,  0.95, 0.21, 0.21, 'chair_n1'),
    ('box',  -6.2,  0.95, 0.21, 0.21, 'chair_n2'),
    ('box',  -5.4,  0.95, 0.21, 0.21, 'chair_n3'),
    ('box',  -4.6,  0.95, 0.21, 0.21, 'chair_n4'),

    # 办公桌 (贴西墙)
    ('box',  -9.3,  3.0, 0.62, 0.33, 'desk1'),
    ('box',  -9.3,  2.1, 0.20, 0.20, 'desk1_chair'),
    ('box',  -9.3, -3.0, 0.62, 0.33, 'desk2'),
    ('box',  -9.3, -2.1, 0.20, 0.20, 'desk2_chair'),

    # 工作台W3 (贴北墙)
    ('box',  -6.5,  4.47, 2.0, 0.32, 'wbench3'),

    # 东区实验台 (L形)
    ('box',   6.5, -1.2, 1.8, 0.40, 'lab1'),
    ('box',   8.05, 0.2, 0.40, 1.0, 'lab2'),

    # 服务器机柜 (贴东墙)
    ('box',   9.65,  2.2, 0.30, 0.52, 'rack1'),
    ('box',   9.65,  0.9, 0.30, 0.52, 'rack2'),
    ('box',   9.65, -0.4, 0.30, 0.52, 'rack3'),

    # 东区工作台 (贴北墙)
    ('box',   6.5,  4.47, 1.4, 0.32, 'wbench_e1'),

    # 地面散落纸箱
    ('box',  -3.5,  2.0, 0.30, 0.30, 'cbox1'),
    ('box',  -3.0,  2.6, 0.25, 0.40, 'cbox2'),
    ('box',  -1.5, -2.8, 0.30, 0.22, 'cbox3'),
    ('box',   3.5, -3.8, 0.35, 0.25, 'cbox4'),
    ('box',   2.8, -3.2, 0.22, 0.22, 'cbox5'),
    ('box',   3.2,  3.5, 0.28, 0.28, 'cbox6'),

    # 结构柱
    ('box',  -3.3,  4.8, 0.15, 0.15, 'col1'),
    ('box',  -3.3, -4.8, 0.15, 0.15, 'col2'),
    ('box',   3.3,  4.8, 0.15, 0.15, 'col3'),
    ('box',   3.3, -4.8, 0.15, 0.15, 'col4'),

    # 消防栓箱
    ('box',  -9.92, 0, 0.08, 0.25, 'fire_box'),
    # 配电柜
    ('box',   4.0, -4.88, 0.12, 0.50, 'elec_panel'),
    # 移动推车
    ('box',   2.5,  1.5, 0.35, 0.25, 'cart'),
    # 废料桶
    ('circle', -8.5,  4.7, 0.18, 0, 'bin1'),
    ('circle',  9.0,  4.7, 0.18, 0, 'bin2'),

    # 台面仪器 (小箱体, 仅标注位置)
    ('box',  -7.8, -4.47, 0.18, 0.16, 'inst1'),
    ('box',  -7.1, -4.47, 0.12, 0.12, 'inst2'),
    ('box',  -2.5, -4.47, 0.15, 0.18, 'inst3'),
    ('box',   5.8, -1.2, 0.20, 0.20, 'inst4'),
    ('box',   6.5, -1.2, 0.14, 0.14, 'inst5'),

    # 动态障碍物 (静态地图中也标记)
    ('circle', 0.5, -3.0, 0.22, 0, 'dyn_person'),
    ('box',   -1.5,  1.8, 0.30, 0.25, 'dyn_box'),
    ('box',    3.5, -3.0, 0.25, 0.20, 'dyn_crate'),
]


def point_to_box_dist(px, py, cx, cy, hx, hy):
    """点到矩形的有符号距离 (负=内部, 正=外部)"""
    dx = abs(px - cx) - hx
    dy = abs(py - cy) - hy
    if dx < 0 and dy < 0:
        # 在矩形内部: 到最近边的距离 (取绝对值较小的)
        return -min(abs(dx), abs(dy))
    return math.sqrt(max(dx, 0)**2 + max(dy, 0)**2)


def point_to_circle_dist(px, py, cx, cy, r):
    """点到圆的有符号距离"""
    return math.sqrt((px - cx)**2 + (py - cy)**2) - r


def generate_map(resolution=0.05, inflation_radius=0.0,
                 x_range=(-10.5, 10.5), y_range=(-5.5, 5.5)):
    w = int((x_range[1] - x_range[0]) / resolution)
    h = int((y_range[1] - y_range[0]) / resolution)

    print(f"Grid: {w} x {h} pixels, {w*resolution:.1f}m x {h*resolution:.1f}m")
    print(f"Resolution: {resolution}m, Inflation: {inflation_radius}m")

    # ROS PGM convention: 0=black=OCCUPIED, 254=white=FREE
    grid = bytearray([254] * (w * h))  # default all FREE (white)

    for gy in range(h):
        wy = y_range[0] + gy * resolution
        pgm_row = h - 1 - gy  # PGM Y flip
        for gx in range(w):
            wx = x_range[0] + gx * resolution

            min_dist = float('inf')
            for gtype, cx, cy, s1, s2, name in GEOMS:
                if gtype == 'box':
                    d = point_to_box_dist(wx, wy, cx, cy, s1, s2)
                else:
                    d = point_to_circle_dist(wx, wy, cx, cy, s1)

                if d < min_dist:
                    min_dist = d
                    if min_dist <= 0:
                        break

            if min_dist <= 0:
                grid[pgm_row * w + gx] = 0       # OCCUPIED (black)
            elif min_dist <= inflation_radius:
                grid[pgm_row * w + gx] = 0       # inflated obstacle (black)

    return grid, w, h


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='navigation/humanoid_sim/maps/lab_env_map',
                        help='Output basename')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--inflation', type=float, default=0.35)
    args = parser.parse_args()

    grid, w, h = generate_map(
        resolution=args.resolution,
        inflation_radius=args.inflation
    )

    pgm_file = f'{args.output}.pgm'
    yaml_file = f'{args.output}.yaml'

    x_range = (-10.5, 10.5)
    y_range = (-5.5, 5.5)

    # Save PGM
    with open(pgm_file, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{w} {h}\n'.encode())
        f.write(b'255\n')
        f.write(grid)

    # Save YAML
    with open(yaml_file, 'w') as f:
        f.write(f'image: {args.output.split("/")[-1]}.pgm\n')
        f.write(f'mode: trinary\n')
        f.write(f'resolution: {args.resolution}\n')
        f.write(f'origin: [{x_range[0]}, {y_range[0]}, 0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: 0.65\n')
        f.write(f'free_thresh: 0.25\n')

    # ROS convention: 254=FREE(white), 0=OCCUPIED(black)
    free = sum(1 for b in grid if b > 200)
    wall = sum(1 for b in grid if b < 50)
    print(f"\nGenerated: {pgm_file}")
    print(f"  Size: {w}x{h}, Free: {free} ({free/len(grid)*100:.1f}%), Wall: {wall} ({wall/len(grid)*100:.1f}%)")

    # 验证
    def check(wx, wy, label):
        px = int((wx - x_range[0]) / args.resolution)
        py = h - 1 - int((wy - y_range[0]) / args.resolution)
        v = grid[py * w + px] if 0 <= py < h and 0 <= px < w else -1
        s = 'FREE' if v > 200 else 'WALL' if v < 50 else '?'
        print(f"  {label:25s} ({wx:5.1f},{wy:5.1f}): {v:3d} {s}")

    print("\n=== Verification ===")
    check(0, 0, "Robot origin")
    check(5, 0, "Target (5,0)")
    check(2, 0, "Partition wall X=2,Y=0")
    check(2, -3.0, "Passage A center")
    check(2, -3.3, "Passage A edge1")
    check(2, -2.7, "Passage A edge2")
    check(5, -1.5, "Glass gap center")
    check(0.5, -3.0, "Dyn person pos")
    check(-5, 0, "West room center")
    check(6, -2, "East room center")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
从 MuJoCo lab_env.xml 场景自动生成 Nav2 兼容的 2D 占据栅格地图。

原理：
  1. 加载 MuJoCo 模型
  2. 获取所有 conaffinity=7 的几何体（可碰撞 = 障碍物）
  3. 从正上方向下投影，生成 2D 占据图
  4. 膨胀处理（模拟 inflation_layer）
  5. 输出 PGM + YAML

用法：
  python3 generate_nav_map.py --model <path_to_xyber_x1_nav.xml> --output navigation/humanoid_sim/maps/lab_env_generated
"""

import argparse
import math
import numpy as np


def parse_mujoco_geoms(xml_path):
    """从 MuJoCo XML 中提取所有障碍物的 2D 投影"""
    import mujoco
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    geoms = []
    for i in range(model.ngeom):
        # 只关注可碰撞几何体 (conaffinity 包含 7)
        contype = model.geom_contype[i]
        conaffinity = model.geom_conaffinity[i]
        if (contype & 7) == 0 and (conaffinity & 7) == 0:
            continue

        geom_type = model.geom_type[i]
        pos = model.geom_pos[i].copy()
        size = model.geom_size[i].copy()
        name = model.geom(i).name or f"geom_{i}"

        # 跳过地板平面
        if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            continue

        # 获取 2D 投影 (X-Y 平面)
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            # 矩形: half_x, half_y
            geoms.append({
                'type': 'box',
                'cx': pos[0], 'cy': pos[1],
                'hx': size[0], 'hy': size[1],
                'name': name
            })
        elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            # 圆柱: radius
            geoms.append({
                'type': 'circle',
                'cx': pos[0], 'cy': pos[1],
                'r': size[0],
                'name': name
            })
        elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            # 球体: 投影为圆
            geoms.append({
                'type': 'circle',
                'cx': pos[0], 'cy': pos[1],
                'r': size[0],
                'name': name
            })
        elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
            # 胶囊: 近似为圆
            geoms.append({
                'type': 'circle',
                'cx': pos[0], 'cy': pos[1],
                'r': size[0],
                'name': name
            })

    return geoms


def generate_occupancy_grid(geoms, x_range, y_range, resolution, inflation_radius):
    """生成占据栅格"""
    w = int((x_range[1] - x_range[0]) / resolution)
    h = int((y_range[1] - y_range[0]) / resolution)

    grid = np.zeros((h, w), dtype=np.uint8)  # 0 = free

    for gy in range(h):
        wy = y_range[0] + gy * resolution
        for gx in range(w):
            wx = x_range[0] + gx * resolution

            for g in geoms:
                dx = wx - g['cx']
                dy = wy - g['cy']
                dist = math.sqrt(dx*dx + dy*dy)

                if g['type'] == 'box':
                    # 点到矩形最近距离
                    nx = max(abs(dx) - g['hx'], 0)
                    ny = max(abs(dy) - g['hy'], 0)
                    dist = math.sqrt(nx*nx + ny*ny)
                elif g['type'] == 'circle':
                    dist = dist - g['r']

                if dist <= 0:
                    grid[h-1-gy, gx] = 254  # wall
                elif dist <= inflation_radius:
                    grid[h-1-gy, gx] = 254  # inflated wall

    return grid, w, h


def save_pgm(filename, grid, w, h):
    """保存为 PGM 格式"""
    with open(filename, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{w} {h}\n'.encode())
        f.write(b'255\n')
        f.write(grid.tobytes())


def save_yaml(filename, pgm_filename, resolution, origin):
    """保存为 YAML 格式"""
    with open(filename, 'w') as f:
        f.write(f'image: {pgm_filename}\n')
        f.write(f'mode: trinary\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin[0]}, {origin[1]}, 0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: 0.65\n')
        f.write(f'free_thresh: 0.25\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Path to xyber_x1_nav.xml')
    parser.add_argument('--output', default='lab_env_generated',
                        help='Output basename (without extension)')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--inflation', type=float, default=0.3,
                        help='Inflation radius (m) around obstacles')
    args = parser.parse_args()

    print(f"Loading model: {args.model}")
    geoms = parse_mujoco_geoms(args.model)
    print(f"Found {len(geoms)} collision geoms")

    for g in geoms:
        if g['type'] == 'box':
            print(f"  {g['name']}: box ({g['cx']:.1f},{g['cy']:.1f}) half={g['hx']:.2f}x{g['hy']:.2f}")
        else:
            print(f"  {g['name']}: circle ({g['cx']:.1f},{g['cy']:.1f}) r={g['r']:.2f}")

    # 地图范围匹配 lab_env.xml (20m x 10m)
    x_range = (-10.5, 10.5)
    y_range = (-5.5, 5.5)

    print(f"\nGenerating grid: {x_range[1]-x_range[0]:.1f}m x {y_range[1]-y_range[0]:.1f}m, "
          f"res={args.resolution}, inflation={args.inflation}m")

    grid, w, h = generate_occupancy_grid(
        geoms, x_range, y_range, args.resolution, args.inflation)

    pgm_file = f'{args.output}.pgm'
    yaml_file = f'{args.output}.yaml'

    save_pgm(pgm_file, grid, w, h)
    save_yaml(yaml_file, f'{args.output}.pgm', args.resolution,
              [x_range[0], y_range[0]])

    free = np.sum(grid < 50)
    wall = np.sum(grid > 200)
    print(f"\nGenerated: {pgm_file} ({w}x{h})")
    print(f"  Free: {free} ({free/(w*h)*100:.1f}%)")
    print(f"  Wall: {wall} ({wall/(w*h)*100:.1f}%)")
    print(f"  Origin: [{x_range[0]}, {y_range[0]}]")

    # 验证关键位置
    def check(wx, wy, label):
        px = int((wx - x_range[0]) / args.resolution)
        py = h - 1 - int((wy - y_range[0]) / args.resolution)
        v = grid[py, px] if 0 <= py < h and 0 <= px < w else -1
        status = 'FREE' if v < 50 else 'WALL' if v > 200 else 'UNKNOWN'
        print(f"  {label} ({wx:.1f},{wy:.1f}): {v} {status}")

    print("\n=== Verification ===")
    check(0, 0, "Robot origin")
    check(5, 0, "Target")
    check(2, 0, "Partition wall")
    check(2, -3, "Passage A center")
    check(-5, 0, "West room center")


if __name__ == '__main__':
    main()

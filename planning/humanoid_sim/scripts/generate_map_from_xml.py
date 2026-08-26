#!/usr/bin/env python3
"""
从 lab_env.xml 直接解析几何体生成 Nav2 2D 占据栅格地图 (mujoco_lab.pgm/yaml)。

背景: 早期版本使用手工抄录的 GEOMS 几何表, 与世界 XML 演化脱节后地图失真
(通道A 被画窄 ~0.3m, 导致 inflation 后可行走廊仅剩 ~0.1m, 全局规划极脆弱)。
本版本直接解析 MuJoCo XML, 保证地图与世界永远一致。

规则:
  1. 仅取 conaffinity=7 (可碰撞) 的 geom
  2. 高度过滤: 顶面 z_max >= min_top_z (默认 0.15m, 高于足底可通过的坎)
     且底面 z_min <= max_bottom_z (默认 1.30m, 机器人本体最高可碰撞高度)
     —— 排除天花板/高位横梁等 2D 导航无关几何
  3. 动态障碍 (dyn_*) 默认排除 —— 设计意图: 动态避障由 VoxelLayer + 重规划
     在线感知, 静态地图不含它们; 需要 --include-dynamic 可包含
  4. cell 中心落入几何足迹则视为占据 (无偏栅格化, 不额外膨胀; inflation 由
     Nav2 InflationLayer 在线完成)

用法:
  python3 generate_map_from_xml.py [--xml PATH] [--output-dir DIR] [--include-dynamic]

纯标准库实现, 可在任何机器离线运行 (CI/本地均无需 ROS)。
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

# ─── 默认路径 (相对 navigation 仓库根) ───
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_XML = os.path.join(
    HERE, '..', '..', '..', '..',
    'motion_control', 'module', 'sim_module', 'model', 'mjcf',
    'environment', 'lab_env.xml')
DEFAULT_OUT_DIR = os.path.join(HERE, '..', 'maps')

RESOLUTION = 0.05
# 地图边界: 完整包含外墙 (±10.1 / ±5.1) 并留半格余量
MAP_X0, MAP_Y0 = -10.15, -5.15
MAP_NX = 406          # 406 * 0.05 = 20.30m -> X ∈ [-10.15, 10.15]
MAP_NY = 206          # 206 * 0.05 = 10.30m -> Y ∈ [-5.15, 5.15]

MIN_TOP_Z = 0.15      # 障碍顶面低于此高度 → 机器人可跨越/足底间隙 → 不入图
MAX_BOTTOM_Z = 1.30   # 障碍底面高于此 → 只会打到头部以上传感器 → 仍入图保守处理? 否, 不入图

PGM_OCC = 0           # 占据像素值
PGM_FREE = 254        # 自由像素值 (trinary 模式下与 205 等价均为 free)


def parse_geoms(xml_path):
    """解析 XML 中全部 geom, 返回 dict 列表 (含足迹类型/参数与 z 波段)."""
    tree = ET.parse(xml_path)
    geoms = []
    for g in tree.getroot().iter('geom'):
        name = (g.get('name') or '').strip()
        gtype = (g.get('type') or 'sphere').strip()
        try:
            size = [float(v) for v in (g.get('size') or '0').split()]
            pos = [float(v) for v in (g.get('pos') or '0 0 0').split()]
        except ValueError as e:
            raise SystemExit(f"[错误] geom '{name}' size/pos 解析失败: {e}")
        while len(pos) < 3:
            pos.append(0.0)
        conaffinity = int(g.get('conaffinity') or '0')

        item = dict(name=name, type=gtype, size=size, pos=pos,
                    conaffinity=conaffinity)
        # 计算 2D 足迹与 z 波段
        if gtype == 'box':
            sx, sy, sz = size[0], size[1], size[2]
            item['footprint'] = ('rect', pos[0], pos[1], sx, sy)
            item['zband'] = (pos[2] - sz, pos[2] + sz)
        elif gtype == 'cylinder':
            r, half_h = size[0], size[1]
            item['footprint'] = ('circle', pos[0], pos[1], r)
            item['zband'] = (pos[2] - half_h, pos[2] + half_h)
        elif gtype == 'sphere':
            r = size[0]
            item['footprint'] = ('circle', pos[0], pos[1], r)
            item['zband'] = (pos[2] - r, pos[2] + r)
        elif gtype == 'plane':
            item['footprint'] = None      # 地面, 不入图
            item['zband'] = None
        else:
            # capsule/ellipsoid/mesh 等保守处理: 目前环境未使用
            item['footprint'] = None
            item['zband'] = None
        geoms.append(item)
    return geoms


def geom_is_2d_obstacle(g, include_dynamic):
    """判定 geom 是否进入 2D 占据图."""
    if g['footprint'] is None:
        return False, 'type-skipped'
    if g['conaffinity'] != 7:
        return False, 'no-collision'
    if not include_dynamic and g['name'].startswith('dyn_'):
        return False, 'dynamic-excluded'
    z_min, z_max = g['zband']
    if z_max < MIN_TOP_Z:
        return False, f'too-low(top={z_max:.2f}m)'
    if z_min > MAX_BOTTOM_Z:
        return False, f'too-high(bottom={z_min:.2f}m)'
    return True, 'included'


def rasterize(geoms, include_dynamic):
    """返回 (grid, included, excluded): grid[y][x] 1=占据."""
    grid = [[0] * MAP_NX for _ in range(MAP_NY)]
    included, excluded = [], []
    for g in geoms:
        ok, why = geom_is_2d_obstacle(g, include_dynamic)
        (included if ok else excluded).append((g['name'], why))

        if not ok:
            continue
        ftype = g['footprint'][0]
        if ftype == 'rect':
            _, cx, cy, sx, sy = g['footprint']
            # 几何足迹在 X/Y 方向的占据范围
            x_lo, x_hi = cx - sx, cx + sx
            y_lo, y_hi = cy - sy, cy + sy
            # 覆盖的 cell 中心范围 (中心在足迹内即占据)
            i0 = max(0, math.ceil((x_lo - MAP_X0) / RESOLUTION - 0.5))
            i1 = min(MAP_NX - 1, math.floor((x_hi - MAP_X0) / RESOLUTION - 0.5))
            j0 = max(0, math.ceil((y_lo - MAP_Y0) / RESOLUTION - 0.5))
            j1 = min(MAP_NY - 1, math.floor((y_hi - MAP_Y0) / RESOLUTION - 0.5))
            for j in range(j0, j1 + 1):
                y = MAP_Y0 + (j + 0.5) * RESOLUTION
                for i in range(i0, i1 + 1):
                    grid[j][i] = 1
        else:  # circle
            _, cx, cy, r = g['footprint']
            i0 = max(0, math.ceil((cx - r - MAP_X0) / RESOLUTION - 0.5))
            i1 = min(MAP_NX - 1, math.floor((cx + r - MAP_X0) / RESOLUTION - 0.5))
            j0 = max(0, math.ceil((cy - r - MAP_Y0) / RESOLUTION - 0.5))
            j1 = min(MAP_NY - 1, math.floor((cy + r - MAP_Y0) / RESOLUTION - 0.5))
            r2 = r * r
            for j in range(j0, j1 + 1):
                y = MAP_Y0 + (j + 0.5) * RESOLUTION
                for i in range(i0, i1 + 1):
                    x = MAP_X0 + (i + 0.5) * RESOLUTION
                    if (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                        grid[j][i] = 1
    return grid, included, excluded


def write_pgm(path, grid):
    h, w = len(grid), len(grid[0])
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'# generated by generate_map_from_xml.py\n'.encode())
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(bytes(PGM_OCC if grid[j][i] else PGM_FREE
                      for j in range(h) for i in range(w)))


def write_yaml(path, pgm_name):
    with open(path, 'w') as f:
        f.write(f'image: {pgm_name}\n')
        f.write('resolution: 0.05\n')
        f.write(f'origin: [{MAP_X0:.2f}, {MAP_Y0:.2f}, 0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')
        f.write('mode: trinary\n')


def world_to_cell(x, y):
    i = math.floor((x - MAP_X0) / RESOLUTION)
    j = math.floor((y - MAP_Y0) / RESOLUTION)
    return i, j


def free_width_at_x(grid, x, y_lo, y_hi):
    """统计给定 X 列在 [y_lo, y_hi] 内的自由竖直连续段 (用于通道宽度校验)."""
    i, _ = world_to_cell(x, 0)
    runs, run = [], 0
    j0, j1 = world_to_cell(0, y_lo)[1], world_to_cell(0, y_hi)[1]
    for j in range(j0, j1 + 1):
        if grid[j][i] == 0:
            run += 1
        else:
            if run:
                runs.append(run)
            run = 0
    if run:
        runs.append(run)
    return [(n * RESOLUTION) for n in runs]


def main():
    ap = argparse.ArgumentParser(description='从 lab_env.xml 生成 Nav2 2D 占据地图')
    ap.add_argument('--xml', default=os.path.normpath(DEFAULT_XML),
                    help='lab_env.xml 路径 (默认: motion_control .../environment/lab_env.xml)')
    ap.add_argument('--output-dir', default=os.path.normpath(DEFAULT_OUT_DIR),
                    help='输出目录 (默认: humanoid_sim/maps)')
    ap.add_argument('--basename', default='mujoco_lab',
                    help='输出文件基名 (默认 mujoco_lab -> mujoco_lab.pgm/.yaml)')
    ap.add_argument('--include-dynamic', action='store_true',
                    help='包含 dyn_* 动态障碍 (默认排除, 保留动态避障测试意义)')
    args = ap.parse_args()

    if not os.path.isfile(args.xml):
        sys.exit(f"[错误] 找不到世界 XML: {args.xml}")

    geoms = parse_geoms(args.xml)
    grid, included, excluded = rasterize(geoms, args.include_dynamic)

    os.makedirs(args.output_dir, exist_ok=True)
    pgm_path = os.path.join(args.output_dir, args.basename + '.pgm')
    yaml_path = os.path.join(args.output_dir, args.basename + '.yaml')
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, os.path.basename(pgm_path))

    occ = sum(sum(row) for row in grid)
    total = MAP_NX * MAP_NY
    print(f"[OK] 解析 {len(geoms)} 个 geom; 入图 {len(included)} / 排除 {len(excluded)}")
    print(f"[OK] 地图 {MAP_NX}x{MAP_NY} @ {RESOLUTION}m, 占据 {occ} 格 "
          f"({100.0 * occ / total:.1f}%), 自由 {total - occ} 格")
    print(f"[OK] 写出: {pgm_path}")
    print(f"[OK] 写出: {yaml_path}")

    print("\n── 排除明细 ──")
    for name, why in excluded:
        print(f"  {name:22s} {why}")
    print("\n── 关键通道校验 (cell 级自由宽度) ──")
    for label, x, ylo, yhi in [
            ('通道A (X=2.0, Y -4.5~-1.5)', 2.0, -4.5, -1.5),
            ('通道B (X=2.0, Y +2.5~+3.9)', 2.0, 2.5, 3.9),
            ('玻璃缺口 (X=5.0, Y -2.2~+1.4)', 5.0, -2.2, 1.4)]:
        widths = free_width_at_x(grid, x, ylo, yhi)
        print(f"  {label}: 自由段 {[f'{w:.2f}m' for w in widths]}")

    print("\n── 关键点状态 ──")
    for label, x, y in [('原点(0,0)', 0, 0), ('A目标(5,0)', 5, 0),
                        ('B目标(8,-3)', 8, -3), ('D目标(6,3.2)', 6, 3.2),
                        ('E/F目标(-7.8,-3.9)', -7.8, -3.9),
                        ('通道A中心(2,-3)', 2, -3)]:
        i, j = world_to_cell(x, y)
        s = 'FREE' if grid[j][i] == 0 else 'OCCUPIED'
        print(f"  {label:22s} {s}")


if __name__ == '__main__':
    main()

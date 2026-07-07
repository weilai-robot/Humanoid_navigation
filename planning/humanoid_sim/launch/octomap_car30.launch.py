"""
OctoMap 建图 Launch — 小车模型 (car_nav.xml)
雷达高度: 0.20m (离地)
  car body pos z=0.10m + lidar_site offset z=0.10m = 0.20m
FastLIO2地图坐标: Z=0 = 雷达位置 (物理0.20m高度)
  地面在地图中 Z ≈ -0.20m
  occupancy_min_z 取 -0.05m (对应物理0.15m，滤除地面噪声)
  occupancy_max_z 取  1.50m (对应物理1.70m高障碍物)
对应仿真脚本: car_nav.py
对应FastLIO2: car_mid360.yaml
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05,
                'frame_id': 'camera_init',      # FastLIO2 全局坐标系
                'base_frame_id': 'body',        # 机器人基座坐标系

                'latch': False,
                'transform_timeout': 2.0,
                'frame_skip': 1,

                'sensor_model.max_range': 10.0,

                # 地面过滤
                # RANSAC 自动检测点云中最大水平面作为地面，比固定 Z 阈值更自适应
                'filter_ground_plane': True,
                'ground_filter.distance': 0.20,      # RANSAC 内点阈值：离检测平面 0.20m 内视为地面点
                'ground_filter.plane_distance': 0.3, # 允许地面平面距传感器最远 0.25m（雷达离地 0.20m）

                # 占据体素高度范围（基于 FastLIO2 地图坐标系）
                # 地面在 Z≈-0.30m，障碍物从 Z=-0.20m 向上到 Z=1.50m
                # 对应物理高度: -0.20+0.30=0.10m 到 1.50+0.30=1.80m
                'occupancy_min_z': -0.05,       # 比原机器人(-1.10m)高1.01m
                'occupancy_max_z':  1.50,       # 与原机器人相同

                # 概率模型
                # occupancy_min 从 0.16 → 0.30：要求统计上更一致的击中才标记为占据，
                # 偶发地面散点（击中 1-2 次）不再能通过阈值，实际障碍物反复扫到仍能建出
                'sensor_model.hit': 0.8,
                'sensor_model.miss': 0.4,       # 0.3 → 0.4：更积极清除空闲区域，散点概率衰减更快
                'occupancy_min': 0.85,          # >sensor_model.hit(0.8): 等价于 mark_threshold=2，单次飞点不触发
                'occupancy_max': 0.97,
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body')
            ]
        ),
    ])

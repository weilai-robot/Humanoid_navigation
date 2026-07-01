"""
OctoMap 建图 Launch — 实机版 (Jetson Orin Nano)
雷达: Mid360 正装，高度 0.30m 离地
FastLIO2 地图坐标系: Z=0 = 雷达位置 (物理0.30m高度)
  地面在地图中 Z ≈ -0.30m
  occupancy_min_z 取 -0.20m (高于地面0.10m，滤除地面噪声)
  occupancy_max_z 取  1.50m (对应物理1.80m高障碍物)
对应配置: car_30_mid360_real.yaml
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
                'use_sim_time': False,          # 实机使用真实时钟
                'resolution': 0.05,
                'frame_id': 'camera_init',      # FastLIO2 全局坐标系
                'tf_tolerance': 0.1,

                'latch': False,
                'transform_timeout': 2.0,
                'frame_skip': 1,

                'sensor_model.max_range': 10.0,

                # 地面过滤
                'filter_ground_plane': False,
                'ground_filter.distance': 0.15,

                # 占据体素高度范围（基于 FastLIO2 地图坐标系）
                'occupancy_min_z': -0.20,
                'occupancy_max_z':  1.50,

                # 概率模型
                'sensor_model.hit': 0.8,
                'sensor_model.miss': 0.3,
                'occupancy_min': 0.16,
                'occupancy_max': 0.97,
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body')
            ]
        ),
    ])

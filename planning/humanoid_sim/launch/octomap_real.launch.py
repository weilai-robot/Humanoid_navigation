"""
OctoMap 建图 Launch — 实机版 (Jetson Orin Nano / 本地 bag 回放)
雷达: Mid360 正装，高度 0.30m 离地
FastLIO2 地图坐标系: Z=0 = 雷达位置 (物理0.30m高度)
  地面在地图中 Z ≈ -0.30m
  occupancy_min_z 取 -0.20m (高于地面0.10m，滤除地面噪声)
  occupancy_max_z 取  1.50m (对应物理1.80m高障碍物)
对应配置: car_30_mid360_real.yaml

离线播包时务必:
  ros2 launch humanoid_sim octomap_real.launch.py use_sim_time:=true
  且 bag play 带 --clock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='true for bag replay with --clock; false for live robot'
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'frame_id': 'camera_init',      # FastLIO2 全局坐标系
                'base_frame_id': 'body',
                'tf_tolerance': 0.1,

                'latch': False,
                'transform_timeout': 2.0,
                'frame_skip': 1,

                'sensor_model.max_range': 10.0,

                # 地面过滤：靠 Z 切片滤地面；真机点云噪声大时勿开 RANSAC
                'filter_ground_plane': False,
                'ground_filter.distance': 0.15,

                # 占据体素高度范围（基于 FastLIO2 地图坐标系，雷达离地 0.30m）
                'occupancy_min_z': -0.20,
                'occupancy_max_z':  1.50,

                # 概率模型（比仿真车更松，避免真机稀疏击中建不出墙）
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

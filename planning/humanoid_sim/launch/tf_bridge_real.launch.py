"""
TF 桥接 Launch 文件 — 实机版 (Jetson + X1 + Mid360)
启动内容：
  1. odom_bridge 节点 — 将 FastLIO2 的 /Odometry 转换为 odom->base_footprint TF
  注意：
  - map->odom 和 map->camera_init 由 open3d_loc (ICP) 节点动态发布，无需静态TF
  - use_sim_time: False，使用真实时钟
  - body_to_footprint_z: -1.25m（X1 LiDAR/IMU 位于胸部，距地面约 1.25m）
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---- odom_bridge: FastLIO2 Odometry -> odom->base_footprint TF ----
        # FastLIO2 发布 /Odometry (frame: camera_init, child: body)
        # 本节点将其转换为 odom->base_footprint，供 Nav2 使用
        Node(
            package='humanoid_sim',
            executable='odom_bridge.py',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'body_to_footprint_z': -1.25,   # X1 LiDAR/IMU 到地面的 Z 偏移（约 1.25m）
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'input_topic': '/Odometry',      # FastLIO2 发布的里程计话题
                'output_topic': '/odom',
            }]
        ),

    ])

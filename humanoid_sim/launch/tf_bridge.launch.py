"""
TF 桥接 Launch 文件
启动内容：
  1. odom_bridge 节点 — 将 FastLIO2 的 /Odometry 转换为 odom->base_footprint TF
  2. 静态TF: map -> odom（初始为单位变换，后续 AMCL 接管）
  3. 静态TF: map -> camera_init（FastLIO2/OctoMap 使用）
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---- 1. odom_bridge: FastLIO2 Odometry -> odom->base_footprint TF ----
        Node(
            package='humanoid_sim',
            executable='odom_bridge.py',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'body_to_footprint_z': -1.31,   # body(雷达/IMU处) 到 base_footprint(地面) 的Z偏移
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'input_topic': '/Odometry',      # FastLIO2 发布的里程计话题
            }]
        ),

        # ---- 2. 静态TF: map -> odom (初始为单位变换) ----
        # AMCL 当前未启用 (tf_broadcast=False 且无 /scan)，由此静态 TF 提供 map->odom，
        # 否则全局 costmap 无法在 map 系定位机器人，导航目标会被 bt_navigator 拒绝。
        # future: 启用 AMCL 动态发布 map->odom 时，再注释掉此节点。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_odom',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # ---- 3. 静态TF: map -> camera_init (FastLIO2 的世界坐标系) ----
        # FastLIO2 和 OctoMap 使用 camera_init 作为全局参考系
        # camera_init 在雷达安装高度(1.31m)处初始化, 不在地面
        # 必须加上高度偏移, 否则 VoxelLayer 的传感器原点会在 Z=0 (地面), 导致 raytrace 失败
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_camera_init',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '1.31', '0', '0', '0', 'map', 'camera_init']
        ),
    ])

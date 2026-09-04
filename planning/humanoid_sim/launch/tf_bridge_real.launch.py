"""
TF 桥接 Launch 文件 — 实机版 (X1 + Mid360 倒装)
启动内容：
  1. odom_bridge — FastLIO2 /Odometry → odom→base_footprint
                 + /cmd_vel → /cmd_vel_limiter（AimRT 只订 limiter）
  2. 静态 TF（非精确标定，仅接通 TF 树，几何近似仿真）:
       base_footprint → base_link (z≈0.65)
       base_link → lidar_link (z≈0.66, Rx≈180° 倒装)
  注意：
  - map→odom / map→camera_init 由 open3d_loc 动态发布，勿再发静态
  - use_sim_time: False
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---- odom_bridge: Odometry → odom→base_footprint + cmd_vel 限幅中继 ----
        Node(
            package='humanoid_sim',
            executable='odom_bridge.py',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'body_to_footprint_z': -1.25,   # X1 LiDAR/IMU 到地面约 1.25m
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'input_topic': '/Odometry',
                'output_topic': '/odom',
                'mount_rpy': [0.0, 0.0, -90.0],   # 雷达正面朝左 → mount yaw -90；odom_bridge 扣 R⁻¹=Rz(+90) 让 base+X=cam+Y=前。2026-09-04 RViz 实测 +90 朝后、-90 朝前
                # 显式写死：AimRT x1_cfg 只订 /cmd_vel_limiter，不可靠默认值
                'enable_cmd_vel_relay': True,
                'cmd_vel_input_topic': '/cmd_vel',
                'cmd_vel_output_topic': '/cmd_vel_limiter',
                # 与 nav2_real.yaml MPPI ax_max/az_max 对齐（真机更保守）
                'max_ax': 1.0,
                'max_ay': 0.5,
                'max_az': 1.0,
            }]
        ),

        # ---- 静态 TF: base_footprint → base_link ----
        # 非精确标定，仅接通 TF 树（避免 base_link 孤儿）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            parameters=[{'use_sim_time': False}],
            arguments=['0', '0', '0.65', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # ---- 静态 TF: base_link → lidar_link ----
        # 非精确标定；qx=1 近似倒装 Mid360（与仿真 tf_bridge 一致）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_to_lidar_link',
            parameters=[{'use_sim_time': False}],
            arguments=['0', '0', '0.66', '1', '0', '0', '0', 'base_link', 'lidar_link']
        ),
    ])

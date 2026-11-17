"""
TF 桥接 Launch 文件
启动内容：
  1. odom_bridge 节点 — 将 FastLIO2 的 /Odometry 转换为 odom->base_footprint TF
     + cmd_vel relay: /cmd_vel → /cmd_vel_limiter (加速度限幅)
  2. 静态TF: map -> odom（初始为单位变换，后续 AMCL 接管）
  3. 静态TF: map -> camera_init（FastLIO2/OctoMap 使用）
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---- 1. odom_bridge: FastLIO2 Odometry -> odom->base_footprint TF ----
        #        + cmd_vel relay: /cmd_vel -> /cmd_vel_limiter (加速度限幅)
        Node(
            package='humanoid_sim',
            executable='odom_bridge.py',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'body_to_footprint_z': -1.25,   # body(雷达/IMU处) 到 base_footprint(地面) 的Z偏移 (X1 LiDAR高度≈1.25m)
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'input_topic': '/Odometry',      # FastLIO2 发布的里程计话题
                # cmd_vel 加速度限幅中继 (Nav2 /cmd_vel → aimrt_main /cmd_vel_limiter)
                # 本限幅器是链路唯一加速度权威（nav2 MPPI 加速度死键已于 2026-11-08 删除）
                'enable_cmd_vel_relay': True,
                'cmd_vel_input_topic': '/cmd_vel',
                'cmd_vel_output_topic': '/cmd_vel_limiter',
                'max_ax': 1.5,    # m/s²  加速向上界（对称夹）
                'max_ay': 0.5,    # m/s²  侧步保守值（DiffDrive vy=0 不触发）
                'max_az': 2.0,    # rad/s² wz 加速向上界（仅截断异常）
                # P6a 非对称减速限幅（急停温柔化）：sim 0.6 由 §3.2 v2 口径背书
                # （干净巡航持续刹车需求 ≤0.46、深帧 0.28% 孤立单帧自愈）
                'max_decel': 0.6,
                # wz 衰减夹板：0 = 沿用 max_az（行为不变）；收紧方向待证，勿擅改
                'max_decel_wz': 0.0,
                # P6b 死人开关：/cmd_vel 沉默期非零末目标强制归零超时（s）
                # 依据 §3.3：巡航帧间隙 p99=0.053s（9.4× 裕量）、观测 max 0.330s 且命令恒定天然容忍
                'cmd_silence_timeout_s': 0.5,
            }]
        ),

        # ---- 2. 静态TF: map -> odom (初始为单位变换) ----
        # 如果你接入 ICP / AMCL 动态发布 map->odom，则需要注释掉静态 map->odom，
        # 否则会造成 TF 来源重复（看起来能跑，但定位融合语义不干净）。
        #
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tf_map_to_odom',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # ),

        # ---- 3. 静态TF: map -> camera_init (FastLIO2 的世界坐标系) ----
        # FastLIO2 和 OctoMap 使用 camera_init 作为全局参考系
        # camera_init 在 X1 LiDAR 高度(1.25m)处初始化, 不在地面
        # map z=0 对应 car 建图时 LiDAR 高度(0.20m), 故 camera_init 在 map 中 z = 1.25 - 0.20 = 1.05m
        # 必须加上高度偏移, 否则 VoxelLayer 的传感器原点会在 Z=0 (地面), 导致 raytrace 失败
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_camera_init',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '1.05', '0', '0', '0', 'map', 'camera_init']
        ),

        # ---- 4. 静态TF: base_footprint -> base_link ----
        # MuJoCo navigation does not launch simulation.launch.py, so robot_state_publisher
        # is not running to publish the fixed joints from robot.xacro.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0.65', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # ---- 5. 静态TF: base_link -> lidar_link ----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_to_lidar_link',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0.66', '1', '0', '0', '0', 'base_link', 'lidar_link']
        ),
    ])

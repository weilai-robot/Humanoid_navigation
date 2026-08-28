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
        # ---- 1b. cloud_retimer: 重盖 FastLIO 点云时间戳 → /cloud_registered_body_fresh ----
        # FastLIO 点云 stamp=lidar_end_time; RTF<0.5 积压下早于 TF 缓冲区最旧条目,
        # nav2 MessageFilter 永久丢弃观测 ("earlier than all the data in the
        # transform cache", transform_tolerance 等待对它无效) → costmap 对动态
        # 障碍半盲 (通道A线上 dyn_person/dyn_crate 不在静态地图, CI 每场景 1 碰撞)。
        # nav2 观测源已改订 fresh 话题 (见 nav2_mujoco.yaml)。
        Node(
            package='humanoid_sim',
            executable='cloud_retimer.py',
            name='cloud_retimer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),


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
                # 参数与 nav2_mujoco.yaml 中 MPPI ax_max/az_max 对齐
                'enable_cmd_vel_relay': True,
                'cmd_vel_input_topic': '/cmd_vel',
                'cmd_vel_output_topic': '/cmd_vel_limiter',
                'max_ax': 1.0,    # m/s²  (MPPI ax_max=1.0)
                'max_ay': 0.5,    # m/s²  (侧步保守值，DiffDrive vy=0 不触发)
                'max_az': 1.0,    # rad/s² (MPPI az_max=1.0)
                # 幅值硬限幅 —— 人形安全上限, 拦截任何旁路超速指令
                # (behavior_server 默认 spin 0.798 rad/s 曾直接导致摔倒, 2026-07-14)
                'max_vx_abs': 0.45,   # m/s
                'max_vy_abs': 0.1,    # m/s
                'max_wz_abs': 0.45,   # rad/s
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

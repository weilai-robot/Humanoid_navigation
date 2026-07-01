"""
TF 桥接 Launch 文件
启动内容：
  1. odom_bridge 节点 — 将 FastLIO2 的 /Odometry 转换为 odom->base_footprint TF
  2. (已移除) map -> odom 静态 TF — AMCL 动态发布
  3. 静态TF: odom -> camera_init — 连接 odom 树和 FastLIO2 树
  4. 静态TF: base_footprint -> base_link
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

        # ---- 2. (已移除) map -> odom 静态 TF ----
        # AMCL 已启用 (tf_broadcast=True), 会动态发布 map→odom。
        # 如果保留此静态 TF, 会与 AMCL 的动态 TF 冲突。

        # ---- 3. 静态TF: odom -> camera_init ----
        # 连接 odom 树和 FastLIO2 树, 解决 TF 死锁
        # (之前 map→camera_init + AMCL→map→odom 导致两棵树不连通)
        # camera_init 在雷达高度 (1.31m), odom 在地面 (0m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_to_camera_init',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '1.31', '0', '0', '0', 'odom', 'camera_init']
        ),

        # ---- 4. 静态TF: base_footprint -> base_link ----
        # 正常应由 URDF/robot_state_publisher 发布，但 navigation.launch.py 未启动它，
        # 导致 base_link 不存在，Costmap/Controller (robot_base_frame=base_link) 报
        # "invalid frame id base_link" 并拒绝导航目标。
        # base_footprint 是 base_link(骨盆)在地面的投影：x=y=0、无旋转，
        # Z≈0.86m (base_link 在 odom≈-0.45m, base_footprint 在 odom≈-1.31m)。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_to_base_link',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0.86', '0', '0', '0', 'base_footprint', 'base_link']
        ),
    ])

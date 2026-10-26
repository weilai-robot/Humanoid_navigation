"""
OctoMap 建图 Launch — 机器人吊架 · 倒装 native（X1 头部 Mid360 倒装，离地 ~1.2~1.25m）

SOP: docs/sim2real/真机建图SOP-机器人吊架native-2026-10-26.md（附录 B2）
基底: octomap_real.launch.py（小车正装线，保持不动）；本文件仅机器人建图用
对应 FastLIO2 配置: F1_mapping_mid360.yaml

坐标系口径（倒装 native，SOP §2.4 ⚠ 人因风险）:
  FastLIO2 camera_init = 启动时 IMU 系，z 轴朝物理下方、无重力对齐（IMU_Processing.hpp init 只做均值）
  地面 = +h（h = 雷达离地高度，本 launch 的 h 参数，卷尺实测，吊架建议 1.2~1.25m）
  离地占据窗 [0.10, 1.60] → z 窗 [h-1.6, h-0.1]
  ⚠ z 朝下系: 数值大 = 物理低处；RViz 看 map 系世界上下颠倒是预期

离线播包时务必:
  ros2 launch humanoid_sim octomap_robot_mapping.launch.py use_sim_time:=true h:=<实测h>
  且 bag play 带 --clock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    h = LaunchConfiguration('h')
    # 倒装系: z = h - 离地高度；数值大 = 物理低处
    min_z = ParameterValue(PythonExpression([h, ' - 1.6']), value_type=float)
    max_z = ParameterValue(PythonExpression([h, ' - 0.1']), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true for bag replay with --clock; false for live robot'
        ),
        DeclareLaunchArgument(
            'h', default_value='1.25',
            description='雷达离地高度 m（卷尺实测；吊架建议 1.2~1.25m ≈ 站立导航高度）'
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'frame_id': 'camera_init',      # FastLIO2 全局坐标系（倒装 native）
                'base_frame_id': 'body',
                'tf_tolerance': 0.1,

                'latch': False,
                'transform_timeout': 2.0,
                'frame_skip': 1,

                'sensor_model.max_range': 10.0,

                # 地面过滤：靠 Z 切片滤地面；真机点云噪声大时勿开 RANSAC
                'filter_ground_plane': False,
                'ground_filter.distance': 0.15,

                # 倒装 native 系: z 朝下，地面 = +h；离地 [0.10, 1.60] → z [h-1.6, h-0.1]
                'occupancy_min_z': min_z,
                'occupancy_max_z': max_z,

                # 概率模型（沿用小车真机口径：比仿真松，避免稀疏击中建不出墙）
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

"""
真机导航一键启动 Launch — X1 + Mid360 + car_30 建图
启动顺序:
  1. tf_bridge_real  — odom_bridge (FastLIO2 /Odometry -> odom->base_footprint TF)
  2. open3d_loc      — ICP 全局定位 (发布 map->odom + map->camera_init TF)
  3. nav2_bringup    — Nav2 全栈 (全局规划 + MPPI 局部控制, 发布 /cmd_vel)

前置条件 (需在其他终端手动启动):
  终端A: ros2 launch livox_ros_driver2 msg_MID360_launch.py
  终端B: ros2 launch fast_lio mapping_real.launch.py

TF 链:
  map <-(ICP)-- camera_init <-(FastLIO2)-- body
  map <-(ICP)-- odom <-(odom_bridge)-- base_footprint

地图:
  2D Nav2 地图: humanoid_sim/maps/school_room_30.yaml
  3D ICP 地图:  open3d_loc/maps/car_30_map.pcd
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_open3d_loc = get_package_share_directory('open3d_loc')

    map_file = os.path.join(pkg_humanoid, 'maps', 'school_room_30.yaml')
    params_file = os.path.join(pkg_humanoid, 'config', 'nav2_real.yaml')
    pcd_map_file = os.path.join(pkg_open3d_loc, 'maps', 'car_30_map.pcd')

    # ====== 1. TF 桥接 (odom_bridge: /Odometry -> odom->base_footprint) ======
    tf_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'tf_bridge_real.launch.py')
        )
    )

    # ====== 2. ICP 全局定位 (open3d_loc: 发布 map->odom TF) ======
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_open3d_loc, 'launch', 'open3d_loc_x1.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_file': pcd_map_file,
        }.items()
    )

    # ====== 3. Nav2 核心 (use_sim_time=false, 真实时钟) ======
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        tf_bridge_launch,
        open3d_loc_launch,
        nav2_launch,
    ])

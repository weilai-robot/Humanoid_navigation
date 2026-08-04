"""
真机导航一键启动 Launch — X1 + Mid360 倒装
启动顺序:
  1. tf_bridge_real  — odom_bridge (FastLIO2 /Odometry -> odom->base_footprint TF)
  2. open3d_loc      — ICP 全局定位 (open3d_loc_x1_real.launch.py)
  3. nav2_bringup    — Nav2 全栈 (params: nav2_real.yaml, 发布 /cmd_vel)

前置条件 (需在其他终端手动启动，或用 F1/scripts/run_nav_real.sh):
  终端A: ros2 launch livox_ros_driver2 msg_MID360_launch.py
  终端B: ros2 launch fast_lio mapping_real.launch.py config_file:=F1_real_mid360.yaml

TF 链:
  map <-(ICP)-- camera_init <-(FastLIO2)-- body
  map <-(ICP)-- odom <-(odom_bridge)-- base_footprint

地图 (真机 FastLIO 建图产物):
  2D Nav2 地图: humanoid_sim/maps/car30_real_fastlio.yaml
  3D ICP 地图:  open3d_loc/maps/car_30_real_map.pcd
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_open3d_loc = get_package_share_directory('open3d_loc')

    map_file = os.path.join(pkg_humanoid, 'maps', 'car30_real_fastlio.yaml')
    params_file = os.path.join(pkg_humanoid, 'config', 'nav2_real.yaml')

    # ====== 1. TF 桥接 (odom_bridge: /Odometry -> odom->base_footprint) ======
    tf_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'tf_bridge_real.launch.py')
        )
    )

    # ====== 2. ICP 全局定位 (真机专用 launch，默认 car_30_real_map.pcd) ======
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_open3d_loc, 'launch', 'open3d_loc_x1_real.launch.py')
        )
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

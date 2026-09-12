"""
真机 Nav2 启动 (拆自 navigation_real.launch.py) — tf_bridge_real + nav2_bringup

对齐仿真 navigation.launch.py 的组织：tf_bridge 和 nav2 在同一个 launch
（仿真就是 tf_bridge.launch.py + nav2_bringup）。tf_bridge 负责 odom->base_footprint
TF + cmd_vel 限幅中继 + 静态 TF；nav2_bringup 负责控制/规划/代价地图。
真机比仿真多出的 ICP (open3d_loc) 是定位纠偏，单独在另一窗口跑（仿真无 ICP，
用 nav2_bringup 内的 AMCL）。

启动内容:
  1. tf_bridge_real  — odom_bridge (FastLIO2 /Odometry -> odom->base_footprint)
                       + 静态 TF (base_footprint->base_link->lidar_link)
                       + /cmd_vel -> /cmd_vel_limiter 限幅中继
  2. nav2_bringup    — controller_server (RotationShim+MPPI) / planner_server
                       (Navfn) / bt_navigator / costmap_2d (VoxelLayer) / map_server

前置:
  1. livox + fastlio 已起
  2. ICP (open3d_loc_x1_real.launch.py) 已起且 map->odom TF 稳定
     （run_nav_real_split.sh 的 [icp] 窗口，须先于此窗口起）

cmd_vel 链路: Nav2 /cmd_vel -> odom_bridge /cmd_vel_limiter -> aimrt_main
  (odom_bridge 在本 launch 的 tf_bridge_real 里，与 nav2 同窗口，对齐仿真)

注: 原文件 navigation_real.launch.py 未改动，仍可供 run_real_all.sh 合体一键启动。
    分窗口（ICP/Nav2 各一终端看日志）用 run_nav_real_split.sh。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_humanoid, 'maps', 'car30_real_fastlio.yaml')
    params_file = os.path.join(pkg_humanoid, 'config', 'nav2_real.yaml')

    # 1. TF 桥接 (odom_bridge + 静态 TF + cmd_vel 限幅中继)
    #    与仿真 navigation.launch.py 一致：tf_bridge 放 nav2 侧，不跟 ICP 捆
    tf_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'tf_bridge_real.launch.py')
        )
    )

    # 2. Nav2 核心 (use_sim_time=false, 真实时钟)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'False',
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        tf_bridge_launch,
        nav2_launch,
    ])

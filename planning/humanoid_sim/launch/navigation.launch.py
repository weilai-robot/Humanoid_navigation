import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 地图/参数默认沿用既有 sim 场景 (mujoco_lab); real2sim P2 起 run_mujoco_nav_real.sh
# 传 map_yaml:=robot_lab_20261030.yaml —— sim nav2 与真机同用一张 GIMP 2D 图。


def _setup(context, *args, **kwargs):
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 指向刚才保存的地图和新建的参数文件
    map_file = os.path.join(pkg_humanoid, 'maps', LaunchConfiguration('map_yaml').perform(context))
    params_file = os.path.join(pkg_humanoid, 'config', LaunchConfiguration('params_yaml').perform(context))

    # ====== 1. 包含 tf_bridge (发布 odom 和 map->camera_init 静态 TF) ======
    tf_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'tf_bridge.launch.py')
        )
    )

    # ====== 2. 包含 pc2scan (将 3D 点云转换为 2D 激光给 AMCL) ======
    # pc2scan_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_humanoid, 'launch', 'pc2scan.launch.py')
    #     )
    # )

    # ====== 3. 包含 Nav2 核心 (带 AMCL 等) ======
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'True',  # true
            'autostart': 'True'
        }.items()
    )

    # OpaqueFunction.function 必须返回 list(可迭代); 包 LaunchDescription 会
    # 抛 "'LaunchDescription' object is not iterable" (Humble 实测)
    return [
        tf_bridge_launch,
        # pc2scan_launch,
        nav2_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value='mujoco_lab.yaml',
                              description='maps/ 下的 2D 地图 yaml 文件名'),
        DeclareLaunchArgument('params_yaml', default_value='nav2_mujoco.yaml',
                              description='config/ 下的 nav2 参数文件名'),
        OpaqueFunction(function=_setup),
    ])

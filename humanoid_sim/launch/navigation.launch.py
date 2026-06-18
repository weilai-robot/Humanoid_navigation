import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 是否使用仿真时间（真机启动时传 use_sim_time:=False）
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 地图和参数文件
    map_file = os.path.join(pkg_humanoid, 'maps', 'mujoco_car.yaml')  # f1_test1 school_room school_room2
    params_file = os.path.join(pkg_humanoid, 'config', 'nav2_mujoco.yaml')

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

    # ====== 3. 包含 Nav2 核心 (AMCL + MPPI + Costmap) ======
    # Nav2 controller_server 在 nav2_mujoco.yaml 中配置 cmd_vel_topic: /cmd_vel_limiter
    # 直接输出到 control_module 订阅的 topic，无需额外 remap
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        tf_bridge_launch,
        # pc2scan_launch,
        nav2_launch
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 是否使用仿真时间（真机启动时传 use_sim_time:=False）
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 地图和参数文件
    map_file = os.path.join(pkg_humanoid, 'maps', 'lab_env_map.yaml')
    params_file = os.path.join(pkg_humanoid, 'config', 'nav2_mujoco.yaml')

    # ====== 1. 包含 tf_bridge (发布 odom 和 map->camera_init 静态 TF) ======
    tf_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'tf_bridge.launch.py')
        )
    )

    # ====== 2. pc2scan (3D 点云 → 2D 激光线扫描, 供 AMCL 用) ======
    pc2scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'pc2scan.launch.py')
        )
    )

    # ====== 3. 包含 Nav2 核心 (AMCL + MPPI + Costmap) + RViz ======
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

    # ====== 4. RViz (Fixed Frame: map) ======
    rviz_config = os.path.join(pkg_humanoid, 'rviz_cfg', 'nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        tf_bridge_launch,
        pc2scan_launch,
        nav2_launch,
        rviz_node,
    ])

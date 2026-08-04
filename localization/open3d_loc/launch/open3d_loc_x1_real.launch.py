"""
Open3D ICP 全局定位 — X1 【真机】
默认:
  use_sim_time:=false
  map_file:= open3d_loc/maps/car_30_real_map.pcd

仿真请用: open3d_loc_x1.launch.py
一键导航会 Include 本文件: humanoid_sim/navigation_real.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='真机使用系统时钟'
    )

    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_x1.yaml'
    ])

    default_map_path = PathJoinSubstitution([
        open3d_loc_share,
        'maps',
        'car_30_real_map.pcd'
    ])
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path,
        description='真机 3D 全局地图 (.pcd / .ply)'
    )
    map_file = LaunchConfiguration('map_file')

    # base_link -> motion_link（与仿真 launch 一致；Nav2 主链用 base_footprint）
    static_tf_base_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_center_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0',
                   '1', 'base_link', 'motion_link']
    )

    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                'path_map': map_file,
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.2,
                'voxelsize_fine': 0.05,
                # FPFH 过阈 + ICP-only 复核锁定；低分不写 map→odom TF
                'threshold_fitness': 0.75,
                'threshold_fitness_init': 0.85,
                # 正立先验：挡 roll≈180° 高分假峰
                'max_init_roll_deg': 30.0,
                'max_init_pitch_deg': 30.0,
                'max_init_retries': 20,
                'loc_frequence': 2.5,
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        static_tf_base_center,
        global_localization_node,
    ])

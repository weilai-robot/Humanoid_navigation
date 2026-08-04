"""
Open3D ICP 全局定位 — X1 【真机】
默认:
  use_sim_time:=false
  map_file:= open3d_loc/maps/car_30_real_map.pcd

仿真请用: open3d_loc_x1.launch.py
一键导航会 Include 本文件: humanoid_sim/navigation_real.launch.py

TF 约定（避免双父）:
  - 不发 base_link→motion_link：节点会动态发 map→motion_link
  - 发 base_link→imu_link（identity）：消掉 init 时 lookup ERROR；
    切勿发 imu_link→base_link（会与 base_footprint→base_link 争父）
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

    # parent=base_link, child=imu_link（identity）—— 方向必须对：
    # lookupTransform("base_link","imu_link") 需要 imu_link 挂在 base_link 下；
    # 若写成 imu_link→base_link，会与 tf_bridge_real 的 base_footprint→base_link 双父。
    static_tf_baselink2imulink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink2imulink',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
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
        # 不发 base_link→motion_link（避免与节点 map→motion_link 双父）
        # mat_baselink2motionlink_ 缺 TF 时回落 Identity，数值正确
        static_tf_baselink2imulink,
        global_localization_node,
    ])

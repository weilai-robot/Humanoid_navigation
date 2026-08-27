"""
Open3D ICP 全局定位 — X1 【仿真】
默认: use_sim_time:=true, map:=mujoco_lab.pcd

真机请用: open3d_loc_x1_real.launch.py
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
        default_value='true',
        description='仿真默认用 /clock；真机请用 open3d_loc_x1_real.launch.py'
    )

    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_x1.yaml'
    ])

    default_map_path = PathJoinSubstitution([
        open3d_loc_share,
        'maps',
        'mujoco_lab.pcd'
    ])
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path,
        description='Path to the global map point cloud file (.pcd or .ply)'
    )
    map_file = LaunchConfiguration('map_file')

    static_tf_camera_init2odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']
    )

    static_tf_imulink2baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imulink2baselink',
        # 方向修正 (对齐 open3d_loc_x1_real.launch.py 的教训):
        # parent=base_link, child=imu_link — 让 imu_link 挂在 base_link 下。
        # 旧方向 (imu_link→base_link) 双重错误: base_link 争父 (tf_bridge 已发
        # base_footprint→base_link, 双父被 TF 拒) + imu_link 成无父根从 map 不可
        # 达 → lookupTransform("base_link","imu_link") 报 "imu_link does not
        # exist" → map→odom 不发布 → 全局 costmap 无 robot pose → NavFn 无法
        # 规划 (run 33034736667: failed to create plan, 机器人静止 0/6)。
        # lookupTransform 支持沿 edge 反向遍历, base_link→imu_link 同样满足查询。
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

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
                'threshold_fitness': 0.75,
                'threshold_fitness_init': 0.85,
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

    pointcloud_transformer_node = Node(
        package='open3d_loc',
        executable='pointcloud_transformer_node',
        name='pointcloud_transformer_node',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered_body_1',
            'output_topic': '/cloud_registered_map',
            'global_map_topic': '/global_map',
            'source_frame': 'base_link',
            'target_frame': 'map',
            'voxel_leaf_size': 0.1,
            'map_voxel_leaf_size': 0.2,
            'max_global_points': 1000000,
            'map_publish_frequency': 1.0,
            'enable_global_map': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        # static_tf_camera_init2odom,
        # static_tf_imulink2baselink,
        static_tf_base_center,
        global_localization_node,
        # pointcloud_transformer_node
    ])

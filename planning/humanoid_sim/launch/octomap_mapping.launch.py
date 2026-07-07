import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05,  #地图体素（voxel）分辨率，单位米。越小精度越高但内存/计算量越大。
                'frame_id': 'camera_init',  # 地图的参考坐标系（通常是全局静止坐标系） map
                # 'base_frame_id': 'body',  #机器人基座坐标系，用于判断哪些点属于“地面以下”等。base_footprint

                'latch': False,                      # 若设为 True，只会发布一次地图；设为 False 才能持续更新增量地图。
                'transform_timeout': 2.0,            # TF 查找超时时间（秒）。增大以适应低频 TF 发布（如 SLAM 频率较低时）
                # 'transform_tolerance': 0.2,         # 增加变换容差
                # 'message_filter_queue_size': 500,  # 增大队列，缓解频率不匹配
                # 'tf_delay': 0.1, 
                'frame_skip': 1,                     # 每接收 1 帧点云就尝试插入地图（不跳过）。若设为 5，则每 5 帧处理 1 帧。

                'sensor_model.max_range': 10.0,   # 注意用 . 而不是 / , 只处理距离传感器 ≤20 米的点。超出部分被忽略，避免无效更新和拖尾
                
                # 地面过滤（ROS2 正确参数名）
                'filter_ground_plane': False,  #是否启用 RANSAC 自动检测并滤除地面平面。设为 False 表示不滤地面（由上游如 FAST-LIO 处理更佳）。
                'ground_filter.distance': 0.15,    # RANSAC 平面距离阈值，可调大一点如 0.15
                
                # 输入点云高度过滤（防止低矮噪声或过高点）  似乎该参数没有起作用（不知道是否是bug）
                # 'point_cloud_min_z': -2.2,
                # 'point_cloud_max_z': 4.0,
                
                # 最终地图 occupied voxel 高度范围（清理无用层）  似乎基于传感器的高度
                'occupancy_min_z': -0.75,  #需要基于该参数过滤天花板和地板  #1.31m圆柱体使用-1.1
                'occupancy_max_z': 1.5,

                # --- 1. 解决更新慢/一条线问题 (激进概率) --- miss 越小 → 空区域更新越快；hit 越大 → 墙壁越稳定。
                # 扫到障碍物，确信度 (0.5~1.0)，越高墙越实  
                'sensor_model.hit': 0.8,  #当激光“击中”某点（障碍物），该体素被标记为 occupied 的置信度。值越高，障碍越“实”。
                # 扫到空地，确信度 (0.0~0.5)，越低白地出得越快！
                # [关键] 改为 0.1，让它看一眼就确信是空地，解决"细线"问题
                'sensor_model.miss': 0.3, 
                
                # 判定阈值：超过这个值变黑，低于这个值变白
                'occupancy_min': 0.16, 
                'occupancy_max': 0.97,
                
                # 可选：更激进的过滤（如果仍太大）
                # 'filter_limit_min': -10.0,   # 部分分支支持的额外参数
                # 'filter_limit_max': 30.0,
            }],
            remappings=[
                # ('cloud_in', '/cloud_clamped')   # 或直接用 FAST-LIO 的 /cloud_registered
                ('cloud_in', '/cloud_registered_body')
            ]
        ),

        # TF 桥梁 A: map → camera_init (FAST-LIO 常用)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tf_map_camera',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
        # ),
        
        # TF 桥梁 B: body → base_footprint
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tf_body_footprint',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['0', '0', '-1.31', '0', '0', '0', 'body', 'base_footprint']
        # ),
    ])
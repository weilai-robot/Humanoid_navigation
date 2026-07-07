from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[   
                ('cloud_in', '/cloud_registered_body'),  # 订阅实时的点云
                ('scan', '/scan')                        # 输出给 AMCL
            ],
            parameters=[{
                'target_frame': 'base_footprint', # 在底盘坐标系下进行水平切片
                'transform_tolerance': 0.5,
                'min_height': 0.8,                # ⭐ 黄金切片底线：0.8米
                'max_height': 1.0,                # ⭐ 黄金切片顶线：1.0米
                'angle_min': -3.14159,            
                'angle_max': 3.14159,
                'angle_increment': 0.0087,        
                'scan_time': 0.1,                 
                'range_min': 0.3,                 
                'range_max': 8.0,                 
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': True              
            }]
        )
    ])
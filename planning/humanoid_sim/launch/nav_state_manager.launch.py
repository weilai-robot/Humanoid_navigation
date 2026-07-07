from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto_start',        default_value='true'),
        DeclareLaunchArgument('startup_delay_s',   default_value='1.0'),
        DeclareLaunchArgument('zero_hold_s',       default_value='2.5'),
        DeclareLaunchArgument('stand_hold_s',      default_value='2.5'),
        DeclareLaunchArgument('stop_to_stand_s',   default_value='0.5'),
        DeclareLaunchArgument('stand_to_zero_s',   default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_timeout_s', default_value='1.5'),

        Node(
            package='humanoid_sim',
            executable='nav_state_manager.py',
            name='nav_state_manager',
            output='screen',
            parameters=[{
                'auto_start':        LaunchConfiguration('auto_start'),
                'startup_delay_s':   LaunchConfiguration('startup_delay_s'),
                'zero_hold_s':       LaunchConfiguration('zero_hold_s'),
                'stand_hold_s':      LaunchConfiguration('stand_hold_s'),
                'stop_to_stand_s':   LaunchConfiguration('stop_to_stand_s'),
                'stand_to_zero_s':   LaunchConfiguration('stand_to_zero_s'),
                'cmd_vel_timeout_s': LaunchConfiguration('cmd_vel_timeout_s'),
            }],
        ),
    ])

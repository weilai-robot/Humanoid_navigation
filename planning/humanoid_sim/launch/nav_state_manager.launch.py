from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # auto_start 已废弃（纯看门狗无启动序列，模式切换归人/脚本）；
        # 保留参数只为兼容旧调用，传 true 节点侧只打 WARN
        # startup_delay_s / zero_hold_s / stand_hold_s 已删（启动序列无消费方）
        DeclareLaunchArgument('auto_start',        default_value='false'),
        DeclareLaunchArgument('stop_to_stand_s',   default_value='0.5'),
        DeclareLaunchArgument('stand_to_zero_s',   default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('no_cmd_vel_warn_s', default_value='6.0'),

        # float 参数包 ParameterValue：launch 传参全是字符串，手动写
        # `:=3`（int 形态）时节点会在 declare 后因类型不符启动即崩
        # （InvalidParameterTypeException）——value_type=float 统一兜成
        # float（'3'→3.0）。auto_start 走 yaml 解析路径（'false'→bool），
        # 不包（bool 语义下 'false' 会被强转 True）
        Node(
            package='humanoid_sim',
            executable='nav_state_manager.py',
            name='nav_state_manager',
            output='screen',
            parameters=[{
                'auto_start':        LaunchConfiguration('auto_start'),
                'stop_to_stand_s':   ParameterValue(
                    LaunchConfiguration('stop_to_stand_s'), value_type=float),
                'stand_to_zero_s':   ParameterValue(
                    LaunchConfiguration('stand_to_zero_s'), value_type=float),
                'cmd_vel_timeout_s': ParameterValue(
                    LaunchConfiguration('cmd_vel_timeout_s'), value_type=float),
                'no_cmd_vel_warn_s': ParameterValue(
                    LaunchConfiguration('no_cmd_vel_warn_s'), value_type=float),
            }],
        ),
    ])

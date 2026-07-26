import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """仅启动路径跟踪控制器和本次试验记录器."""
    params = os.path.join(
        get_package_share_directory('centerline_extraction'),
        'config',
        'field_row_follow.yaml')

    max_linear_speed = LaunchConfiguration('max_linear_speed')
    quality_aware = LaunchConfiguration('quality_aware')
    max_distance = LaunchConfiguration('max_distance')
    max_time = LaunchConfiguration('max_time')

    declarations = [
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.05',
            description='最大线速度（m/s）；摸底0.05，正式试验锁定为0.18。'),
        DeclareLaunchArgument(
            'quality_aware',
            default_value='true',
            choices=['true', 'false'],
            description='true=质量约束PID，false=固定名义速度PID。'),
        DeclareLaunchArgument(
            'max_distance',
            default_value='0.0',
            description='最大累计行驶距离（m）；0表示禁用距离自动停车。'),
        DeclareLaunchArgument(
            'max_time',
            default_value='180.0',
            description='本次试验最大运行时间（s）。'),
        DeclareLaunchArgument('trial_id', default_value='commissioning'),
        DeclareLaunchArgument('scenario', default_value='normal'),
        DeclareLaunchArgument('repeat_index', default_value='1'),
        DeclareLaunchArgument(
            'output_dir',
            default_value='field_trial_results'),
    ]

    controller = Node(
        package='centerline_extraction',
        executable='cornfield_navigation_node',
        name='pid_controller',
        arguments=['--controller_type', 'pid', '--disable_obstacle_detector'],
        output='screen',
        parameters=[
            params,
            {
                'max_linear_speed': ParameterValue(
                    max_linear_speed, value_type=float),
                'use_quality_aware_control': ParameterValue(
                    quality_aware, value_type=bool),
                'require_quality_metrics': ParameterValue(
                    quality_aware, value_type=bool),
                'max_row_follow_distance': ParameterValue(
                    max_distance, value_type=float),
                'max_row_follow_time': ParameterValue(
                    max_time, value_type=float),
            },
        ],
    )
    logger = Node(
        package='centerline_extraction',
        executable='field_trial_logger.py',
        name='field_trial_logger',
        output='screen',
        parameters=[
            params,
            {
                'trial_id': LaunchConfiguration('trial_id'),
                'experiment_type': 'closed_loop',
                'scenario': LaunchConfiguration('scenario'),
                'perception_method': 'full',
                'repeat_index': ParameterValue(
                    LaunchConfiguration('repeat_index'), value_type=int),
                'nominal_speed': ParameterValue(
                    max_linear_speed, value_type=float),
                'quality_aware': ParameterValue(
                    quality_aware, value_type=bool),
                'output_dir': LaunchConfiguration('output_dir'),
            },
        ],
    )

    return LaunchDescription(declarations + [controller, logger])

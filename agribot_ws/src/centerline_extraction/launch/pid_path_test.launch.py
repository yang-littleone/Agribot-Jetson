import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动固定室内轨迹发布器和 PID 跟踪器。

    底盘、传感器和 /odometry/filtered 应先由 turn_on_agribot 启动。
    不要与 corn_row_detector_projection 同时运行：两者会发布同一个中心线话题。
    """
    params = os.path.join(
        get_package_share_directory('centerline_extraction'),
        'config',
        'pid_path_test.yaml')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.12',
            description='PID 测试最大线速度（m/s）；0.25 m 半径圆弧建议不高于 0.15 m/s。'),
        Node(
            package='centerline_extraction',
            executable='test_path_publisher',
            name='test_path_publisher',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='centerline_extraction',
            executable='cornfield_navigation_node',
            arguments=['--controller_type', 'pid', '--disable_obstacle_detector'],
            output='screen',
            parameters=[
                params,
                {
                    'max_linear_speed': ParameterValue(
                        max_linear_speed, value_type=float),
                },
            ],
        ),
        Node(
            package='centerline_extraction',
            executable='pid_tracking_evaluator.py',
            name='pid_tracking_evaluator',
            output='screen',
            parameters=[params],
        ),
    ])

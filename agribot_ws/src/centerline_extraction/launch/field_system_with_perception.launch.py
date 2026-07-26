import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动实车基础系统和玉米行中心线感知."""
    params = os.path.join(
        get_package_share_directory('centerline_extraction'),
        'config',
        'field_row_follow.yaml')
    agribot_launch = os.path.join(
        get_package_share_directory('turn_on_agribot'),
        'launch',
        'turn_on_agribot.launch.py')

    odometry_mode = LaunchConfiguration('odometry_mode')
    imu_source = LaunchConfiguration('imu_source')
    use_rviz = LaunchConfiguration('use_rviz')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')

    declarations = [
        DeclareLaunchArgument(
            'odometry_mode',
            default_value='fastlio',
            choices=['fastlio', 'legacy'],
            description='基础系统里程计模式。田间默认使用 fastlio。'),
        DeclareLaunchArgument(
            'imu_source',
            default_value='h30',
            choices=['h30', 'onboard'],
            description='融合系统使用的IMU来源。'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='是否启动基础系统RViz。'),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/livox/lidar',
            description='中心线检测器输入点云话题。'),
    ]

    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(agribot_launch),
        launch_arguments={
            'odometry_mode': odometry_mode,
            'imu_source': imu_source,
            'use_rviz': use_rviz,
        }.items(),
    )
    centerline_perception = Node(
        package='centerline_extraction',
        executable='corn_row_detector_projection',
        name='corn_row_detector_projection',
        output='screen',
        parameters=[
            params,
            {'point_cloud_topic': point_cloud_topic},
        ],
    )

    return LaunchDescription(
        declarations + [base_system, centerline_perception])

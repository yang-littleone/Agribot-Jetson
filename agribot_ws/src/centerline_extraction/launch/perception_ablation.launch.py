import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """仅启动感知和记录器，用同一 rosbag 依次运行四种消融配置."""
    params = os.path.join(
        get_package_share_directory('centerline_extraction'),
        'config',
        'field_row_follow.yaml')
    boolean_arguments = (
        ('enable_innermost', 'true'),
        ('enable_robust', 'true'),
        ('use_parallel', 'true'),
        ('enable_temporal', 'true'),
        ('enable_quality', 'true'),
    )
    declarations = [
        DeclareLaunchArgument('trial_id', default_value='ablation_bag_01'),
        DeclareLaunchArgument('scenario', default_value='normal'),
        DeclareLaunchArgument('perception_method', default_value='full'),
        DeclareLaunchArgument('output_dir', default_value='field_trial_results'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ] + [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in boolean_arguments
    ]
    overrides = {
        'enable_innermost_row_extraction': ParameterValue(
            LaunchConfiguration('enable_innermost'), value_type=bool),
        'enable_robust_refinement': ParameterValue(
            LaunchConfiguration('enable_robust'), value_type=bool),
        'use_parallel_row_model': ParameterValue(
            LaunchConfiguration('use_parallel'), value_type=bool),
        'enable_temporal_tracking': ParameterValue(
            LaunchConfiguration('enable_temporal'), value_type=bool),
        'enable_quality_evaluation': ParameterValue(
            LaunchConfiguration('enable_quality'), value_type=bool),
    }
    nodes = [
        Node(
            package='centerline_extraction',
            executable='corn_row_detector_projection',
            name='corn_row_detector_projection',
            output='screen',
            parameters=[
                params,
                overrides,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
        ),
        Node(
            package='centerline_extraction',
            executable='field_trial_logger.py',
            name='field_trial_logger',
            output='screen',
            parameters=[
                params,
                {
                    'trial_id': LaunchConfiguration('trial_id'),
                    'experiment_type': 'perception_ablation',
                    'scenario': LaunchConfiguration('scenario'),
                    'perception_method': LaunchConfiguration('perception_method'),
                    'output_dir': LaunchConfiguration('output_dir'),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
        ),
    ]
    return LaunchDescription(declarations + nodes)

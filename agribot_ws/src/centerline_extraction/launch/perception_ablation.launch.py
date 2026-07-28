import os
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def choose_default_output_dir(workspace_root):
    """优先选择空间充足的已挂载U盘，否则返回工作空间目录."""
    workspace_output = Path(workspace_root) / 'field_trial_results'
    if os.environ.get('FIELD_STORAGE_FORCE_WORKSPACE', '').lower() in {
            '1', 'true', 'yes'}:
        return str(workspace_output)

    try:
        minimum_free_gib = float(
            os.environ.get('FIELD_STORAGE_MIN_FREE_GIB', '1'))
    except ValueError:
        minimum_free_gib = 1.0
    minimum_free_bytes = max(0.0, minimum_free_gib) * 1024 ** 3

    configured_mount = os.environ.get('FIELD_USB_MOUNT', '').strip()
    candidates = []
    if configured_mount:
        candidates.append(Path(configured_mount).expanduser())
    else:
        media_root = Path('/media') / os.environ.get('USER', 'wheeltec')
        if media_root.is_dir():
            candidates.extend(
                path for path in media_root.iterdir() if path.is_dir())

    usable = []
    for mount_path in candidates:
        try:
            if not mount_path.is_mount() or not os.access(mount_path, os.W_OK):
                continue
            free_bytes = shutil.disk_usage(mount_path).free
            if free_bytes >= minimum_free_bytes:
                usable.append((free_bytes, mount_path))
        except OSError:
            continue
    if not usable:
        return str(workspace_output)
    _, selected_mount = max(usable, key=lambda item: item[0])
    return str(
        selected_mount / 'agribot_field_data' / 'field_trial_results')


def generate_launch_description():
    """仅启动感知和记录器，用同一 rosbag 依次运行四种消融配置."""
    package_share = get_package_share_directory('centerline_extraction')
    params = os.path.join(package_share, 'config', 'field_row_follow.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(package_share))))
    workspace_output_dir = os.path.join(
        workspace_root, 'field_trial_results')
    default_output_dir = choose_default_output_dir(workspace_root)
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
        DeclareLaunchArgument(
            'output_dir',
            default_value=default_output_dir,
            description='消融CSV和参数快照目录；默认优先U盘，不可用时回退工作空间。'),
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
                    'fallback_output_dir': workspace_output_dir,
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
        ),
    ]
    storage_message = LogInfo(
        msg=['消融CSV和参数快照目录: ', LaunchConfiguration('output_dir')])
    return LaunchDescription(declarations + [storage_message] + nodes)

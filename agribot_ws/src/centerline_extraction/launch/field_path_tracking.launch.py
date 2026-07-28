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
    """仅启动路径跟踪控制器和本次试验记录器."""
    package_share = get_package_share_directory('centerline_extraction')
    params = os.path.join(package_share, 'config', 'field_row_follow.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(package_share))))
    workspace_output_dir = os.path.join(
        workspace_root, 'field_trial_results')
    default_output_dir = choose_default_output_dir(workspace_root)

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
            default_value=default_output_dir,
            description='CSV和参数快照目录；默认优先U盘，不可用时回退工作空间。'),
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
                'fallback_output_dir': workspace_output_dir,
            },
        ],
    )

    storage_message = LogInfo(
        msg=['田间CSV和参数快照目录: ', LaunchConfiguration('output_dir')])
    return LaunchDescription(
        declarations + [storage_message, controller, logger])

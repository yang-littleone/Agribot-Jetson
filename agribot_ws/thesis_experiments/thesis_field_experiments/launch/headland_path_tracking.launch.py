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
    """Use a writable USB drive when available, matching the paper trials."""
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
    """Start the frozen controller and logger with headland-turn overrides."""
    centerline_share = get_package_share_directory('centerline_extraction')
    params = os.path.join(
        centerline_share, 'config', 'field_row_follow.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(centerline_share))))
    workspace_output_dir = os.path.join(workspace_root, 'field_trial_results')
    default_output_dir = choose_default_output_dir(workspace_root)

    max_linear_speed = LaunchConfiguration('max_linear_speed')
    row_target_distance = LaunchConfiguration('target_distance')
    quality_aware = LaunchConfiguration('quality_aware')
    max_distance = LaunchConfiguration('max_distance')
    max_time = LaunchConfiguration('max_time')
    enable_headland_turn = LaunchConfiguration('enable_headland_turn')
    turn_direction = LaunchConfiguration('headland_turn_direction')
    max_turns = LaunchConfiguration('max_headland_turns')
    row_spacing = LaunchConfiguration('headland_row_spacing')
    turn_speed = LaunchConfiguration('headland_turn_linear_speed')
    turn_target_distance = LaunchConfiguration(
        'headland_turn_target_distance')
    min_follow_distance = LaunchConfiguration(
        'headland_min_follow_distance')
    turn_radius = LaunchConfiguration('headland_turn_radius')
    exit_distance = LaunchConfiguration('headland_exit_distance')
    settle_distance = LaunchConfiguration('headland_settle_distance')
    headland_path_step = LaunchConfiguration('headland_path_step')
    use_continuous_turn = LaunchConfiguration(
        'headland_use_continuous_curvature_turn')
    turn_forward_extension = LaunchConfiguration(
        'headland_turn_forward_extension')
    use_safety_margin_speed = LaunchConfiguration(
        'headland_turn_use_safety_margin_speed')
    turn_goal_tolerance = LaunchConfiguration(
        'headland_turn_goal_tolerance')
    turn_heading_tolerance = LaunchConfiguration(
        'headland_turn_heading_tolerance')
    reacquire_confidence = LaunchConfiguration(
        'headland_reacquire_confidence')
    reacquire_track_confidence = LaunchConfiguration(
        'headland_reacquire_track_confidence')
    reacquire_search_speed = LaunchConfiguration(
        'headland_reacquire_search_speed')
    reacquire_search_angular_speed = LaunchConfiguration(
        'headland_reacquire_search_angular_speed')
    reacquire_max_distance = LaunchConfiguration(
        'headland_reacquire_max_distance')
    reacquire_max_time = LaunchConfiguration(
        'headland_reacquire_max_time')
    reacquire_prediction_length = LaunchConfiguration(
        'headland_reacquire_prediction_length')
    reacquire_frames = LaunchConfiguration('headland_reacquire_frames')

    declarations = [
        # Stable/seldom changed settings are listed first.
        DeclareLaunchArgument(
            'quality_aware', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'max_distance', default_value='0.0'),
        DeclareLaunchArgument(
            'max_time', default_value='600.0'),
        DeclareLaunchArgument(
            'enable_headland_turn', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'max_headland_turns', default_value='1',
            description='Use 1 for one turn and 4 for a multi-row mission.'),
        DeclareLaunchArgument(
            'headland_min_follow_distance', default_value='1.50',
            description='Minimum row travel before a headland turn is allowed.'),
        DeclareLaunchArgument(
            'headland_turn_radius', default_value='0.0',
            description='U-turn radius; 0 uses half of row spacing.'),
        DeclareLaunchArgument(
            'headland_exit_distance', default_value='0.15',
            description='Straight distance after headland detection before turning.'),
        DeclareLaunchArgument(
            'headland_settle_distance', default_value='0.60',
            description='Straight segment after the 180-degree arc.'),
        DeclareLaunchArgument(
            'headland_path_step', default_value='0.05',
            description='Sampling interval of turn/reacquisition paths.'),
        DeclareLaunchArgument(
            'headland_use_continuous_curvature_turn', default_value='false',
            choices=['true', 'false'],
            description='Use a smooth continuous-curvature turn instead of a semicircle.'),
        DeclareLaunchArgument(
            'headland_turn_forward_extension', default_value='0.30',
            description='Forward extension used by continuous-curvature turns only.'),
        DeclareLaunchArgument(
            'headland_turn_use_safety_margin_speed', default_value='true',
            choices=['true', 'false'],
            description='Apply safety-margin speed reduction during U-turn.'),
        DeclareLaunchArgument(
            'headland_turn_goal_tolerance', default_value='0.20',
            description='Position tolerance for completing the U-turn, in metres.'),
        DeclareLaunchArgument(
            'headland_turn_heading_tolerance', default_value='0.35',
            description='Heading tolerance for completing the U-turn, in radians.'),
        DeclareLaunchArgument(
            'headland_reacquire_confidence', default_value='0.70',
            description='Confidence required for stable next-row reacquisition.'),
        DeclareLaunchArgument(
            'headland_reacquire_track_confidence', default_value='0.35',
            description='Confidence required to fuse a detected next-row line.'),
        DeclareLaunchArgument(
            'headland_reacquire_search_speed', default_value='0.15',
            description='Maximum speed during next-row reacquisition.'),
        DeclareLaunchArgument(
            'headland_reacquire_search_angular_speed', default_value='0.0',
            description='Fallback angular speed when no reacquisition path exists.'),
        DeclareLaunchArgument(
            'headland_reacquire_max_distance', default_value='1.20',
            description='Maximum next-row search distance.'),
        DeclareLaunchArgument(
            'headland_reacquire_max_time', default_value='8.0',
            description='Maximum next-row search time.'),
        DeclareLaunchArgument(
            'headland_reacquire_prediction_length', default_value='1.50',
            description='Length of the predicted next-row reference line.'),
        DeclareLaunchArgument(
            'headland_reacquire_frames', default_value='5',
            description='Consecutive confident frames required to resume row following.'),
        DeclareLaunchArgument(
            'output_dir', default_value=default_output_dir),

        # Frequently changed trial settings are deliberately listed last.
        DeclareLaunchArgument(
            'trial_id', default_value='headland_commissioning'),
        DeclareLaunchArgument(
            'scenario', default_value='headland_turn'),
        DeclareLaunchArgument(
            'repeat_index', default_value='1'),
        DeclareLaunchArgument(
            'headland_turn_direction', default_value='left',
            choices=['left', 'right'],
            description='Direction of the first turn; later turns alternate.'),
        DeclareLaunchArgument(
            'headland_row_spacing', default_value='0.60',
            description='Measured adjacent row-center spacing in metres.'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='0.08',
            description='Row-follow speed in m/s; use 0.08 for commissioning.'),
        DeclareLaunchArgument(
            'headland_turn_linear_speed', default_value='0.08',
            description='Turn speed in m/s; increase only after commissioning.'),
        DeclareLaunchArgument(
            'target_distance', default_value='0.20',
            description='Look-ahead distance during row following and reacquisition.'),
        DeclareLaunchArgument(
            'headland_turn_target_distance', default_value='0.20',
            description='Look-ahead distance used only during U_TURN, in metres.'),
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
                'target_distance': ParameterValue(
                    row_target_distance, value_type=float),
                'use_quality_aware_control': ParameterValue(
                    quality_aware, value_type=bool),
                'require_quality_metrics': ParameterValue(
                    quality_aware, value_type=bool),
                'max_row_follow_distance': ParameterValue(
                    max_distance, value_type=float),
                'max_row_follow_time': ParameterValue(
                    max_time, value_type=float),
                'enable_headland_turn': ParameterValue(
                    enable_headland_turn, value_type=bool),
                'headland_turn_direction': turn_direction,
                'max_headland_turns': ParameterValue(
                    max_turns, value_type=int),
                'headland_min_follow_distance': ParameterValue(
                    min_follow_distance, value_type=float),
                'headland_row_spacing': ParameterValue(
                    row_spacing, value_type=float),
                'headland_turn_radius': ParameterValue(
                    turn_radius, value_type=float),
                'headland_exit_distance': ParameterValue(
                    exit_distance, value_type=float),
                'headland_settle_distance': ParameterValue(
                    settle_distance, value_type=float),
                'headland_path_step': ParameterValue(
                    headland_path_step, value_type=float),
                'headland_use_continuous_curvature_turn': ParameterValue(
                    use_continuous_turn, value_type=bool),
                'headland_turn_forward_extension': ParameterValue(
                    turn_forward_extension, value_type=float),
                'headland_turn_use_safety_margin_speed': ParameterValue(
                    use_safety_margin_speed, value_type=bool),
                'headland_turn_goal_tolerance': ParameterValue(
                    turn_goal_tolerance, value_type=float),
                'headland_turn_heading_tolerance': ParameterValue(
                    turn_heading_tolerance, value_type=float),
                'headland_turn_linear_speed': ParameterValue(
                    turn_speed, value_type=float),
                'headland_turn_target_distance': ParameterValue(
                    turn_target_distance, value_type=float),
                'headland_reacquire_confidence': ParameterValue(
                    reacquire_confidence, value_type=float),
                'headland_reacquire_track_confidence': ParameterValue(
                    reacquire_track_confidence, value_type=float),
                'headland_reacquire_search_speed': ParameterValue(
                    reacquire_search_speed, value_type=float),
                'headland_reacquire_search_angular_speed': ParameterValue(
                    reacquire_search_angular_speed, value_type=float),
                'headland_reacquire_max_distance': ParameterValue(
                    reacquire_max_distance, value_type=float),
                'headland_reacquire_max_time': ParameterValue(
                    reacquire_max_time, value_type=float),
                'headland_reacquire_prediction_length': ParameterValue(
                    reacquire_prediction_length, value_type=float),
                'headland_reacquire_frames': ParameterValue(
                    reacquire_frames, value_type=int),
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
                # Keep compatibility with the existing field-bag validator,
                # which requires this identity for every moving trial.
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

    settings = LogInfo(msg=[
        'Thesis trial: id=', LaunchConfiguration('trial_id'),
        ', turn=', enable_headland_turn,
        ', first_direction=', turn_direction,
        ', max_turns=', max_turns,
        ', row_spacing=', row_spacing,
        ', row_speed=', max_linear_speed,
        ', row_lookahead=', row_target_distance,
        ', turn_speed=', turn_speed,
        ', turn_lookahead=', turn_target_distance,
        ', exit=', exit_distance,
        ', settle=', settle_distance,
        ', goal_tolerance=', turn_goal_tolerance,
    ])
    storage = LogInfo(
        msg=['Field CSV and parameter snapshots: ',
             LaunchConfiguration('output_dir')])
    return LaunchDescription(
        declarations + [settings, storage, controller, logger])

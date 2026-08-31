import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Run one synthetic 1 m row, one real U-turn, and a second 1 m row."""
    centerline_share = get_package_share_directory('centerline_extraction')
    params = os.path.join(
        centerline_share, 'config', 'field_row_follow.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(centerline_share))))
    output_default = os.path.join(workspace_root, 'field_trial_results')

    trial_id = LaunchConfiguration('trial_id')
    scenario = LaunchConfiguration('scenario')
    repeat_index = LaunchConfiguration('repeat_index')
    row_speed = LaunchConfiguration('max_linear_speed')
    turn_speed = LaunchConfiguration('headland_turn_linear_speed')
    row_spacing = LaunchConfiguration('headland_row_spacing')
    turn_direction = LaunchConfiguration('headland_turn_direction')
    row_target_distance = LaunchConfiguration('target_distance')
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
    first_length = LaunchConfiguration('first_line_length')
    second_length = LaunchConfiguration('second_line_length')
    start_delay = LaunchConfiguration('start_delay')
    max_time = LaunchConfiguration('max_time')
    output_dir = LaunchConfiguration('output_dir')

    declarations = [
        DeclareLaunchArgument(
            'trial_id', default_value='indoor_turn_left_r1'),
        DeclareLaunchArgument(
            'scenario', default_value='indoor_synthetic_headland'),
        DeclareLaunchArgument(
            'repeat_index', default_value='1'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='0.12',
            description='Straight-row speed in m/s.'),
        DeclareLaunchArgument(
            'headland_turn_linear_speed', default_value='0.10',
            description='U-turn speed in m/s.'),
        DeclareLaunchArgument(
            'target_distance', default_value='0.20',
            description='Straight-row and reacquisition look-ahead in metres.'),
        DeclareLaunchArgument(
            'headland_turn_target_distance', default_value='0.20',
            description='U-turn look-ahead in metres.'),
        DeclareLaunchArgument(
            'headland_min_follow_distance', default_value='0.80',
            description='Minimum row travel before a headland turn is allowed.'),
        DeclareLaunchArgument(
            'headland_row_spacing', default_value='0.60',
            description='Synthetic spacing between the two parallel rows.'),
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
            'headland_turn_direction', default_value='left',
            choices=['left', 'right']),
        DeclareLaunchArgument(
            'headland_reacquire_confidence', default_value='0.70',
            description='Confidence required for stable next-row reacquisition.'),
        DeclareLaunchArgument(
            'headland_reacquire_track_confidence', default_value='0.35',
            description='Confidence required to fuse a detected next-row line.'),
        DeclareLaunchArgument(
            'headland_reacquire_search_speed', default_value='0.12',
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
            'first_line_length', default_value='1.00'),
        DeclareLaunchArgument(
            'second_line_length', default_value='1.00'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Stationary delay before the first path is published.'),
        DeclareLaunchArgument(
            'max_time', default_value='120.0'),
        DeclareLaunchArgument(
            'output_dir', default_value=output_default),
    ]

    simulator = Node(
        package='thesis_field_experiments',
        executable='indoor_headland_simulator.py',
        output='screen',
        parameters=[{
            'first_line_length': ParameterValue(
                first_length, value_type=float),
            'second_line_length': ParameterValue(
                second_length, value_type=float),
            'row_spacing': ParameterValue(
                row_spacing, value_type=float),
            'turn_direction': turn_direction,
            'start_delay': ParameterValue(
                start_delay, value_type=float),
            'path_step': 0.05,
            'publish_period': 0.10,
            'valid_confidence': 0.95,
            'invalid_confidence': 0.10,
            'safety_margin': 0.20,
        }],
    )

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
                    row_speed, value_type=float),
                'target_distance': ParameterValue(
                    row_target_distance, value_type=float),
                'use_quality_aware_control': True,
                'require_quality_metrics': True,
                'max_row_follow_distance': 0.0,
                'max_row_follow_time': ParameterValue(
                    max_time, value_type=float),
                'stop_at_path_end': False,
                'enable_headland_turn': True,
                # The publisher triggers at 1 m; keep this gate below 1 m.
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
                'headland_turn_direction': turn_direction,
                'max_headland_turns': 1,
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
                'trial_id': trial_id,
                'experiment_type': 'closed_loop',
                'scenario': scenario,
                'perception_method': 'indoor_synthetic_rows',
                'repeat_index': ParameterValue(
                    repeat_index, value_type=int),
                'nominal_speed': ParameterValue(
                    row_speed, value_type=float),
                'quality_aware': True,
                'output_dir': output_dir,
                'fallback_output_dir': output_default,
            },
        ],
    )

    settings = LogInfo(msg=[
        'INDOOR TEST: id=', trial_id,
        ', first=', first_length, 'm, second=', second_length,
        'm, spacing=', row_spacing,
        'm, direction=', turn_direction,
        ', row_speed=', row_speed,
        ', turn_speed=', turn_speed,
        ', turn_lookahead=', turn_target_distance,
        ', exit=', exit_distance,
        ', settle=', settle_distance,
        ', goal_tolerance=', turn_goal_tolerance,
        'm. Robot starts after ', start_delay, 's.',
    ])
    return LaunchDescription(
        declarations + [settings, simulator, controller, logger])

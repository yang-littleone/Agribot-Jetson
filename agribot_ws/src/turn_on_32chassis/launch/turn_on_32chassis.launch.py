import os

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def load_bringup_config(config_path):
    """Load launch defaults and node parameters from the package YAML file."""
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    if 'launch' not in config:
        raise RuntimeError(f"Missing 'launch' section in {config_path}")
    return config


def generate_launch_description():
    """Create the complete real-robot bringup graph."""
    package_share_dir = get_package_share_directory('turn_on_32chassis')
    bringup_config = os.path.join(
        package_share_dir, 'config', 'bringup.yaml')
    config = load_bringup_config(bringup_config)
    launch_defaults = config['launch']
    chassis_params = config['turn_on_32chassis_node']['ros__parameters']
    h30_params = config['yesense_pub']['ros__parameters']
    selector_params = config['imu_source_selector']['ros__parameters']
    slip_aware_odom_params = config['slip_aware_odometry']['ros__parameters']
    onboard_imu_tf = launch_defaults['onboard_imu_tf']

    odometry_mode = LaunchConfiguration('odometry_mode')
    imu_source = LaunchConfiguration('imu_source')
    start_h30_driver = LaunchConfiguration('start_h30_driver')
    h30_topic = LaunchConfiguration('h30_topic')
    h30_serial_port = LaunchConfiguration('h30_serial_port')
    h30_baud_rate = LaunchConfiguration('h30_baud_rate')
    h30_frame_id = LaunchConfiguration('h30_frame_id')
    use_rviz = LaunchConfiguration('use_rviz')

    declarations = [
        launch.actions.DeclareLaunchArgument(
            name='odometry_mode',
            default_value=str(launch_defaults['odometry_mode']),
            choices=['fastlio', 'legacy'],
            description=(
                'Odometry source: fastlio for slip-aware LIO, '
                'legacy for wheel speed plus H30'),
        ),
        launch.actions.DeclareLaunchArgument(
            name='imu_source',
            default_value=str(launch_defaults['imu_source']),
            choices=['onboard', 'h30'],
            description='Select the IMU used by the EKF: onboard or h30',
        ),
        launch.actions.DeclareLaunchArgument(
            name='start_h30_driver',
            default_value=str(
                launch_defaults['start_h30_driver']).lower(),
            choices=['true', 'false'],
            description=(
                'Start the Yesense H30 driver when imu_source is h30'),
        ),
        launch.actions.DeclareLaunchArgument(
            name='h30_topic',
            default_value=str(launch_defaults['h30_topic']),
            description='H30 sensor_msgs/Imu topic',
        ),
        launch.actions.DeclareLaunchArgument(
            name='h30_serial_port',
            default_value=str(launch_defaults['h30_serial_port']),
            description='Serial device used by the H30 IMU',
        ),
        launch.actions.DeclareLaunchArgument(
            name='h30_baud_rate',
            default_value=str(launch_defaults['h30_baud_rate']),
            description='Serial baud rate used by the H30 IMU',
        ),
        launch.actions.DeclareLaunchArgument(
            name='h30_frame_id',
            default_value=str(launch_defaults['h30_frame_id']),
            description='TF frame assigned to H30 sensor_msgs/Imu messages',
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value=str(launch_defaults['use_rviz']).lower(),
            choices=['true', 'false'],
            description='Start RViz2 with the Agribot diagnostic view',
        ),
    ]

    onboard_condition = IfCondition(
        PythonExpression(["'", imu_source, "' == 'onboard'"]))
    fastlio_odometry_condition = IfCondition(
        PythonExpression(["'", odometry_mode, "' == 'fastlio'"]))
    legacy_odometry_condition = IfCondition(
        PythonExpression(["'", odometry_mode, "' == 'legacy'"]))
    start_h30_condition = IfCondition(PythonExpression([
        "'", imu_source, "' == 'h30' and '",
        start_h30_driver, "' == 'true'",
    ]))
    selected_imu_topic = PythonExpression([
        "'", h30_topic, "' if '", imu_source,
        "' == 'h30' else '/imu/data'",
    ])

    default_urdf_path = os.path.join(
        package_share_dir, 'urdf', 'urdf', 'agribot.urdf.xacro')
    declarations.append(launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_path,
        description='URDF absolute path',
    ))

    ekf_config = os.path.join(package_share_dir, 'config', 'ekf.yaml')
    legacy_ekf_config = os.path.join(
        package_share_dir, 'config', 'ekf_legacy.yaml')
    imu_config = os.path.join(package_share_dir, 'config', 'imuconfig.yaml')
    fast_lio_config = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'mid360.yaml',
    )
    default_rviz_config_path = os.path.join(
        package_share_dir, 'config', 'rviz2_config_agribot.rviz')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command([
            'xacro ',
            LaunchConfiguration('model'),
        ]),
        value_type=str,
    )

    base_to_gyro = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=[
            '--x', str(onboard_imu_tf['x']),
            '--y', str(onboard_imu_tf['y']),
            '--z', str(onboard_imu_tf['z']),
            '--roll', str(onboard_imu_tf['roll']),
            '--pitch', str(onboard_imu_tf['pitch']),
            '--yaw', str(onboard_imu_tf['yaw']),
            '--frame-id', str(onboard_imu_tf['parent_frame']),
            '--child-frame-id', str(onboard_imu_tf['child_frame']),
        ],
        condition=onboard_condition,
    )

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    drive_node = launch_ros.actions.Node(
        package='turn_on_32chassis',
        executable='turn_on_32chassis_node',
        name='turn_on_32chassis_node',
        output='screen',
        parameters=[chassis_params],
    )
    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config],
        condition=onboard_condition,
    )
    h30_driver_node = launch_ros.actions.Node(
        package='yesense_std_ros2',
        executable='yesense_node_publisher',
        name='yesense_pub',
        output='screen',
        parameters=[h30_params, {
            'serial_port': h30_serial_port,
            'baud_rate': h30_baud_rate,
            'frame_id': h30_frame_id,
            'imu_topic_ros': h30_topic,
        }],
        condition=start_h30_condition,
    )
    imu_source_selector_node = launch_ros.actions.Node(
        package='turn_on_32chassis',
        executable='imu_source_selector_node',
        name='imu_source_selector',
        output='screen',
        parameters=[selector_params],
        remappings=[('imu/input', selected_imu_topic)],
    )
    fast_lio_node = launch_ros.actions.Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[fast_lio_config, {'use_sim_time': False}],
        condition=fastlio_odometry_condition,
    )
    slip_aware_odometry_node = launch_ros.actions.Node(
        package='turn_on_32chassis',
        executable='slip_aware_odometry_node',
        name='slip_aware_odometry',
        output='screen',
        parameters=[slip_aware_odom_params],
        condition=fastlio_odometry_condition,
    )
    fastlio_ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/fused_internal'),
        ],
        condition=fastlio_odometry_condition,
    )
    legacy_ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[legacy_ekf_config],
        condition=legacy_odometry_condition,
    )

    # This is the only MID360 driver include in the complete bringup chain.
    mid360_driver = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('livox_ros_driver2'),
            '/launch_ROS2/msg_MID360_launch.py',
        ])
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return launch.LaunchDescription(declarations + [
        robot_state_publisher,
        joint_state_publisher,
        drive_node,
        imu_filter_node,
        h30_driver_node,
        imu_source_selector_node,
        base_to_gyro,
        fast_lio_node,
        slip_aware_odometry_node,
        fastlio_ekf_node,
        legacy_ekf_node,
        mid360_driver,
        rviz_node,
    ])

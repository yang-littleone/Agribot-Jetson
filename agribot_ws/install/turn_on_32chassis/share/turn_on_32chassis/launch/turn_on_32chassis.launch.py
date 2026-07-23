import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory #通过功能包的名字找到 share 目录
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
import yaml
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def load_bringup_config(config_path):
    """Load launch defaults and node parameters from the package YAML file."""
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    if 'launch' not in config:
        raise RuntimeError(f"Missing 'launch' section in {config_path}")

    return config

def generate_launch_description():
    package_share_dir = get_package_share_directory('turn_on_32chassis')
    bringup_config = os.path.join(package_share_dir, 'config', 'bringup.yaml')
    config = load_bringup_config(bringup_config)
    launch_defaults = config['launch']
    chassis_params = config['turn_on_32chassis_node']['ros__parameters']
    h30_params = config['yesense_pub']['ros__parameters']
    selector_params = config['imu_source_selector']['ros__parameters']
    onboard_imu_tf = launch_defaults['onboard_imu_tf']

    imu_source = LaunchConfiguration('imu_source')
    start_h30_driver = LaunchConfiguration('start_h30_driver')
    h30_topic = LaunchConfiguration('h30_topic')
    h30_serial_port = LaunchConfiguration('h30_serial_port')
    h30_baud_rate = LaunchConfiguration('h30_baud_rate')
    h30_frame_id = LaunchConfiguration('h30_frame_id')

    action_declare_imu_source = launch.actions.DeclareLaunchArgument(
        name='imu_source',
        default_value=str(launch_defaults['imu_source']),
        choices=['onboard', 'h30'],
        description='Select the IMU used by the EKF: onboard or h30',
    )
    action_declare_start_h30_driver = launch.actions.DeclareLaunchArgument(
        name='start_h30_driver',
        default_value=str(launch_defaults['start_h30_driver']).lower(),
        choices=['true', 'false'],
        description='Start the Yesense H30 driver when imu_source is h30',
    )
    action_declare_h30_topic = launch.actions.DeclareLaunchArgument(
        name='h30_topic',
        default_value=str(launch_defaults['h30_topic']),
        description='H30 sensor_msgs/Imu topic',
    )
    action_declare_h30_serial_port = launch.actions.DeclareLaunchArgument(
        name='h30_serial_port',
        default_value=str(launch_defaults['h30_serial_port']),
        description='Serial device used by the H30 IMU',
    )
    action_declare_h30_baud_rate = launch.actions.DeclareLaunchArgument(
        name='h30_baud_rate',
        default_value=str(launch_defaults['h30_baud_rate']),
        description='Serial baud rate used by the H30 IMU',
    )
    action_declare_h30_frame_id = launch.actions.DeclareLaunchArgument(
        name='h30_frame_id',
        default_value=str(launch_defaults['h30_frame_id']),
        description='TF frame assigned to H30 sensor_msgs/Imu messages',
    )

    onboard_condition = IfCondition(
        PythonExpression(["'", imu_source, "' == 'onboard'"])
    )
    start_h30_condition = IfCondition(
        PythonExpression([
            "'", imu_source, "' == 'h30' and '",
            start_h30_driver, "' == 'true'",
        ])
    )
    selected_imu_topic = PythonExpression([
        "'", h30_topic, "' if '", imu_source,
        "' == 'h30' else '/imu/data'",
    ])

    # 获取默认的urdf路径
    default_urdf_path = os.path.join(package_share_dir, 'urdf/urdf','agribot.urdf.xacro')

    ekf_config = os.path.join(package_share_dir, 'config','ekf.yaml')


    # 获取默认的rviz配置文件路径
    default_rviz_config_path = os.path.join(package_share_dir, 'config','rviz2_config_agribot.rviz')
    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_urdf_path),description='URDF的绝对路径'
    )

    imu_config = os.path.join(package_share_dir, 'config','imuconfig.yaml')

    """ 通过文件路径，获取内容，并转化为参数值对象，以供传入 robot_state_publisher """
    # 获取文件内容
    substitutions_command_result = launch.substitutions.Command(command=['xacro ',launch.substitutions.LaunchConfiguration('model')])
    # 将内容转换为参数值对象
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)

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

    # 状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    # 关节状态发布节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    action_drive_node = launch_ros.actions.Node(
        package='turn_on_32chassis',
        executable='turn_on_32chassis_node',
        name='turn_on_32chassis_node',
        output='screen',
        parameters=[chassis_params],
    )

    imu_filter_node =  launch_ros.actions.Node(
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

    action_ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
    )

    # 启动 MID360 激光雷达驱动
    # 使用get_package_share_directory获取包的share目录时，必须在.bashrc中source工作空间的setup.bash文件，以确保环境变量正确设置。
    # when using get_package_share_directory to get the share directory of a package, you must source the setup.bash file of your workspace in your .bashrc to ensure that the environment variables are set correctly.
    action_launch_mid360 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/msg_MID360_launch.py'])
    )
    # RViz 节点
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        # 使用保存的配置文件
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        action_declare_imu_source,
        action_declare_start_h30_driver,
        action_declare_h30_topic,
        action_declare_h30_serial_port,
        action_declare_h30_baud_rate,
        action_declare_h30_frame_id,
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_drive_node,
        imu_filter_node,
        h30_driver_node,
        imu_source_selector_node,
        base_to_gyro,
        action_ekf_node,  # 启用EKF节点
        action_launch_mid360,  # 启动 MID360 激光雷达驱动
        action_rviz_node,
    ])

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory #通过功能包的名字找到 share 目录
import os

def generate_launch_description():
    # 获取默认的urdf路径
    package_share_dir = get_package_share_directory('turn_on_32chassis')
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

    base_to_footprint = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_footprint',
            arguments=['0', '0', '0','0', '0','0','base_link','base_footprint'],
    )

    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_link','imu_link'],
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
        parameters=[
            {'odom_frame_id': 'odom'},
            {'robot_frame_id': 'base_footprint'},
            {'gyro_frame_id': 'imu_link'}
    ]
    )

    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )

    action_ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered','odom')]
    )


    # RViz 节点
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        # 使用保存的配置文件
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_drive_node,
        imu_filter_node,
        base_to_footprint,
        base_to_gyro,
        action_ekf_node,  # 启用EKF节点
        action_rviz_node,
    ])
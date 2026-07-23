import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory #通过功能包的名字找到 share 目录
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_prefix
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    package_share_dir_centerline_extraction_path = get_package_share_directory('centerline_extraction')

    # 启动 MID360 激光雷达驱动
    # 使用get_package_share_directory获取包的share目录时，必须在.bashrc中source工作空间的setup.bash文件，以确保环境变量正确设置。
    # when using get_package_share_directory to get the share directory of a package, you must source the setup.bash file of your workspace in your .bashrc to ensure that the environment variables are set correctly.
    action_launch_mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/msg_MID360_launch.py'])
    )
    
    action_launch_32chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('turn_on_32chassis'), '/launch/turn_on_32chassis.launch.py'])
    )

    action_corndetector = launch_ros.actions.Node(
        package='centerline_extraction',
        executable='corn_row_detector_projection',
        name='corn_row_detector_projection',
    )

    return launch.LaunchDescription([
        action_launch_32chassis,
        # 启动32chassis完成后再启动MID360
        action_launch_mid360,
        # action_corndetector,
    ])
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Include the single chassis-owned real-robot bringup graph."""
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turn_on_32chassis'),
            '/launch/turn_on_32chassis.launch.py',
        ])
    )
    return LaunchDescription([chassis_launch])

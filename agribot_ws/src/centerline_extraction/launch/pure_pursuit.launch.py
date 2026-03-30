from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for Pure Pursuit Controller"""
    
    pure_pursuit_node = Node(
        package='centerline_extraction',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            {'lookahead_distance': 0.5},          # 前视距离（米）
            {'min_lookahead_distance': 0.3},      # 最小前视距离
            {'max_lookahead_distance': 1.0},      # 最大前视距离
            {'linear_velocity': 0.3},             # 期望线速度（m/s）
            {'max_linear_velocity': 0.5},         # 最大线速度
            {'min_linear_velocity': 0.1},         # 最小线速度
            {'max_angular_velocity': 1.0},        # 最大角速度（rad/s）
            {'velocity_gain': 0.5},               # 速度增益
            {'adaptive_lookahead': True},         # 自适应前视距离
            {'debug_mode': True},                 # 调试模式
        ]
    )
    
    return LaunchDescription([
        pure_pursuit_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab02_pkg',          # name of your ROS 2 package
            executable='bump_go_controller',  # name of the entry point from setup.py
            name='bump_and_go',             # optional: node name override
            output='screen',
            parameters=[
                {'linear_speed': 0.2},
                {'angular_speed': 1.5},
                {'control_frequency': 10.0},
                {'min_distance_threshold': 0.6},
            ]
        )
    ])

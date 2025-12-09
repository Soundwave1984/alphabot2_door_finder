from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alphabot2_door_detection',
            executable='door_detector',
            name='door_detector',
            output='screen'
        ),
        Node(
            package='alphabot2_door_detection',
            executable='door_search',
            name='door_search',
            output='screen'
        ),
    ])

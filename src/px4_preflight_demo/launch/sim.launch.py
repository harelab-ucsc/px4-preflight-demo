from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="px4_preflight_demo",
                executable="status_listener.py",
                name="status_listener",
                output="screen",
            ),
        ]
    )

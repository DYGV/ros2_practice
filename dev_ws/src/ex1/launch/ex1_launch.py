from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="ex1", node_executable="talker", output="screen"),
            Node(package="ex1", node_executable="listener", output="screen"),
        ]
    )

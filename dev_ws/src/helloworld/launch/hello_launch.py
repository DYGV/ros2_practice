from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="helloworld", node_executable="talker", output="screen"),
            Node(package="helloworld", node_executable="listener", output="screen"),
        ]
    )

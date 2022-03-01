from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="face_detect",
                node_executable="client",
                output="screen",
                parameters=[{"video_device": -1, "freq": 30}],
            ),
            Node(
                package="face_detect",
                node_executable="server",
                output="screen",
                parameters=[
                    {
                        "cascade_path": "/home/eisuke/dev_ws/src/face_detect/haarcascade_frontalface_default.xml"
                    }
                ],
            ),
        ]
    )

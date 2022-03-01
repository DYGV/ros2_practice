from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ex2",
                node_executable="talker",
                parameters=[{"rand_min": 100, "rand_max": 200, "freq": 2}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                node_name="listener_1",
                parameters=[{"N": 2}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                node_name="listener_2",
                parameters=[{"N": 3}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                node_name="listener_3",
                parameters=[{"N": 4}],
                output="screen",
            ),
        ]
    )

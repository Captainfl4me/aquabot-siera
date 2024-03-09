from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="siera_python",
            executable="main",
            name="siera_convert_node",
        ),
        Node(
            package="aquabot_main",
            executable="aquabot_main",
            name="siera_main_node",
        ),
        Node(
            package="aquabot_view",
            executable="aquabot_view",
            name="siera_view_node",
        )
    ])

"""Start read-only NMPC reference tracking diagnostics in simulation."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("panda_nmpc"), "config", "nmpc.yaml"
    )
    return LaunchDescription([
        Node(
            package="panda_nmpc",
            executable="nmpc_node",
            name="panda_nmpc",
            parameters=[config],
            output="screen",
        )
    ])

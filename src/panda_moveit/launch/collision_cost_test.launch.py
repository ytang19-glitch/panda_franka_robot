"""Read-only geometry experiment; requires an already running Gazebo + MoveIt."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = get_package_share_directory("panda_moveit")
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="world"),
        DeclareLaunchArgument("planning_group", default_value="arm"),
        DeclareLaunchArgument("probe_link", default_value="panda_hand"),
        DeclareLaunchArgument("collision_weight", default_value="100.0"),
        DeclareLaunchArgument("csv_path", default_value="/tmp/panda_collision_cost.csv"),
        Node(package="panda_moveit", executable="collision_cost_test",
             name="collision_cost_test", output="screen",
             parameters=[os.path.join(share, "config", "obstacles.yaml"),
                         {"use_sim_time": True,
                          "base_frame": LaunchConfiguration("base_frame"),
                          "planning_group": LaunchConfiguration("planning_group"),
                          "probe_link": LaunchConfiguration("probe_link"),
                          "collision_weight": ParameterValue(LaunchConfiguration("collision_weight"), value_type=float),
                          "csv_path": LaunchConfiguration("csv_path")}])])

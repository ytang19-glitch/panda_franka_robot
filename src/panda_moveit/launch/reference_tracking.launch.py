"""Start the planning-only client against an already running MoveIt server."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def setup(context):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in ("panda_gazebo", "fr3_gazebo", "fr3_real"):
        raise RuntimeError("Unknown robot profile")
    application = LaunchConfiguration("application").perform(context)
    if application not in ("baseline", "welding", "deburring", "polishing", "clamp_avoidance", "pick_and_place", "surfacing"):
        raise RuntimeError("Unknown application")
    share = get_package_share_directory("panda_moveit")
    return [Node(package="panda_moveit", executable="reference_tracker",
                 name="reference_tracker", output="screen",
                 parameters=[os.path.join(share, "config", "profiles", profile + ".yaml"),
                             os.path.join(share, "config", "cost_weights.yaml"),
                             os.path.join(share, "config", "applications", application + ".yaml"),
                             {"cost_csv_path": ParameterValue(LaunchConfiguration("cost_csv_path"), value_type=str)}]) ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="panda_gazebo"),
        DeclareLaunchArgument("application", default_value="baseline"),
        DeclareLaunchArgument("cost_csv_path", default_value=""),
        OpaqueFunction(function=setup)])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("application", default_value="baseline"),
        DeclareLaunchArgument("cost_csv_path", default_value=""),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("panda_moveit"), "launch", "reference_tracking.launch.py")),
        launch_arguments={"profile": "fr3_real", "application": LaunchConfiguration("application"), "cost_csv_path": LaunchConfiguration("cost_csv_path")}.items())])

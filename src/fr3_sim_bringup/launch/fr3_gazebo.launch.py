"""Simulation-only Franka FR3 Gazebo + optional RViz, without demo motions.

Uses Franka's existing simulation Xacro and gz_ros2_control configuration.
Never starts a motion/effort controller. No real robot connection.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description_nodes(context):
    franka_share = get_package_share_directory("franka_gazebo_bringup")
    xacro_path = os.path.join(franka_share, "urdf", "franka_arm.gazebo.xacro")
    robot_type = context.perform_substitution(LaunchConfiguration("robot_type"))
    if robot_type != "fr3":
        raise RuntimeError("This bringup is restricted to the seven-joint fr3")
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "robot_type": "fr3",
            "hand": "false",
            "ee_id": "franka_hand",
            "gazebo_effort": "true",
        },
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": doc.toxml(), "use_sim_time": True}
            ],
        )
    ]


def generate_launch_description():
    frank_desc = get_package_share_directory("franka_description")
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    rviz_config = os.path.join(frank_desc, "rviz", "visualize_franka.rviz")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )
    # Avoid loading the official joint impedance / cyclic-motion example.
    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type", default_value="fr3",
            description="Only fr3 is supported in this simulation"
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Launch RViz alongside Gazebo"
        ),
        DeclareLaunchArgument(
            "gz_args", default_value="-r empty.sdf",
            description="Arguments forwarded to Gazebo Sim"
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.dirname(frank_desc)
        ),
        gazebo,
        OpaqueFunction(function=_robot_description_nodes),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_config, "-f", "world"],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        spawn,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[broadcaster])
        ),
    ])

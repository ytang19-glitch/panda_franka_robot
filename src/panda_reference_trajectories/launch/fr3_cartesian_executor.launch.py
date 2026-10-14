import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def read_yaml(package, relative_path):
    filename = os.path.join(get_package_share_directory(package), relative_path)
    with open(filename, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    hand = LaunchConfiguration('load_gripper')
    ee_id = LaunchConfiguration('ee_id')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    path_topic = LaunchConfiguration('path_topic')
    execute = LaunchConfiguration('execute')


    urdf_xacro = os.path.join(
        get_package_share_directory('franka_bringup'),
        'urdf', 'franka_arm.urdf.xacro')
    srdf_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro')

    robot_description = {
        'robot_description': ParameterValue(Command([
            FindExecutable(name='xacro'), ' ', urdf_xacro,
            ' hand:=', hand,
            ' robot_type:=fr3',
            ' robot_ip:=', robot_ip,
            ' ee_id:=', ee_id,
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
        ]), value_type=str)
    }
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(Command([
            FindExecutable(name='xacro'), ' ', srdf_xacro,
            ' hand:=', hand,
            ' ee_id:=', ee_id,
        ]), value_type=str)
    }
    kinematics_config = {
        'robot_description_kinematics': read_yaml(
            'franka_fr3_moveit_config', 'config/kinematics.yaml')
    }
    joint_limits_config = {
        'robot_description_planning': read_yaml(
            'franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml')
    }

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0',
                              description='Use the same robot_ip as the running FR3 MoveIt launch.'),
        DeclareLaunchArgument('load_gripper', default_value='true'),
        DeclareLaunchArgument('ee_id', default_value='franka_hand'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),
        DeclareLaunchArgument('path_topic', default_value='/fr3_reference/cartesian_path'),
        DeclareLaunchArgument('execute',default_value='false',description='Execute the planned trajectories through MoveIt.'),
        
        Node(
            package='panda_reference_trajectories',
            executable='cartesian_path_executor',
            name='cartesian_path_executor',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
                {
                    'path_topic': path_topic,
                    'planning_group': 'fr3_arm',
                    'execute': ParameterValue(execute, value_type=bool),
                    'move_to_start': True,
                    'keep_current_orientation': False,
                    'avoid_collisions': True,
                    'velocity_scaling': 0.1,
                    'acceleration_scaling': 0.1,
                    'eef_step': 0.005,
                    'minimum_fraction': 0.99,
                },
            ],
        ),
    ])

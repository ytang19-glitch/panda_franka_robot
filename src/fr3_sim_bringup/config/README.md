# Simulation configuration

The first bringup uses the official `franka_gazebo_bringup` Gazebo hardware/controller YAML
embedded in the Franka Gazebo Xacro. This directory is reserved for future
simulation-only controller overlays.

Do not replace the official controller-manager configuration with a partial YAML.
The custom `fr3_nmpc_controller` is currently disabled at activation and cannot
command Gazebo. The bringup loads only `joint_state_broadcaster`.

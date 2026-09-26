# panda_nmpc

ROS 2 Panda reference-tracking scaffold. It reads measured joint positions and
a `trajectory_msgs/msg/JointTrajectory` reference, validates the reference,
and logs the position error. **It does not solve NMPC or send robot commands.**

## Layout

- `panda_nmpc/nmpc_node.py`: ROS subscriptions and read-only diagnostics.
- `panda_nmpc/reference_trajectory.py`: joint mapping and interpolation.
- `panda_nmpc/safety.py`: measured-state validation and error metric.
- `panda_nmpc/robot_model.py`: boundary for validated robot dynamics.
- `panda_nmpc/optimizer.py`: boundary for the future constrained solver.
- `config/nmpc.yaml`: ROS parameters.
- `launch/nmpc_sim.launch.py`: diagnostics launch file.

Build with `colcon build --packages-select panda_nmpc`, source the workspace,
then run `ros2 launch panda_nmpc nmpc_sim.launch.py`. Publish a MoveIt
`RobotTrajectory.joint_trajectory` to
`/panda_nmpc/reference_trajectory` to inspect tracking error. A trajectory
is a *reference* here; this node does not execute it. For a future controller,
implement dynamics, constraints, solver failure handling, and a separately
validated command interface, then test in simulation before hardware.

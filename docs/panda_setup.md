1. Create the package
Run:
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd src

ros2 pkg create panda_reference_trajectories \
--build-type ament_cmake \
--license Apache-2.0 \
--dependencies \
rclcpp \
geometry_msgs \
nav_msgs \
trajectory_msgs \
moveit_msgs \
moveit_ros_planning_interface \
tf2_geometry_msgs

Create the remaining directories:
cd ~/panda_franka_robot/src/panda_reference_trajectories

mkdir -p \
config \
launch \
include/panda_reference_trajectories \
src/generators \
test

Create the initial files:
touch README.md

touch config/industrial_profiles.yaml

touch launch/reference_experiment.launch.py

touch include/panda_reference_trajectories/path_generator.hpp
touch include/panda_reference_trajectories/moveit_trajectory_builder.hpp

touch src/reference_generator_node.cpp
touch src/moveit_trajectory_builder.cpp

touch src/generators/point_to_point.cpp
touch src/generators/linear_path.cpp
touch src/generators/arc_path.cpp
touch src/generators/spline_path.cpp
touch src/generators/raster_path.cpp
touch src/generators/approach_retract.cpp

touch test/test_path_generators.cpp

2. New repository structure
Your project should become:
panda_franka_robot/
├── experiment_data/                       # Generated experiment results; do not put in src
│   ├── baseline/
│   └── mpc/
├── src/
│   ├── panda_description/
│   │   ├── urdf/
│   │   ├── meshes/
│   │   ├── models/
│   │   ├── world/
│   │   ├── config/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── panda_controller/
│   │   ├── config/
│   │   ├── test_panda_controller/
│   │   └── launch/
│   │
│   ├── panda_moveit/
│   │   ├── config/
│   │   ├── launch/
│   │   └── rviz/
│   │
│   ├── panda_vision/
│   │   └── panda_vision/
│   │       └── color_detector.py
│   │
│   ├── panda_commander/
│   │   └── src/
│   │       └── panda_commander.cpp
│   │
│   ├── panda_reference_trajectories/      # New industrial reference package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── config/
│   │   │   └── industrial_profiles.yaml
│   │   ├── launch/
│   │   │   └── reference_experiment.launch.py
│   │   ├── include/
│   │   │   └── panda_reference_trajectories/
│   │   │       ├── path_generator.hpp
│   │   │       └── moveit_trajectory_builder.hpp
│   │   ├── src/
│   │   │   ├── reference_generator_node.cpp
│   │   │   ├── moveit_trajectory_builder.cpp
│   │   │   └── generators/
│   │   │       ├── point_to_point.cpp
│   │   │       ├── linear_path.cpp
│   │   │       ├── arc_path.cpp
│   │   │       ├── spline_path.cpp
│   │   │       ├── raster_path.cpp
│   │   │       └── approach_retract.cpp
│   │   └── test/
│   │       └── test_path_generators.cpp
│   │
│   ├── panda_nmpc/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── panda_nmpc/
│   │   │   ├── __init__.py
│   │   │   ├── bridge.py
│   │   │   ├── nmpc_node.py
│   │   │   ├── reference_trajectory.py
│   │   │   ├── safety.py
│   │   │   ├── optimizer.py
│   │   │   └── robot_model.py
│   │   ├── resource/
│   │   ├── test/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── panda_bringup/
│       └── launch/
│           ├── pick_and_place.launch.xml
│           └── industrial_reference_test.launch.py  # Add later
│
├── README.md
└── .gitignore

3. Purpose of each new file
File	Responsibility
path_generator.hpp	Common interface used by every path generator
linear_path.cpp	General line between any two Cartesian poses
arc_path.cpp	Circular segment in a selectable plane
spline_path.cpp	Smooth path through arbitrary control poses
raster_path.cpp	Back-and-forth surface coverage
point_to_point.cpp	Pick-and-place and machine-tending motion
approach_retract.cpp	Approach, dwell and retract sequence
moveit_trajectory_builder.cpp	Cartesian waypoints → collision-checked JointTrajectory
reference_generator_node.cpp	Select profile, generate path, publish or execute it
industrial_profiles.yaml	Welding, cleaning and assembly profiles
reference_experiment.launch.py	Starts one trajectory experiment

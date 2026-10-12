# Franka Jazzy launch failure caused by mixed workspace environments

## Symptoms

A Franka launch attempted to load a Xacro file from the older workspace:

```text
FileNotFoundError: [Errno 2] No such file or directory:
'/home/yujietang/franka_ros2_ws/install/franka_bringup/share/franka_bringup/urdf/franka_arm.urdf.xacro'

xacro.XacroException: No such file or directory:
 /home/yujietang/franka_ros2_ws/install/franka_bringup/share/franka_bringup/urdf/franka_arm.urdf.xacro

AttributeError: module 'xml' has no attribute 'parsers'
```

The intended workspace was `~/franka_ros2_ws_jazzy`, but the failing path pointed to `~/franka_ros2_ws`.

## Cause and observed resolution

In this session, the launch failure was resolved by clearing inherited workspace environment variables and sourcing only ROS 2 Jazzy and the intended Franka workspace.

Sourcing several ROS workspaces in one shell can leave package lookup paths pointing to an unintended installation. Check the resolved package paths before changing launch files or rebuilding packages.

The missing Xacro file is the first failure in this traceback. The `xml.parsers` exception occurs while Xacro is handling that failure; this traceback alone does not establish a separate Python installation problem.

## Solution: start a shell without inherited workspace paths

Run this in the terminal used for the Franka launch:

```bash
env -u AMENT_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH \
    -u LD_LIBRARY_PATH \
    bash --noprofile --norc
```

Then, inside the new shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/local_setup.bash

export ROS_DOMAIN_ID=20
```

Using `local_setup.bash` applies this workspace without automatically sourcing its recorded underlay chain. The ROS Jazzy underlay is sourced explicitly first.

This command clears the five listed variables; it does not erase every inherited environment variable.

## Verify package resolution

```bash
for pkg in franka_bringup franka_description franka_fr3_moveit_config; do
    echo "===== $pkg ====="
    ros2 pkg prefix --share "$pkg"
done
```

For packages built in the intended workspace, the returned paths should point into `~/franka_ros2_ws_jazzy/install`. None should unexpectedly resolve to the older `~/franka_ros2_ws/install`.

After checking these paths, rerun the Franka launch command that previously failed from the same shell.

## Prevent recurrence

- Use separate terminals for Panda simulation and the real Franka stack.
- In the Franka terminal, source ROS Jazzy and the intended Franka workspace.
- Inspect shell startup files for automatic sourcing of the older workspace:

```bash
rg -n 'franka_ros2_ws|panda_franka_robot|setup\.bash|local_setup\.bash' \
    ~/.bashrc ~/.profile ~/.bash_profile 2>/dev/null
```

Review matching lines and remove or comment out unwanted automatic workspace sourcing.

`ROS_DOMAIN_ID=20` controls ROS communication discovery. It does not select a workspace or repair package lookup paths.

## If the error persists

1. Check that the three package paths above resolve to the intended installations.
2. Check whether the Xacro file referenced by the launch actually exists.
3. Inspect the launch file for a hard-coded old workspace path or a Xacro filename that does not match the installed package version.
4. If `xml.parsers` still fails after the missing-file issue is fixed, investigate that exception separately.

**Reported outcome:** the user confirmed that the launch worked after cleaning the mixed workspace environment. These commands were documented from that report; they were not executed on the robot host during this documentation update.

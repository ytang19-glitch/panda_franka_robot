# FR3 Simulation — Git Pull, Build, and Run Guide

This guide covers the **existing** `fr3_sim_bringup` package in
`panda_franka_robot/src/fr3_sim_bringup` (ROS 2 Jazzy).
The package starts **Gazebo FR3 + RViz + joint_state_broadcaster**, but **does
not activate NMPC or any arm motion controller**.

## 1. Update your local repository from GitHub

Open a terminal on Ubuntu:

```bash
cd ~/panda_franka_robot
git status
git branch --show-current
```

**Important:** If `git status` shows uncommitted changes, save them before
pulling. For example, to keep local work safely without committing:

```bash
git stash push -u -m "before fr3_sim_bringup update"
```

Only do this if you intend to stash **all** current local changes. Restore
later with `git stash list` followed by `git stash pop`, after checking
for conflicts. Alternatively, commit your changes to your current branch.

The `fr3_sim_bringup` package is now merged into **main**; **do not** switch
to the old `feat/fr3-sim-bringup` branch just to obtain it.

```bash
git fetch origin
git switch main
git pull --ff-only origin main
```

If `git switch main` refuses because files would be overwritten, **do not
force it**. Save or commit those changes first. If `git pull --ff-only`
reports divergent branches, inspect the history with
`git log --oneline --graph --decorate -10` before deciding how to merge.

Verify the package and its launch file:

```bash
ls src/fr3_sim_bringup
ls src/fr3_sim_bringup/launch
git log -1 --oneline
```

Expected files include `package.xml`, `CMakeLists.txt`, `README.md`,
and `launch/fr3_gazebo.launch.py`.

### Later daily updates

For a clean working tree already on `main`:

```bash
cd ~/panda_franka_robot
git pull --ff-only origin main
```

`git fetch` updates remote-tracking refs; `git pull` fetches and integrates
changes into the **currently checked-out branch**. `--ff-only` refuses
a non-fast-forward merge rather than silently creating a merge commit.

## 2. Source ROS and build only the simulation package

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash

cd ~/panda_franka_robot
colcon build --packages-select fr3_sim_bringup --symlink-install
source install/setup.bash
```

Verify that ROS 2 can find the package:

```bash
ros2 pkg prefix fr3_sim_bringup
ros2 launch fr3_sim_bringup fr3_gazebo.launch.py --show-args
```

If `ros2 pkg prefix` reports that the package is not found, check the build
output and re-source `install/setup.bash` in **that terminal**. A new
terminal must source all three environments again.

If you modify any files under `src/fr3_sim_bringup`, rebuild the package
and re-source the workspace. `--symlink-install` simplifies working on
installed non-compiled resources but is not a substitute for rebuilding after
changing package setup/install rules.

## 3. Start simulation in Terminal 1

Stop any existing Panda/FR3 Gazebo instances first to avoid duplicate
`/controller_manager` nodes.

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch fr3_sim_bringup fr3_gazebo.launch.py rviz:=true
```

This starts official FR3 Gazebo, RViz, the clock bridge and
`joint_state_broadcaster`. Unlike the official joint impedance example,
it does **not** request periodic J4/J5 motion.

**Warning:** No arm effort controller is activated, so the model can sag
under gravity. This is a bringup/integration check, not gravity hold or
closed-loop NMPC. Pause Gazebo physics if you need to inspect the initial pose.

For Gazebo without RViz, add `rviz:=false`.

## 4. Inspect in Terminal 2

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 node list
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

Expected:
- `joint_state_broadcaster` is active.
- `fr3_joint1` through `fr3_joint7` have valid position and velocity
  state interfaces.
- `fr3_joint*/effort` command interfaces are available and **unclaimed**.
- No `joint_impedance_example_controller` or custom NMPC controller is
  active.

If the controller manager service times out, inspect Terminal 1 for Gazebo
errors and check that the simulation clock exists:

```bash
ros2 service list | grep controller_manager
timeout 5 ros2 topic echo /clock --once
```

A listed service is not proof that it is responsive. Confirm that a single
Gazebo instance is running with the same `ROS_DOMAIN_ID`.

## 5. Optional: send changes back to GitHub

Only if **you actually edited files** and want to upload them:

```bash
cd ~/panda_franka_robot
git status
git add src/fr3_sim_bringup
git commit -m "docs: update FR3 simulation setup"
git push origin main
```

`git pull` downloads/integrates changes **from** GitHub;
`git push` uploads local commits **to** GitHub. If your remote has
branch protection or `git push` is rejected, use a feature branch
and a pull request instead; do not force-push `main`.

## 6. Next development milestone

The NMPC backend has passed basic offline Hold/Move model tests, but the
existing `fr3_nmpc_controller` intentionally rejects activation and writes
no actuator efforts. Bringing up Gazebo is **not** NMPC execution. Next,
implement a simulation-only, bounded and fail-safe effort-control adapter,
check the gravity/torque conventions and model match, then test Gazebo
gravity hold before attempting MoveIt joint-trajectory tracking.

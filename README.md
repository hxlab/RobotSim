# RobotSim Design Artifacts
**Authors:** Steven Yang, Aidan Kirwin, Dawang Zhang
**Platform:** ROS 2 Humble | Ubuntu 22.04 | Franka FR3

---

## Overview

This workspace implements a Cartesian impedance controller for the Franka FR3 robot arm in simulation. 
- The controller maps position errors to end-effector motion using a virtual first-order spring-damper system. 
- Joint torques are computed via a Jacobian pseudoinverse and sent to a modified low-level torque controller.

## Dependencies

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Ubuntu | 22.04 |
| franka_ros2 | https://github.com/frankaemika/franka_ros2 |
| franka_gazebo | Included with franka_ros2 |
| Eigen3 | System package (`libeigen3-dev`) |

## Opening and Building the Workspace

### 1. Open in VS Code Dev Container

When prompted by VS Code, click **"Reopen in Container"** (or use `Ctrl+Shift+P` → *Dev Containers: Reopen in Container*).

The dev container entrypoint script (`franka_entrypoint.sh`) will automatically clone all Franka dependencies into `/ros2_ws` using `vcs import`.

### 2. Build the workspace

```bash
cd /ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **Note:** You must run `source install/setup.bash` in every new terminal, or add it to your `~/.bashrc`.

---

## Running the Simulation

### 1. Launch the full simulation

```bash
ros2 launch controller controller.launch.py
```

This brings up:
- Ignition Gazebo with the custom `robot_table.sdf` world
- Franka FR3 robot spawned in simulation
- `joint_state_publisher`
- The controller node

### 2. Run the haptic device node

```bash
ros2 launch haptic_device haptic_device.launch.py
```

Or test the gripper

```bash
ros2 topic pub /haptic/buttons std_msgs/msg/Int32 "{data: 1}" --once
```

### 3. Run the GUI node
```bash
ros2 launch gui gui.launch.py
```
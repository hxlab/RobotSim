# ME780 Assignment 2 — Admittance Control (Track A)
**Author:** Steven Yang  
**Course:** ME780 – Collaborative Robotics
**Platform:** ROS 2 Humble | Ubuntu 22.04 | Franka FR3

---

## Overview

THE CODE I DEVELOPED IS THE IN:  franka_ros2/src/MY_ADMITTANCE_CONTROL 
THE CONTROLLER CODE IS IN: franka_ros2/src/MY_ADMITTANCE_CONTROL/src/admittance_control_node.cpp

I also modified the file joint_velocity_example_controller.cpp to read ros2 topics to command the joint velocities

This workspace implements a Cartesian admittance controller for the Franka FR3 robot arm in simulation. The controller maps external forces and position errors to compliant end-effector motion using a virtual second-order spring-damper system. Joint velocities are computed via a damped least-squares Jacobian pseudoinverse and sent to a modified low-level velocity controller.

## Dependencies

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Ubuntu | 22.04 |
| franka_ros2 | https://github.com/frankaemika/franka_ros2 |
| franka_gazebo | Included with franka_ros2 |
| MoveIt 2 | For Humble (`ros-humble-moveit`) |
| Eigen3 | System package (`libeigen3-dev`) |
| ign_ros2_control | For velocity controller interface |

The admittance node interfaces with the **Franka joint velocity example controller** (`franka_example_controllers`), which has been modified to subscribe to a ROS 2 topic for desired joint velocities. The robot is spawned and simulated using **franka_gazebo** (Ignition Gazebo / Gazebo Fortress).

---

## Repository Structure

The workspace is based on the `franka_ros2` repository. The custom admittance controller lives alongside the Franka packages under `src/`:

```
franka_ros2/                          ← Open this folder in VS Code
├── .devcontainer/                    ← Dev container config (Docker)
├── src/
│   ├── franka_example_controllers/   ← MODIFIED: joint_velocity_example_controller
│   │   └── src/
│   │       └── joint_velocity_example_controller.cpp   ← Reads from /target_joint_velocities topic
│   ├── franka_gazebo/                ← Franka Gazebo simulation
│   ├── franka_description/           ← Robot URDF/xacro
│   ├── ... (other franka_ros2 packages)
│   └── MY_ADMITTANCE_CONTROL/        ← MY CODE, CUSTOM PACKAGE (package name: admittance_controller)
│       ├── src/
│       │   └── admittance_control_node.cpp
│       ├── scripts/
│       │   ├── record_run.sh
│       │   ├── extract_bag.py
│       │   └── plot_results.py
│       ├── launch/
│       │   └── sim_admittance.launch.py
│       ├── worlds/
│       │   └── robot_table.sdf
│       ├── data_{SCENARIO}/
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
```

---

## Modification to `franka_example_controllers`

The `JointVelocityExampleController` in `franka_example_controllers` has been modified to accept desired joint velocities from an external ROS 2 topic instead of computing them internally. It subscribes to:

```
/target_joint_velocities   [std_msgs/Float64MultiArray]
```

The admittance controller node publishes to this topic to command the robot.

---

## Workflow: Opening and Building the Workspace

### 1. Open in VS Code Dev Container

```bash
cd ~/franka_ros2          # or wherever the repo is cloned
code .                    # open in VS Code
```

When prompted by VS Code, click **"Reopen in Container"** (or use `Ctrl+Shift+P` → *Dev Containers: Reopen in Container*).

The dev container entrypoint script (`franka_entrypoint.sh`) will automatically clone all Franka dependencies into `/ros2_ws/src` using `vcs import`.

### 2. Build the workspace

```bash
cd /ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

To build only specific packages:

```bash
colcon build --packages-select admittance_controller franka_example_controllers
```

> **Note:** You must run `source install/setup.bash` in every new terminal, or add it to your `~/.bashrc`.

---

## Running the Simulation

### 1. Launch the full simulation

```bash
ros2 launch admittance_controller sim_admittance.launch.py
```

This brings up:
- Ignition Gazebo with the custom `robot_table.sdf` world
- Franka FR3 robot spawned in simulation
- `robot_state_publisher`
- `joint_state_publisher`
- The admittance controller node

### 2. Send a goal position

```bash
ros2 topic pub --once /set_goal_pose geometry_msgs/msg/Pose "{
  position: {x: 0.4, y: 0.0, z: 0.2},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}"
```

### 3. Inject an external force (disturbance scenario)

```bash
ros2 topic pub --once /set_force geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 40.0, z: 0.0}"
```

### 4. Manually command joint velocities (for testing the velocity controller directly)

```bash
ros2 topic pub --once /target_joint_velocities std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]"
```

---

## Node: `admittance_control_node`

### Control Architecture

```
F_ext (sensor or manual)
        |
        v
[ Admittance Law ]  <-- position error (x_actual - x_goal)
  a = (F - D*v - K*e) / M
        |
        v
[ Euler Integration ]
  v += a*dt,  x += v*dt
        |
        v
[ Jacobian Pseudoinverse ]  (damped least squares)
  q_dot = J† * x_dot
        |
        v
[ /target_joint_velocities topic ]
        |
        v
[ JointVelocityExampleController ]  (ign_ros2_control)
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `mass` | `10.0` | Virtual inertia M_d (kg) |
| `damping` | `140.0` | Virtual damping D_d (Ns/m) — overridden by `d_calculated` |
| `stiffness` | `500.0` | Virtual stiffness K_d (N/m) |

**Note:** These parameters are modified by changing the values in the source code

### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/set_goal_pose` | `geometry_msgs/Pose` | Desired EE goal position |
| `/set_force` | `geometry_msgs/Vector3` | Manually injected external force |
| `/joint_states` | `sensor_msgs/JointState` | Joint position feedback |
| `/joint_wrenches/fr3_joint7` | `geometry_msgs/WrenchStamped` | Raw wrench at EE |
| `haptic/pose` | `geometry_msgs/Pose` | Optional haptic device input |

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/target_joint_velocities` | `std_msgs/Float64MultiArray` | Joint velocity commands to velocity controller |
| `ee_wrench` | `geometry_msgs/Vector3` | Effective force used in control loop |
| `ee_pos_error` | `geometry_msgs/Vector3` | Cartesian position error |
| `/ik_scaled_pose` | `geometry_msgs/PoseStamped` | Target EE pose for RViz |

---

## Data Recording and Plotting

### Record a bag (auto-deletes previous bag, waits 2s before sending goal)

```bash
bash scripts/record_run.sh
```

Edit `BAG_NAME` and `DELAY` at the top of the script as needed.

### Extract data to .npz

Edit `RUN_LABEL` in `extract_bag.py` to match the experiment (e.g. `"500StiffnessCritDamping10Mass"`), then:

```bash
python3 scripts/extract_bag.py
```

Data is saved to `data/<RUN_LABEL>_ee_wrench.npz` and `data/<RUN_LABEL>_ee_pos_error.npz`. Time is anchored to when the goal pose was published, ensuring consistent alignment across runs.

### Generate figures

Edit the `RUNS` and `RUN_LABELS` lists in `plot_results.py`, then:

```bash
python3 scripts/plot_results.py
```

Figures are saved as PDF to `figures/`.

---

## Key Implementation Notes

- **Damping ratio control:** `d_calculated = 2 * zeta * sqrt(M * K)` is used instead of the raw `damping` parameter, preserving the damping ratio when M or K are varied.
- **Virtual contact:** A virtual plane at z = 0.05 m with stiffness 5000 N/m simulates a rigid table surface. Contact force is proportional to penetration depth (normal direction only).
- **Force sensor:** Gravity compensation and velocity-dependent damping correction are applied to the raw wrench before it enters the control loop. The `readForce` flag toggles between sensor-driven and manually-injected force modes.
- **Singularity handling:** Damped least-squares pseudoinverse with λ = 0.05 prevents large joint velocity commands near kinematic singularities.

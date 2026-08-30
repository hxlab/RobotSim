# Shared Control Project Design Artifacts
**Authors:** Aidan Kirwin, Dawang Zhang, Steven Yang | 
**Platform:** ROS 2 Humble,  Ubuntu 22.04, Franka Panda Arm

---

## Overview

This workspace implements a Cartesian impedance controller for the Franka Panda robot arm in simulation or on the real robot. 
- The controller maps position errors to end-effector motion using a virtual first-order spring-damper system. 
- Joint torques are computed via a Jacobian pseudoinverse and sent to a modified low-level torque controller.

The workspace also includes:
- Grasp pose generation using UOIS unseen object segmentation and NVIDIA Contact-GraspNet.
- A GUI to view the RGB camera data, depth map, object segmentation, grasp candidates, and modify certain parameters at runtime.

## Opening and Building the Workspace

1. Install Docker Engine (https://docs.docker.com/engine/install/ubuntu/)
2. Clone this repository
```bash
git clone https://github.com/hxlab/RobotSim.git
cd RobotSim
git submodule update --init --recursive
```
3. Install the NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- NOTE: If using the BFG, this is already installed and configured.

### 1. Set up the Docker container(s)

The Docker containers are structured as follows:

```
                              ROS 2
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                     │
          ▼                     ▼                     ▼
┌──────────────────┐   ┌──────────────────┐   ┌───────────────────────┐
│       USER       │   │      ROBOT       │   │ CONTACT_GRASPNET_ROS2 │
│                  │   │                  │   │                       │
│  gui             │   │   controller     │   │    grasp_processor    │
│  haptic_device   │   │  shared_control  │   │                       │
│                  │   │                  │   │                       │
└──────────────────┘   └──────────────────┘   └───────────────────────┘
       │                         │                      │
   /robot_ws                 /user_ws              /cgn_ros2_ws
```

#### Dependencies by Container
| Container | Dependencies |
| --------- | ------------ |
| user      | ROS2 Humble, PyQT5, TouchDriver2022_04_04, OpenHaptics 3.4.0 |
| robot     | ROS2 Humble, ros-humble-ros-gz (Gazebo), ros-humble-librealsense2 (RealSense Cameras), ros-humble-moveit (MoveIt)  |
| contact_graspnet_ros2 | NVIDIA CUDA 12.1, CGN-PyTorch 0.4.3 | 

Then
```bash
cd RobotSim
docker compose build --progress=plain {CONTAINER_NAME}
docker compose up {CONTAINER_NAME}
docker exec -it {CONTAINER_NAME} bash
```

Build each container (robot, user, contact_graspnet) individually using the commands above and swapping {CONTAINER_NAME} for each container's name. Note that there is a `docker-compose.yml` file.

### 2. Build the workspace(s)

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **Note:** You must run `source install/setup.bash` in every new terminal, or add it to your `~/.bashrc`.

You will need to run this in each container.

---

## Running the Workspace

### Contact-GraspNet (NVIDIA computer)

1. Download the UOIS models from [here](https://drive.google.com/uc?export=download&id=1nbYuSjx7kukRPG7i-zq9G6ZBHcS2yhFs)
2. Re-build the workspace
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
3. Launch the workspace
```bash
ros2 launch grasp_processor grasp_processor.launch.py
```

This will load a pre-trained Contact-GraspNet model and start the `grasp_processor` node, which subscribes to:
- `PointCloud2`, `/camera/depth/color/points`
- `Image`, `/camera/depth/image`
- `Image`, `/camera/color/image`

and publishes:
- `Grasps`, which includes:
```
geometry_msgs/Pose[] poses
float32[] scores
int32[] object_ids
```

### Controller (robot system)

```bash
ros2 launch controller controller.launch.py
```

This brings up:
- Ignition Gazebo with the custom `robot_table.sdf` world
- Franka FR3 robot spawned in simulation
- `joint_state_publisher`
- The controller node

### Haptic device (user system)

```bash
ros2 launch haptic_device haptic_device.launch.py
```

Or test the gripper

```bash
ros2 topic pub /haptic/buttons std_msgs/msg/Int32 "{data: 1}" --once
```

### GUI (user system)
```bash
ros2 launch gui gui.launch.py
```

## To do

### Development
1. Make the controller launch file conditionally load the custom_franka_description OR franka_ros2/franka_description depending on whether we are in simulation or using real hardware
2. Test the grasp generation pipeline
5. Set up a hand-eye calibration node

### Other
1. Add subscribers and publishers for each node to this README
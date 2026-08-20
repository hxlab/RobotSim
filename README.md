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

## Dependencies

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Ubuntu | 22.04 |
| franka_ros2 | https://github.com/frankaemika/franka_ros2 |
| franka_gazebo | Included with franka_ros2 |
| Eigen3 | System package (`libeigen3-dev`) |

## Opening and Building the Workspace

1. Install Docker Engine (https://docs.docker.com/engine/install/ubuntu/)
2. Clone this repository
```bash
git clone https://github.com/hxlab/RobotSim.git
cd RobotSim
git submodule update --init --recursive
```
3. Install the NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### 1. Set up the Docker container(s)

The Docker containers are structured as follows:

```
                              ROS 2
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                     │
          ▼                     ▼                     ▼
┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐
│       USER       │   │      ROBOT       │   │ CONTACT_GRASPNET │
│                  │   │                  │   │                  │
│  GUI             │   │   Controller     │   │                  │
│  Haptic Device   │   │   franka_ros2    │   │ Contact-GraspNet │
│                  │   │                  │   │ CUDA / PyTorch   │
│  PyQt5           │   │   Gazebo (sim)   │   │                  │
│  OpenHaptics     │   │                  │   │                  │
└──────────────────┘   └──────────────────┘   └──────────────────┘
       │                         │                      │
   /robot_ws                 /user_ws            /root/graspnet_ws
```

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

You will need to run this in each container.s

---

## Running the Workspace

### Contact-GraspNet (NVIDIA computer)

NOTE: you must [download](https://drive.google.com/drive/folders/1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl) a model, make a `contact_graspnet/contact_graspnet/checkpoints` directory, and place the model (i.e., all contents of the downloaded model folder; recommended `scene_test_2048_hor_sigma_001`) in the directory.

Once both CGN containers (`contact_graspnet_ros` and `contact_graspnet`) are built and running, enter the `contact_graspnet` container and run
```bash
conda run -n contact-graspnet bash compile_pointnet_tfops.sh
```

If any of the tests return an error about GLIBCXX_3.4.29 you might need to run

```bash
conda activate contact-graspnet
conda install -c conda-forge libstdcxx-ng
strings $CONDA_PREFIX/lib/libstdc++.so.6 | grep GLIBCXX_3.4.29
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
1. Make the controller launch file conditionally load the custom_franka_description OR franka_ros2/franka_description depending on whether we are in simulation or using real hardware
2. Re-write the CGN container to be a single ROS2 node which: (a) receives depth and segmentation data, (b) runs the CGN inference, (c) returns the grasps and scores
3. Get the CGN container running on the BFG system
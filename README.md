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

Clone the repository and install docker.

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

## Running the Simulation

### 1. Launch the simulation OR run with the real robot

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

## To do
1. Make the controller launch file conditionally load the custom_franka_description OR franka_ros2/franka_description depending on whether we are in simulation or using real hardware
2. Test contact_graspnet container (and document how this works, note that the container diagram isn't entirely up to date since we are now using a DiD setup for the ROS2 CGN wrapper)

## Testing

Running the CGN containers in Podman (on the BFG computer)

For contact_graspnet_ros2:

```bash
podman build \
  -t contact_graspnet_ros2 \
  -f ./contact_graspnet/Dockerfile \
  --build-arg USER_UID="${USER_UID}" \
  --build-arg USER_GID="${USER_GID}" \
  --build-arg USERNAME=user \
  ./contact_graspnet
```

then:

```bash
podman run -it \
  --name contact_graspnet_ros2 \
  --user root \
  --network host \
  --ipc host \
  --privileged \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  --env ROS_LOCALHOST_ONLY=0 \
  --group-add dialout \
  --cap-add SYS_NICE \
  --cap-add SYS_PTRACE \
  --ulimit rtprio=99:99 \
  --ulimit memlock=102400:102400 \
  -v "$(pwd)/contact_graspnet:/cgn_ros2_ws" \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.docker.xauth:/tmp/.docker.xauth:ro" \
  -v "$(pwd)/limits.conf:/etc/security/limits.conf" \
  contact_graspnet_ros2 \
  /bin/bash
```

For the GPU contact_graspnet container:

```bash
podman build \
  -t contact_graspnet \
  -f ./contact_graspnet/contact_graspnet/Dockerfile \
  --build-arg USER_UID="${USER_UID}" \
  --build-arg USER_GID="${USER_GID}" \
  --build-arg USERNAME=contact_graspnet \
  ./contact_graspnet/contact_graspnet
```

then:

```bash
podman run -it \
  --name contact_graspnet \
  --network host \
  --ipc host \
  --shm-size 32g \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  --env ROS_LOCALHOST_ONLY=0 \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  --device nvidia.com/gpu=all \
  -v "$(pwd)/contact_graspnet/contact_graspnet:/cgn_ws" \
  contact_graspnet \
  bash -lc 'conda run -n contact-graspnet bash compile_pointnet_tfops.sh && exec bash -l'
```
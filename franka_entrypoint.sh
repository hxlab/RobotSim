#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/franka_deps < /ros2_ws/dependency.repos --recursive --skip-existing

exec "$@"
#!/bin/bash

DELAY=2.0
BAG_NAME="my_bag"

# Delete existing bag if it exists
if [ -d "$BAG_NAME" ]; then
    echo "Deleting existing bag..."
    rm -rf "$BAG_NAME"
fi

# Start recording in background
ros2 bag record /ee_wrench /ee_pos_error /set_goal_pose -o $BAG_NAME &
BAG_PID=$!

# Wait until bag folder exists (recorder is ready)
echo "Waiting for bag to initialize..."
until [ -d "${BAG_NAME}" ]; do
    sleep 0.05
done
echo "Bag confirmed recording, waiting ${DELAY}s before sending goal..."

sleep $DELAY

# Send goal
ros2 topic pub --once /set_goal_pose geometry_msgs/msg/Pose "{
  position: {x: 0.4, y: 0.0, z: 0.2},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}"

echo "Goal sent. Press Ctrl+C when motion is complete to stop recording."
wait $BAG_PID
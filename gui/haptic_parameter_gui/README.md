# Haptic Controller Parameter GUI

Standalone prototype GUI for changing haptic teleoperation controller parameters at runtime.

The GUI collects the current values into one parameter object and sends that object through a
selected transport. For local testing, the console transport prints the object. For ROS 2 testing,
the ROS transport publishes the same object to a topic.

The intended system shape is:

```text
3D Systems Touch ROS node -> controller node -> robot/sim, currently Franka Panda
                         ^
                         |
       this GUI -> parameter update bridge
```

This GUI is not a 3D Systems Touch driver and it is not Franka-specific. It is a controller tuning
panel. The current defaults come from Steven's Franka admittance controller, but the payload is meant
to stay usable if the robot target changes later.

## Install

From this folder:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
```

## Run With Console Transport

```bash
make run
```

Click **Apply** to print the full parameter object.

Close the window or click **Quit** to stop the GUI process.

Equivalent direct command:

```bash
python -m haptic_parameter_gui.main --transport console
```

## Run With ROS 2

Source your ROS 2 environment first, then run:

```bash
make run-ros
```

In this mode, clicking **Apply** publishes a JSON string to:

```text
/haptic_controller/parameters
```

The ROS message wrapper looks like:

```json
{
  "schema_version": 1,
  "source": "haptic_parameter_gui",
  "parameters": {
    "controller": {},
    "workspace_limits": {},
    "shared_control": {}
  }
}
```

For quick ROS-side inspection:

```bash
ros2 topic echo /haptic_controller/parameters
```

## Payload Shape

```json
{
  "controller": {
    "zeta": 1.0,
    "mass": 10.0,
    "damping": 140.0,
    "haptic_damping": 15.0,
    "k_orient": 50.0
  },
  "workspace_limits": {
    "x_min": 0.2,
    "x_max": 0.6,
    "y_min": -0.25,
    "y_max": 0.25,
    "z_min": 0.1,
    "z_max": 0.7
  },
  "shared_control": {
    "enabled": false,
    "grasp_assist": 0.0,
    "target_pursuance_assist": 0.0,
    "desired_trajectory_track": 0.0
  }
}
```

## ROS 2 Integration Later

The GUI app itself does not import ROS. ROS-specific code lives in `haptic_parameter_gui/ros_transport.py`,
which keeps the UI testable without a ROS 2 install.

The ROS transport publishes the full parameter object using `std_msgs/String`. That is useful for
early testing because it does not require defining a custom ROS message. For a production
controller, the next step should be either:

- a typed custom message for these parameter groups, or
- a subscriber that unpacks this JSON and applies updates to the controller's ROS parameters.

## Linux Notes

The GUI uses PySide6 and the Qt Fusion style, so the same code should run on Ubuntu/Linux desktops.
If running inside Docker or a remote Linux machine, the main extra setup is display forwarding
through X11 or Wayland. Running directly on the lab Linux desktop should not need extra GUI setup.

## ROS 2 Package Notes

This folder is structured as a normal Python package with `setup.py`, `setup.cfg`, and
`console_scripts`. It can also be built by ROS 2 as an `ament_python` package if this `gui/` folder is
included in the workspace source tree.

After a ROS 2 build:

```bash
colcon build --packages-select haptic_parameter_gui
source install/setup.bash
ros2 run haptic_parameter_gui haptic-parameter-gui --transport ros2
```

The `ros2 run` command is available after a ROS 2 `colcon build`. During local editable installs,
use `make run` or `python -m haptic_parameter_gui.main` because ROS-style `setup.cfg` installs
console scripts under `lib/haptic_parameter_gui` for ROS discovery.

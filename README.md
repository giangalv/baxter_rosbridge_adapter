# Baxter ROS2 Rosbridge Adapter

ROS2 package for controlling the Baxter Research Robot via rosbridge WebSocket connection.

This package provides a bridge between ROS1 (running on Baxter) and ROS2 (running on external workstation), enabling:
- Joint state monitoring and visualization in RViz2
- Gripper control
- Head pan control
- Arm joint position commands
- Interactive CLI for manual robot control

---

## Features

- **Joint State Bridge**: subscribes to ROS1 `/robot/joint_states` and republishes merged states to ROS2 `/joint_states`
- **RViz2 Visualization**: live robot model display with real-time joint updates
- **Gripper Control**: open/close/position commands for both grippers
- **Head Control**: pan head left/right
- **Arm Control**: position control for left and right arms with predefined poses
- **Interactive CLI**: terminal-based interface for all robot functions

---

## Dependencies

### ROS2 packages
- `rclpy`
- `sensor_msgs`
- `robot_state_publisher`
- `tf2_ros`
- `rviz2`

### Python packages
- `roslibpy` (install with `pip install roslibpy`)

### External dependencies
- **Baxter robot** running ROS1 with rosbridge_server active
- **baxter_common_ros2** (URDF, meshes, messages)

  This package was originally developed by Centrale Nantes Robotics:  
  https://github.com/CentraleNantesRobotics/baxter_common_ros2

---

## Installation

### 1. Create workspace and clone dependencies

```bash
mkdir -p ~/baxter_ros2_ws/src
cd ~/baxter_ros2_ws/src

# Clone baxter_common_ros2
git clone https://github.com/CentraleNantesRobotics/baxter_common_ros2.git

# Clone this package
git clone https://github.com/YOUR_USERNAME/baxter_rosbridge_adapter.git
```

### 2. Install roslibpy
```bash
pip3 install roslibpy
```

### 3.Build workspace
```bash
cd ~/baxter_ros2_ws
colcon build
source install/setup.bash
```

## Configuration

### Baxter rosbridge connection
By default, the package connects to:

- Host: `130.251.13.31`
- Port: `9090`

You can override these parameters at runtime:
```bash
ros2 run baxter_rosbridge_adapter baxter_cli \
  --ros-args -p baxter_host:=YOUR_IP -p baxter_port:=9090
```

## Gripper ID
Default gripper ID is `65538`. If your Baxter uses a different gripper type, you can override:
```bash
ros2 run baxter_rosbridge_adapter baxter_cli \
  --ros-args -p gripper_id:=YOUR_ID
```

## Auto-calibration
By default, grippers are not auto-calibrated at startup.

To enable auto-calibration:
```bash
ros2 run baxter_rosbridge_adapter baxter_cli \
  --ros-args -p auto_calibrate:=true
```

## Usage
### Launch Rviz2 visualization
```bash
ros2 launch baxter_rosbridge_adapter baxter_visualize.launch.py
```

This starts:

- Joint state bridge (ROS1 → ROS2)
- `robot_state_publisher`
- RViz2 with Baxter model

The robot model will update in real-time with live joint states.

## Interactive CLI
```bash
ros2 run baxter_rosbridge_adapter baxter_cli
```

Available commands:

Robot control
- enable — enable the robot
- disable — disable the robot
- status — print full robot status

Grippers
- lo / lc — left gripper open / close
- ro / rc — right gripper open / close
- lp <0-100> — left gripper position (0=closed, 100=open)
- rp <0-100> — right gripper position
- open_both / close_both — both grippers
- cal — calibrate grippers

Head
- head <angle_rad> — head pan in radians (-1.39 to 1.39)
- head_deg <angle_deg> — head pan in degrees
- head_center — center the head

Left arm
- lhome — move to home pose
- lneutral — move to neutral pose
- ltuck — move to tucked pose
- ltable_high — move above table (approach)
- ltable_low — move to table manipulation pose
- lpose j0 j1 j2 j3 j4 j5 j6 — move to specific joint positions
- ljoint <idx> <value> — set single joint (idx 0-6)
- lcurrent — print current joint positions

Right arm
- rhome / rneutral / rtuck
- rtable_high / rtable_low
- rpose j0 j1 j2 j3 j4 j5 j6
- rjoint <idx> <value>
- rcurrent

Both arms
- bhome / bneutral / btuck
- btable_high / btable_low

Both-arm commands execute simultaneously.

Other
- help — show all commands
- q / quit / exit — quit CLI


## Example workflow
### Visualization only
```bash
ros2 launch baxter_rosbridge_adapter baxter_visualize.launch.py
```

### Interactive control
```bash
# Terminal 1: launch visualization
ros2 launch baxter_rosbridge_adapter baxter_visualize.launch.py

# Terminal 2: launch CLI
ros2 run baxter_rosbridge_adapter baxter_cli
```

## Costumizing table poses
The default table poses are:
```python
LEFT_TABLE_HIGH  = [-0.55, -0.75,  0.10,  1.45,  0.00,  0.85,  0.00]
LEFT_TABLE_LOW   = [-0.55, -0.95,  0.10,  1.70,  0.00,  1.05,  0.00]
RIGHT_TABLE_HIGH = [ 0.55, -0.75, -0.10,  1.45,  0.00,  0.85,  0.00]
RIGHT_TABLE_LOW  = [ 0.55, -0.95, -0.10,  1.70,  0.00,  1.05,  0.00]
```

## Architecture
### Nodes
`joint_state_bridge`
- Subscribes to ROS1 `/robot/joint_states` via rosbridge
- Merges fragmented joint state messages
- Publishes complete joint states to ROS2 `/joint_states`

`baxter_cli`

Connects to Baxter rosbridge (WebSocket)

Publishes commands to ROS1 topics:
* `/robot/set_super_enable`
* `/robot/end_effector/{left|right}_gripper/command`
* `/robot/head/command_head_pan`
* `/robot/limb/{left|right}/joint_command`

Subscribes to state topics:
* `/robot/state`
* `/robot/joint_states`
* `/robot/end_effector/{left|right}_gripper/state`
- Provides interactive terminal interface

### Launch files
`baxter_visualize.launch.py`
- Starts `joint_state_bridge`
- Starts `robot_state_publisher` with Baxter URDF
- Publishes static `world → base` transform
- Launches RViz2 with Baxter model


## Acknowledgments
- baxter_common_ros2: Centrale Nantes Robotics
https://github.com/CentraleNantesRobotics/baxter_common_ros2
- Baxter Research Robot: Rethink Robotics

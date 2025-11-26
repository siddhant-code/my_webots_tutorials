## Overview
This project implements a **Roomba‑like walker behavior** for the **e-puck robot** in ROS 2 Humble.  
The controller uses a **State Design Pattern** with two states:

- **ForwardState:**  drives straight until an obstacle is detected  
- **RotateState:** rotates the robot until the path ahead becomes clear (Alternating between clockwise and anticlockwise) 

---

## Assumptions & Dependencies
- **ROS 2 Humble** installed
- Python 3.10 or higher
- `colcon` for building workspaces
- Webots: Install using ```sudo apt-get install ros-humble-webots-ros2 ```. For installation instructions: [Webots Installation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html)
- Webots epuck : Installtion guide : [Installation guide](https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-E-puck-for-ROS2-Beginners). make sure ```ros2 launch webots_ros2_epuck robot_launch.py``` runs before proceeding.
- Packages for controller:
  - `geometry_msgs`
  - `sensor_msgs`
  - `rclcpp`
  - `rosbag2` (for recording/playback)
---

## Building the Package

Create a directory name ros2_ws. Clone this repository inside the directory and build the package.

```bash
# 1. Create directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this package
git clone https://github.com/siddhant-code/my_webots_tutorials.git

# 3. Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# 4. Source the workspace
source install/setup.bash
```

---

## Running the Simulation

Launch normally (no rosbag recording):

```bash
ros2 launch walker walker_launch.py record_bag:=false
```

Start with rosbag recording enabled:

```bash
ros2 launch walker walker_launch.py record_bag:=true output_dir:=results
```

The `walker` node will:
- Subscribe to `/scan`
- Publish velocity commands to `/cmd_vel`
- Switch states depending on laser scan distance
- Note: The obstacle and clearance threshold should be changed as per requirement.

---

## Recording Bag Files

### Disable recording
```bash
ros2 launch walker walker_launch.py record_bag:=false
```

### Enable recording and save to a custom directory
```bash
ros2 launch walker walker_launch.py record_bag:=true output_dir:=results
```

## Inspecting Bag Files

List topics recorded:

```bash
ros2 bag info results
```


## Playing Back Bag Files

Replay a recorded session:

```bash
ros2 bag play results
```

View contents of specific topics in another terminal after sourcing:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /scan
```

---

You may run visualization tools like rviz in parallel during playback.

---

## Node Summary

| Component | Description |
|----------|-------------|
| **Node name** | `walker` |
| **Subscriptions** | `/scan` |
| **Publications** | `/cmd_vel` |
| **States** | `ForwardState`, `RotateState` |
| **Logic** | Roomba-like obstacle avoidance |

---
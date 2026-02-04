<img width="1735" height="950" alt="image" src="https://github.com/user-attachments/assets/35011c26-d1f5-423f-a6b5-9a50e8360caf" />

# 🤖 Double Pendulum Arm Robot - ROS 2 Jazzy

A multi-segment robot (Double Pendulum Arm) visualization package for ROS 2 Jazzy. This package demonstrates URDF/Xacro robot description, RViz visualization, and joint control using the Joint State Publisher GUI.

![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 📋 Overview

This package creates a double pendulum arm robot with the following structure:

```
World (Fixed Base)
    └── Sphere 1 (Red) - Base joint
            └── Cylinder 1 (Blue) - First arm segment
                    └── Sphere 2 (Green) - Middle joint
                            └── Cylinder 2 (White) - Second arm segment
```

**Features:**
- ✅ URDF/Xacro robot description
- ✅ Interactive joint control via GUI sliders
- ✅ Real-time visualization in RViz2
- ✅ Two continuous revolving joints

## 🚀 Quick Start

### Prerequisites
- Ubuntu 24.04 LTS
- ROS 2 Jazzy installed
- Required packages: `joint-state-publisher-gui`, `xacro`, `robot-state-publisher`

### Installation

```bash
# Clone/navigate to workspace
cd ~/ros2_ws

# Install dependencies
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-robot-state-publisher

# Build the package
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Launch

```bash
ros2 launch my_robot_description display.launch.py
```

## 📁 Package Structure

```
my_robot_description/
├── launch/
│   └── display.launch.py      # Main launch file
├── rviz/
│   └── view_robot.rviz        # RViz configuration
├── urdf/
│   └── robot.urdf.xacro       # Robot description
├── resource/
│   └── my_robot_description   # Package marker
├── my_robot_description/
│   └── __init__.py            # Python package
├── package.xml                # Package manifest
├── setup.cfg                  # Setup configuration
├── setup.py                   # Build configuration
├── README.md                  # This file
└── TUTORIAL.md                # Step-by-step guide
```

## 🎮 Usage

1. **Launch the visualization** - Opens RViz and Joint State Publisher GUI
2. **Control joints** - Use the sliders in the popup window to move `joint_1` and `joint_2`
3. **View robot movement** - Watch the pendulum arms swing in RViz

## 🔧 ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint angles |
| `/robot_description` | `std_msgs/String` | URDF XML string |
| `/tf` | `tf2_msgs/TFMessage` | Coordinate transforms |

## 🔍 Inspecting the System

```bash
# List running nodes
ros2 node list

# List active topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states
```

<img width="859" height="746" alt="image" src="https://github.com/user-attachments/assets/928aba6e-d97f-439d-a3f4-11efa81171fc" />


## 📖 Learn More

See [TUTORIAL.md](TUTORIAL.md) for a comprehensive step-by-step guide on how this package was created.

## 📄 License

MIT License - Feel free to use and modify!

## 👤 Author

Syed Nazmus Sakib


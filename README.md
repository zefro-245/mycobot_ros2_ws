# MyCobot ROS 2 Workspace

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-green.svg)
![MoveIt 2](https://img.shields.io/badge/MoveIt-2.0-purple.svg)

This repository contains a complete ROS 2 workspace for the **MyCobot robotic arm**, featuring simulation, visualization, and motion planning capabilities.

## 📋 Overview

The workspace provides:
- **URDF robot description** for the MyCobot arm
- **Gazebo simulation** with physics integration
- **RViz visualization** tools
- **MoveIt 2** motion planning configuration
- **ros2_control** integration for controller management

## 📁 Repository Structure

```
src/
└── mycobot_ros2/
    ├── arm_gazebo/              # Gazebo-specific configurations
    ├── arm_moveit_config/       # MoveIt configuration
    ├── mycobot_bringup/         # Launch and system bringup
    ├── mycobot_description/     # URDF and meshes
    ├── mycobot_gazebo/          # Gazebo simulation
    └── mycobot_moveit_config/   # MoveIt setup for MyCobot
```

## ⚙️ Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **colcon** build system
- **Gazebo** (Gazebo Classic recommended)
- **MoveIt 2**

## 🛠️ Installation & Build

### 1. Clone and Build

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws

# Clone the repository (adjust the path as needed)
git clone <repository-url> src/mycobot_ros2

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2. Install Dependencies

```bash
# Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install additional packages if needed
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs
```

## 🚀 Usage

### Launch Gazebo Simulation

```bash
ros2 launch arm_gazebo spawn_arm_gazebo.launch.py
```

### Launch RViz with MoveIt

```bash
ros2 arm_moveit_config moveit_gazebo.launch.py
```


## 🎥 Demonstration

Watch the MyCobot in action:

[[MyCobot ROS 2 Demo](https://drive.google.com/file/d/1uVbSuryEDiWIEvxfniRph-N-8lTyWHxC/view?usp=sharing)]

## 🎯 Features

- **Complete Simulation**: Full physics simulation in Gazebo
- **Motion Planning**: Advanced planning with MoveIt 2
- **Controller Integration**: ros2_control for hardware abstraction
- **Visualization**: RViz markers and robot visualization
- **Configuration**: Easy-to-modify configurations for different setups

## 🔧 Technical Details

- **ROS 2 Version**: Humble Hawksbill
- **Control Framework**: ros2_control
- **Simulation**: Gazebo Classic with ROS 2 plugins
- **Planning Framework**: MoveIt 2
- **Tested Platform**: Ubuntu 22.04 LTS

## 📝 Notes

- The workspace uses `ros2_control` for controller management
- Built and tested on ROS 2 Humble
- `build/`, `install/`, and `log/` directories are git-ignored
- Make sure to source the workspace in every new terminal


## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 👤 Author

**Subhradeep Pal**

---

*This workspace is designed for educational and research purposes with the MyCobot robotic arm.*

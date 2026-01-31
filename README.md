# mycobot ROS 2 Workspace

This repository contains a ROS 2 workspace for the **myCobot robotic arm**, including:

- URDF and robot description
- Gazebo simulation
- RViz visualization
- MoveIt 2 configuration
- ros2_control integration

The setup allows the robot to be visualized in RViz and simulated in Gazebo with controllers loaded.

---

## 📦 Repository Structure

src/
└── mycobot_ros2/
├── arm_gazebo
├── arm_moveit_config
├── mycobot_bringup
├── mycobot_description
├── mycobot_gazebo
└── mycobot_moveit_config

yaml
Copy code

---

## ⚙️ Requirements

- Ubuntu 22.04
- ROS 2 Humble
- colcon
- Gazebo
- MoveIt 2

---

## 🛠️ Build Instructions

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
▶️ Running the Simulation
1️⃣ Launch Gazebo
bash
Copy code
ros2 launch mycobot_gazebo mycobot_gazebo.launch.py
2️⃣ Launch RViz + MoveIt
bash
Copy code
ros2 launch mycobot_moveit_config moveit_rviz.launch.py
(Use the appropriate launch filenames if yours differ.)

🎥 Demo Video
Below is a demonstration of the robot running in Gazebo and RViz:

https://drive.google.com/file/d/1uVbSuryEDiWIEvxfniRph-N-8lTyWHxC/view?usp=sharing
🧠 Notes
This workspace uses ros2_control for controller management.

Built and tested on ROS 2 Humble.

build/, install/, and log/ directories are intentionally ignored.

📌 Author
Subhradeep Pal
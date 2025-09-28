# ARL ROS2 Project - Turtle Drawing

This project allows a turtle in `turtlesim` to draw various shapes using ROS 2 nodes.

---

## Requirements

- ROS 2 Humble
- Python 3
- Standard ROS 2 packages: `turtlesim`, `rclpy`, `geometry_msgs`, `std_msgs`

---

## Quick Start (Copy-Paste in One Terminal)

```bash
# 1️⃣ Source ROS 2
source /opt/ros/humble/setup.bash

# 2️⃣ Clone the repository
cd ~/ros2_ws/src
git clone <[the repo](https://github.com/martinmelad1/ros2_turtle_project.git)> arl_ros2_project

# 3️⃣ Make Python scripts executable
cd ~/ros2_ws/src/arl_ros2_project/arl_ros2_project
chmod +x turtle_commander.py
chmod +x shape_node.py

# 4️⃣ Build the package
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 5️⃣ Launch turtlesim and your nodes all in one terminal
ros2 launch arl_ros2_project full_turtle_launch.py

## Demo Video
Watch the project in action:[ros2 project](https://drive.google.com/file/d/1hv2d7dFDgnGvRXv5FSyQF1h0pHz3tiPu/view?usp=drive_link)

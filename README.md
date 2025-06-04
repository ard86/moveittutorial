# MoveIt Tutorial for Real Robot Integration

🚀 A detailed guide to set up, execute, and program in moveit 


## 📚 Overview

Summary of this tutorial
- Setting up a robot arm in ROS2 and MoveIt
- Planning and executing motion in RViz
- Integrating simulation with Gazebo
- Controlling real hardware via `ros2_control`
- Creating complex tasks like:
  1. pick-and-place
  2. 


## 🛠 Software Needed

- ROS2 (Humble)
- MoveIt 

## 🧰 Installation: Pre-reqs

### Prepare the VM

```bash
sudo apt install ros-humble-moveit
git clone https://github.com/yourusername/moveit-tutorial-gbx.git
cd moveit-tutorial-gbx
colcon build
source install/setup.bash

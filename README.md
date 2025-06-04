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

### 1. Prepare the VM (Skip if Linux User)
- [] Rent a GPU on vast.ai: pick a Linux vm that is within your budget.
- [] Connect using SSH from your computer: copy your public key, which you can get in your terminal, into your VM's ~/.ssh/authorized_keys in the VM's konsole. 

### 2. Install ROS2 Humble OR Rolling
Depending on your needs, pick either ROS2 Humble or Rolling. 

| Platform       | Ubuntu Version | ROS 2 Version to Use     | Notes                          |
|----------------|----------------|---------------------------|--------------------------------|
| macOS (via VM) | 22.04           | 🟢 Humble   | Most stable and supported      |
| Linux native   | 22.04           | 🟢 Humble    | Ideal setup                    |
| Linux native   | 24.04           | 🟡 Rolling   | Only option right now for 24.04 |


<u> [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)</u>


```bash
sudo apt install ros-humble-moveit
git clone https://github.com/yourusername/moveit-tutorial-gbx.git
cd moveit-tutorial-gbx
colcon build
source install/setup.bash

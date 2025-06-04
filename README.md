# MoveIt Tutorial for Real Robot Integration

🚀 Annie's detailed guide to set up, execute, and program in Moveit 


## Overview

Summary of this tutorial
- Setting up a robot arm in ROS2 and MoveIt
- Planning and executing motion in RViz
- Integrating simulation with Gazebo
- Controlling real hardware via `ros2_control`
- Creating complex tasks like:
  1. pick-and-place
  2. 


## Software Needed

- ROS2 (Humble)
- Colon
- MoveIt 

## Installation: Pre-reqs

### 1. Prepare the VM (Skip if Linux User)
- [ ] Rent a GPU on vast.ai: pick a Linux vm that is within your budget.
- [ ] Connect using SSH from your computer: copy your public key, which you can get in your terminal, into your VM's ~/.ssh/authorized_keys in the VM's konsole. 

### 2. Install ROS2 Humble OR Rolling
<u> [Install ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)</u> on Ubuntu 22.04, or <u>[Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html)<u> on Ubuntu 24.04, depending on your ROS development goals.

| Platform       | Ubuntu Version | ROS 2 Version to Use     | Advantage                          |
|----------------|----------------|---------------------------|--------------------------------|
| MacOS (via VM) | 22.04           | 🟢 Humble   | Ideal for stable, long-term robotics setup     |
| Linux    | 22.04           | 🟢 Humble    |                     |
| Linux    | 24.04           | 🟡 Rolling   | Only option right now for 24.04, ideal for testing new ROS features; currently experimental, stablility not guaranteed |

install system dependencies & update packages:
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```

### 3. Install and set up Colcon, the official build tool to customize and extend Moveit for your robot in ROS2.

#### 1. Install: 
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
sudo apt install python3-vcstool
```

#### 2. Create a colcon workspace:
```bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
```

#### 3. Download tutorials to prepare for custom coding in Moveit & Build workspace
```bash
git clone https://github.com/ros2/examples src/examples -b humble
source install/setup.bash
```

🛑 You should only proceed to the next step when you get a pop up saying all colcon packages have been successfully installed. If there is even one that failed to build, follow this 👉 [Colcon Build Troubleshooting Guide](./Troubleshooting_colcon_build.md).

## Installation: Lets MOVEIT 
### 1. Download the source code & install dependencies
```bash
sudo apt remove ros-$ROS_DISTRO-moveit*
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
### 2. Configure colcon workspace
```bash
cd ~/ws_moveit
colcon build --symlink-install
```
### 3. Source colcon workspace
```bash
source ~/ws_moveit/install/setup.bash
```
### 4. Launch RViz (quick motion planning demo) or Gazebo (full sim test) for MoveIt:
RViz: 
```bash
ros2 launch moveit2_tutorials demo.launch.py
```
Gazebo: 
```bash
ros2 launch moveit2_tutorials gazebo.launch.py
```
### Congrats on setting things up! Now we will move onto how to use moveit like a prooo.

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

### 3. Install Moveit!

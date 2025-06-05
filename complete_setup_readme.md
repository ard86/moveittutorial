# Panda Arm Bolt Pick-and-Place System

A ROS2-based system for detecting and picking up bolts using a Franka Panda robot arm with YOLO object detection and MoveIt2 motion planning.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Package Setup](#package-setup)
- [Building the System](#building-the-system)
- [Running the System](#running-the-system)
- [Customizing Drop-off Location](#customizing-drop-off-location)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage: dropping off in constraint spaces](#advanced-usage)

## Prereqs

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- X11 forwarding enabled (if using SSH)

## Installation

### Step 1: Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 2: Install Required Dependencies

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Install MoveIt2 and other ROS packages
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs

# Install Python packages
pip3 install ultralytics opencv-python PyQt5 scipy numpy
```

### Step 3: Install YOLO ROS2 Messages

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git
cd ~/ros2_ws
colcon build --packages-select yolov8_msgs
source install/setup.bash
```

## Package Setup

### Step 1: Create Workspace and Copy Files

```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Copy your files from Downloads
cp -r ~/Downloads/moveit_obb/* ~/ros2_ws/src/
```

### Step 2: Organize Package Structure

```bash
cd ~/ros2_ws/src

# Create package directories
mkdir -p panda_moveit_config/launch
mkdir -p panda_moveit_config/scripts
mkdir -p panda_moveit_config/config
mkdir -p yolov8_obb/launch
mkdir -p yolov8_obb/scripts

# Move files to correct locations
mv ~/Downloads/moveit_obb/bolt_selector.py panda_moveit_config/scripts/
mv ~/Downloads/moveit_obb/arm_control_from_UI.py panda_moveit_config/scripts/
mv ~/Downloads/moveit_obb/bolt_selector_window.py panda_moveit_config/scripts/
mv ~/Downloads/moveit_obb/moveit_gazebo_obb.py panda_moveit_config/launch/
mv ~/Downloads/moveit_obb/moveit_rviz.launch.py panda_moveit_config/launch/

mv ~/Downloads/moveit_obb/yolov8_obb_publisher.py yolov8_obb/scripts/
mv ~/Downloads/moveit_obb/yolov8_obb_subscriber.py yolov8_obb/scripts/
mv ~/Downloads/moveit_obb/yolov8_obb.launch.py yolov8_obb/launch/

# Make scripts executable
chmod +x panda_moveit_config/scripts/*.py
chmod +x yolov8_obb/scripts/*.py
```

### Step 3: Create Package Configuration Files

#### panda_moveit_config/package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>panda_moveit_config</name>
  <version>0.0.1</version>
  <description>Panda MoveIt Config with Bolt Pick Place</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <exec_depend>moveit_ros_planning_interface</exec_depend>
  <exec_depend>moveit_ros_perception</exec_depend>
  <exec_depend>moveit_servo</exec_depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <exec_depend>moveit_planners</exec_depend>
  <exec_depend>moveit_simple_controller_manager</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>
  <exec_depend>yolov8_msgs</exec_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### panda_moveit_config/CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.5)
project(panda_moveit_config)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/bolt_selector.py
  scripts/arm_control_from_UI.py
  scripts/bolt_selector_window.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files if they exist
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

ament_package()
```

#### yolov8_obb/package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>yolov8_obb</name>
  <version>0.0.1</version>
  <description>YOLOv8 OBB Detection</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>yolov8_msgs</exec_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### yolov8_obb/CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.5)
project(yolov8_obb)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/yolov8_obb_publisher.py
  scripts/yolov8_obb_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
```

### Step 4: Set Up YOLO Model

```bash
# Create directory for models
mkdir -p ~/ros2_ws/src/yolov8_obb/scripts/

# If you don't have a trained model, create a placeholder
python3 << EOF
from ultralytics import YOLO

# Download a pretrained model as base
model = YOLO('yolov8n.pt')

# Save it (you'll need to train this on your bolt dataset)
model.save('${HOME}/ros2_ws/src/yolov8_obb/scripts/best.pt')
print("Model saved. Note: You need to train this on your bolt dataset!")
EOF
```

### Step 5: Download Panda Description

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ros-planning/moveit_resources.git
cd ~/ros2_ws
colcon build --packages-select moveit_resources_panda_description
source install/setup.bash
```

## Building the System

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the System

### Test Launch

Create a test launch file:

```bash
cat > ~/ros2_ws/src/panda_moveit_config/launch/test_system.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch MoveIt
        ExecuteProcess(
            cmd=['ros2', 'launch', 'moveit_resources_panda_moveit_config', 'demo.launch.py'],
            shell=False,
        ),
        
        # Launch YOLO detector
        Node(
            package='yolov8_obb',
            executable='yolov8_obb_publisher.py',
            name='yolo_detector',
            output='screen',
        ),
        
        # Launch bolt selector GUI
        Node(
            package='panda_moveit_config',
            executable='bolt_selector.py',
            name='bolt_selector',
            output='screen',
        ),
        
        # Launch arm controller
        Node(
            package='panda_moveit_config',
            executable='arm_control_from_UI.py',
            name='arm_controller',
            output='screen',
        ),
    ])
EOF
```

Run the system:

```bash
# Terminal 1: Launch everything
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panda_moveit_config test_system.launch.py

# Terminal 2: 
ros2 topic echo /target_point
```

## Customizing Drop-off Location: 

The current system drops bolts at a fixed location (0.3, -0.3). Here are several ways to customize this:

### Method 1: Edit Drop-off Position in Code

Edit `arm_control_from_UI.py` and find this line:
```python
self.move_to(0.3, -0.3, self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)
```

Change to your desired position:
```python
# Example: Drop at different location
self.move_to(0.4, -0.2, self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)
```

### Method 2: Add ROS Parameters

```python
#!/usr/bin/env python3
# save as: configurable_arm_control.py

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.core.kinematic_constraints import construct_joint_constraint

class ConfigurableController(Node):
    def __init__(self):
        super().__init__('configurable_controller')
        
        # Declare parameters for drop-off location
        self.declare_parameter('drop_x', 0.3)
        self.declare_parameter('drop_y', -0.3)
        self.declare_parameter('drop_z', 0.3)
        
        # Get parameters
        self.drop_x = self.get_parameter('drop_x').value
        self.drop_y = self.get_parameter('drop_y').value
        self.drop_z = self.get_parameter('drop_z').value
        
        # Rest of initialization...
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
            self.listener_callback,
            10)
        
        # MoveIt setup (same as original)
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        self.panda = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.panda_hand = self.panda.get_planning_component("hand")
        self.logger = get_logger("moveit_py.pose_goal")
        
        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)
        
        self.height = 0.18
        self.pick_height = 0.126
        self.carrying_height = 0.3
        self.init_angle = -0.3825
        
        self.get_logger().info(f"Drop-off location: ({self.drop_x}, {self.drop_y}, {self.drop_z})")
    
    def listener_callback(self, data):
        # Pick sequence (same as original)
        self.move_to(data.data[0], data.data[1], self.height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)
        self.gripper_action("open")
        self.move_to(data.data[0], data.data[1], self.pick_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)
        self.gripper_action("close")
        self.move_to(data.data[0], data.data[1], self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)
        
        # Move to configurable drop location
        self.move_to(self.drop_x, self.drop_y, self.drop_z, 1.0, self.init_angle + data.data[2], 0.0, 0.0)
        self.gripper_action("open")
    
    # Include move_to and gripper_action methods from original
```

Then launch with custom parameters:
```bash
ros2 run panda_moveit_config configurable_arm_control.py --ros-args -p drop_x:=0.4 -p drop_y:=-0.2 -p drop_z:=0.35
```

### Method 3: Interactive Drop Location Selection

Set drop location dynamically:

```python
# Add to controller class:
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

class InteractiveController(Node):
    def __init__(self):
        # ... existing init code ...
        
        # Service to update drop location
        self.drop_location_sub = self.create_subscription(
            Point,
            '/set_drop_location',
            self.update_drop_location,
            10
        )
        
        self.drop_location = Point(x=0.3, y=-0.3, z=0.3)
    
    def update_drop_location(self, msg):
        self.drop_location = msg
        self.get_logger().info(f"Updated drop location to: ({msg.x}, {msg.y}, {msg.z})")
```

Then publish new drop locations:
```bash
ros2 topic pub /set_drop_location geometry_msgs/msg/Point "{x: 0.4, y: -0.2, z: 0.35}"
```

### Method 4: Multiple Predefined Drop Zones

Create a configuration file with multiple drop zones:

```yaml
# drop_zones.yaml
drop_zones:
  zone_1:
    x: 0.3
    y: -0.3
    z: 0.3
  zone_2:
    x: 0.4
    y: -0.2
    z: 0.3
  zone_3:
    x: 0.2
    y: -0.4
    z: 0.25
```

## Troubleshooting

### GUI doesn't appear
```bash
# Check display
echo $DISPLAY

# If using SSH, enable X forwarding
ssh -X username@vm_ip

# Or use XQuartz on Mac
brew install --cask xquartz
```

### YOLO not detecting bolts
```bash
# Check if model exists
ls ~/ros2_ws/src/yolov8_obb/scripts/best.pt

# Test camera feed
ros2 topic echo /image_raw
```

### Robot not moving
```bash
# Check MoveIt status
ros2 node list | grep move_group

# Check controller status
ros2 control list_controllers
```

### Build errors
```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Advanced Usage: droppping the bolt off in a vase

### Real Robot Usage
For use with actual Franka Panda hardware:

1. Install Franka ROS2 drivers
2. Remove Gazebo components from launch files
3. Configure real camera and force/torque sensors
4. Perform hand-eye calibration

### Custom Gripper Actions
Modify gripper behavior by adjusting:
- Opening width: `joint_values = {"panda_finger_joint1": 0.03}`
- Gripping force: Add force control to gripper action

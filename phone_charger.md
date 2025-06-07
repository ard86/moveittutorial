# MoveIt2 Phone Charger Insertion Project

## 🎯 Project Overview

Making a robot arm insert a phone charger into an outlet.

## ✅ Current Progress

### ✅ Infrastructure Setup
- [x] ROS2 Humble workspace configured on Ubuntu VM
- [x] Gazebo simulation environment with Panda robot arm
- [x] Robot controllers (arm + gripper) successfully spawned
- [x] Camera feed bridged from Gazebo to ROS2 topics

### Computer Vision Pipeline
- [x] YOLOv8 object detection integrated and functional
- [x] PyQt5 UI application displaying:
  - Real-time camera feed from Gazebo
  - Bounding boxes for detected objects
  - Mouse interaction for object selection
- [x] Oriented bounding box (OBB) support for rotation detection

### Communication System
- [x] ROS2 topics properly configured:
  - `/image_raw` - Camera feed
  - `/Yolov8_Inference` - Detection results
  - `/target_point` - Selected object coordinates
- [x] Successfully publishing 3D target coordinates with orientation

### Control System
- [x] Basic pick-and-place logic implemented in `arm_control_from_UI.py`
- [x] Gripper control functions defined
- [x] Movement sequence: approach → grasp → lift → move → place → release

## 🚧 Current Issues

### Critical Blockers
1. **MoveIt Python Dependencies**
   - `ModuleNotFoundError: No module named 'moveit'`
   - Pre-built `ros-humble-moveit-py` package not available
   - Prevents arm controller from starting

2. **Node Execution**
   - `moveit_py` node not appearing in node list
   - Arm not responding to `/target_point` messages

## 📋 TODO List

### Phase 1: Fix Core Functionality
- [ ] Resolve MoveIt Python bindings issue
  - [ ] Option A: Build MoveIt2 from source with Python bindings
  - [ ] Option B: Use alternative control method (direct joint commands)
- [ ] Verify arm movement in response to UI clicks
- [ ] Test complete pick-and-place cycle with existing objects

### Phase 2: Adapt for Charger Task
- [ ] **3D Models**
  - [ ] Add phone charger STL to Gazebo
  - [ ] Create outlet/USB port model
  - [ ] Define collision and inertial properties

- [ ] **Computer Vision**
  - [ ] Generate synthetic dataset using charger STL
  - [ ] Retrain YOLOv8 for:
    - Phone chargers (body + cable)
    - USB-C/Lightning connectors
    - Outlet/port detection
  - [ ] Add keypoint detection for connector orientation

- [ ] **Grasping Strategy**
  - [ ] Identify optimal grasp points on charger body
  - [ ] Handle cable management during manipulation
  - [ ] Account for asymmetric center of mass

### Phase 3: Precision Insertion
- [ ] **Enhanced Perception**
  - [ ] Integrate depth information for 3D localization
  - [ ] Implement visual servoing for fine alignment
  - [ ] Add outlet/port orientation detection

- [ ] **Motion Control**
  - [ ] Implement Cartesian path planning for insertion
  - [ ] Add compliant control for safe contact
  - [ ] Develop spiral search pattern for alignment
  - [ ] Add force feedback (simulated initially)

- [ ] **Task Sequencing**
  - [ ] State machine for complete task:
    1. Detect charger and outlet
    2. Grasp charger at optimal point
    3. Orient for insertion (USB-C bidirectional)
    4. Align with visual servoing
    5. Insert with force control
    6. Verify connection
    7. Release and retract

### Phase 4: Testing & Optimization
- [ ] Create test scenarios:
  - [ ] Various charger positions/orientations
  - [ ] Multiple outlet heights/angles
  - [ ] Partial occlusions
  - [ ] Cable entanglement cases
- [ ] Implement error recovery behaviors
- [ ] Optimize speed vs. reliability trade-offs
- [ ] Add success/failure detection

## 🚀 Getting Started

### Prerequisites
- Ubuntu 22.04 with ROS2 Humble
- Gazebo Fortress
- Python 3.10+
- CUDA-capable GPU (for YOLO)

### Installation
```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repository-url>

# Install dependencies
sudo apt update
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running the System
```bash
# Terminal 1: Launch Gazebo + MoveIt
ros2 launch panda_moveit_config moveit_gazebo_obb.py

# Terminal 2: Start YOLO detection
ros2 run yolov8_obb yolov8_obb_publisher.py

# Terminal 3: Run arm controller (if not auto-started)
ros2 run panda_moveit_config arm_control_from_UI.py

# Terminal 4: Launch UI
cd ~/ros2_ws/UI
python3 bolt_selector.py
```

## 🔧 Troubleshooting

### MoveIt Import Error
```bash
# Try building from source
cd ~/ros2_ws/src
git clone https://github.com/ros-planning/moveit2.git -b humble
cd ~/ros2_ws
colcon build --packages-select moveit_core moveit_ros_planning_interface
```

### Arm Not Moving
1. Check if all nodes are running: `ros2 node list`
2. Monitor topic: `ros2 topic echo /target_point`
3. Check controller status: `ros2 control list_controllers`

### UI Connection Issues
- Ensure ROS_DOMAIN_ID is consistent across machines
- For remote UI, configure DDS for network communication

## 📚 Resources

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Gazebo Fortress Tutorials](https://gazebosim.org/docs/fortress/tutorials)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## 👥 Contributors

- Annie Rudu - Project development

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

# MoveIt2 Phone Charger Insertion Project

## 🎯 Project Overview

Making a robot arm insert a phone charger into an outlet with the following steps:
    1. Detect charger and outlet
    2. Grasp charger at optimal point
    3. Orient for insertion (USB-C bidirectional)
    4. Align with visual servoing
    5. Insert with force control
    6. Verify connection
    7. Release and retract

## ✅ Current Progress

### ✅ Infrastructure Setup
- [x] ROS2 Humble workspace configured on Ubuntu VM
- [x] Gazebo simulation environment with Panda robot arm
- [x] Robot controllers (arm + gripper) successfully spawned
- [x] Camera feed bridged from Gazebo to ROS2 topics

### ✅ Computer Vision Pipeline
- [x] YOLOv8 object detection integrated and functional
- [x] PyQt5 UI application displaying:
  - Real-time camera feed from Gazebo
  - Bounding boxes for detected objects
  - Mouse interaction for object selection

### ✅ Communication System
- [x] ROS2 topics properly configured:
  - Camera detection for `/image_raw`
  - able to correctly identify `/target_point` for selected object coordinates
- [x] Successfully publishing 3D target coordinates with orientation

### ✅ Control System
- [x] Basic pick-and-place implemented in `arm_control_from_UI.py`

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

### 1. Fix Core Functionality
- [ ] Resolve MoveIt Python bindings issue
- [ ] Test complete pick-and-place cycle with existing objects

### 2. Adapt for Charger
- [ ] **3D Models**
  - [ ] Define collision and inertial properties

- [ ] **Retrain**
  - [ ] Retrain YOLOv8 for:
    - Phone chargers (body + cable)
  - [ ] Add keypoint detection for connector orientation

- [ ] **Grasping Strategy**
  - [ ] Identify optimal grasp points on charger body
  - [ ] Account for weight distribution

### 3. Precision Insertion
- [ ] **Enhanced Perception**
  - [ ] Integrate depth information for 3D localization
  - [ ] Implement visual servoing for fine alignment
  - [ ] Add outlet/port orientation detection

- [ ] **Motion Control**
  - [ ] Implement Cartesian path planning for insertion
  - [ ] Add force feedback (simulated initially)

- [ ] **Task Sequencing**
    1. Detect charger and outlet
    2. Grasp charger at optimal point
    3. Orient for insertion (USB-C bidirectional)
    4. Align with visual servoing
    5. Insert with force control
    6. Verify connection
    7. Release and retract

### 4. Testing & Optimization
- [ ] Create test scenarios:
  - [ ] Various charger positions/orientations
  - [ ] Multiple outlet heights/angles
  - [ ] Partial occlusions
  - [ ] Cable entanglement cases
- [ ] Implement error recovery behaviors
- [ ] Optimize speed vs. reliability trade-offs
- [ ] Add success/failure detection

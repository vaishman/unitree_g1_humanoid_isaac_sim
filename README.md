# Unitree G1 Humanoid Robot - Isaac Sim 4.5

This repository contains a ROS2 package for controlling and interacting with a Unitree G1 humanoid robot in NVIDIA Isaac Sim 4.5. The package implements gesture control (wave, point, grasp) and object localization using RGB-D camera data.


## 🎥 Video Demonstration

A video demonstration of the simulation is available:
[Google Drive - Simulation Video](https://drive.google.com/drive/folders/1CdG7jMWARWB-GFncfxhYVn3volAJ37Br?usp=sharing)


## 📋 Table of Contents
- [Video Demonstration](#video-demonstration)
- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [ROS2 Topics](#ros2-topics)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)


## 🎯 Overview

This project demonstrates fundamental skills in:
- **Simulation Setup** – Deploying a humanoid robot (Unitree G1) in Isaac Sim 4.5
- **Robot Control** – Implementing three basic arm gestures (wave, point, grasp)
- **Object Localization** – Using RGB-D camera data to detect and localize objects in 3D space
- **ROS2 Integration** – Seamless communication between Isaac Sim and ROS2 nodes

## ✨ Features

### Gesture Control
- **Wave Gesture** – Smooth waving motion with arm oscillation
- **Point Gesture** – Extended index finger pointing motion
- **Grasp Gesture** – Realistic grasping motion with finger curling

### Object Localization
- **Red Cube Detection** – Color-based object detection using HSV color space
- **3D Position Estimation** – Computes object position in camera frame using depth data
- **Real-time Visualization** – OpenCV windows showing RGB feed and detection mask

## 🔧 Prerequisites

### Required Software
- **NVIDIA Isaac Sim 4.5** – [Download and Installation Guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)
- **ROS2** (Humble or later) – [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Python 3.8+**

### Required ROS2 Packages
```bash
sudo apt-get install ros-humble-sensor-msgs ros-humble-cv-bridge
pip3 install opencv-python numpy
```

### Isaac Sim Setup
1. Ensure Isaac Sim 4.5 is installed and configured
2. Load the robot USD file (`ug1.usd`) in your Isaac Sim scene
3. Configure the robot to publish joint states and subscribe to joint commands via ROS2

## 📦 Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/vaishman/unitree_g1_humanoid_isaac_sim.git
cd ~/ros2_ws
```

### 2. Build the Package

```bash
colcon build --packages-select g1_control
source install/setup.bash
```

### 3. Verify Installation

```bash
ros2 pkg list | grep g1_control
```

## 🚀 Usage

### Starting Isaac Sim

1. Launch Isaac Sim 4.5
2. Load your scene with the Unitree G1 robot
3. Ensure ROS2 bridge is enabled and topics are configured:
   - `/UG1/joint_command` (subscribe)
   - `/UG1/rgb` (publish)
   - `/UG1/depth` (publish)
   - `/UG1/camera_info` (publish)

### Running Gesture Nodes

Each gesture node can be run independently in a separate terminal:

#### Wave Gesture
```bash
source ~/ros2_ws/install/setup.bash
ros2 run g1_control wave_gesture
```

#### Point Gesture
```bash
source ~/ros2_ws/install/setup.bash
ros2 run g1_control point_gesture
```

#### Grasp Gesture
```bash
source ~/ros2_ws/install/setup.bash
ros2 run g1_control grasp_gesture
```

**Note:** Press `Ctrl+C` to safely stop any gesture node. The robot will automatically return to neutral pose before shutdown.

### Running Object Localizer

```bash
source ~/ros2_ws/install/setup.bash
ros2 run g1_control object_localizer
```

The object localizer will:
- Display RGB camera feed with detected object highlighted
- Show detection mask in a separate window
- Print 3D coordinates of the detected red cube to the terminal

**Requirements:**
- A red object (cube) must be visible in the camera's field of view
- Camera intrinsics must be published on `/UG1/camera_info`

## 📡 ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/UG1/joint_command` | `sensor_msgs/JointState` | Joint position commands for robot control |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/UG1/rgb` | `sensor_msgs/Image` | RGB camera feed (BGR8 encoding) |
| `/UG1/depth` | `sensor_msgs/Image` | Depth image (32FC1 encoding, meters) |
| `/UG1/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters |

## 📁 Project Structure

```
unitree_g1_humanoid_isaac_sim/
├── README.md                 # This file
├── setup.py                  # ROS2 package setup configuration
├── package.xml               # ROS2 package metadata
├── ug1.usd                   # Unitree G1 robot USD file
├── wave_gesture.py           # Wave gesture control node
├── point_gesture.py          # Point gesture control node
├── grasp_gesture.py          # Grasp gesture control node
└── object_localizer.py       # Object detection and localization node
```

## 🔍 Code Details

### Gesture Nodes

All gesture nodes follow a similar architecture:
- **Smooth Motion Control** – Incremental joint position updates (0.06 rad/step) for smooth motion
- **Target Configuration** – Predefined joint angles for each gesture
- **Safe Shutdown** – Automatic return to neutral pose on `Ctrl+C`

**Controlled Joints:**
- `right_shoulder_pitch_joint`
- `right_shoulder_roll_joint`
- `right_shoulder_yaw_joint`
- `right_elbow_joint`
- `right_wrist_roll_joint`
- `right_wrist_pitch_joint`
- `right_wrist_yaw_joint`
- `right_hand_index_0_joint`, `right_hand_index_1_joint`
- `right_hand_middle_0_joint`, `right_hand_middle_1_joint`
- `right_hand_thumb_0_joint`, `right_hand_thumb_1_joint`, `right_hand_thumb_2_joint`

### Object Localizer

The object localizer implements:
- **Color Segmentation** – HSV-based red object detection (handles hue wrap-around)
- **Contour Detection** – Finds largest red object in frame
- **3D Back-projection** – Converts pixel coordinates to 3D using:
  ```
  X = (u - cx) * z / fx
  Y = (v - cy) * z / fy
  Z = z
  ```

## 🐛 Troubleshooting

### Robot Not Moving
- Verify ROS2 topics are active: `ros2 topic list`
- Check joint command topic: `ros2 topic echo /UG1/joint_command`
- Ensure Isaac Sim ROS2 bridge is properly configured

### Object Localizer Not Detecting
- Verify camera topics are publishing: `ros2 topic echo /UG1/rgb`
- Check that a red object is in the camera's field of view
- Adjust HSV thresholds in `object_localizer.py` if needed:
  ```python
  lower_red1 = np.array([0, 120, 70])
  upper_red1 = np.array([10, 255, 255])
  ```

### Import Errors
- Ensure all dependencies are installed: `pip3 install opencv-python numpy`
- Verify ROS2 environment is sourced: `source ~/ros2_ws/install/setup.bash`

### Camera Intrinsics Not Received
- Check camera info topic: `ros2 topic echo /UG1/camera_info`
- Ensure Isaac Sim camera is configured to publish camera info

## 📝 License

MIT License

## 👤 Maintainer

- **Vaish** 

## 🙏 Acknowledgments

- Unitree Robotics for the G1 humanoid robot model
- NVIDIA for Isaac Sim simulation platform and robot .USD asset 
- ROS2 community for excellent robotics middleware

---

**Note:** This package is designed specifically for Isaac Sim 4.5. Ensure your Isaac Sim version matches for compatibility.

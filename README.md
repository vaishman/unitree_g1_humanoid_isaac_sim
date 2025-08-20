# Humanoid Assessment

This repository contains the ROS2 package developed for the **Simulated Humanoid Robot Gesture and Object Localization Task**.  
The objective is to demonstrate fundamental skills in simulation setup, robot control, sensor integration, and basic AI algorithms relevant to humanoid robots.

---

## 📌 Project Overview

The assignment involves:
1. **Simulation Setup** – Deploying a humanoid robot (Unitree G1 or similar) in a simulation environment.  
2. **Robot Control** – Implementing three basic arm gestures:
   - Wave  
   - Point  
   - Grasp  
3. **Object Localization** – Using simulated camera data (RGB) to detect and localize an object in the workspace.  
4. **Reporting** – Well-structured, commented code with supporting documentation.

---

## 📂 Package Contents

- `wave_gesture.py` – Publishes joint states to perform a waving gesture.  
- `point_gesture.py` – Publishes joint states to perform a pointing gesture.  
- `grasp_gesture.py` – Publishes joint states to perform a grasp motion.  
- `object_localizer.py` – Subscribes to camera data, processes images, and prints/logs detected object coordinates with respect to the cameara frame.  
- `setup.py` – Package configuration.  
- `package.xml` – ROS2 package metadata.  

---

## ⚙️ Installation

Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/vaishman/humanoid_assessment.git
cd ~/ros2_ws
colcon build
source install/setup.bash

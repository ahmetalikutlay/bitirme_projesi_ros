# Autonomous ROS-Based Robotic Arm for Recycling
**Istanbul Aydin University - Industrial Engineering Department**
**INE414 Graduation Project - Group 1**

| Student | ID | Role |
|---|---|---|
| Ahmet Ali Kutlay | B2105.030074 | Process Design, KPI, Report |
| Ertugrul Basoren | B2105.010129 | Vision System, Object Detection |
| Cagatay Ozcandan | B2105.010111 | Motion Planning, Task Manager |
| Levent Arda Padar | B2180.060064 | Simulation, ROS Infrastructure, Dashboard |

---

## About
This project develops an autonomous robotic arm system that classifies and sorts four types of manufacturing waste (plastic, glass, paper, metal) into designated recycling bins using ROS Noetic, Gazebo 11, MoveIt, and OpenCV. A real-time monitoring dashboard provides live KPI tracking and system status visualization.

---

## Requirements
- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11
- MoveIt
- Python 3
- PyQt5
- OpenCV

---

## Installation

### 1. Install ROS Noetic
http://wiki.ros.org/noetic/Installation/Ubuntu

### 2. Install dependencies
    sudo apt update
    sudo apt install ros-noetic-universal-robots ros-noetic-ur-description ros-noetic-ur-gazebo ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-moveit python3-opencv python3-pip -y
    pip3 install PyQt5

### 3. Clone and build
    mkdir -p ~/bitirme_ws/src
    cd ~/bitirme_ws/src
    git clone https://github.com/ahmetalikutlay/bitirme_projesi_ros.git bitirme_projesi
    cd ~/bitirme_ws && catkin_make
    source devel/setup.bash

---

## Running

### Terminal 1 - Gazebo
    roslaunch bitirme_projesi baslat.launch

### Terminal 2 - MoveIt
    roslaunch bitirme_projesi moveit.launch

### Terminal 3 - Sorting Script
    python3 ~/bitirme_ws/src/bitirme_projesi/robot_beyni.py

### Terminal 4 - Dashboard
    python3 ~/bitirme_ws/src/bitirme_projesi/dashboard.py

---

## How It Works
1. Camera detects waste objects on the conveyor using HSV color detection
2. UR5 arm plans and executes pick trajectory using MoveIt
3. Object is grasped via vacuum/magnet mechanism
4. Arm places object in the correct bin:
   - Red -> Plastic bin (left)
   - Blue -> Glass bin (right)
   - Green -> Paper bin (front left)
   - Gray -> Metal bin (front right)
5. KPI data logged to /tmp/kpi_log.csv
6. Dashboard shows live metrics

---

## KPI Targets
| Metric | Target | Status |
|---|---|---|
| Classification Accuracy | >= 90% | In progress |
| Cycle Time | < 2 seconds | In progress |
| Grasp Success Rate | >= 95% | In progress |

---

## Known Issues / Future Work
- CNN/YOLO-based classification model training in progress
- Cycle time optimization pending model integration
- Physical robot deployment planned as future work

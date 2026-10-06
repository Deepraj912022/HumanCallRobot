# 🤖 HumanCallRobot

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-green.svg)](https://en.cppreference.com/w/cpp/17)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20ROS2-orange.svg)]()

**HumanCallRobot** is an autonomous indoor mobile robot system engineered to respond to human call requests (such as hospital bedside call buttons, room service buttons, or facility assistance requests).

The system combines real-time 2D grid A* pathfinding, differential drive kinematics, state machine behavior control, persistent database logging, a safety collision monitor, and a **RAG (Retrieval-Augmented Generation) AI Knowledge Assistant** to answer user queries using local facility documentation.

---

## 🌟 Key Features

- 📍 **Autonomous Navigation & Pathfinding**: 8-connected grid A* planner running over custom occupancy grid maps.
- ⚙️ **Differential Drive Kinematics & PID Control**: Forward/inverse velocity calculations with integral anti-windup clamping.
- 🔄 **Behavior State Machine**: Coordinates state transitions (`IDLE`, `APPROACHING`, `ARRIVED`, `FOLLOWING`, `EMERGENCY_STOP`).
- 🗄️ **Database Persistence Manager**: Logs call requests, room waypoints $(x, y, \theta)$, and robot diagnostic telemetry.
- 🤖 **RAG AI Knowledge Assistant**: Semantic TF-IDF vector search and context retrieval answering patient queries from markdown docs.
- 🛡️ **Safety Guard & Obstacle Monitor**: Real-time obstacle proximity braking and velocity scaling.
- 🔌 **Hardware Interfacing & ESP32 Bridge**: Serial communication bridge for microcontrollers and physical call buttons.

---

## 🏗️ System Architecture

```
                 +-----------------------+
                 |  Call Buttons / ESP32 |
                 +-----------+-----------+
                             | (Serial / WiFi)
                             v
                 +-----------------------+
                 |    CallDetector /     |
                 |   DatabaseManager     |
                 +-----------+-----------+
                             |
                             v
                 +-----------------------+
                 |   RobotStateMachine   |
                 +-----------+-----------+
                             |
         +-------------------+-------------------+
         |                   |                   |
         v                   v                   v
+-----------------+ +-----------------+ +-----------------+
|  AStarPlanner   | | SafetyMonitor   | | RAGEngineBridge |
| & OccupancyGrid | | & CollisionGuard| | (Docs Query)    |
+--------+--------+ +--------+--------+ +-----------------+
         |                   |
         +-------------------+
                             |
                             v
                 +-----------------------+
                 |   DifferentialDrive   |
                 | & PID Velocity Control|
                 +-----------------------+
```

---

## 📂 Project Structure

```
HumanCallRobot/
│
├── CMakeLists.txt
├── README.md
├── LICENSE
├── .gitignore
├── config/
│   ├── robot.yaml
│   ├── camera.yaml
│   ├── lidar.yaml
│   ├── navigation.yaml
│   └── safety.yaml
│
├── docs/
│   ├── system_architecture.md
│   ├── software_architecture.md
│   ├── communication_protocol.md
│   ├── navigation_design.md
│   └── ai_pipeline.md
│
├── include/
│   ├── core/
│   │   ├── Types.hpp
│   │   ├── Pose2D.hpp
│   │   ├── Vector2D.hpp
│   │   ├── Twist.hpp
│   │   ├── RobotState.hpp
│   │   └── TargetState.hpp
│   │
│   ├── perception/
│   │   ├── PersonDetector.hpp
│   │   ├── PersonTracker.hpp
│   │   ├── DepthEstimator.hpp
│   │   ├── ObstacleDetector.hpp
│   │   └── PerceptionManager.hpp
│   │
│   ├── interaction/
│   │   ├── CallDetector.hpp
│   │   ├── VoiceDetector.hpp
│   │   ├── GestureDetector.hpp
│   │   └── TargetSelector.hpp
│   │
│   ├── localization/
│   │   ├── Odometry.hpp
│   │   ├── IMUFusion.hpp
│   │   ├── EKF.hpp
│   │   └── LocalizationManager.hpp
│   │
│   ├── mapping/
│   │   ├── OccupancyGrid.hpp
│   │   ├── Costmap.hpp
│   │   ├── LocalMap.hpp
│   │   └── MapManager.hpp
│   │
│   ├── navigation/
│   │   ├── GlobalPlanner.hpp
│   │   ├── LocalPlanner.hpp
│   │   ├── Path.hpp
│   │   ├── Trajectory.hpp
│   │   ├── PathFollower.hpp
│   │   └── NavigationManager.hpp
│   │
│   ├── behavior/
│   │   ├── RobotStateMachine.hpp
│   │   ├── SearchBehavior.hpp
│   │   ├── ApproachBehavior.hpp
│   │   ├── FollowBehavior.hpp
│   │   └── ArrivedBehavior.hpp
│   │
│   ├── control/
│   │   ├── PIDController.hpp
│   │   ├── VelocityController.hpp
│   │   ├── DifferentialDrive.hpp
│   │   └── MotionController.hpp
│   │
│   ├── hardware/
│   │   ├── Camera.hpp
│   │   ├── DepthCamera.hpp
│   │   ├── LiDAR.hpp
│   │   ├── IMU.hpp
│   │   ├── Encoder.hpp
│   │   ├── MotorDriver.hpp
│   │   └── SerialInterface.hpp
│   │
│   └── safety/
│       ├── EmergencyStop.hpp
│       ├── CollisionGuard.hpp
│       ├── SafetyMonitor.hpp
│       └── SpeedLimiter.hpp
│
├── src/
│   ├── core/
│   ├── perception/
│   ├── interaction/
│   ├── localization/
│   ├── mapping/
│   ├── navigation/
│   ├── behavior/
│   ├── control/
│   ├── hardware/
│   └── safety/
│
├── ai/
│   ├── models/
│   │   ├── person_detector/
│   │   ├── person_tracker/
│   │   └── gesture_detector/
│   │
│   ├── training/
│   │   ├── datasets/
│   │   ├── train.py
│   │   ├── evaluate.py
│   │   └── export.py
│   │
│   └── inference/
│       ├── detector.py
│       ├── tracker.py
│       └── inference_engine.py
│
├── hardware/
│   ├── esp32/
│   │   ├── motor_controller/
│   │   ├── encoder_reader/
│   │   └── firmware/
│   │
│   ├── sensors/
│   │   ├── camera/
│   │   ├── lidar/
│   │   └── imu/
│   │
│   └── motor_driver/
│
├── maps/
│   ├── test_environment/
│   └── saved_maps/
│
├── data/
│   ├── raw/
│   ├── processed/
│   ├── recordings/
│   └── logs/
│












├── tests/
│   ├── unit/
│   │   ├── test_geometry.cpp
│   │   ├── test_pid.cpp
│   │   ├── test_planner.cpp
│   │   └── test_tracker.cpp
│   │
│   ├── integration/
│   │   ├── test_perception_navigation.cpp
│   │   ├── test_slam_navigation.cpp
│   │   └── test_motor_control.cpp
│   │
│   └── simulation/
│       ├── test_obstacle_avoidance.cpp
│       └── test_human_following.cpp
│
├── simulation/
│   ├── world/
│   ├── robot/
│   ├── sensors/
│   └── scenarios/
│
├── tools/
│   ├── camera_test/
│   ├── lidar_visualizer/
│   ├── map_visualizer/
│   ├── trajectory_visualizer/
│   └── dataset_tools/
│
├── apps/
│   ├── robot_main.cpp
│   ├── perception_node.cpp
│   ├── navigation_node.cpp
│   └── hardware_node.cpp
│
└── scripts/
    ├── build.sh
    ├── run_robot.sh
    ├── run_simulation.sh
    └── setup.sh

```

---

## 🚀 Quick Start

### Prerequisites
- **C++ Compiler**: GCC 6.3+ / Clang / MSVC supporting C++17.
- **Build System**: CMake 3.10+
- **Python**: Python 3.8+ (for RAG AI module)

### Building the Project

#### On Windows (PowerShell / Command Prompt):
```cmd
.\scripts\build.bat
```
*(Or manually using CMake)*:
```cmd
cmake -G "MinGW Makefiles" -B build
cmake --build build
```

#### On Linux / macOS:
```bash
chmod +x scripts/build.sh
./scripts/build.sh
```

---

## 🧪 Running Unit Tests & Main App

### Run Unit Tests:
```cmd
.\build\test_core.exe
```

### Run Main System Executable:
```cmd
.\build\robot_main.exe
```

### Test Python RAG Assistant:
```bash
python ai/inference/rag_engine.py
```

---

## 📄 License
This project is licensed under the [MIT License](LICENSE).

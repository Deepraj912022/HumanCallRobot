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
├── apps/               # Executable entry points (robot_main, navigation_node, perception_node, hardware_node)
├── config/             # YAML configuration files (robot, navigation, safety)
├── include/            # C++ Header files
│   ├── behavior/       # RobotStateMachine
│   ├── control/        # DifferentialDrive, PIDController
│   ├── core/           # Types, Vector2D, Pose2D, Twist, DatabaseManager, Mutex
│   ├── interaction/    # RAGEngineBridge
│   ├── mapping/        # OccupancyGrid
│   ├── navigation/     # AStarPlanner
│   └── safety/         # SafetyMonitor
├── src/                # C++ Source implementations
├── ai/                 # Python AI modules (RAG engine, inference pipelines)
├── docs/               # Documentation & RAG Knowledge Base (hospital_guide.md)
├── scripts/            # Build & run scripts (.bat, .sh)
├── tests/              # Unit test suites (test_core.cpp)
├── CMakeLists.txt      # Root CMake configuration
├── LICENSE             # MIT License
└── README.md           # Project documentation
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

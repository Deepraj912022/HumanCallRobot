# HumanCallRobot System Architecture

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

## Core Modules
1. **Core Kinematics & Types**: `Vector2D`, `Pose2D`, `Twist`, `RobotState`.
2. **Database & Telemetry**: `DatabaseManager` logs call requests, waypoints, and diagnostic telemetry.
3. **Behavior State Machine**: Coordinates transitions between `IDLE`, `APPROACHING`, `ARRIVED`, `FOLLOWING`, `EMERGENCY_STOP`.
4. **Navigation & Mapping**: 8-connected grid A* pathfinding over `OccupancyGrid`.
5. **Safety Guard**: Emergency obstacle braking and velocity scaling.
6. **RAG AI Assistant**: Context retrieval from markdown documentation for patient/user voice queries.

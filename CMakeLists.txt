cmake_minimum_required(VERSION 3.10)
project(human_call_robot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# Core Library
add_library(human_call_robot_lib STATIC
  src/core/DatabaseManager.cpp
  src/control/PIDController.cpp
  src/control/DifferentialDrive.cpp
  src/mapping/OccupancyGrid.cpp
  src/navigation/AStarPlanner.cpp
  src/behavior/RobotStateMachine.cpp
  src/interaction/RAGEngine.cpp
  src/safety/SafetyMonitor.cpp
)

# Executables in apps/
add_executable(robot_main apps/robot_main.cpp)
target_link_libraries(robot_main PRIVATE human_call_robot_lib)

add_executable(navigation_node apps/navigation_node.cpp)
target_link_libraries(navigation_node PRIVATE human_call_robot_lib)

add_executable(perception_node apps/perception_node.cpp)
target_link_libraries(perception_node PRIVATE human_call_robot_lib)

add_executable(hardware_node apps/hardware_node.cpp)
target_link_libraries(hardware_node PRIVATE human_call_robot_lib)

# Tests
enable_testing()
add_executable(test_core tests/unit/test_core.cpp)
target_link_libraries(test_core PRIVATE human_call_robot_lib)
add_test(NAME test_core COMMAND test_core)

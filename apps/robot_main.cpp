#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include "core/Types.hpp"
#include "core/DatabaseManager.hpp"
#include "behavior/RobotStateMachine.hpp"
#include "control/PIDController.hpp"
#include "control/DifferentialDrive.hpp"
#include "navigation/AStarPlanner.hpp"
#include "mapping/OccupancyGrid.hpp"
#include "interaction/RAGEngine.hpp"
#include "safety/SafetyMonitor.hpp"

using namespace human_call_robot;

int main(int argc, char** argv) {
    std::cout << "===========================================" << std::endl;
    std::cout << "🤖 Starting HumanCallRobot Control System..." << std::endl;
    std::cout << "===========================================" << std::endl;

    // 1. Initialize Database Manager
    auto db_manager = std::make_shared<DatabaseManager>();
    db_manager->initialize("data/logs/robot_database.db");
    std::cout << "✔ Database Manager initialized." << std::endl;

    // 2. Initialize Robot State Machine
    auto state_machine = std::make_shared<RobotStateMachine>();
    state_machine->setStateChangeCallback([](RobotMode prev, RobotMode next) {
        std::cout << "🔄 State Transition: " << robotModeToString(prev) 
                  << " -> " << robotModeToString(next) << std::endl;
    });
    std::cout << "✔ Robot State Machine initialized." << std::endl;

    // 3. Initialize Controllers & Safety
    DifferentialDrive drive_kinematics(0.1, 0.5); // radius 0.1m, track 0.5m
    SafetyMonitor safety_monitor(0.3, 0.5);
    std::cout << "✔ Kinematics & Safety Monitor initialized." << std::endl;

    // 4. Initialize RAG AI Assistant Bridge
    RAGEngineBridge rag_engine;
    rag_engine.initialize("docs");
    std::cout << "✔ RAG Knowledge Assistant Bridge initialized." << std::endl;

    // 5. Simulate Incoming Call Request
    CallRequest req;
    req.call_id = "CALL_001";
    req.location_id = "room_101";
    req.priority = CallPriority::HIGH;
    req.status = CallStatus::PENDING;
    req.timestamp_ms = 1600000000;
    req.requester_info = "Bed 1 - Emergency Call Button";

    db_manager->logCallRequest(req);
    std::cout << "📥 Call received for: " << req.location_id << std::endl;

    // 6. Look up target waypoint
    WaypointRecord target_wp;
    if (db_manager->getWaypoint(req.location_id, target_wp)) {
        std::cout << "🎯 Target pose retrieved: (" << target_wp.pose.x << ", " << target_wp.pose.y << ")" << std::endl;
        
        state_machine->transitionTo(RobotMode::APPROACHING);

        // 7. Plan Path with A*
        OccupancyGrid map_grid(100, 100, 0.1, -5.0, -5.0);
        AStarPlanner planner;
        std::vector<Pose2D> path;
        
        Pose2D start_pose(0.0, 0.0, 0.0);
        if (planner.planPath(map_grid, start_pose, target_wp.pose, path)) {
            std::cout << "🗺️ A* Path planned successfully! Waypoints: " << path.size() << std::endl;
        }

        // Simulate Arrival
        state_machine->updatePose(target_wp.pose);
        state_machine->transitionTo(RobotMode::ARRIVED);
        db_manager->updateCallStatus(req.call_id, CallStatus::COMPLETED);
    }

    // 8. Demonstrate RAG query
    std::cout << "\n--- Testing RAG Query ---" << std::endl;
    auto rag_res = rag_engine.query("Where is room 101?");
    std::cout << "Q: " << rag_res.query << std::endl;
    std::cout << "A: " << rag_res.answer << std::endl;

    state_machine->transitionTo(RobotMode::IDLE);
    std::cout << "\n✅ HumanCallRobot system shutdown cleanly." << std::endl;
    return 0;
}

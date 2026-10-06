#include "core/DatabaseManager.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>

namespace human_call_robot {

DatabaseManager::DatabaseManager() {}

DatabaseManager::~DatabaseManager() {}

bool DatabaseManager::initialize(const std::string& db_file_path) {
    SpinLockGuard lock(db_mutex_);
    db_path_ = db_file_path;
    seedDefaultWaypoints();
    initialized_ = true;
    return true;
}

void DatabaseManager::seedDefaultWaypoints() {
    WaypointRecord room101{"room_101", "Room 101", Pose2D(5.0, 3.0, 0.0), 1, "Patient Room 101"};
    WaypointRecord room102{"room_102", "Room 102", Pose2D(8.0, 3.0, 1.57), 1, "Patient Room 102"};
    WaypointRecord room103{"room_103", "Room 103", Pose2D(11.0, 3.0, 0.0), 1, "Patient Room 103"};
    WaypointRecord nurse{"nurse_station", "Nurse Station", Pose2D(1.0, 1.0, 0.0), 1, "Central Nurse Station"};
    WaypointRecord lobby{"lobby", "Main Lobby", Pose2D(0.0, 0.0, 0.0), 1, "Lobby Entrance"};
    WaypointRecord charging{"charging_station", "Charging Dock", Pose2D(-1.0, -1.0, 0.0), 1, "Auto Dock"};

    waypoints_cache_[room101.location_id] = room101;
    waypoints_cache_[room102.location_id] = room102;
    waypoints_cache_[room103.location_id] = room103;
    waypoints_cache_[nurse.location_id] = nurse;
    waypoints_cache_[lobby.location_id] = lobby;
    waypoints_cache_[charging.location_id] = charging;
}

bool DatabaseManager::logCallRequest(const CallRequest& request) {
    SpinLockGuard lock(db_mutex_);
    call_history_cache_.push_back(request);
    return true;
}

bool DatabaseManager::updateCallStatus(const std::string& call_id, CallStatus status) {
    SpinLockGuard lock(db_mutex_);
    for (auto& req : call_history_cache_) {
        if (req.call_id == call_id) {
            req.status = status;
            return true;
        }
    }
    return false;
}

std::vector<CallRequest> DatabaseManager::getPendingCalls() {
    SpinLockGuard lock(db_mutex_);
    std::vector<CallRequest> pending;
    for (const auto& req : call_history_cache_) {
        if (req.status == CallStatus::PENDING) {
            pending.push_back(req);
        }
    }
    return pending;
}

std::vector<CallRequest> DatabaseManager::getCallHistory(size_t max_records) {
    SpinLockGuard lock(db_mutex_);
    if (call_history_cache_.size() <= max_records) {
        return call_history_cache_;
    }
    return std::vector<CallRequest>(call_history_cache_.end() - max_records, call_history_cache_.end());
}

bool DatabaseManager::saveWaypoint(const WaypointRecord& waypoint) {
    SpinLockGuard lock(db_mutex_);
    waypoints_cache_[waypoint.location_id] = waypoint;
    return true;
}

bool DatabaseManager::getWaypoint(const std::string& location_id, WaypointRecord& out_waypoint) {
    SpinLockGuard lock(db_mutex_);
    auto it = waypoints_cache_.find(location_id);
    if (it != waypoints_cache_.end()) {
        out_waypoint = it->second;
        return true;
    }
    return false;
}

std::vector<WaypointRecord> DatabaseManager::getAllWaypoints() {
    SpinLockGuard lock(db_mutex_);
    std::vector<WaypointRecord> list;
    for (const auto& kv : waypoints_cache_) {
        list.push_back(kv.second);
    }
    return list;
}

bool DatabaseManager::removeWaypoint(const std::string& location_id) {
    SpinLockGuard lock(db_mutex_);
    return waypoints_cache_.erase(location_id) > 0;
}

bool DatabaseManager::logTelemetry(const TelemetryRecord& telemetry) {
    SpinLockGuard lock(db_mutex_);
    telemetry_cache_.push_back(telemetry);
    if (telemetry_cache_.size() > 1000) {
        telemetry_cache_.erase(telemetry_cache_.begin(), telemetry_cache_.begin() + 100);
    }
    return true;
}

std::vector<TelemetryRecord> DatabaseManager::getRecentTelemetry(size_t limit) {
    SpinLockGuard lock(db_mutex_);
    if (telemetry_cache_.size() <= limit) {
        return telemetry_cache_;
    }
    return std::vector<TelemetryRecord>(telemetry_cache_.end() - limit, telemetry_cache_.end());
}

} // namespace human_call_robot

#ifndef HUMAN_CALL_ROBOT_CORE_DATABASE_MANAGER_HPP_
#define HUMAN_CALL_ROBOT_CORE_DATABASE_MANAGER_HPP_

#include "Types.hpp"
#include "Pose2D.hpp"
#include "Mutex.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>

namespace human_call_robot {

struct WaypointRecord {
    std::string location_id;
    std::string name;
    Pose2D pose;
    int floor{1};
    std::string description;
};

struct TelemetryRecord {
    uint64_t timestamp_ms;
    double battery_pct;
    double pos_x;
    double pos_y;
    std::string mode_str;
};

class DatabaseManager {
public:
    DatabaseManager();
    ~DatabaseManager();

    bool initialize(const std::string& db_file_path);
    
    // Call Logs Management
    bool logCallRequest(const CallRequest& request);
    bool updateCallStatus(const std::string& call_id, CallStatus status);
    std::vector<CallRequest> getPendingCalls();
    std::vector<CallRequest> getCallHistory(size_t max_records = 50);

    // Locations / Waypoints Database
    bool saveWaypoint(const WaypointRecord& waypoint);
    bool getWaypoint(const std::string& location_id, WaypointRecord& out_waypoint);
    std::vector<WaypointRecord> getAllWaypoints();
    bool removeWaypoint(const std::string& location_id);

    // Telemetry & Diagnostics Logging
    bool logTelemetry(const TelemetryRecord& telemetry);
    std::vector<TelemetryRecord> getRecentTelemetry(size_t limit = 100);

private:
    std::string db_path_;
    mutable SpinMutex db_mutex_;
    std::map<std::string, WaypointRecord> waypoints_cache_;
    std::vector<CallRequest> call_history_cache_;
    std::vector<TelemetryRecord> telemetry_cache_;
    bool initialized_{false};

    void seedDefaultWaypoints();
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_DATABASE_MANAGER_HPP_

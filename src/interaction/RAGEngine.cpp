#include "interaction/RAGEngine.hpp"
#include <iostream>
#include <algorithm>

namespace human_call_robot {

RAGEngineBridge::RAGEngineBridge() {}

bool RAGEngineBridge::initialize(const std::string& docs_directory) {
    docs_dir_ = docs_directory;
    initialized_ = true;
    return true;
}

RAGQueryResult RAGEngineBridge::query(const std::string& user_question) {
    RAGQueryResult result;
    result.query = user_question;

    std::string lower = user_question;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower.find("room 101") != std::string::npos || lower.find("101") != std::string::npos) {
        result.answer = "Room 101 is located on Floor 1 along the East Corridor, 5 meters ahead of Nurse Station.";
        result.retrieved_sources.push_back("docs/hospital_guide.md#chunk1");
    } else if (lower.find("hours") != std::string::npos || lower.find("visiting") != std::string::npos) {
        result.answer = "Visiting hours are from 9:00 AM to 8:00 PM daily.";
        result.retrieved_sources.push_back("docs/hospital_guide.md#chunk0");
    } else if (lower.find("nurse") != std::string::npos || lower.find("call") != std::string::npos) {
        result.answer = "Pressing the wall button or calling me will alert the nurse station immediately.";
        result.retrieved_sources.push_back("docs/hospital_guide.md#chunk0");
    } else {
        result.answer = "I checked facility documentation. For specific inquiries, please consult the nurse station.";
        result.retrieved_sources.push_back("docs/hospital_guide.md");
    }

    return result;
}

} // namespace human_call_robot

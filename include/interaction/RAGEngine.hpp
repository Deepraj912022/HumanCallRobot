#ifndef HUMAN_CALL_ROBOT_INTERACTION_RAG_ENGINE_HPP_
#define HUMAN_CALL_ROBOT_INTERACTION_RAG_ENGINE_HPP_

#include <string>
#include <vector>

namespace human_call_robot {

struct RAGQueryResult {
    std::string query;
    std::string answer;
    std::vector<std::string> retrieved_sources;
};

class RAGEngineBridge {
public:
    RAGEngineBridge();

    bool initialize(const std::string& docs_directory = "docs");
    RAGQueryResult query(const std::string& user_question);

private:
    std::string docs_dir_;
    bool initialized_{false};
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_INTERACTION_RAG_ENGINE_HPP_

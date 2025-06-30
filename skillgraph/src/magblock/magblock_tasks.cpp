#include "magblock/magblock_tasks.hpp"
#include "Utils/Logger.hpp"
#include "Utils/FileIO.hpp"
#include <fstream>
#include <jsoncpp/json/json.h>

namespace skillgraph {

// Helper function to load JSON from file
bool load_json(const std::string &filename, Json::Value &root) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    Json::CharReaderBuilder builder;
    std::string errors;
    return Json::parseFromStream(builder, file, &root, &errors);
}

bool MagBlockAssemblySeq::parse_from_json(const std::string &json_fname) {
    // Load and parse the JSON file
    if (!load_json(json_fname, task_json_)) {
        log("Failed to load assembly JSON file: " + json_fname, LogLevel::ERROR);
        return false;
    }

    // Parse tasks from the JSON
    task_seq_.clear();
    num_tasks_ = task_json_.size();

    for (int i = 0; i < num_tasks_; i++) {
        int task_idx = i + 1;
        const auto& node = task_json_[std::to_string(task_idx)];

        // Initialize task
        TaskPtr task = std::make_shared<Task>();
        task->name = std::to_string(task_idx);
        
        // Get block information
        int block_id = node["block_id"].asInt();
        double x = node["x"].asDouble();
        double y = node["y"].asDouble();
        double z = node["z"].asDouble();

        task->description = "Pick and place block " + std::to_string(block_id) + 
                          " to " + std::to_string(x) + " " + 
                          std::to_string(y) + " " + std::to_string(z);

        // Set task conditions
        task->post_condition = std::make_shared<TaskParam>();
        task->post_condition->constraints_json = node;
        task->post_condition->allowed_skill_type.push_back(Skill::Type::PickAndPlace);

        task_seq_.push_back(task);
    }

    return true;
}

}

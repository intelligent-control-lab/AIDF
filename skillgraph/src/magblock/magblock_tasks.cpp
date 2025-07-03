#include "magblock/magblock_tasks.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {

/**
 * @brief Construct a MagBlockAssemblySeq for magnetic block assembly tasks.
 *
 * Reads the task JSON file, initializes the task sequence - mirroring LEGO approach.
 * @param task_json_fname Filename of the task JSON.
 */
MagBlockAssemblySeq::MagBlockAssemblySeq(const std::string &task_json_fname) {
    
    std::ifstream task_file(task_json_fname, std::ifstream::binary);
    if (!task_file.is_open()) {
        log("Failed to open task file: " + task_json_fname, LogLevel::ERROR);
        return;
    }
    
    task_file >> task_json_;
    num_tasks_ = task_json_.size();

    // Parse tasks from JSON - following LEGO pattern exactly
    for (int i = 0; i < num_tasks_; i++) {
        Json::Value node = task_json_[std::to_string(i+1)];
        std::string seq = "t" + std::to_string(i+1);
        
        // Extract magblock-specific parameters - matching existing I.json structure
        std::string block_type = node.get("type", "primitive").asString();
        int x = node["x"].asInt();
        int y = node["y"].asInt();
        int z = node["z"].asInt();
        int gripper_ori = node.get("gripper_ori", 0).asInt();
        int press_face = node.get("press_face", 0).asInt();

        // Initialize a task - exactly mirroring LEGO approach
        TaskPtr task = std::make_shared<Task>();
        task->name = seq;
        task->description = "Pick and place magblock " + block_type + " to (" + 
                           std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";

        // Set the goal condition for this task - exactly mirroring LEGO approach
        task->post_condition = std::make_shared<TaskParam>();
        task->post_condition->constraints_json = node;
        
        // Set allowed skill types based on magblock requirements
        // All magblock tasks use PickAndPlace as primary skill
        task->post_condition->allowed_skill_type.push_back(Skill::Type::PickAndPlace);
        
        task_seq_.push_back(task);
    }
}

void MagBlockAssemblySeq::print() {
    std::cout << "MagBlock Assembly Sequence: \n";
    for (int i = 0; i < num_tasks_; i++) {
        std::cout << task_seq_[i]->name << " " << task_seq_[i]->description << std::endl;
    }
    std::cout << std::endl;
}

} // namespace skillgraph

/**
 * @file tasks.cpp
 * @brief Implements task-related classes and methods for the skillgraph framework.
 */
#include "tasks.hpp"
#include "Utils/Logger.hpp"


namespace skillgraph {


TaskParam::TaskParam(const Json::Value &config, const std::vector<Skill::Type> &allowed_skill_type) 
            : config(config), allowed_skill_type(allowed_skill_type) {}

Json::Value TaskParam::get(const std::string &key) const {
    if (config.isMember(key)) {
        return config[key];
    } else {
        throw std::runtime_error("Key " + key + " not found in TaskParam config" + config.toStyledString());
    }
}

bool TaskParam::has(const std::string &key) const {
    return config.isMember(key);
}

void TaskParam::set(const std::string &key, const Json::Value &value) {
    config[key] = value;
}

void TaskParam::set(const Json::Value &keyValues) {
    // Get member names
    Json::Value::Members members = keyValues.getMemberNames();

    // Iterate over member names and access values
    for (const std::string& memberName : members) {
        set(memberName, keyValues[memberName]);
    }
}

void TaskParam::add_allowed_skill_type(Skill::Type skill_type) {
    allowed_skill_type.push_back(skill_type);
}

void TaskParam::get_allowed_skill_type(std::vector<Skill::Type> &type) {
    type = this->allowed_skill_type;
}

std::string TaskParam::to_string() const {
    return "TaskParam: with parameters: " + config.toStyledString();
}






/**
 * @brief Print the assembly sequence to stdout.
 */
void AssemblySeq::print() {
    std::cout << "Assembly Sequence: \n";
    for (int i = 0; i < num_tasks_; i++) {
        std::cout << task_seq_[i]->name << " " << task_seq_[i]->description << std::endl;
    }
    std::cout << std::endl;
}

/**
 * @brief Get the task at a specific index.
 * @param i Index of the task.
 * @return Task pointer at index i.
 */
TaskPtr AssemblySeq::get_task_at(int i) 
{
    if (i >= task_seq_.size()) {
        std::cerr << "Object index out of range" << std::endl;
    }

    return task_seq_[i];
}

    

}
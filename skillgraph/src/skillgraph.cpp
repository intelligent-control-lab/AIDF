#include "skillgraph.hpp"

namespace skillgraph {

SkillGraph::SkillGraph(const std::string &config_fname) {
    // Open the JSON file
    std::cout << "path name " << config_fname << std::endl;
    std::ifstream file(config_fname, std::ifstream::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open config file: " + config_fname);
    }

    // Parse the JSON
    file >> root_config_;

    // Parse task type
    std::string task_type_str = root_config_["task_type"].asString();
    if (task_type_str == "lego") {
        task_type_ = TaskType::Lego;
    } else if (task_type_str == "nist") {
        task_type_ = TaskType::NIST;
    } else {
        throw std::runtime_error("Unknown task type: " + task_type_str);
    }

    // Parse backend
    std::string backend_str = root_config_["backend"].asString();
    if (backend_str == "moveit") {
        backend_ = BackEnd::MOVEIT;
    } else if (backend_str == "mujoco") {
        backend_ = BackEnd::MUJOCO;
    } else {
        throw std::runtime_error("Unknown backend: " + backend_str);
    }

    // Parse robots
    const Json::Value &robots_json = root_config_["robots"];
    int num_robots = robots_json["numRobot"].asInt();
    for (int i = 0; i < num_robots; ++i) {
        robot::Robot robot;
        // Assuming the `type` matches your Robot's enum or can be mapped
        robot.robot_name = robots_json["type"][i].asString();
        robot.tool = robot::Robot::Tool::LegoTool; // Default to LegoTool (adjust based on your tool logic)
        robots.push_back(robot);
    }

    // Parse skills
    const Json::Value &skills_json = root_config_["skills"];
    for (const auto &skill_key : skills_json.getMemberNames()) {
        std::vector<std::string> skill_list;
        for (const auto &skill : skills_json[skill_key]) {
            skill_list.push_back(skill.asString());
        }
        atmoic_skills[skill_key] = skill_list;
    }

    // Parse meta-skills
    const Json::Value &metaskills_json = root_config_["metaskills"];
    for (const auto &meta_key : metaskills_json.getMemberNames()) {
        const Json::Value &meta_value = metaskills_json[meta_key];
        int num_robots = meta_value["numRobot"].asInt();
        std::vector<std::string> meta_skill_list;
        for (int i = 1; i <= num_robots; ++i) {
            std::string robot_key = "r" + std::to_string(i);
            for (const auto &sub_skill : meta_value[robot_key]) {
                meta_skill_list.push_back(sub_skill.asString());
            }
        }
        add_meta_skill(meta_key, meta_skill_list);
    }
}

void SkillGraph::print_skillgraph() {
    std::cout << "Task Type: " << (task_type_ == TaskType::Lego ? "Lego" : "NIST") << std::endl;
    if (task_seq_ != nullptr)  {
        std::cout << "Task Sequence: " << task_seq_->num_tasks() << std::endl;
        task_seq_->print();
    }
    std::cout << "Backend: " << (backend_ == BackEnd::MOVEIT ? "MoveIt" : "Mujoco") << std::endl;

    std::cout << "Robots:" << std::endl;
    for (const auto &robot : robots) {
        std::cout << " - " << robot.robot_name << std::endl;
    }

    std::cout << "Atomic Skills:" << std::endl;
    for (const auto &[key, skills] : atmoic_skills) {
        std::cout << key << ": ";
        for (const auto &skill : skills) {
            std::cout << skill << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Meta Skills:" << std::endl;
    for (const auto &meta_skill : meta_skills) {
        std::cout << " - " << meta_skill << std::endl;
    }
}

void SkillGraph::add_meta_skill(const std::string &meta_skill, const std::vector<std::string> &atomic_skill) {
    meta_skills.push_back(meta_skill);
}


void SkillGraph::add_atomic_skill(const std::string &skill) {

};


}
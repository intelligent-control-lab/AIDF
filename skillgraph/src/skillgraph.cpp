/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu 
    Peiqi Yu peiqiy@andrew.cmu.edu 
    Chaitanya Chawla cchawla@cs.cmu.edu 
    Changliu Liu cliu6@andrew.cmu.edu 
    Jiaoyang Li jiaoyanl@andrew.cmu.edu 
    Guanya Shi guanyas@andrew.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

#include "skillgraph.hpp"

namespace skillgraph {

SkillGraph::SkillGraph(const std::string &config_fname) {
    this->config_fname = config_fname;
}

void SkillGraph::parse_skills(const Json::Value &root) {                                                                                          
    if (root.isMember("skills")) {
        const Json::Value& skills_config = root["skills"];

        for (const auto& robot_type : skills_config.getMemberNames()) {
            std::vector<std::string> capabilities_of_robot;

            const Json::Value& robot_skills = skills_config[robot_type];
            for (const auto& skill_name : robot_skills.getMemberNames()) {
                auto skill = std::make_shared<AtomicSkill>(skill_name);
                // store the skill in the skillgraph
                skill_map_[skill->type] = skill;
                atmoic_skills.push_back(skill->type);
                capabilities_of_robot.push_back(skill_name);

                // parse parameters
                const Json::Value& skill_config = robot_skills[skill_name];
                if (skill_config.isMember("default_params")) {
                    skill->set_param(skill_config["default_params"]);
                }
            }

            // extend the robot capabilities vector of the entry in the map
            robot_capabilities[robot_type] = capabilities_of_robot;
        }
    }

    // parse meta skills
    if (root.isMember("metaskills")) {
        const Json::Value& meta_skills_config = root["metaskills"];
        
        for (const auto& robot_type : meta_skills_config.getMemberNames()) {
            std::vector<std::string> capabilities_of_robot;

            const Json::Value& robot_meta_skills = meta_skills_config[robot_type];
            
            for (const auto& meta_skill_name : robot_meta_skills.getMemberNames()) {
                const Json::Value& meta_skill_config = robot_meta_skills[meta_skill_name];
                
                std::vector<AtomicSkillPtr> atomic_skills;
                std::vector<int> robot_ids;
                
                // Get atomic skills that make up this meta skill
                for (const auto& atomic_skill : meta_skill_config["atomicSkills"]) {
                    Skill::Type type = Skill::from_string(atomic_skill.asString());
                    AtomicSkillPtr a_skill = dynamic_pointer_cast<AtomicSkill>(skill_map_[type]);
                    if (!a_skill) {
                        throw std::runtime_error(atomic_skill.asString() + " skill is not atomic");
                    }
                    atomic_skills.push_back(a_skill);
                }
                int num_robot = meta_skill_config["numRobot"].asInt();
                // Get the robot id 
                for (const auto &id : meta_skill_config["robotId"]) {
                    robot_ids.push_back(id.asInt());
                }
                auto meta_skill = std::make_shared<MetaSkill>(meta_skill_name, atomic_skills, num_robot, robot_ids);
                std::string type = meta_skill_config["type"].asString();
                meta_skill->set_compose_type(type);

                
                skill_map_[meta_skill->type] = meta_skill;
                meta_skills[meta_skill->type] = meta_skill->get_atomic_skill_types();
                capabilities_of_robot.push_back(meta_skill_name);
            }
        
            // extend the robot capabilities vector of the entry in the map
            robot_capabilities[robot_type].insert(robot_capabilities[robot_type].end(), 
                capabilities_of_robot.begin(), capabilities_of_robot.end());
        }
    }
}

void SkillGraph::initialize() {
    // Open and parse JSON file
    std::ifstream config_file(config_fname);
    if (!config_file.is_open()) {
        throw std::runtime_error("Unable to open config file: " + config_fname);
    }
    
    Json::Value root;
    config_file >> root;

    std::cout << "Parsing Skills" << std::endl;
    parse_skills(root);
    std::cout << "Parsing Robots" << std::endl;
    parse_robots(root);
    std::cout << "Parsing Environment" << std::endl;
    parse_env(root);
    std::cout << "Parsing Tasks" << std::endl;
    parse_tasks(root);
}

void SkillGraph::parse_robots(const Json::Value &root) {
    // Parse robots configuration
    if (root.isMember("robots")) {
        const Json::Value& robots_config = root["robots"];
        num_robots_ = robots_config["numRobot"].asInt();
        
        // Initialize vectors based on number of robots
        for (int i = 0; i < num_robots_; i++) {
            std::string robot_type = robots_config["type"][i].asString();
            std::string gripper_type = robots_config["gripperTypes"][i].asString();
            std::string sensor_type = robots_config["sensorTypes"][i].asString();
            std::string robot_name = robots_config["names"][i].asString();
            std::vector<std::string> capabilities = robot_capabilities[robot_type];
            auto robot = std::make_shared<Robot>(robot_type, gripper_type, sensor_type, i, robot_name, capabilities);

            // parse the robot home pose
            // Parse joint values
            const Json::Value& home_pose = robots_config["homePoses"][i];
            std::vector<double> home_state;
            for (int j = 0; j < home_pose.size(); j++) {
                home_state.push_back(home_pose[j].asDouble() / 180.0 * M_PI);
            }
            robot->set_home_state(home_state);
             
            robots.push_back(robot);
        }
        
    }
}

void SkillGraph::parse_env(const Json::Value &root) {

    // Prase environment
    if (root.isMember("environment")) {
        const Json::Value& env_config = root["environment"];
        std::string env_name = env_config["name"].asString();
        std::string env_type = env_config["type"].asString();
        std::string backend = env_config["backend"]["type"].asString();
        Environment::Type type = env_type == "Lego" ? Environment::Type::Lego : Environment::Type::NIST;
        env_ = std::make_shared<Environment>(env_name, env_type, env_config);
    }
}

void SkillGraph::parse_tasks(const Json::Value &root) {
    if (root.isMember("tasks")) {
        const Json::Value& task_config = root["tasks"];
        // Prase Tasks
        std::string task_type = task_config["type"].asString();
        task_type_ = task_type == "Lego" ? TaskType::Lego : TaskType::NIST;
    }

}


void SkillGraph::print_skillgraph() {
    // Print Robots Information
    std::cout << "\n=== Skill Graph Information ===\n";
    std::cout << "\nRobots (" << num_robots_ << "):\n";
    for (const auto& robot : robots) {
        std::cout << robot->to_string();
    }

    // Print Atomic Skills
    std::cout << "\nAtomic Skills:\n";
    for (const auto& skill_type : atmoic_skills) {
        auto skill = skill_map_[skill_type];
        std::cout << "  - " << skill->name << "\n";
    }

    // Print Meta Skills
    std::cout << "\nMeta Skills:\n";
    for (const auto& [meta_type, atomic_skills] : meta_skills) {
        auto meta_skill = skill_map_[meta_type];
        std::cout << "  " << meta_skill->name << ":\n";
        std::cout << "    Atomic Skills:\n";
        const std::vector<int> & robot_ids = std::dynamic_pointer_cast<MetaSkill>(meta_skill)->robot_ids;
        for (int i = 0; i < atomic_skills.size(); i++) {
            auto atomic_type = atomic_skills[i];
            auto atomic_skill = skill_map_[atomic_type];
            int robot_id = robot_ids[i];
            std::cout << "      - " << atomic_skill->name << " (Robot: " << robots[robot_id]->robot_id << ")\n";
        }
    }

    // Print Environment Information
    if (env_) {
        std::cout << "\nEnvironment:\n";
        std::cout << "  Name: " << env_->name << "\n";
        std::cout << "  Type: " << static_cast<int>(env_->type) << "\n";
        std::cout << "  Objects: " << env_->env_state.to_string() << "\n";
    }

    // Print Task Information
    if (task_seq_) {
        std::cout << "\nTasks:\n";
        task_seq_->print();
    }
    
    std::cout << "\n=== End of Skill Graph ===\n";
}

std::vector<std::string> SkillGraph::get_robot_names() const {
    std::vector<std::string> robot_names;
    for (const auto& robot : robots) {
        robot_names.push_back(robot->robot_name);
    }
    return robot_names;
}

std::vector<std::string> SkillGraph::get_hand_names() const {
    std::vector<std::string> hand_names;
    for (const auto& robot : robots) {
        hand_names.push_back(robot->tool_string());
    }
    return hand_names;
}

SkillPtr SkillGraph::get_skill(const std::string &skill_name) const {
    Skill::Type type = Skill::from_string(skill_name);
    if (skill_map_.find(type) == skill_map_.end()) {
        throw std::runtime_error("Skill " + skill_name + " not found in skill graph");
    }
    return skill_map_.at(type);
}

RobotPtr SkillGraph::get_robot(const std::string &robot_name) const {
    for (const auto& robot : robots) {
        if (robot->robot_name == robot_name) {
            return robot;
        }
    }
    throw std::runtime_error("Robot " + robot_name + " not found in skill graph");
}


}
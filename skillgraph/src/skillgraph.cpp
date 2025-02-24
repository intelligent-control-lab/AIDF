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
                if (skill_config.isMember("param")) {
                    skill->set_param(skill_config["param"]);
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
                
                auto meta_skill = std::make_shared<MetaSkill>(meta_skill_name);
                
                // Get atomic skills that make up this meta skill
                const Json::Value& atomic_skills = meta_skill_config["atomicSkills"];
                for (const auto& atomic_skill : atomic_skills) {
                    Skill::Type type = Skill::from_string(atomic_skill.asString());

                    meta_skill->atomic_skills.push_back(type);
                }
                
                skill_map_[meta_skill->type] = meta_skill;
                meta_skills[meta_skill->type] = meta_skill->atomic_skills;
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
            std::string robot_name = robots_config["robotNames"][i].asString();
            std::vector<std::string> capabilities = robot_capabilities[robot_type];
            auto robot = std::make_shared<Robot>(robot_type, gripper_type, sensor_type, robot_name, capabilities);
             
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
        std::cout << "  Robot: " << robot->robot_name << "\n";
        std::cout << "    Type: " << robot->robot_name << "\n";
        std::cout << "    Tool: " << static_cast<int>(robot->tool) << "\n";
        std::cout << "    Capabilities:\n";
        for (const auto& cap : robot->capabilities) {
            std::cout << "      - " << cap << "\n";
        }
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
        for (const auto& atomic_type : atomic_skills) {
            auto atomic_skill = skill_map_[atomic_type];
            std::cout << "      - " << atomic_skill->name << "\n";
        }
    }

    // Print Environment Information
    if (env_) {
        std::cout << "\nEnvironment:\n";
        std::cout << "  Name: " << env_->name << "\n";
        std::cout << "  Type: " << static_cast<int>(env_->type) << "\n";
        std::cout << "  Objects: " << env_->objects_.size() << "\n";
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

}
#include "magblock/magblock_skills.hpp"
#include "magblock/magblock_skillgraph.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {

/**
 * @brief Construct a MagBlockSkillGraph from a configuration file.
 * @param config_file Path to the configuration file.
 */
MagBlockSkillGraph::MagBlockSkillGraph(const std::string &config_file) : SkillGraph(config_file)
{
    // Initialize robot transformation configurations using left arm as reference frame
    // Block coordinates are now defined relative to the left arm with specified offset
    
    RobotConfig left_arm_config;
    // Left arm serves as the origin, so no transformation needed except for the offset
    left_arm_config.x_origin_blocks = 0.0;  // Block origin at left arm with offset
    left_arm_config.y_origin_blocks = 0.0;
    left_arm_config.default_orientation_deg = {0, -180, 90};
    left_arm_config.block_to_robot_matrix << 1, 0, 0, 1;  // Identity transformation
    robot_configs_["left_arm"] = left_arm_config;

    RobotConfig right_arm_config;
    // Right arm transformation: RIGHT robot base frame coordinates
    // Block frame origin is offset from RIGHT robot base by:
    // - x=40cm (to the right of right robot base) 
    // - y=11cm (in front of right robot base)
    // - z=0cm (same height as right robot base)
    //
    // Coordinate directions:
    // - Moving right of right robot base is +X robot, but -X in block frame
    // - Moving forward from right robot base is +Y robot, and +Y in block frame 
    // - Moving up from right robot base is +Z robot, and +Z in block frame
    right_arm_config.x_origin_blocks = 16.0;  // 40cm / 2.5cm = 16 block units
    right_arm_config.y_origin_blocks = -4.4;  // -11cm / 2.5cm = -4.4 block units
    right_arm_config.default_orientation_deg = {0, -180, 90};
    right_arm_config.block_to_robot_matrix << -1, 0, 0, -1;  // Invert X, negate Y
    robot_configs_["right_arm"] = right_arm_config;

    RobotConfig center_arm_config;
    // Center arm transformation relative to left arm
    center_arm_config.x_origin_blocks = 0.0;
    center_arm_config.y_origin_blocks = 0.0;
    center_arm_config.default_orientation_deg = {0, -180, 90};
    center_arm_config.block_to_robot_matrix << 1, 0, 0, 1;
    robot_configs_["center_arm"] = center_arm_config;
}

/**
 * @brief Parse the environment configuration for the MagBlock skill graph.
 *
 * This method sets up the backend, MoveIt instance, and robot parameters from the JSON configuration,
 * exactly mirroring the LEGO implementation.
 * @param root_config Root JSON configuration.
 */
void MagBlockSkillGraph::parse_env(const Json::Value &root_config) {
    // First, use the base class implementation - exactly mirroring LEGO approach
    SkillGraph::parse_env(root_config);
 
    const Json::Value& env_config = root_config["environment"];
    const Json::Value& backend_config = env_config["backend"];
    std::string moveitConfigPkg = backend_config["moveitConfigPkg"].asString();
    
    std::string backend = backend_config["type"].asString();
    if (backend != "moveit") {
        throw std::runtime_error("Backend must be moveit for MagBlock assembly! " + backend);
    }

    // Create the moveit instance backend - exactly mirroring LEGO approach
    auto plan_instance_ = std::make_shared<MoveitInstance>(backend_config["moveit_group_name"].asString(), 
                            moveitConfigPkg);

    // Set the number of robots and their names - exactly mirroring LEGO approach
    plan_instance_->setNumberOfRobots(num_robots_);
    std::vector<std::string> robot_names;
    for (int i = 0; i < num_robots_; i++) {
        plan_instance_->setRobotDOF(i, robots[i]->getDOF()+1); // the moveit backend adds a dummy gripper dof
    }
    plan_instance_->setRobotNames(get_robot_names());
    if (backend_config.isMember("l1_vmax")) {
        plan_instance_->setVmax(backend_config["l1_vmax"].asDouble());
    }
    
    env_->setBackend(plan_instance_);
    
    log("MagBlock environment initialized with " + std::to_string(num_robots_) + " robots", LogLevel::INFO);
}

/**
 * @brief Check if we are at the target state.
 *
 * This method checks if all magblock assembly tasks have been completed, mirroring the LEGO approach.
 * @param state Current state.
 * @return True if at target.
 */
bool MagBlockSkillGraph::at_target(const State &state) {
    // Check if all assembly tasks are complete - following LEGO pattern
    return state.assembled_steps >= task_seq_->num_tasks();
}

/**
 * @brief Get feasible skills for the current state.
 *
 * This method determines which skills can be executed in the current state, mirroring the LEGO approach.
 * @param state Current state.
 * @return Vector of feasible skills.
 */
std::vector<SkillPtr> MagBlockSkillGraph::feasible_u(const State &state) {
    std::vector<SkillPtr> feasible_skills;
    
    // Debug: Check if skill_map_ is populated
    log("Debug: skill_map_ size: " + std::to_string(skill_map_.size()), LogLevel::INFO);
    for (const auto& [skill_type, skill] : skill_map_) {
        log("Debug: skill_map_ contains skill type: " + std::to_string(static_cast<int>(skill_type)) + " (" + skill->name + ")", LogLevel::INFO);
    }
    
    // Get the next task to execute - following LEGO pattern
    int assembled_steps = state.assembled_steps;
    if (assembled_steps >= task_seq_->num_tasks()) {
        log("All magblock assembly tasks completed", LogLevel::INFO);
        return feasible_skills;
    }
    
    TaskPtr current_task = task_seq_->get_task_at(assembled_steps);
    
    // Check if task is valid
    if (!current_task) {
        log("Invalid task at step " + std::to_string(assembled_steps), LogLevel::ERROR);
        return feasible_skills;
    }
    
    // Debug: Check current task
    log("Debug: Current task allowed skill types: " + std::to_string(current_task->post_condition->allowed_skill_type.size()), LogLevel::INFO);
    
    // Add allowed skills for this task - following LEGO pattern
    for (const auto& skill_type : current_task->post_condition->allowed_skill_type) {
        log("Debug: Looking for skill type: " + std::to_string(static_cast<int>(skill_type)), LogLevel::INFO);
        
        // Convert skill type to string and create AtomicSkill
        std::string skill_name;
        switch(skill_type) {
            case Skill::Type::Pick:
                skill_name = "Pick";
                break;
            case Skill::Type::PlaceTop:
                skill_name = "PlaceTop";
                break;
            case Skill::Type::PlaceBottom:
                skill_name = "PlaceBottom";
                break;
            case Skill::Type::Transit:
                skill_name = "Transit";
                break;
            case Skill::Type::PickAndPlace:
                skill_name = "PickAndPlace";
                break;
            default:
                skill_name = "Pick"; // Default fallback
                break;
        }
        
        // Create atomic skill with executor and post condition - following LEGO pattern
        auto atomic_skill = std::make_shared<AtomicSkill>(skill_name);
        auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(skill_type, env_->backend_);
        
        // Set post-condition from current task
        atomic_executor->set_post_condition(current_task->post_condition);
        atomic_skill->executor = atomic_executor;
        
        feasible_skills.push_back(atomic_skill);
        log("Debug: Added feasible skill: " + skill_name, LogLevel::INFO);
    }
    
    log("Found " + std::to_string(feasible_skills.size()) + " feasible skills for task " + 
        std::to_string(assembled_steps), LogLevel::INFO);
    
    return feasible_skills;
}

/**
 * @brief Get the next state given a skill.
 * @param state The current state.
 * @param gs The skill to apply.
 * @param next_state The resulting next state.
 * @param cost The cost of the transition.
 * @return True if successful.
 */
bool MagBlockSkillGraph::get_next_state(const State& state, SkillPtr gs, State &next_state, double &cost) {
    // Copy current state
    next_state = state;
    next_state.assembled_steps++;
    cost = 1.0; // Simple cost model
    
    // Apply the skill effects - this would involve updating object positions
    // For now, just increment the assembly steps
    log("Applied skill, advanced from step " + std::to_string(state.assembled_steps) + 
        " to " + std::to_string(next_state.assembled_steps), LogLevel::INFO);
    
    return true;
}

/**
 * @brief Check if a skill is feasible in the given state.
 * @param state The current state.
 * @param skill_config JSON configuration for the skill.
 * @param gs Output parameter for the feasible skill.
 * @return True if the skill is feasible.
 */
bool MagBlockSkillGraph::is_feasible(const State&state, Json::Value &skill_config, SkillPtr &gs) {
    std::string skillname = skill_config["type"].asString();
    Skill::Type skill_type = Skill::from_string(skillname);
    
    if (skill_type == Skill::Type::PickAndPlace) {
        // For PickAndPlace skills, create meta skill - following LEGO pattern
        auto base_skill = std::dynamic_pointer_cast<skillgraph::MetaSkill>(get_skill(skillname));
        skillgraph::MetaSkillPtr gs_meta = std::make_shared<skillgraph::MetaSkill>(*base_skill);
        gs = gs_meta;
        
        // Set robot for the skill
        int rid = skill_config.get("robot_id", 0).asInt();
        skillgraph::RobotPtr robot = robots[rid];
        gs_meta->set_robot({robot});
        
        // Set target object
        for (auto& obj : state.env_state.objects) {
            gs_meta->set_object(obj);
            break; // Use first available object for now
        }
        
        // Create executor for the meta skill
        auto meta_executor = std::make_shared<skillgraph::MetaSkillExecutor>(
            gs_meta->type, skillgraph::MetaSkill::ComposeType::Temporal, gs_meta->atomic_skills);
        
        // Set post-condition from skill config
        skillgraph::TaskParamPtr post_condition = std::make_shared<skillgraph::TaskParam>();
        post_condition->constraints_json = skill_config["target_location"];
        
        meta_executor->set_post_condition(post_condition);
        
        // Create atomic executors for each atomic skill in the meta skill - following LEGO pattern
        for (auto& atomic_skill : gs_meta->atomic_skills) {
            auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(atomic_skill->type, env_->backend_);
            atomic_executor->set_post_condition(post_condition);
            atomic_skill->executor = atomic_executor;
            meta_executor->add_atomic_executor(atomic_executor);
        }
        
        gs_meta->set_executor(meta_executor);
        
        return true;
    }
    
    // For other atomic skills (Pick, Place, Transit), create atomic skill
    auto atomic_skill = std::make_shared<AtomicSkill>(skillname);
    gs = atomic_skill;
    
    // Create and set executor with proper post condition - following LEGO pattern
    auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(atomic_skill->type, env_->backend_);
    
    // Set post-condition from skill config
    skillgraph::TaskParamPtr post_condition = std::make_shared<skillgraph::TaskParam>();
    post_condition->constraints_json = skill_config;
    
    atomic_executor->set_post_condition(post_condition);
    atomic_skill->executor = atomic_executor;
    
    return true;
}

/**
 * @brief Determine which robot to use for a given location.
 */
int MagBlockSkillGraph::determineRobotForLocation(double x, double y, double z) {
    // Simple workspace-based robot selection
    // This can be enhanced with reachability analysis from skillgraph
    
    // For now, use simple spatial partitioning:
    // Left arm: x < 10
    // Center arm: 10 <= x <= 20  
    // Right arm: x > 20
    
    if (x < 10) {
        return 0;  // left_arm
    } else if (x > 20) {
        return 2;  // right_arm
    } else {
        return 1;  // center_arm
    }
}

/**
 * @brief Transform block coordinates to robot coordinates.
 * Block coordinates are now defined relative to the left arm with a specific offset.
 * The offset is: x=11cm, y=-40cm, z=0cm in the left arm's frame.
 */
bool MagBlockSkillGraph::blockToRobotFrame(const std::string& robot_name, 
                                         double x_blocks, double y_blocks, double z_blocks,
                                         double& x_robot, double& y_robot, double& z_robot,
                                         double& rx_deg, double& ry_deg, double& rz_deg) {
    
    // Check if robot config exists
    auto config_it = robot_configs_.find(robot_name);
    if (config_it == robot_configs_.end()) {
        log("Robot config not found for: " + robot_name, LogLevel::ERROR);
        return false;
    }
    
    const RobotConfig& config = config_it->second;
    
    // For right arm, apply the proper transformation based on magblock_assembly_executor.cpp
    if (robot_name == "right_arm") {
        // Transform block frame coordinates to RIGHT robot base frame coordinates
        // Block frame origin is offset from RIGHT robot base by:
        // - x=40cm (to the right of right robot base) 
        // - y=11cm (in front of right robot base)
        // - z=0cm (same height as right robot base)
        //
        // Coordinate directions:
        // - Moving right of right robot base is +X robot, but -X in block frame
        // - Moving forward from right robot base is +Y robot, and +Y in block frame 
        // - Moving up from right robot base is +Z robot, and +Z in block frame
        //
        // Each block unit = 2.5cm = 0.025m
        
        double block_size = 0.025; // 2.5cm per block unit
        double block_frame_offset_x = 0.40; // 40cm offset to right of right robot base
        double block_frame_offset_y = 0.11; // 11cm offset in front of right robot base 
        double block_frame_offset_z = 0.0; // Same height as right robot base
        double table_height = 0.195; // Table height above ground
        double block_height_offset = 0.0125; // Half block height (2.5cm / 2) to place on table surface
        
        // Transform from block frame to right robot base frame
        // Note: X direction is inverted between block frame and robot frame
        x_robot = block_frame_offset_x - (x_blocks * block_size); // Inverted X direction
        y_robot = -(block_frame_offset_y + (y_blocks * block_size)); // Negative Y for right robot arm
        z_robot = table_height + block_frame_offset_z + (z_blocks * block_size) + block_height_offset; // Place on table surface
        
        // Set default orientation
        rx_deg = config.default_orientation_deg[0];
        ry_deg = config.default_orientation_deg[1];
        rz_deg = config.default_orientation_deg[2];
        
        log("Right arm block coords (" + std::to_string(x_blocks) + "," + 
            std::to_string(y_blocks) + "," + std::to_string(z_blocks) + 
            ") to robot coords (" + std::to_string(x_robot) + "," + 
            std::to_string(y_robot) + "," + std::to_string(z_robot) + ")", LogLevel::DEBUG);
        
        return true;
    }
    
    // For other robots, use the matrix transformation (simplified for now)
    Eigen::Vector2d block_pos(x_blocks, y_blocks);
    Eigen::Vector2d robot_pos = config.block_to_robot_matrix * block_pos;
    
    // Add origin offset
    x_robot = robot_pos(0) + config.x_origin_blocks;
    y_robot = robot_pos(1) + config.y_origin_blocks;
    z_robot = z_blocks * block_size_;  // Convert block units to meters
    
    // Set default orientation
    rx_deg = config.default_orientation_deg[0];
    ry_deg = config.default_orientation_deg[1];
    rz_deg = config.default_orientation_deg[2];
    
    log("Transformed block coords (" + std::to_string(x_blocks) + "," + 
        std::to_string(y_blocks) + "," + std::to_string(z_blocks) + 
        ") to robot coords (" + std::to_string(x_robot) + "," + 
        std::to_string(y_robot) + "," + std::to_string(z_robot) + ")", LogLevel::DEBUG);
    
    return true;
}

/**
 * @brief Load assembly task from JSON file.
 */
bool MagBlockSkillGraph::loadAssemblyTask(const std::string& task_file, const std::string& env_setup_file) {
    try {
        // Load assembly tasks
        std::ifstream task_stream(task_file);
        if (!task_stream.is_open()) {
            log("Failed to open task file: " + task_file, LogLevel::ERROR);
            return false;
        }
        task_stream >> assembly_tasks_;
        task_stream.close();
        
        // Load environment setup
        std::ifstream env_stream(env_setup_file);
        if (!env_stream.is_open()) {
            log("Failed to open env setup file: " + env_setup_file, LogLevel::ERROR);
            return false;
        }
        env_stream >> env_setup_;
        env_stream.close();
        
        // Create assembly sequence
        assembly_seq_ = std::make_shared<MagBlockAssemblySeq>(task_file);
        task_seq_ = assembly_seq_;  // For compatibility
        
        log("Successfully loaded magblock assembly task and environment", LogLevel::INFO);
        return true;
    } catch (const std::exception& e) {
        log("Exception loading assembly task: " + std::string(e.what()), LogLevel::ERROR);
        return false;
    }
}

/**
 * @brief Get the assembly sequence for trajectory planning.
 */
std::shared_ptr<MagBlockAssemblySeq> MagBlockSkillGraph::getAssemblySequence() const {
    return assembly_seq_;
}

/**
 * @brief Set the assembly sequence.
 */
void MagBlockSkillGraph::setAssemblySequence(std::shared_ptr<MagBlockAssemblySeq> assembly_seq) {
    assembly_seq_ = assembly_seq;
    task_seq_ = assembly_seq;  // For compatibility
}

/**
 * @brief Set environment configuration.
 */
void MagBlockSkillGraph::setEnvironmentConfig(const Json::Value& env_config) {
    env_setup_ = env_config;
}

/**
 * @brief Get optimal robot for given block coordinates.
 */
int MagBlockSkillGraph::getOptimalRobot(double x_blocks, double y_blocks, double z_blocks) {
    return determineRobotForLocation(x_blocks, y_blocks, z_blocks);
}

/**
 * @brief Transform block coordinates to robot frame - renamed for clarity.
 */
bool MagBlockSkillGraph::transformBlockToRobot(const std::string& robot_name, 
                                              double x_blocks, double y_blocks, double z_blocks,
                                              double& x_robot, double& y_robot, double& z_robot,
                                              double& rx_deg, double& ry_deg, double& rz_deg) {
    return blockToRobotFrame(robot_name, x_blocks, y_blocks, z_blocks,
                            x_robot, y_robot, z_robot, rx_deg, ry_deg, rz_deg);
}

/**
 * @brief Get pick pose for a task from skillgraph logic.
 */
bool MagBlockSkillGraph::getPickPose(TaskPtr task, double& x, double& y, double& z) {
    // Implementation: Get storage/source location for the block
    // For now, return false to use default table pick location
    // This can be enhanced with actual storage location logic
    return false;
}

} // namespace skillgraph

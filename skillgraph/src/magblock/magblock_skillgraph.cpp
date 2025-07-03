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
    
    // Get the next task to execute - following LEGO pattern
    int assembled_steps = state.assembled_steps;
    if (assembled_steps >= task_seq_->num_tasks()) {
        log("All magblock assembly tasks completed", LogLevel::INFO);
        return feasible_skills;
    }
    
    TaskPtr current_task = task_seq_->get_task_at(assembled_steps);
    
    // Add allowed skills for this task - following LEGO pattern
    for (const auto& skill_type : current_task->post_condition->allowed_skill_type) {
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
        
        SkillPtr skill = std::make_shared<AtomicSkill>(skill_name);
        feasible_skills.push_back(skill);
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
        gs_meta->set_executor(meta_executor);
        
        return true;
    }
    
    // For other skills, use simple atomic skill creation
    gs = std::make_shared<AtomicSkill>(skillname);
    return true;
}

/**
 * @brief Determine which robot to use for a given location.
 * @param x World x coordinate.
 * @param y World y coordinate.
 * @param z World z coordinate.
 * @return Robot ID (0, 1, or 2).
 */
int MagBlockSkillGraph::determineRobotForLocation(double x, double y, double z) {
    // Simple spatial partitioning for three arms
    // This can be refined based on actual workspace geometry
    if (x < -0.3) {
        return 0; // Left robot
    } else if (x > 0.3) {
        return 2; // Right robot
    } else {
        return 1; // Center robot
    }
}

} // namespace skillgraph

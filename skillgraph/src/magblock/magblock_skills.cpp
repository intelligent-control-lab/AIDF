#include "magblock/magblock_skills.hpp"
#include "Utils/Logger.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>

namespace skillgraph {

/**
 * @brief Construct a MagBlockSkillExecutor for a specific skill type and backend.
 * @param type The skill type.
 * @param backend Shared pointer to the PlanInstance backend.
 */
MagBlockSkillExecutor::MagBlockSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend) 
    : SkillExecutor(type), backend_(backend), skill_type_(type) {
    
    // Initialize MoveitControl for robot motion - mirroring LEGO approach exactly
    auto moveit_backend = std::dynamic_pointer_cast<MoveitInstance>(backend_);
    controller_ = std::make_shared<MoveitControl>(moveit_backend, true);
    
    log("MagBlockSkillExecutor initialized for skill type: " + std::to_string(static_cast<int>(type)), LogLevel::INFO);
}

/**
 * @brief Execute the MagBlock skill on the current state.
 *
 * This method executes the skill logic using MoveIt2 motion planning, exactly mirroring the LEGO approach.
 * @param current_state The current state to execute on.
 * @return True if execution was successful, false otherwise.
 */
bool MagBlockSkillExecutor::execute(State &current_state) {
    // Check for valid post condition - following LEGO pattern exactly
    if (post_condition == nullptr) {
        log("Post condition is null", LogLevel::ERROR);
        return false;
    }

    log("Executing MagBlock skill: " + std::to_string(static_cast<int>(skill_type_)), LogLevel::INFO);

    // Handle specific skill types for magnetic blocks
    if (skill_type_ == Skill::Type::Pick) {
        return execute_pick_skill(current_state);
    } else if (skill_type_ == Skill::Type::PlaceTop || skill_type_ == Skill::Type::PlaceBottom) {
        return execute_place_skill(current_state);
    } else if (skill_type_ == Skill::Type::Transit) {
        return execute_transit_skill(current_state);
    } else if (skill_type_ == Skill::Type::PickAndPlace) {
        return execute_pick_and_place_skill(current_state);
    } else {
        // Use MoveitControl for other skills - exactly mirroring LEGO approach
        log("Using MoveitControl for general skill execution", LogLevel::INFO);
        bool success = controller_->move(post_condition, planned_trajectory_);
        
        // Update state after successful execution - following LEGO pattern
        if (success) {
            current_state.robot_states = post_condition->target_state.robot_states;
            current_state.env_state = post_condition->target_state.env_state;
        }
        
        // Brief pause for stability - following LEGO pattern  
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return success;
    }
}

bool MagBlockSkillExecutor::execute_pick_skill(State &current_state) {
    log("Executing pick skill for magnetic block", LogLevel::INFO);
    
    // Extract pick parameters from constraints - following LEGO pattern
    const Json::Value& constraints = post_condition->constraints_json;
    
    double approach_distance = constraints.get("approach_distance", 0.1).asDouble();
    double grip_width = constraints.get("grip_width", 0.08).asDouble();
    
    log("Pick parameters - approach_distance: " + std::to_string(approach_distance) + 
        ", grip_width: " + std::to_string(grip_width), LogLevel::INFO);
    
    // Use MoveitControl for motion execution - exactly mirroring LEGO approach
    bool success = controller_->move(post_condition, planned_trajectory_);
    
    if (success) {
        // Update state to reflect picked object - following LEGO pattern
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        
        log("Pick skill executed successfully", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        log("Pick skill execution failed", LogLevel::ERROR);
    }
    
    return success;
}

bool MagBlockSkillExecutor::execute_place_skill(State &current_state) {
    log("Executing place skill for magnetic block", LogLevel::INFO);
    
    // Extract place parameters from constraints - following LEGO pattern
    const Json::Value& constraints = post_condition->constraints_json;
    
    double approach_distance = constraints.get("approach_distance", 0.1).asDouble();
    double release_distance = constraints.get("release_distance", 0.002).asDouble();
    
    log("Place parameters - approach_distance: " + std::to_string(approach_distance) + 
        ", release_distance: " + std::to_string(release_distance), LogLevel::INFO);
    
    // Use MoveitControl for motion execution - exactly mirroring LEGO approach
    bool success = controller_->move(post_condition, planned_trajectory_);
    
    if (success) {
        // Update state to reflect placed object - following LEGO pattern
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        
        log("Place skill executed successfully", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        log("Place skill execution failed", LogLevel::ERROR);
    }
    
    return success;
}

bool MagBlockSkillExecutor::execute_transit_skill(State &current_state) {
    log("Executing transit skill for magnetic block", LogLevel::INFO);
    
    // Extract transit parameters from constraints - following LEGO pattern
    const Json::Value& constraints = post_condition->constraints_json;
    
    double speed_scale = constraints.get("speed_scale", 0.5).asDouble();
    
    log("Transit parameters - speed_scale: " + std::to_string(speed_scale), LogLevel::INFO);
    
    // Use MoveitControl for motion execution - exactly mirroring LEGO approach
    bool success = controller_->move(post_condition, planned_trajectory_);
    
    if (success) {
        // Update state after transit - following LEGO pattern
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        
        log("Transit skill executed successfully", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    } else {
        log("Transit skill execution failed", LogLevel::ERROR);
    }
    
    return success;
}

bool MagBlockSkillExecutor::execute_pick_and_place_skill(State &current_state) {
    log("Executing pick and place skill for magnetic block", LogLevel::INFO);
    
    // This combines pick, transit, and place operations - following LEGO pattern
    // Use MoveitControl for the composite motion
    bool success = controller_->move(post_condition, planned_trajectory_);
    
    if (success) {
        // Update state after complete pick and place - following LEGO pattern
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        
        log("Pick and place skill executed successfully", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
        log("Pick and place skill execution failed", LogLevel::ERROR);
    }
    
    return success;
}

} // namespace skillgraph

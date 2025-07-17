#include "magblock/magblock_skills.hpp"
#include "magblock/magblock_algorithms.hpp"
#include "Utils/Logger.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <thread>
#include <chrono>
#include <cmath>
#include <tuple>
#include <fstream>
#include <jsoncpp/json/json.h>

namespace skillgraph {

/**
 * @brief Construct a MagBlockSkillExecutor for a specific skill type and backend.
 * @param type The skill type.
 * @param backend Shared pointer to the PlanInstance backend.
 */
MagBlockSkillExecutor::MagBlockSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend) 
    : SkillExecutor(type), backend_(backend), skill_type_(type) {
    
    // Initialize MoveitControl for robot motion
    auto moveit_backend = std::dynamic_pointer_cast<MoveitInstance>(backend_);
    controller_ = std::make_shared<MoveitControl>(moveit_backend, true);
    
    log("MagBlockSkillExecutor initialized for skill type: " + std::to_string(static_cast<int>(type)), LogLevel::INFO);
}

/**
 * @brief Execute the MagBlock skill on the current state.
 *
 * This method executes the skill logic using MoveIt2 motion planning
 * @param current_state The current state to execute on.
 * @return True if execution was successful, false otherwise.
 */
bool MagBlockSkillExecutor::execute(State &current_state) {
    // Check for valid post condition
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
        // Use MoveitControl for other skills
        log("Using MoveitControl for general skill execution", LogLevel::INFO);
        bool success = controller_->move(post_condition, planned_trajectory_);
        
        // Update state after successful execution
        if (success) {
            current_state.robot_states = post_condition->target_state.robot_states;
            current_state.env_state = post_condition->target_state.env_state;
        }
        
        // Brief pause for stability
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return success;
    }
}

bool MagBlockSkillExecutor::execute_pick_skill(State &current_state) {
    log("Executing pick skill for magnetic block", LogLevel::INFO);
    
    // Extract pick parameters from constraints
    const Json::Value& constraints = post_condition->constraints_json;
    
    // Get block name from constraints or generate it
    std::string block_name = constraints.get("object", "").asString();
    if (block_name.empty()) {
        block_name = "b" + std::to_string(current_state.assembled_steps + 1);
    }
    
    // Get pick position from environment file (not constraints)
    double x_blocks, y_blocks, z_blocks;
    if (!getBlockPickPosition(block_name, x_blocks, y_blocks, z_blocks)) {
        log("Failed to get pick position for " + block_name, LogLevel::ERROR);
        return false;
    }
    
    // Get press_face from constraints (for orientation calculation)
    int press_face = constraints.get("press_face", 0).asInt();
    int gripper_ori = constraints.get("gripper_ori", 0).asInt();
    
    log("Pick skill - block " + block_name + " coordinates: (" + std::to_string(x_blocks) + ", " + 
        std::to_string(y_blocks) + ", " + std::to_string(z_blocks) + 
        "), press_face: " + std::to_string(press_face), LogLevel::INFO);
    
    // Transform from skillgraph to robot coordinates
    double x_robot, y_robot, z_robot, rx, ry, rz;
    transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
    
    // Determine robot name (default to right arm for magblock tasks)
    std::string robot_name = "right_arm";
    
    // Calculate pick orientation using IK-based approach
    double pick_thetax = 0.0;   // Always approach from above
    double pick_thetay = 180.0; // Gripper pointing down
    double pick_thetaz = findOptimalThetaZ(press_face);
    
    // Create pick pose (table surface for safe pick)
    geometry_msgs::msg::Pose pick_pose = createPoseWithOrientation(x_robot, y_robot, z_robot, 
                                                                  pick_thetax, pick_thetay, pick_thetaz);
    
    // Create place pose (this is simplified - in practice would come from task constraints)
    geometry_msgs::msg::Pose place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
    
    // Plan and execute pick trajectory using joint-space IK planning
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::string object_name = "block_" + std::to_string(current_state.assembled_steps);
    
    bool success = planPickPlaceTrajectory(this->getMoveitInstance(), robot_name, pick_pose, place_pose, object_name, press_face, trajectories);
    
    if (success) {
        // Update state to reflect picked object
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
    
    // Extract place parameters from constraints
    const Json::Value& constraints = post_condition->constraints_json;
    
    // Get block name from constraints or generate it
    std::string block_name = constraints.get("object", "").asString();
    if (block_name.empty()) {
        block_name = "b" + std::to_string(current_state.assembled_steps + 1);
    }
    
    // Get place position from assembly file (not constraints)
    double x_blocks, y_blocks, z_blocks;
    int press_face, gripper_ori;
    if (!getBlockPlacePosition(block_name, x_blocks, y_blocks, z_blocks, press_face, gripper_ori)) {
        log("Failed to get place position for " + block_name, LogLevel::ERROR);
        return false;
    }
    
    log("Place skill - block " + block_name + " coordinates: (" + std::to_string(x_blocks) + ", " + 
        std::to_string(y_blocks) + ", " + std::to_string(z_blocks) + 
        "), press_face: " + std::to_string(press_face), LogLevel::INFO);
    
    // Transform from skillgraph to robot coordinates
    double x_robot, y_robot, z_robot, rx, ry, rz;
    transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
    
    // Determine robot name
    std::string robot_name = "right_arm";
    
    // Calculate place orientation using IK-based approach
    double pick_thetaz = findOptimalThetaZ(press_face);
    geometry_msgs::msg::Pose place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
    
    // For place skill, we assume object is already in hand, so create approach pose
    geometry_msgs::msg::Pose approach_pose = createPlaceApproachPose(place_pose, press_face, 0.1);
    
    // Plan and execute place trajectory using joint-space IK planning
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::string object_name = "block_" + std::to_string(current_state.assembled_steps);
    
    bool success = planPickPlaceTrajectory(this->getMoveitInstance(), robot_name, approach_pose, place_pose, object_name, press_face, trajectories);
    
    if (success) {
        // Update state to reflect placed object
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        current_state.assembled_steps++; // Increment assembly progress
        
        log("Place skill executed successfully", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        log("Place skill execution failed", LogLevel::ERROR);
    }
    
    return success;
}

bool MagBlockSkillExecutor::execute_transit_skill(State &current_state) {
    log("Executing transit skill for magnetic block", LogLevel::INFO);
    
    // Extract transit parameters from constraints
    const Json::Value& constraints = post_condition->constraints_json;
    
    double speed_scale = constraints.get("speed_scale", 0.5).asDouble();
    
    log("Transit parameters - speed_scale: " + std::to_string(speed_scale), LogLevel::INFO);
    
    // Use MoveitControl for motion execution
    bool success = controller_->move(post_condition, planned_trajectory_);
    
    if (success) {
        // Update state after transit
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
    
    // Extract parameters from constraints
    const Json::Value& constraints = post_condition->constraints_json;
    
    // Get block name from constraints or generate it
    std::string block_name = constraints.get("object", "").asString();
    if (block_name.empty()) {
        block_name = "b" + std::to_string(current_state.assembled_steps + 1);
    }
    
    // Get pick position from environment file
    double pick_x_blocks, pick_y_blocks, pick_z_blocks;
    if (!getBlockPickPosition(block_name, pick_x_blocks, pick_y_blocks, pick_z_blocks)) {
        log("Failed to get pick position for " + block_name, LogLevel::ERROR);
        return false;
    }
    
    // Get place position from assembly file
    double place_x_blocks, place_y_blocks, place_z_blocks;
    int press_face, gripper_ori;
    if (!getBlockPlacePosition(block_name, place_x_blocks, place_y_blocks, place_z_blocks, press_face, gripper_ori)) {
        log("Failed to get place position for " + block_name, LogLevel::ERROR);
        return false;
    }
    
    log("Pick and place skill - block " + block_name + 
        " pick: (" + std::to_string(pick_x_blocks) + ", " + std::to_string(pick_y_blocks) + ", " + std::to_string(pick_z_blocks) + ") " +
        " place: (" + std::to_string(place_x_blocks) + ", " + std::to_string(place_y_blocks) + ", " + std::to_string(place_z_blocks) + ") " +
        " press_face: " + std::to_string(press_face), LogLevel::INFO);
    
    // Transform coordinates to robot frame
    double pick_x_robot, pick_y_robot, pick_z_robot, pick_rx, pick_ry, pick_rz;
    double place_x_robot, place_y_robot, place_z_robot, place_rx, place_ry, place_rz;
    
    transformSkillgraphToRobot(pick_x_blocks, pick_y_blocks, pick_z_blocks, pick_x_robot, pick_y_robot, pick_z_robot, pick_rx, pick_ry, pick_rz);
    transformSkillgraphToRobot(place_x_blocks, place_y_blocks, place_z_blocks, place_x_robot, place_y_robot, place_z_robot, place_rx, place_ry, place_rz);
    
    // Determine robot name
    std::string robot_name = "right_arm";
    
    // Calculate optimal pick orientation using IK-based approach
    double pick_thetax = 0.0;   // Always approach from above
    double pick_thetay = 180.0; // Gripper pointing down
    double pick_thetaz = findOptimalThetaZ(press_face);
    
    // Create pick and place poses
    geometry_msgs::msg::Pose pick_pose = createPoseWithOrientation(pick_x_robot, pick_y_robot, pick_z_robot, 
                                                                  pick_thetax, pick_thetay, pick_thetaz);
    geometry_msgs::msg::Pose place_pose = createPlacePose(place_x_robot, place_y_robot, place_z_robot, press_face, gripper_ori, pick_thetaz);
    
    // Plan and execute complete pick and place trajectory using joint-space IK planning
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::string object_name = "block_" + std::to_string(current_state.assembled_steps);
    
    bool success = planPickPlaceTrajectory(this->getMoveitInstance(), robot_name, pick_pose, place_pose, object_name, press_face, trajectories);
    
    if (success) {
        // Update state after complete pick and place
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        current_state.assembled_steps++; // Increment assembly progress
        
        log("Pick and place skill executed successfully", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
        log("Pick and place skill execution failed", LogLevel::ERROR);
    }
    
    return success;
}

} // namespace skillgraph

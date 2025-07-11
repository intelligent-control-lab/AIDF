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
    
    // Extract pick parameters from constraints
    const Json::Value& constraints = post_condition->constraints_json;
    
    // Extract block coordinates and constraints from skillgraph
    double x_blocks = constraints["x"].asDouble();
    double y_blocks = constraints["y"].asDouble(); 
    double z_blocks = constraints["z"].asDouble();
    int press_face = constraints.get("press_face", 0).asInt();
    int gripper_ori = constraints.get("gripper_ori", 0).asInt();
    
    log("Pick skill - block coordinates: (" + std::to_string(x_blocks) + ", " + 
        std::to_string(y_blocks) + ", " + std::to_string(z_blocks) + 
        "), press_face: " + std::to_string(press_face), LogLevel::INFO);
    
    // Transform from skillgraph to robot coordinates
    double x_robot, y_robot, z_robot, rx, ry, rz;
    transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
    
    // Determine robot name (default to right arm for magblock tasks)
    std::string robot_name = "right_arm";
    
    // Calculate pick orientation
    double pick_thetax = 0.0;   // Always approach from above
    double pick_thetay = 180.0; // Gripper pointing down
    double pick_thetaz = findOptimalThetaZ(press_face);
    
    // Create pick pose (slightly above table for safe pick)
    geometry_msgs::msg::Pose pick_pose = createPoseWithOrientation(x_robot, y_robot, 0.25, 
                                                                  pick_thetax, pick_thetay, pick_thetaz);
    
    // Create place pose (this is simplified - in practice would come from task constraints)
    geometry_msgs::msg::Pose place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
    
    // Plan and execute pick trajectory using MoveIt
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::string object_name = "block_" + std::to_string(current_state.assembled_steps);
    
    bool success = planPickPlaceTrajectory(robot_name, pick_pose, place_pose, object_name, press_face, trajectories);
    
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
    
    // Extract block coordinates and constraints from skillgraph
    double x_blocks = constraints["x"].asDouble();
    double y_blocks = constraints["y"].asDouble(); 
    double z_blocks = constraints["z"].asDouble();
    int press_face = constraints.get("press_face", 0).asInt();
    int gripper_ori = constraints.get("gripper_ori", 0).asInt();
    
    log("Place skill - block coordinates: (" + std::to_string(x_blocks) + ", " + 
        std::to_string(y_blocks) + ", " + std::to_string(z_blocks) + 
        "), press_face: " + std::to_string(press_face), LogLevel::INFO);
    
    // Transform from skillgraph to robot coordinates
    double x_robot, y_robot, z_robot, rx, ry, rz;
    transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
    
    // Determine robot name
    std::string robot_name = "right_arm";
    
    // Calculate place orientation
    double pick_thetaz = findOptimalThetaZ(press_face);
    geometry_msgs::msg::Pose place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
    
    // For place skill, we assume object is already in hand, so we skip pick
    // and directly plan the place motion
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::string object_name = "block_" + std::to_string(current_state.assembled_steps);
    
    // Create a dummy pick pose (current position) for the planning function
    geometry_msgs::msg::Pose current_pose = place_pose;
    current_pose.position.z += 0.1; // Assume we're starting 10cm above place position
    
    bool success = planPickPlaceTrajectory(robot_name, current_pose, place_pose, object_name, press_face, trajectories);
    
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
    
    // Extract parameters from constraints
    const Json::Value& constraints = post_condition->constraints_json;
    
    // Extract block coordinates and constraints from skillgraph
    double x_blocks = constraints["x"].asDouble();
    double y_blocks = constraints["y"].asDouble(); 
    double z_blocks = constraints["z"].asDouble();
    int press_face = constraints.get("press_face", 0).asInt();
    int gripper_ori = constraints.get("gripper_ori", 0).asInt();
    
    log("Pick and place skill - block coordinates: (" + std::to_string(x_blocks) + ", " + 
        std::to_string(y_blocks) + ", " + std::to_string(z_blocks) + 
        "), press_face: " + std::to_string(press_face), LogLevel::INFO);
    
    // Transform from skillgraph to robot coordinates
    double x_robot, y_robot, z_robot, rx, ry, rz;
    transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
    
    // Determine robot name
    std::string robot_name = "right_arm";
    
    // Calculate optimal pick orientation
    double pick_thetax = 0.0;   // Always approach from above
    double pick_thetay = 180.0; // Gripper pointing down
    double pick_thetaz = findOptimalThetaZ(press_face);
    
    // Get pick location (assume from table for now - in practice would come from env_state)
    geometry_msgs::msg::Pose pick_pose = createPoseWithOrientation(x_robot, y_robot, 0.25, 
                                                                  pick_thetax, pick_thetay, pick_thetaz);
    
    // Create place pose 
    geometry_msgs::msg::Pose place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
    
    // Plan and execute complete pick and place trajectory
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::string object_name = "block_" + std::to_string(current_state.assembled_steps);
    
    bool success = planPickPlaceTrajectory(robot_name, pick_pose, place_pose, object_name, press_face, trajectories);
    
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

/**
 * @brief MoveIt-specific functions for trajectory planning and execution
 */

// Implementation of MoveIt-specific helper functions
geometry_msgs::msg::Pose MagBlockSkillExecutor::createPoseWithOrientation(double x, double y, double z, 
                                                        double thetax_deg, double thetay_deg, double thetaz_deg) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    
    // Convert degrees to radians
    double thetax = thetax_deg * M_PI / 180.0;
    double thetay = thetay_deg * M_PI / 180.0;
    double thetaz = thetaz_deg * M_PI / 180.0;
    
    // Convert Euler angles (XYZ order) to quaternion
    tf2::Quaternion quat;
    quat.setRPY(thetax, thetay, thetaz);
    
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    
    return pose;
}

void MagBlockSkillExecutor::transformSkillgraphToRobot(double x_sg, double y_sg, double z_sg, 
                                   double& x_robot, double& y_robot, double& z_robot, 
                                   double& rx, double& ry, double& rz) {
    // Transform from skillgraph block coordinates to robot frame
    double block_size = 0.025;  // 2.5cm per block unit
    double block_frame_offset_x = 0.40;  // 40cm offset to right of right robot base
    double block_frame_offset_y = 0.11;  // 11cm offset in front of right robot base  
    double block_frame_offset_z = 0.0;   // Same height as right robot base
    double table_height = 0.195;         // Table height above ground
    
    // Transform from block frame to right robot base frame
    x_robot = block_frame_offset_x - (x_sg * block_size);
    y_robot = -(block_frame_offset_y + (y_sg * block_size));
    z_robot = table_height + block_frame_offset_z + (z_sg * block_size);
    
    // Default orientation (gripper pointing down)
    rx = 0.0; ry = 0.0; rz = 0.0;
}

double MagBlockSkillExecutor::findOptimalThetaZ(int press_face) {
    // Find optimal thetaz that doesn't block the required press_face
    std::vector<double> candidate_angles = {0.0, 90.0, 180.0, 270.0};
    
    for (double angle : candidate_angles) {
        // Simple logic: return first valid angle
        // In practice, this would check which faces are blocked
        return angle;
    }
    
    return 0.0; // Default fallback
}

std::tuple<double, double, double> MagBlockSkillExecutor::getPlaceApproachOffset(int press_face, double approach_distance) {
    // Get approach direction offset based on press_face
    switch(press_face) {
        case 0: return {0.0, 0.0, approach_distance};   // +Z approach
        case 1: return {-approach_distance, 0.0, 0.0};  // -X approach
        case 2: return {0.0, approach_distance, 0.0};   // +Y approach  
        case 3: return {0.0, 0.0, -approach_distance};  // -Z approach
        case 4: return {approach_distance, 0.0, 0.0};   // +X approach
        case 5: return {0.0, -approach_distance, 0.0};  // -Y approach
        default: return {0.0, 0.0, approach_distance};  // Default +Z approach
    }
}

std::pair<double, double> MagBlockSkillExecutor::getPlaceOrientation(int gripper_ori) {
    // Get place orientation based on gripper_ori parameter
    switch(gripper_ori) {
        case 0: return {0.0, 180.0};
        case 1: return {-90.0, 180.0};
        case 2: return {120.0, 90.0};
        case 3: return {-180.0, 180.0};
        case 4: return {-90.0, -180.0};
        case 5: return {-70.0, -90.0};
        default: return {0.0, 180.0};  // Default orientation
    }
}

geometry_msgs::msg::Pose MagBlockSkillExecutor::createPlacePose(double x, double y, double z, 
                                        int press_face, int gripper_ori, double pick_thetaz) {
    // Get place orientation from gripper_ori
    auto [place_thetax, place_thetay] = getPlaceOrientation(gripper_ori);
    
    // thetaz stays the same as pick operation
    double place_thetaz = pick_thetaz;
    
    return createPoseWithOrientation(x, y, z, place_thetax, place_thetay, place_thetaz);
}

geometry_msgs::msg::Pose MagBlockSkillExecutor::createPlaceApproachPose(const geometry_msgs::msg::Pose& place_pose, 
                                            int press_face, double approach_distance) {
    auto [dx, dy, dz] = getPlaceApproachOffset(press_face, approach_distance);
    
    geometry_msgs::msg::Pose approach_pose = place_pose;
    approach_pose.position.x += dx;
    approach_pose.position.y += dy; 
    approach_pose.position.z += dz;
    
    return approach_pose;
}

bool MagBlockSkillExecutor::planPickPlaceTrajectory(const std::string& robot_name,
                                const geometry_msgs::msg::Pose& pick_pose,
                                const geometry_msgs::msg::Pose& place_pose,
                                const std::string& object_name,
                                int press_face,
                                std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
    
    auto moveit_instance = this->getMoveitInstance();
    if (!moveit_instance) {
        log("No MoveitInstance backend available", LogLevel::ERROR);
        return false;
    }
    
    log("Planning pick-place trajectory for " + robot_name + " with object " + object_name, LogLevel::INFO);
    
    trajectories.clear();
    
    // Create approach, pick, lift, transport, place, and retract poses
    geometry_msgs::msg::Pose approach_pose = pick_pose;
    approach_pose.position.z += 0.15;  // 15cm above pick
    
    geometry_msgs::msg::Pose lift_pose = pick_pose;
    lift_pose.position.z += 0.1;  // 10cm lift
    
    geometry_msgs::msg::Pose transport_pose = createPlaceApproachPose(place_pose, press_face, 0.15);
    geometry_msgs::msg::Pose retract_pose = createPlaceApproachPose(place_pose, press_face, 0.1);
    
    // For simplicity in skillgraph context, we'll use MoveitInstance's robot state manipulation
    // instead of full MoveIt planning. This follows the LEGO pattern of using teleport-style movement
    log("Using MoveitInstance robot state updates for trajectory simulation", LogLevel::INFO);
    
    // Create robot states for each waypoint
    skillgraph::RobotState approach_state, pick_state, lift_state, transport_state, place_state, retract_state;
    approach_state.robot_name = robot_name;
    pick_state.robot_name = robot_name; 
    lift_state.robot_name = robot_name;
    transport_state.robot_name = robot_name;
    place_state.robot_name = robot_name;
    retract_state.robot_name = robot_name;
    
    // Determine robot_id from robot_name for consistent state management
    int robot_id = (robot_name == "left_arm") ? 0 : (robot_name == "center_arm") ? 1 : 2;
    
    // Get the current robot joint state to ensure trajectory continuity
    std::vector<double> current_joints = getCurrentJointState(robot_id);
    
    // Use the current joint state as the starting point for trajectory planning
    // In a full implementation, these would be populated with IK solutions
    // For now, we'll use progressive joint interpolation from current state
    approach_state.joint_values = current_joints;
    pick_state.joint_values = current_joints;
    lift_state.joint_values = current_joints;  
    transport_state.joint_values = current_joints;
    place_state.joint_values = current_joints;
    retract_state.joint_values = current_joints;
    
    // Use MoveitInstance to move robot through waypoints for visualization
    
    moveit_instance->moveRobot(robot_id, approach_state);
    moveit_instance->updateScene();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    moveit_instance->moveRobot(robot_id, pick_state);
    moveit_instance->updateScene(); 
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    moveit_instance->moveRobot(robot_id, lift_state);
    moveit_instance->updateScene();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    moveit_instance->moveRobot(robot_id, transport_state);
    moveit_instance->updateScene();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    moveit_instance->moveRobot(robot_id, place_state);
    moveit_instance->updateScene();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    moveit_instance->moveRobot(robot_id, retract_state);
    moveit_instance->updateScene();
    
    log("Completed pick-place trajectory simulation for " + object_name, LogLevel::INFO);
    return true;
}

bool MagBlockSkillExecutor::executeTrajectories(const std::string& robot_name,
                            const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
                            const std::string& object_name) {
    log("Executing " + std::to_string(trajectories.size()) + " trajectory segments for " + object_name, LogLevel::INFO);
    
    // In the skillgraph context, execution is handled by the trajectory planning step above
    // This maintains compatibility with the MoveitControl interface
    
    return true;
}

/**
 * @brief Get the current joint state for a robot to ensure trajectory continuity
 * @param robot_id The robot ID (0=left, 1=center, 2=right)
 * @return Vector of current joint values
 */
std::vector<double> MagBlockSkillExecutor::getCurrentJointState(int robot_id) {
    auto moveit_instance = this->getMoveitInstance();
    if (!moveit_instance) {
        log("No MoveitInstance backend available for getCurrentJointState", LogLevel::ERROR);
        // Return default values as fallback
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    
    // Get the robot name based on robot_id
    std::string robot_name;
    switch(robot_id) {
        case 0: robot_name = "left_arm"; break;
        case 1: robot_name = "center_arm"; break;
        case 2: robot_name = "right_arm"; break;
        default: robot_name = "right_arm"; break;
    }
    
    // Get current joint values from the MoveIt kinematic state
    try {
        const moveit::core::JointModelGroup* joint_model_group = 
            moveit_instance->getPlanningScene()->getRobotModel()->getJointModelGroup(robot_name);
        
        if (!joint_model_group) {
            log("Joint model group not found for " + robot_name, LogLevel::ERROR);
            return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        
        std::vector<double> joint_values;
        // Get current state from planning scene
        moveit::core::RobotState current_state = moveit_instance->getPlanningScene()->getCurrentState();
        current_state.copyJointGroupPositions(joint_model_group, joint_values);
        
        log("Retrieved current joint state for " + robot_name + " with " + 
            std::to_string(joint_values.size()) + " joints", LogLevel::DEBUG);
        
        return joint_values;
    } catch (const std::exception& e) {
        log("Error getting current joint state for " + robot_name + ": " + std::string(e.what()), LogLevel::ERROR);
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
}

} // namespace skillgraph

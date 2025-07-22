#include "magblock/magblock_algorithms.hpp"
#include "Utils/PathUtils.hpp"
using skillgraph::utils::PathResolver;
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

static Json::Value task_config_;
static bool task_config_loaded_ = false;

void loadTaskConfig() {
    if (task_config_loaded_) return;

    std::string config_path = PathResolver::resolvePath("config/mag_block_tasks/skillgraph.json");

    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        log("Failed to open task config: " + config_path, LogLevel::ERROR);
        return;
    }

    config_file >> task_config_;
    task_config_loaded_ = true;
}

namespace skillgraph {

/**
 * @brief MagBlock-specific algorithms and helper functions
 * These functions provide utility operations for magnetic block manipulation,
 * including coordinate transformations, pose generation, and trajectory planning.
 */

// Static configuration for robot properties
static Json::Value robot_properties_;
static bool properties_loaded = false;

/**
 * @brief Load robot properties configuration from JSON file
 */
void loadRobotProperties() {
    if (properties_loaded) return;
    
    std::string config_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/robot_properties.json";
    // loadTaskConfig();
    // std::string config_path = PathResolver::resolvePath(task_config_["environment"]["robot_properties"].asString());
    
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        log("Failed to open robot properties file: " + config_path, LogLevel::ERROR);
        log("Using default robot properties", LogLevel::WARN);
        
        // Set default properties for right_arm
        robot_properties_["robots"]["right_arm"]["block_frame_transform"]["offset_x"] = 0.40;
        robot_properties_["robots"]["right_arm"]["block_frame_transform"]["offset_y"] = 0.11;
        robot_properties_["robots"]["right_arm"]["block_frame_transform"]["offset_z"] = 0.0;
        robot_properties_["robots"]["right_arm"]["block_frame_transform"]["x_direction"] = -1.0;
        robot_properties_["robots"]["right_arm"]["block_frame_transform"]["y_direction"] = -1.0;
        robot_properties_["robots"]["right_arm"]["block_frame_transform"]["z_direction"] = 1.0;
        
        robot_properties_["workspace"]["block_size"] = 0.025;
        robot_properties_["workspace"]["table_height"] = 0.159;
        robot_properties_["workspace"]["block_height_offset"] = 0.0125;
        
        properties_loaded = true;
        return;
    }
    
    try {
        config_file >> robot_properties_;
        log("Robot properties loaded successfully from: " + config_path, LogLevel::INFO);
        properties_loaded = true;
    } catch (const std::exception& e) {
        log("Failed to parse robot properties JSON: " + std::string(e.what()), LogLevel::ERROR);
        log("Using default robot properties", LogLevel::WARN);
    }
    
    config_file.close();
}

/**
 * @brief Load environment setup from JSON file
 */
Json::Value loadEnvironmentSetup() {
    // loadTaskConfig();
    // std::string env_path = PathResolver::resolvePath(task_config_["environment"]["object_library"].asString());
    std::string env_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_I.json";
    
    std::ifstream env_file(env_path);
    Json::Value env_setup;
    
    if (!env_file.is_open()) {
        log("Failed to open environment setup file: " + env_path, LogLevel::ERROR);
        return env_setup;
    }
    
    try {
        env_file >> env_setup;
        log("Environment setup loaded successfully from: " + env_path, LogLevel::INFO);
    } catch (const std::exception& e) {
        log("Failed to parse environment setup JSON: " + std::string(e.what()), LogLevel::ERROR);
    }
    
    env_file.close();
    return env_setup;
}

/**
 * @brief Load assembly instructions from JSON file
 */
Json::Value loadAssemblyInstructions() {
    // loadTaskConfig();
    // std::string assembly_path = PathResolver::resolvePath(task_config_["tasks"]["assembly_seq"].asString());
    std::string assembly_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/I.json";
    
    std::ifstream assembly_file(assembly_path);
    Json::Value assembly_data;
    
    if (!assembly_file.is_open()) {
        log("Failed to open assembly instructions file: " + assembly_path, LogLevel::ERROR);
        return assembly_data;
    }
    
    try {
        assembly_file >> assembly_data;
        log("Assembly instructions loaded successfully from: " + assembly_path, LogLevel::INFO);
    } catch (const std::exception& e) {
        log("Failed to parse assembly instructions JSON: " + std::string(e.what()), LogLevel::ERROR);
    }
    
    assembly_file.close();
    return assembly_data;
}

/**
 * @brief Create pose with orientation from Euler angles
 */
geometry_msgs::msg::Pose createPoseWithOrientation(double x, double y, double z, 
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

/**
 * @brief Transform coordinates from skillgraph frame to robot frame
 */
void transformSkillgraphToRobot(double x_sg, double y_sg, double z_sg, 
                               double& x_robot, double& y_robot, double& z_robot, 
                               double& rx, double& ry, double& rz) {
    // Default to right_arm - in practice this should be passed as parameter
    transformSkillgraphToRobot("right_arm", x_sg, y_sg, z_sg, x_robot, y_robot, z_robot, rx, ry, rz);
}

/**
 * @brief Transform coordinates from skillgraph frame to robot frame for specific robot
 */
void transformSkillgraphToRobot(const std::string& robot_name, 
                               double x_sg, double y_sg, double z_sg, 
                               double& x_robot, double& y_robot, double& z_robot, 
                               double& rx, double& ry, double& rz) {
    loadRobotProperties();
    
    // Get workspace parameters
    double block_size = robot_properties_["workspace"]["block_size"].asDouble();
    double block_height_offset = robot_properties_["workspace"]["block_height_offset"].asDouble();
    
    // Get robot-specific transformation parameters
    const Json::Value& robot_transform = robot_properties_["robots"][robot_name]["block_frame_transform"];
    double offset_x = robot_transform["offset_x"].asDouble();
    double offset_y = robot_transform["offset_y"].asDouble();
    double offset_z = robot_transform["offset_z"].asDouble();
    double x_direction = robot_transform["x_direction"].asDouble();
    double y_direction = robot_transform["y_direction"].asDouble();
    double z_direction = robot_transform["z_direction"].asDouble();
    
    // Transform from block frame to robot base frame using robot properties
    // Based on coordinate mapping: +X robot = +Y block, +Y robot = +X block for right arm
    // Use the direction vectors and offsets from robot_properties.json
    
    if (robot_name == "right_arm") {
        // For right arm: +X robot = +Y block, +Y robot = +X block
        // Block frame origin is at (11cm, -40cm) in robot frame  
        x_robot = 0.11 + (y_sg * block_size);      // Block Y maps to robot X
        y_robot = -0.40 + (x_sg * block_size);     // Block X maps to robot Y
    } else {
        // For other robots, use the standard transformation with direction vectors
        x_robot = offset_x + (x_direction * x_sg * block_size);
        y_robot = offset_y + (y_direction * y_sg * block_size);
    }
    
    // Z coordinate: table top is at 0.159m in robot frame
    z_robot = 0.159 + offset_z + (z_direction * z_sg * block_size) + block_height_offset;
    
    // Default orientation (gripper pointing down)
    rx = 0.0; ry = 0.0; rz = 0.0;
    
    log("Detailed coordinate transformation for " + robot_name + ":", LogLevel::DEBUG);
    log("  Input skillgraph coords: (" + std::to_string(x_sg) + ", " + std::to_string(y_sg) + ", " + std::to_string(z_sg) + ")", LogLevel::DEBUG);
    log("  Block size: " + std::to_string(block_size), LogLevel::DEBUG);
    log("  Block height offset: " + std::to_string(block_height_offset), LogLevel::DEBUG);
    log("  Offsets: (" + std::to_string(offset_x) + ", " + std::to_string(offset_y) + ", " + std::to_string(offset_z) + ")", LogLevel::DEBUG);
    log("  Direction vectors: (" + std::to_string(x_direction) + ", " + std::to_string(y_direction) + ", " + std::to_string(z_direction) + ")", LogLevel::DEBUG);
    log("  Final robot coords: (" + std::to_string(x_robot) + ", " + std::to_string(y_robot) + ", " + std::to_string(z_robot) + ")", LogLevel::DEBUG);
}

/**
 * @brief Find optimal theta-Z angle that doesn't block the required press face
 */
double findOptimalThetaZ(int press_face) {
    // Find optimal thetaz that doesn't block the required press_face
    std::vector<double> candidate_angles = {0.0, 90.0, 180.0, 270.0};
    
    for (double angle : candidate_angles) {
        // Simple logic: return first valid angle
        // In practice, this would check which faces are blocked
        return angle;
    }
    
    return 0.0; // Default fallback
}

/**
 * @brief Get blocked faces for a given thetaz orientation
 */
std::set<int> getBlockedFaces(double thetaz_deg) {
    // Normalize angle to 0-360 range
    while (thetaz_deg < 0) thetaz_deg += 360;
    while (thetaz_deg >= 360) thetaz_deg -= 360;
    
    // Round to nearest 90 degree increment for mapping
    int rounded_angle = static_cast<int>(std::round(thetaz_deg / 90.0) * 90) % 360;
    
    switch (rounded_angle) {
        case 0: return {2, 5, 3};
        case 90: return {1, 4, 3};
        case 180: return {2, 5, 3};
        case 270: return {1, 4, 3};
        default: return {2, 5, 3}; // Default fallback
    }
}

/**
 * @brief Get approach direction offset based on press_face for place operation
 */
std::tuple<double, double, double> getPlaceApproachOffset(int press_face, double approach_distance) {
    // Convention for approach direction based on press_face:
    // press_face 0: approach from +Z to press -Z
    // press_face 1: approach from -X to press +X 
    // press_face 2: approach from +Y to press -Y
    // press_face 3: approach from -Z to press +Z
    // press_face 4: approach from +X to press -X
    // press_face 5: approach from -Y to press +Y
    
    switch(press_face) {
        case 0: return {0.0, 0.0, approach_distance}; // +Z approach
        case 1: return {-approach_distance, 0.0, 0.0}; // -X approach
        case 2: return {0.0, approach_distance, 0.0}; // +Y approach 
        case 3: return {0.0, 0.0, -approach_distance}; // -Z approach
        case 4: return {approach_distance, 0.0, 0.0}; // +X approach
        case 5: return {0.0, -approach_distance, 0.0}; // -Y approach
        default: 
            log("Unknown press_face=" + std::to_string(press_face) + ", defaulting to +Z approach", LogLevel::WARN);
            return {0.0, 0.0, approach_distance}; // Default +Z approach
    }
}

/**
 * @brief Get place orientation based on gripper_ori parameter
 */
std::pair<double, double> getPlaceOrientation(int gripper_ori) {
    // Mapping from gripper_ori to (thetax, thetay):
    // 0: (0,180)
    // 1: (-90,180) 
    // 2: (120,90)
    // 3: (-180,180)
    // 4: (-90,-180)
    // 5: (-70,-90)
    
    switch(gripper_ori) {
        case 0: return {0.0, 180.0};
        case 1: return {-90.0, 180.0};
        case 2: return {120.0, 90.0};
        case 3: return {-180.0, 180.0};
        case 4: return {-90.0, -180.0};
        case 5: return {-70.0, -90.0};
        default:
            log("Unknown gripper_ori=" + std::to_string(gripper_ori) + ", defaulting to (0,180)", LogLevel::WARN);
            return {0.0, 180.0}; // Default orientation
    }
}

/**
 * @brief Create place pose with proper orientation
 */
geometry_msgs::msg::Pose createPlacePose(double x, double y, double z, 
                                          int press_face, int gripper_ori, double pick_thetaz) {
    // Get place orientation from gripper_ori
    auto [place_thetax, place_thetay] = getPlaceOrientation(gripper_ori);
    
    // thetaz stays the same as pick operation
    double place_thetaz = pick_thetaz;
    
    return createPoseWithOrientation(x, y, z, place_thetax, place_thetay, place_thetaz);
}

/**
 * @brief Create approach pose for place operation
 */
geometry_msgs::msg::Pose createPlaceApproachPose(const geometry_msgs::msg::Pose& place_pose, 
                                                  int press_face, double approach_distance) {
    auto [dx, dy, dz] = getPlaceApproachOffset(press_face, approach_distance);
    
    geometry_msgs::msg::Pose approach_pose = place_pose;
    approach_pose.position.x += dx;
    approach_pose.position.y += dy; 
    approach_pose.position.z += dz;
    
    return approach_pose;
}

/**
 * @brief Get block pick position from environment setup
 */
bool getBlockPickPosition(const std::string& block_name, double& x, double& y, double& z) {
    Json::Value env_setup = loadEnvironmentSetup();
    
    if (env_setup.isMember(block_name)) {
        const Json::Value& block_data = env_setup[block_name];
        if (block_data.isMember("x") && block_data.isMember("y") && block_data.isMember("z")) {
            x = block_data["x"].asDouble();
            y = block_data["y"].asDouble();
            z = block_data["z"].asDouble();
            
            log("Found pick position for " + block_name + ": (" + 
                std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")", LogLevel::INFO);
            return true;
        }
    }
    
    log("Could not find pick position for block: " + block_name, LogLevel::ERROR);
    return false;
}

/**
 * @brief Get block place position from assembly instructions
 */
bool getBlockPlacePosition(const std::string& block_name, double& x, double& y, double& z, int& press_face, int& gripper_ori) {
    Json::Value assembly_data = loadAssemblyInstructions();
    
    // Extract numeric part from block name (e.g., "b1" -> "1")
    std::string block_number = block_name.substr(1); // Remove 'b' prefix
    
    // Look for block by its numeric key in assembly data
    if (assembly_data.isMember(block_number)) {
        const Json::Value& block_data = assembly_data[block_number];
        if (block_data.isMember("x") && block_data.isMember("y") && block_data.isMember("z")) {
            x = block_data["x"].asDouble();
            y = block_data["y"].asDouble();
            z = block_data["z"].asDouble();
            press_face = block_data.get("press_face", 0).asInt();
            gripper_ori = block_data.get("gripper_ori", 0).asInt();
            
            log("Found place position for " + block_name + " (key=" + block_number + "): (" + 
                std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + 
                ") press_face=" + std::to_string(press_face) + " gripper_ori=" + std::to_string(gripper_ori), LogLevel::INFO);
            return true;
        }
    }
    
    // Fallback: Look through assembly tasks to find this block
    if (assembly_data.isMember("tasks")) {
        for (const auto& task : assembly_data["tasks"]) {
            if (task.isMember("constraints")) {
                const Json::Value& constraints = task["constraints"];
                std::string task_block = constraints.get("object", "").asString();
                
                if (task_block == block_name || 
                    (task_block.empty() && constraints.isMember("x") && constraints.isMember("y"))) {
                    x = constraints["x"].asDouble();
                    y = constraints["y"].asDouble();
                    z = constraints["z"].asDouble();
                    press_face = constraints.get("press_face", 0).asInt();
                    gripper_ori = constraints.get("gripper_ori", 0).asInt();
                    
                    log("Found place position for " + block_name + " in tasks: (" + 
                        std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + 
                        ") press_face=" + std::to_string(press_face) + " gripper_ori=" + std::to_string(gripper_ori), LogLevel::INFO);
                    return true;
                }
            }
        }
    }
    
    log("Could not find place position for block: " + block_name, LogLevel::ERROR);
    return false;
}

/**
 * @brief Plan transit trajectory
 */
bool planTransit(std::shared_ptr<MoveitInstance> moveit_instance,
                             const std::string& robot_name,
                             const geometry_msgs::msg::Pose& goal_pose,
                             std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
    if (!moveit_instance) {
        log("No MoveitInstance backend available for transit planning", LogLevel::ERROR);
        return false;
    }

    trajectories.clear();

    // Get current joint values as seed for IK
    std::vector<double> current_joints;
    if (!moveit_instance->getCurrentJointValues(robot_name, current_joints)) {
        log("Failed to get current joint values for " + robot_name, LogLevel::ERROR);
        return false;
    }

    std::vector<double> solution_joints;
    if (!moveit_instance->solveIK(robot_name, goal_pose, current_joints, solution_joints)) {
        log("IK planning failed for transit goal pose", LogLevel::ERROR);
        return false;
    }

    // Create interpolated trajectories between waypoints
    int steps_per_segment = 10;
    double step_duration = 0.1;  // 100ms per step
    
    std::vector<std::vector<double>> full_trajectory;
    std::vector<std::vector<double>> segment = moveit_instance->interpolateJointTrajectory(current_joints, solution_joints, steps_per_segment);
    // Execute the joint trajectory
    moveit_instance->executeJointTrajectory(robot_name, segment, step_duration);
    for (const auto& joint_values : segment) {
        std::cout << "Joint values: ";
        for (const auto& value : joint_values) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    return true;
}

/**
 * @brief Plan pick-place trajectory
 */
bool planPickPlaceTrajectory(std::shared_ptr<MoveitInstance> moveit_instance,
                             const std::string& robot_name,
                             const geometry_msgs::msg::Pose& pick_pose,
                             const geometry_msgs::msg::Pose& place_pose,
                             const std::string& object_name,
                             int press_face,
                             std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
    
    if (!moveit_instance) {
        log("No MoveitInstance backend available", LogLevel::ERROR);
        return false;
    }
    
    log("Planning pick-place trajectory for " + robot_name + " with object " + object_name, LogLevel::INFO);
    
    trajectories.clear();
    
    // Get current joint values as seed for IK
    std::vector<double> current_joints;
    if (!moveit_instance->getCurrentJointValues(robot_name, current_joints)) {
        log("Failed to get current joint values for " + robot_name, LogLevel::ERROR);
        return false;
    }
    
    log("Current joint values: " + std::to_string(current_joints.size()) + " joints", LogLevel::DEBUG);
    
    // Create waypoint poses for pick and place sequence
    geometry_msgs::msg::Pose approach_pose = pick_pose;
    approach_pose.position.z += 0.15;  // 15cm above pick
    
    geometry_msgs::msg::Pose lift_pose = pick_pose;
    lift_pose.position.z += 0.1;  // 10cm lift
    
    geometry_msgs::msg::Pose transport_pose = createPlaceApproachPose(place_pose, press_face, 0.15);
    geometry_msgs::msg::Pose retract_pose = createPlaceApproachPose(place_pose, press_face, 0.1);
    
    // Solve IK for each waypoint
    std::vector<std::vector<double>> waypoint_joints;
    std::vector<geometry_msgs::msg::Pose> waypoints = {approach_pose, pick_pose, lift_pose, transport_pose, place_pose, retract_pose};
    std::vector<std::string> waypoint_names = {"approach", "pick", "lift", "transport", "place", "retract"};
    
    std::vector<double> seed_joints = current_joints;
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::vector<double> solution_joints;
        
        // Log pose being solved with full orientation details
        log("Solving IK for " + waypoint_names[i] + " pose: [" + 
            std::to_string(waypoints[i].position.x) + ", " + 
            std::to_string(waypoints[i].position.y) + ", " + 
            std::to_string(waypoints[i].position.z) + "]", LogLevel::DEBUG);
            
        // Also log orientation quaternion
        log("  Orientation quat: [" + 
            std::to_string(waypoints[i].orientation.x) + ", " + 
            std::to_string(waypoints[i].orientation.y) + ", " + 
            std::to_string(waypoints[i].orientation.z) + ", " + 
            std::to_string(waypoints[i].orientation.w) + "]", LogLevel::DEBUG);
            
        // Convert quaternion back to Euler for debugging
        tf2::Quaternion q(waypoints[i].orientation.x, waypoints[i].orientation.y, 
                         waypoints[i].orientation.z, waypoints[i].orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        log("  Euler angles (rad): roll=" + std::to_string(roll) + 
            " pitch=" + std::to_string(pitch) + " yaw=" + std::to_string(yaw), LogLevel::DEBUG);
        log("  Euler angles (deg): roll=" + std::to_string(roll * 180.0 / M_PI) + 
            " pitch=" + std::to_string(pitch * 180.0 / M_PI) + " yaw=" + std::to_string(yaw * 180.0 / M_PI), LogLevel::DEBUG);
        
        // First try the enhanced IK solver using MoveGroupInterface planning
        if (!moveit_instance->solveIK(robot_name, waypoints[i], seed_joints, solution_joints)) {
            log("IK planning failed for " + waypoint_names[i] + " pose, trying basic IK solver...", LogLevel::WARN);
                        
                // Try with a more relaxed orientation - modify the pose to have less strict orientation
                log("Trying with relaxed orientation...", LogLevel::WARN);
                geometry_msgs::msg::Pose relaxed_pose = waypoints[i];
                
                // Use a more achievable orientation - slightly tilted instead of straight down
                double relaxed_thetax = 10.0;  // Small tilt
                double relaxed_thetay = 170.0; // Not quite straight down
                double relaxed_thetaz = 0.0;   // No rotation around Z
                
                // Convert to quaternion
                tf2::Quaternion relaxed_quat;
                relaxed_quat.setRPY(relaxed_thetax * M_PI / 180.0, 
                                    relaxed_thetay * M_PI / 180.0, 
                                    relaxed_thetaz * M_PI / 180.0);
                
                relaxed_pose.orientation.x = relaxed_quat.x();
                relaxed_pose.orientation.y = relaxed_quat.y();
                relaxed_pose.orientation.z = relaxed_quat.z();
                relaxed_pose.orientation.w = relaxed_quat.w();
                
                log("Relaxed orientation: roll=" + std::to_string(relaxed_thetax) + 
                    " pitch=" + std::to_string(relaxed_thetay) + " yaw=" + std::to_string(relaxed_thetaz), LogLevel::DEBUG);
                
                // Try enhanced planning with relaxed pose first
                if (!moveit_instance->solveIK(robot_name, relaxed_pose, seed_joints, solution_joints)) {
                    log("IK planning with relaxed orientation failed, trying basic IK...", LogLevel::WARN);
                    
                } else {
                    log("Relaxed orientation IK succeeded for " + waypoint_names[i] + " pose", LogLevel::INFO);
                    waypoints[i] = relaxed_pose; // Update the waypoint to use the successful pose
                }
        } else {
            log("IK planning succeeded for " + waypoint_names[i] + " pose", LogLevel::INFO);
        }
    

        waypoint_joints.push_back(solution_joints);
        seed_joints = solution_joints;  // Use previous solution as seed for next IK
        
        log("IK solved for " + waypoint_names[i] + " pose", LogLevel::DEBUG);
    }

    waypoint_joints.push_back({0.0, 0.0, 0.0, 2.5299, 0.0, 0.6120, 1.570});
    
    // Create interpolated trajectories between waypoints
    int steps_per_segment = 10;
    double step_duration = 0.1;  // 100ms per step
    
    std::vector<std::vector<double>> full_trajectory;
    
    for (size_t i = 0; i < waypoint_joints.size() - 1; ++i) {
        auto segment = moveit_instance->interpolateJointTrajectory(
            waypoint_joints[i], waypoint_joints[i + 1], steps_per_segment);
        
        // Add all points except the last one (to avoid duplication)
        for (size_t j = 0; j < segment.size() - 1; ++j) {
            full_trajectory.push_back(segment[j]);
        }
    }
    
    // Add the final waypoint
    full_trajectory.push_back(waypoint_joints.back());
    
    // Execute the joint trajectory
    moveit_instance->executeJointTrajectory(robot_name, full_trajectory, step_duration);
    
    log("Completed pick-place trajectory execution for " + object_name + " with " + 
        std::to_string(full_trajectory.size()) + " trajectory points", LogLevel::INFO);
    
    return true;
}

/**
 * @brief Get the current joint state for a robot to ensure trajectory continuity
 */
std::vector<double> getCurrentJointState(std::shared_ptr<MoveitInstance> moveit_instance, int robot_id) {
    if (!moveit_instance) {
        log("No MoveitInstance backend available for getCurrentJointState", LogLevel::ERROR);
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
    
    // Get current joint values using MoveitInstance
    std::vector<double> current_joints;
    if (!moveit_instance->getCurrentJointValues(robot_name, current_joints)) {
        log("Failed to get current joint values for " + robot_name, LogLevel::ERROR);
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    
    log("Retrieved current joint state for " + robot_name + " with " + 
        std::to_string(current_joints.size()) + " joints", LogLevel::DEBUG);
    
    return current_joints;
}

/**
 * @brief Interpolate between two joint configurations and move robot for visualization
 */
void interpolateAndMoveRobot(std::shared_ptr<MoveitInstance> moveit_instance,
                             int robot_id,
                             const std::vector<double>& start_joints,
                             const std::vector<double>& end_joints,
                             int steps,
                             int delay_ms) {
    // Get robot name based on robot_id
    std::string robot_name;
    switch(robot_id) {
        case 0: robot_name = "left_arm"; break;
        case 1: robot_name = "center_arm"; break;
        case 2: robot_name = "right_arm"; break;
        default: robot_name = "right_arm"; break;
    }
    
    // Generate interpolated trajectory
    auto trajectory = moveit_instance->interpolateJointTrajectory(start_joints, end_joints, steps);
    
    // Execute with specified delay
    double step_duration = delay_ms / 1000.0;  // Convert to seconds
    moveit_instance->executeJointTrajectory(robot_name, trajectory, step_duration);
}

// ===============================
// Original Planning Algorithm Classes
// ===============================

bool MagBlockGraspGenerator::generate(const Json::Value& constraints, Skill::Type type, int seq_id, State& state) {
    // Enhanced implementation using the algorithms above
    log("Generating grasps for MagBlock skill type: " + std::to_string(static_cast<int>(type)), LogLevel::INFO);
    
    // Use the helper functions for grasp generation based on constraints
    if (constraints.isMember("object")) {
        std::string block_name = constraints["object"].asString();
        
        // Get block positions using our algorithms
        double x, y, z;
        int press_face = 0, gripper_ori = 0;
        
        if (type == Skill::Type::Pick) {
            if (getBlockPickPosition(block_name, x, y, z)) {
                log("Generated pick pose for " + block_name, LogLevel::INFO);
                // Update state with generated grasp information
                return true;
            }
        } else if (type == Skill::Type::PlaceTop || type == Skill::Type::PlaceBottom) {
            if (getBlockPlacePosition(block_name, x, y, z, press_face, gripper_ori)) {
                log("Generated place pose for " + block_name, LogLevel::INFO);
                // Update state with generated grasp information
                return true;
            }
        }
    }
    
    return true; // Return success for basic testing
}

bool MagBlockPlan::plan_skill(const State& current_state,
                             const TaskParam& task_param,
                             Skill::Type type,
                             RobotTrajectory& traj) {
    log("Planning skill for MagBlock assembly. Type: " + std::to_string(static_cast<int>(type)), LogLevel::INFO);
    
    switch (type) {
        case Skill::Type::Pick:
            return plan_pick(current_state, task_param, traj);
        case Skill::Type::PlaceTop:
            return plan_place(current_state, task_param, traj);
        default:
            log("Unsupported skill type for MagBlock planning", LogLevel::WARN);
            return false;
    }
}

bool MagBlockPlan::plan_pick(const State& current_state,
                            const TaskParam& task_param,
                            RobotTrajectory& traj) {
    log("Planning pick operation for magnetic block", LogLevel::INFO);
    
    // Use magblock algorithms for sophisticated pick planning
    // This could leverage the coordinate transformation and pose generation functions
    
    return true;
}

bool MagBlockPlan::plan_place(const State& current_state,
                             const TaskParam& task_param,
                             RobotTrajectory& traj) {
    log("Planning place operation for magnetic block", LogLevel::INFO);
    
    // Use magblock algorithms for sophisticated place planning
    // This could leverage the approach pose and orientation functions
    
    return true;
}

/**
 * @brief Get current robot end-effector pose using forward kinematics.
 */
bool getCurrentRobotPose(std::shared_ptr<PlanInstance> backend,
                        const std::string& robot_name, 
                        geometry_msgs::msg::Pose& current_pose) {
    auto moveit_instance = std::dynamic_pointer_cast<MoveitInstance>(backend);
    if (!moveit_instance) {
        log("MoveIt instance not available for FK calculation", LogLevel::ERROR);
        return false;
    }
    
    // Get current joint values
    std::vector<double> current_joints;
    if (!moveit_instance->getCurrentJointValues(robot_name, current_joints)) {
        log("Failed to get current joint values for " + robot_name, LogLevel::ERROR);
        return false;
    }
    
    // Use MoveIt's robot state to compute forward kinematics
    auto planning_scene = moveit_instance->getPlanningScene();
    if (!planning_scene) {
        log("Planning scene not available for FK calculation", LogLevel::ERROR);
        return false;
    }
    
    moveit::core::RobotState robot_state = planning_scene->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state.getJointModelGroup(robot_name);
        
    if (!joint_model_group) {
        log("Joint model group not found for " + robot_name, LogLevel::ERROR);
        return false;
    }
    
    // Set joint positions and compute FK
    robot_state.setJointGroupPositions(joint_model_group, current_joints);
    const Eigen::Isometry3d& end_effector_state = 
        robot_state.getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());
    
    // Convert to geometry_msgs::Pose
    current_pose.position.x = end_effector_state.translation().x();
    current_pose.position.y = end_effector_state.translation().y();
    current_pose.position.z = end_effector_state.translation().z();
    
    Eigen::Quaterniond q(end_effector_state.rotation());
    current_pose.orientation.x = q.x();
    current_pose.orientation.y = q.y();
    current_pose.orientation.z = q.z();
    current_pose.orientation.w = q.w();
    
    log("Current robot pose for " + robot_name + ": pos(" + 
        std::to_string(current_pose.position.x) + "," + 
        std::to_string(current_pose.position.y) + "," + 
        std::to_string(current_pose.position.z) + ") orient(" +
        std::to_string(current_pose.orientation.w) + "," +
        std::to_string(current_pose.orientation.x) + "," +
        std::to_string(current_pose.orientation.y) + "," +
        std::to_string(current_pose.orientation.z) + ")", LogLevel::DEBUG);
    
    return true;
}

/**
 * @brief Calculate the exact approach pose for a task, consistent with pick/place operations.
 */
bool calculateApproachPose(std::shared_ptr<PlanInstance> backend,
                         const Json::Value& task_constraints,
                         const std::string& robot_name, 
                         geometry_msgs::msg::Pose& approach_pose) {
    // Extract target positions from constraints (block coordinates)
    double target_x = task_constraints.get("x", 0.0).asDouble();
    double target_y = task_constraints.get("y", 0.0).asDouble();
    double target_z = task_constraints.get("z", 0.0).asDouble();
    
    // Transform to robot coordinates using the existing transform function
    double robot_x, robot_y, robot_z, rx_deg, ry_deg, rz_deg;
    transformSkillgraphToRobot(robot_name, target_x, target_y, target_z, 
                              robot_x, robot_y, robot_z, rx_deg, ry_deg, rz_deg);
    
    // Calculate approach position - offset above target for pick operations
    // This should match the offset used in pick operations
    double approach_offset_z = 0.15; // 15cm above target (same as pick approach)
    
    approach_pose.position.x = robot_x;
    approach_pose.position.y = robot_y;
    approach_pose.position.z = robot_z + approach_offset_z;
    
    // Get current robot orientation to preserve it for transit
    geometry_msgs::msg::Pose current_pose;
    if (getCurrentRobotPose(backend, robot_name, current_pose)) {
        // Preserve current orientation for transit
        approach_pose.orientation = current_pose.orientation;
        log("Transit will preserve current robot orientation for " + robot_name, LogLevel::INFO);
    } else {
        // Fallback to proper gripper-down orientation if we can't get current pose
        // Convert the default orientation from degrees to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0.0, M_PI, 0.0);
        
        approach_pose.orientation.x = quat.x();
        approach_pose.orientation.y = quat.y();
        approach_pose.orientation.z = quat.z();
        approach_pose.orientation.w = quat.w();
        
        log("Using default orientation for transit (could not get current pose) for " + robot_name, LogLevel::WARN);
    }
    
    log("Calculated approach pose for " + robot_name + " at task (" + 
        std::to_string(target_x) + "," + std::to_string(target_y) + "," + std::to_string(target_z) + 
        "): pos(" + std::to_string(approach_pose.position.x) + "," + 
        std::to_string(approach_pose.position.y) + "," + 
        std::to_string(approach_pose.position.z) + ") orient(" +
        std::to_string(approach_pose.orientation.w) + "," +
        std::to_string(approach_pose.orientation.x) + "," +
        std::to_string(approach_pose.orientation.y) + "," +
        std::to_string(approach_pose.orientation.z) + ")", LogLevel::INFO);
    
    return true;
}

} // namespace skillgraph

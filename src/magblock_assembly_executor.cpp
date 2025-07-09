#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <cmath>
#include <set>

#include "magblock/magblock_skillgraph.hpp"
#include "magblock/magblock_algorithms.hpp"
#include "Utils/Logger.hpp"

/**
 * @brief MagBlock Assembly Executor with Enhanced Trajectory Planning
 * 
 * This program demonstrates complete magblock assembly trajectory planning by:
 * 1. Loading assembly tasks and environment setup from skillgraph JSON files
 * 2. Planning complete trajectories using MoveIt for pick and place operations
 * 3. Visualizing trajectories in RViz before execution
 * 4. Executing coordinated multi-robot assembly operations
 * 5. Supporting both simulation and real robot execution
 */

class MagBlockAssemblyExecutor 
{
public:
 MagBlockAssemblyExecutor(const rclcpp::Node::SharedPtr& node) 
 : node_(node)
 {
 RCLCPP_INFO(node_->get_logger(), "Starting MagBlock Assembly Executor");
 RCLCPP_INFO(node_->get_logger(), "Waiting for MoveIt2 services...");
 
 // Suppress TF and planning scene monitor warnings for cleaner output
 if (rcutils_logging_set_logger_level(
 "moveit_ros.planning_scene_monitor.planning_scene_monitor", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK) {
 RCLCPP_DEBUG(node_->get_logger(), "Could not set planning scene monitor log level");
 }
 
 // Wait for MoveIt2 services to be available
 if (!waitForMoveItServices()) {
 throw std::runtime_error("Failed to connect to MoveIt2 services");
 }
 
 RCLCPP_INFO(node_->get_logger(), "Initializing MoveIt interfaces...");
 
 // Initialize MoveIt2 interfaces for all three arms
 try {
 RCLCPP_INFO(node_->get_logger(), "Creating MoveGroupInterface objects...");
 
 // Create MoveGroupInterface objects using the shared node
 left_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "left_arm");
 right_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "right_arm"); 
 center_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "center_arm");
 
 // Initialize planning scene interface without the node
 planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
 } catch (const std::exception& e) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MoveIt interfaces: %s", e.what());
 throw;
 }
 
 // Setup minimal publishers for simulation only
 marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
 "/magblock_trajectory_markers", 10);
 
 // Initialize planning scene diff client for simulation
 planning_scene_diff_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
 "apply_planning_scene");
 
 // Wait for planning scene service
 RCLCPP_INFO(node_->get_logger(), "Waiting for apply_planning_scene service...");
 if (!planning_scene_diff_client_->wait_for_service(std::chrono::seconds(10))) {
 RCLCPP_WARN(node_->get_logger(), "apply_planning_scene service not available, continuing anyway");
 }
 
 // Visual tools disabled for cleaner simulation
 RCLCPP_INFO(node_->get_logger(), "Visual tools disabled for cleaner simulation");
 
 // Set up planning parameters
 setupPlanningParameters();
 
 // Initialize the MagBlock skillgraph
 std::string config_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/skillgraph.json";
 skillgraph_ = std::make_shared<skillgraph::MagBlockSkillGraph>(config_path);
 skillgraph_->initialize();
 
 // Control execution flow
 wait_for_confirmation_ = false; // Set to false for automatic execution without prompts
 
 RCLCPP_INFO(node_->get_logger(), "MagBlock Assembly Executor initialized with skillgraph trajectory planning");
 }
 
 void setupPlanningParameters() {
 std::vector<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> arms = 
 {left_arm_group_, right_arm_group_, center_arm_group_};
 
 for (auto& arm : arms) {
 arm->setPlanningTime(15.0); // More time for complex planning
 arm->setMaxVelocityScalingFactor(0.5);
 arm->setMaxAccelerationScalingFactor(0.3);
 arm->setNumPlanningAttempts(10);
 arm->setGoalTolerance(0.01); // 1cm tolerance
 arm->setPlannerId("RRTConnect"); // Use RRT Connect for reliability
 }
 
 // Set pose reference frame to robot base for each arm
 left_arm_group_->setPoseReferenceFrame("left_base_link");
 right_arm_group_->setPoseReferenceFrame("right_base_link");
 center_arm_group_->setPoseReferenceFrame("center_base_link");
 }
 
 /**
 * @brief Interactive prompt - DISABLED for automatic execution
 */
 void prompt(const std::string& message) {
 // Disabled for automatic execution
 RCLCPP_DEBUG(node_->get_logger(), "Prompt disabled: %s", message.c_str());
 }
 
 /**
 * @brief Draw title - DISABLED for cleaner simulation
 */
 void draw_title(const std::string& title) {
 // Disabled for cleaner simulation
 RCLCPP_DEBUG(node_->get_logger(), "Title disabled: %s", title.c_str());
 }
 
 /**
 * @brief Draw trajectory tool path - DISABLED
 */
 void draw_trajectory_tool_path(const moveit_msgs::msg::RobotTrajectory& trajectory, 
 const std::string& robot_name) {
 // Disabled for cleaner simulation
 RCLCPP_DEBUG(node_->get_logger(), "Trajectory visualization disabled for %s", robot_name.c_str());
 }
 
 /**
 * @brief Get visual tools for specific robot - DISABLED
 */
 std::shared_ptr<moveit_visual_tools::MoveItVisualTools> getVisualTools(const std::string& robot_name) {
 // Disabled for cleaner simulation
 return nullptr;
 }
 
 /**
 * @brief Get joint model group for robot
 */
 const moveit::core::JointModelGroup* getJointModelGroup(const std::string& robot_name) {
 auto arm = getRobotArm(getRobotIdFromName(robot_name));
 return arm->getRobotModel()->getJointModelGroup(robot_name);
 }
 
 /**
 * @brief Plan complete trajectory for a pick and place operation
 */
 /**
 * @brief Plan complete trajectory for a pick and place operation and simulate via planning scene updates
 */
 bool planPickPlaceTrajectory(int robot_id, 
 const geometry_msgs::msg::Pose& pick_pose,
 const geometry_msgs::msg::Pose& place_pose,
 const std::string& object_name,
 int press_face,
 std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
 
 auto arm = getRobotArm(robot_id);
 std::string robot_name = getRobotName(robot_id);
 
 RCLCPP_INFO(node_->get_logger(), "Planning pick-place trajectory for %s with object %s", 
 robot_name.c_str(), object_name.c_str());
 
 trajectories.clear();
 
 // Create clear pick-and-place waypoints with straight vertical movements
 
 // 1. Approach Pick: Move to position above pick location (vertical approach)
 geometry_msgs::msg::Pose approach_pick_pose = pick_pose;
 approach_pick_pose.position.z += 0.15; // 15cm straight up above pick
 
 // 2. Pick pose is already defined (gripper at block level)
 
 // 3. Retract Pick: Move straight up from pick location 
 geometry_msgs::msg::Pose retract_pick_pose = pick_pose;
 retract_pick_pose.position.z += 0.15; // 15cm straight up above pick (same as approach)
 
 // 4. Approach Place: Move to position above place location (vertical approach)
 geometry_msgs::msg::Pose approach_place_pose = place_pose;
 approach_place_pose.position.z += 0.15; // 15cm straight up above place
 
 // 5. Place pose is already defined (gripper at place level)
 
 // 6. Retract Place: Move straight up from place location
 geometry_msgs::msg::Pose retract_place_pose = place_pose;
 retract_place_pose.position.z += 0.15; // 15cm straight up above place
 
 RCLCPP_INFO(node_->get_logger(), "=== PLANNING PICK-AND-PLACE TRAJECTORY FOR %s ===", object_name.c_str());
 RCLCPP_INFO(node_->get_logger(), "1. Approach Pick: (%.3f, %.3f, %.3f) - Move above pick location", 
 approach_pick_pose.position.x, approach_pick_pose.position.y, approach_pick_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "2. Pick: (%.3f, %.3f, %.3f) - Move straight DOWN to pick", 
 pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "3. Retract Pick: (%.3f, %.3f, %.3f) - Move straight UP from pick", 
 retract_pick_pose.position.x, retract_pick_pose.position.y, retract_pick_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "4. Approach Place: (%.3f, %.3f, %.3f) - Move above place location", 
 approach_place_pose.position.x, approach_place_pose.position.y, approach_place_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "5. Place: (%.3f, %.3f, %.3f) - Move straight DOWN to place", 
 place_pose.position.x, place_pose.position.y, place_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "6. Retract Place: (%.3f, %.3f, %.3f) - Move straight UP from place", 
 retract_place_pose.position.x, retract_place_pose.position.y, retract_place_pose.position.z);
 
 // Plan clear pick-and-place sequence with vertical movements
 std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> waypoints = {
 {"Approach Pick", approach_pick_pose}, // Move above pick location
 {"Pick", pick_pose}, // Move straight down to pick location
 {"Retract Pick", retract_pick_pose}, // Move straight up from pick location
 {"Approach Place", approach_place_pose}, // Move above place location  
 {"Place", place_pose}, // Move straight down to place location
 {"Retract Place", retract_place_pose} // Move straight up from place location
 };
 
 for (const auto& [segment_name, target_pose] : waypoints) {
 RCLCPP_INFO(node_->get_logger(), "Planning %s segment to (%.3f, %.3f, %.3f)", 
 segment_name.c_str(), target_pose.position.x, target_pose.position.y, target_pose.position.z);
 
 arm->setPoseTarget(target_pose);
 
 moveit::planning_interface::MoveGroupInterface::Plan plan;
 auto success = static_cast<bool>(arm->plan(plan));
 
 if (success) {
 trajectories.push_back(plan.trajectory_);
 RCLCPP_INFO(node_->get_logger(), "Successfully planned %s segment", segment_name.c_str());
 } else {
 RCLCPP_ERROR(node_->get_logger(), "Failed to plan %s segment", segment_name.c_str());
 return false;
 }
 }
 
 RCLCPP_INFO(node_->get_logger(), "Successfully planned all %zu trajectory segments for %s", 
 trajectories.size(), object_name.c_str());
 
 return true;
 }
 
 /**
 * @brief Plan a single trajectory segment
 */
 bool planSingleTrajectory(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm,
 const geometry_msgs::msg::Pose& target_pose,
 const std::string& segment_name,
 std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
 
 arm->setPoseTarget(target_pose);
 
 moveit::planning_interface::MoveGroupInterface::Plan plan;
 bool success = (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
 
 if (success) {
 trajectories.push_back(plan.trajectory_);
 RCLCPP_INFO(node_->get_logger(), "Planned trajectory segment: %s", segment_name.c_str());
 return true;
 } else {
 RCLCPP_ERROR(node_->get_logger(), "Failed to plan trajectory segment: %s", segment_name.c_str());
 return false;
 }
 }
 
 /**
 * @brief Visualize planned trajectories in RViz - DISABLED for cleaner simulation
 */
 void visualizeTrajectories(const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
 const std::string& robot_name,
 const std::string& object_name) {
 // Disabled to reduce visual clutter during simulation
 RCLCPP_DEBUG(node_->get_logger(), "Trajectory visualization disabled for cleaner simulation view");
 }
 
 /**
 * @brief Create trajectory markers for enhanced visualization - DISABLED
 */
 void createTrajectoryMarkers(const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
 const std::string& robot_name,
 const std::string& object_name) {
 // Disabled to reduce visual clutter during simulation
 RCLCPP_DEBUG(node_->get_logger(), "Trajectory markers disabled for cleaner simulation view");
 }
 
 /**
 * @brief Move robot to specific joint configuration via planning scene update (simulation)
 */
 void moveRobot(int robot_id, const std::vector<double>& joint_values) {
 std::string robot_name = getRobotName(robot_id);
 auto arm = getRobotArm(robot_id);
 
 // Create planning scene diff for only this robot in its base frame
 planning_scene_diff_.is_diff = true;
 planning_scene_diff_.robot_state.joint_state.header.stamp = node_->now();
 planning_scene_diff_.robot_state.joint_state.header.frame_id = robot_name + "_base_link"; // Use robot base frame
 
 // Get joint names for this specific robot
 auto joint_model_group = arm->getRobotModel()->getJointModelGroup(robot_name);
 auto joint_names = joint_model_group->getActiveJointModelNames();
 
 // Only update joints for this specific robot
 planning_scene_diff_.robot_state.joint_state.name.clear();
 planning_scene_diff_.robot_state.joint_state.position.clear();
 
 // Set joint values only for this robot
 for (size_t i = 0; i < joint_values.size() && i < joint_names.size(); ++i) {
 planning_scene_diff_.robot_state.joint_state.name.push_back(joint_names[i]);
 planning_scene_diff_.robot_state.joint_state.position.push_back(joint_values[i]);
 }
 
 RCLCPP_DEBUG(node_->get_logger(), "Moving %s to joint configuration with %zu joints in %s frame", 
 robot_name.c_str(), joint_values.size(), (robot_name + "_base_link").c_str());
 }
 
 /**
 * @brief Update the planning scene for visualization
 */
 void updateScene() {
 if (!planning_scene_diff_client_->service_is_ready()) {
 RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
 "Planning scene service not ready");
 return;
 }
 
 auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
 request->scene = planning_scene_diff_;
 
 auto future = planning_scene_diff_client_->async_send_request(request);
 
 // Wait for response with timeout
 if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(100)) == 
 rclcpp::FutureReturnCode::SUCCESS) {
 auto response = future.get();
 if (!response->success) {
 RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
 "Failed to update planning scene");
 }
 }
 }
 
 /**
 * @brief Simulate trajectory execution by interpolating joint values and updating planning scene
 */
 bool simulateTrajectoryExecution(int robot_id, 
 const moveit_msgs::msg::RobotTrajectory& trajectory,
 const std::string& segment_name,
 double playback_speed = 1.0) {
 
 std::string robot_name = getRobotName(robot_id);
 
 RCLCPP_INFO(node_->get_logger(), "=== EXECUTING %s ===", segment_name.c_str());
 
 if (trajectory.joint_trajectory.points.empty()) {
 RCLCPP_WARN(node_->get_logger(), "Empty trajectory for simulation");
 return false;
 }
 
 const auto& joint_trajectory = trajectory.joint_trajectory;
 
 // Use more waypoints for smoother vertical movements
 size_t step_size = std::max(1UL, joint_trajectory.points.size() / 20); // Sample every 20th point for smoother motion
 
 for (size_t i = 0; i < joint_trajectory.points.size(); i += step_size) {
 const auto& point = joint_trajectory.points[i];
 
 // Move robot to this waypoint
 moveRobot(robot_id, point.positions);
 updateScene();
 
 // Slower motion for clear visualization
 rclcpp::sleep_for(std::chrono::milliseconds(50)); // 50ms between waypoints for smoother motion
 
 // Allow ROS to process callbacks
 rclcpp::spin_some(node_);
 }
 
 // Always execute the final point
 if (!joint_trajectory.points.empty()) {
 const auto& final_point = joint_trajectory.points.back();
 moveRobot(robot_id, final_point.positions);
 updateScene();
 }
 
 RCLCPP_INFO(node_->get_logger(), "=== COMPLETED %s ===", segment_name.c_str());
 return true;
 }

 /**
 * @brief Execute planned trajectories via planning scene simulation
 */
 bool executeTrajectories(int robot_id, 
 const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
 const std::string& object_name) {
 
 std::string robot_name = getRobotName(robot_id);
 
 RCLCPP_INFO(node_->get_logger(), "Executing %zu trajectory segments for %s using %s", 
 trajectories.size(), object_name.c_str(), robot_name.c_str());
 
 std::vector<std::string> segment_names = {
 "Approach Pick", "Pick", "Retract Pick", "Approach Place", "Place", "Retract Place"
 };
 
 for (size_t i = 0; i < trajectories.size(); ++i) {
 std::string segment_name = (i < segment_names.size()) ? segment_names[i] : "Segment " + std::to_string(i+1);
 
 RCLCPP_INFO(node_->get_logger(), "Executing %s motion...", segment_name.c_str());
 
 // Simulate trajectory execution using planning scene updates
 if (!simulateTrajectoryExecution(robot_id, trajectories[i], segment_name)) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to simulate trajectory segment %zu (%s)", 
 i, segment_name.c_str());
 return false;
 }
 
 // Add special messages for pick and place actions
 if (segment_name == "Pick") {
 RCLCPP_INFO(node_->get_logger(), "*** PICKED UP %s ***", object_name.c_str());
 } else if (segment_name == "Place") {
 RCLCPP_INFO(node_->get_logger(), "*** PLACED %s ***", object_name.c_str());
 }
 
 // Longer pause between segments to clearly see each movement
 rclcpp::sleep_for(std::chrono::milliseconds(800));
 }
 
 RCLCPP_INFO(node_->get_logger(), "Successfully executed all trajectory segments for %s", 
 object_name.c_str());
 return true;
 }
 
 /**
 * @brief Load and execute complete assembly task with trajectory planning using skillgraph locations
 */
 bool executeAssemblyWithTrajectories(const std::string& task_file, const std::string& env_setup_file) {
 
 RCLCPP_INFO(node_->get_logger(), "Loading assembly task from skillgraph locations...");
 
 // Load and parse assembly task from skillgraph
 if (!loadSkillgraphTasks(task_file, env_setup_file)) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to load skillgraph assembly tasks");
 return false;
 }
 
 // Get the assembly sequence from skillgraph
 auto assembly_seq = skillgraph_->getAssemblySequence();
 if (!assembly_seq) {
 RCLCPP_ERROR(node_->get_logger(), "No assembly sequence found in skillgraph");
 return false;
 }
 
 RCLCPP_INFO(node_->get_logger(), "Found %d assembly tasks in skillgraph", assembly_seq->num_tasks());
 
 // Setup initial environment blocks from skillgraph
 setupSkillgraphEnvironment();
 
 // Plan and execute each assembly step from skillgraph
 for (int task_idx = 0; task_idx < assembly_seq->num_tasks(); ++task_idx) {
 auto task = assembly_seq->get_task_at(task_idx);
 
 RCLCPP_INFO(node_->get_logger(), "Processing skillgraph task %d: %s", 
 task_idx, task->name.c_str());
 
 // Get parsed locations from skillgraph task constraints
 if (!planAndExecuteSkillgraphTask(task, task_idx)) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to execute skillgraph task %d", task_idx);
 return false;
 }
 
 // Brief pause between tasks
 rclcpp::sleep_for(std::chrono::milliseconds(500));
 }
 
 RCLCPP_INFO(node_->get_logger(), "All skillgraph assembly tasks completed successfully!");
 return true;
 }
 
 /**
 * @brief Load skillgraph tasks and initialize environment
 */
 bool loadSkillgraphTasks(const std::string& task_file, const std::string& env_setup_file) {
 try {
 // Parse assembly tasks through skillgraph
 auto assembly_seq = std::make_shared<skillgraph::MagBlockAssemblySeq>(task_file);
 assembly_seq->print();
 
 // Load environment setup through skillgraph 
 std::ifstream env_stream(env_setup_file);
 if (env_stream.is_open()) {
 env_stream >> env_setup_; // Store in member variable
 env_stream.close();
 }
 
 // Store for later use
 skillgraph_->setAssemblySequence(assembly_seq);
 skillgraph_->setEnvironmentConfig(env_setup_);
 
 return true;
 } catch (const std::exception& e) {
 RCLCPP_ERROR(node_->get_logger(), "Exception loading skillgraph tasks: %s", e.what());
 return false;
 }
 }
 
 /**
 * @brief Plan and execute a single skillgraph task with MoveIt trajectory planning
 */
 bool planAndExecuteSkillgraphTask(skillgraph::TaskPtr task, int task_idx) {
 // Get locations from skillgraph task constraints
 const Json::Value& constraints = task->post_condition->constraints_json;
 
 // Extract parsed block coordinates from skillgraph
 double x_blocks = constraints["x"].asDouble();
 double y_blocks = constraints["y"].asDouble(); 
 double z_blocks = constraints["z"].asDouble();
 
 // Extract press_face and gripper_ori parameters for orientation constraints
 int press_face = constraints.get("press_face", 0).asInt();
 int gripper_ori = constraints.get("gripper_ori", 0).asInt();
 
 RCLCPP_INFO(node_->get_logger(), "=== ASSEMBLY TASK %d ===", task_idx);
 RCLCPP_INFO(node_->get_logger(), "Block coordinates: (%.3f, %.3f, %.3f), press_face: %d, gripper_ori: %d", 
 x_blocks, y_blocks, z_blocks, press_face, gripper_ori);
 
 // Get optimal robot for this location
 int robot_id = determineOptimalRobot(x_blocks, y_blocks, z_blocks);
 std::string robot_name = getRobotName(robot_id);
 
 // Transform coordinates and create poses
 double x_robot, y_robot, z_robot, rx, ry, rz;
 transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
 
 // Calculate orientations
 double pick_thetax = 0.0, pick_thetay = 180.0;
 double pick_thetaz = findOptimalThetaZ(press_face);
 
 // Get pick and place poses
 geometry_msgs::msg::Pose pick_pose, place_pose;
 double pick_x, pick_y, pick_z;
 if (getBlockPickPose(task_idx, pick_x, pick_y, pick_z)) {
 // Use the exact position where the block is visualized
 pick_pose = createPoseWithOrientation(pick_x, pick_y, pick_z, pick_thetax, pick_thetay, pick_thetaz);
 RCLCPP_INFO(node_->get_logger(), "Pick pose from environment: (%.3f, %.3f, %.3f)", 
 pick_x, pick_y, pick_z);
 } else {
 // Fallback to default table location with proper height
 pick_pose = createPoseWithOrientation(x_robot, y_robot, z_robot, pick_thetax, pick_thetay, pick_thetaz);
 RCLCPP_INFO(node_->get_logger(), "Pick pose fallback: (%.3f, %.3f, %.3f)", 
 x_robot, y_robot, z_robot);
 }
 
 place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
 
 RCLCPP_INFO(node_->get_logger(), "=== COORDINATE VERIFICATION ===");
 RCLCPP_INFO(node_->get_logger(), "Robot: %s", robot_name.c_str());
 RCLCPP_INFO(node_->get_logger(), "Block visualization location: (%.3f, %.3f, %.3f)", 
 pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "Robot pick target: (%.3f, %.3f, %.3f)", 
 pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "Robot place target: (%.3f, %.3f, %.3f)", 
 place_pose.position.x, place_pose.position.y, place_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "These coordinates should MATCH exactly!");
 RCLCPP_INFO(node_->get_logger(), "=== END VERIFICATION ===");
 
 // Plan trajectory
 std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
 std::string object_name = "block_" + std::to_string(task_idx);
 std::string pick_block_name = "b" + std::to_string(task_idx + 1); // Original block name
 
 RCLCPP_INFO(node_->get_logger(), "Starting trajectory planning for %s", object_name.c_str());
 
 if (!planPickPlaceTrajectory(robot_id, pick_pose, place_pose, object_name, press_face, trajectories)) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to plan trajectory for task %d", task_idx);
 return false;
 }
 
 RCLCPP_INFO(node_->get_logger(), "Trajectory planning successful, now executing simulation");
 
 // Execute trajectory simulation  
 if (!executeTrajectories(robot_id, trajectories, object_name)) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to execute trajectory for task %d", task_idx);
 return false;
 }
 RCLCPP_INFO(node_->get_logger(), "Block %s picked from (%.3f, %.3f, %.3f)",
 pick_block_name.c_str(), pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
 RCLCPP_INFO(node_->get_logger(), "Gripper moved to (%.3f, %.3f, %.3f), block placed at (%.3f, %.3f, %.3f)",
 place_pose.position.x, place_pose.position.y, place_pose.position.z,
 place_pose.position.x, place_pose.position.y, place_pose.position.z);
 
 return true;
 }
 
 
 /**
 * @brief Setup initial environment blocks from skillgraph
 */
 void setupSkillgraphEnvironment() {
 RCLCPP_INFO(node_->get_logger(), "Setting up initial environment from skillgraph...");
 
 // Setup blocks from environment configuration using the EXACT same coordinates as robot planning
 if (!env_setup_.empty()) {
 RCLCPP_INFO(node_->get_logger(), "Setting up blocks from environment configuration...");
 
 for (auto it = env_setup_.begin(); it != env_setup_.end(); ++it) {
 std::string block_name = it.key().asString();
 Json::Value block_data = *it;
 
 if (block_data.isMember("x") && block_data.isMember("y") && block_data.isMember("z")) {
 double x_sg = block_data["x"].asDouble();
 double y_sg = block_data["y"].asDouble(); 
 double z_sg = block_data["z"].asDouble();
 
 // Use the EXACT same coordinate transformation as used for robot pick poses
 double x_robot, y_robot, z_robot, rx, ry, rz;
 transformSkillgraphToRobot(x_sg, y_sg, z_sg, x_robot, y_robot, z_robot, rx, ry, rz);
 
 // Visual block visualization disabled
 // addVisualBlock(block_name, x_robot, y_robot, z_robot);
 
 RCLCPP_INFO(node_->get_logger(), "Block location calculated for %s at: (%.3f,%.3f,%.3f) - visualization disabled",
 block_name.c_str(), x_robot, y_robot, z_robot);
 }
 }
 }
 
 RCLCPP_INFO(node_->get_logger(), "Block visualization disabled - coordinates calculated only");
 }
 
 void setupInitialBlocks(const std::string& env_setup_file) {
 std::ifstream env_stream(env_setup_file);
 if (!env_stream.is_open()) {
 RCLCPP_ERROR(node_->get_logger(), "Failed to open env setup file");
 return;
 }
 
 Json::Value env_setup;
 env_stream >> env_setup;
 env_stream.close();
 
 for (auto it = env_setup.begin(); it != env_setup.end(); ++it) {
 std::string block_name = it.key().asString();
 Json::Value block_data = *it;
 
 double x_blocks = block_data["x"].asDouble();
 double y_blocks = block_data["y"].asDouble();
 double z_blocks = block_data["z"].asDouble();
 
 // Transform to world coordinates for visualization
 double x_world, y_world, z_world, rx, ry, rz;
 if (skillgraph_->blockToRobotFrame("center_arm", x_blocks, y_blocks, z_blocks,
 x_world, y_world, z_world, rx, ry, rz)) {
 addCollisionBlock(block_name, x_world, y_world, 0.25); // Reduced table height (0.8 - 0.55)
 }
 }
 }
 
 void addCollisionBlock(const std::string& block_id, double x, double y, double z, 
 double size_x = 0.025, double size_y = 0.025, double size_z = 0.025) {
 moveit_msgs::msg::CollisionObject collision_object;
 collision_object.header.frame_id = "right_base_link"; // Use robot base frame to match planning
 collision_object.id = block_id;

 shape_msgs::msg::SolidPrimitive primitive;
 primitive.type = primitive.BOX;
 primitive.dimensions.resize(3);
 primitive.dimensions[primitive.BOX_X] = size_x;
 primitive.dimensions[primitive.BOX_Y] = size_y;
 primitive.dimensions[primitive.BOX_Z] = size_z;

 geometry_msgs::msg::Pose block_pose;
 block_pose.orientation.w = 1.0;
 block_pose.position.x = x;
 block_pose.position.y = y;
 block_pose.position.z = z;

 collision_object.primitives.push_back(primitive);
 collision_object.primitive_poses.push_back(block_pose);
 collision_object.operation = collision_object.ADD;

 std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
 collision_objects.push_back(collision_object);
 planning_scene_interface_->addCollisionObjects(collision_objects);
 
 RCLCPP_INFO(node_->get_logger(), "Added collision block: %s at (%.3f, %.3f, %.3f) in right_base_link frame [size: %.3fx%.3fx%.3f]", 
 block_id.c_str(), x, y, z, size_x, size_y, size_z);
 }
 
 /**
 * @brief Add visual block without collision for pick-up locations - DISABLED
 */
 void addVisualBlock(const std::string& block_id, double x, double y, double z, 
 double size_x = 0.025, double size_y = 0.025, double size_z = 0.025) {
 
 // Visual block visualization disabled
 RCLCPP_DEBUG(node_->get_logger(), "Visual block visualization disabled for: %s at (%.3f, %.3f, %.3f)", 
 block_id.c_str(), x, y, z);
 }
 
 /**
 * @brief Remove visual block when picked up - DISABLED
 */
 void removeVisualBlock(const std::string& block_id) {
 // Visual block removal disabled
 RCLCPP_DEBUG(node_->get_logger(), "Visual block removal disabled for: %s", block_id.c_str());
 }
 
 geometry_msgs::msg::Pose createPose(double x, double y, double z, double rx = 0.0, double ry = 0.0, double rz = 0.0) {
 geometry_msgs::msg::Pose pose;
 pose.position.x = x;
 pose.position.y = y;
 pose.position.z = z;
 // Simple orientation - could be enhanced with proper quaternion conversion
 pose.orientation.w = 1.0;
 pose.orientation.x = 0.0;
 pose.orientation.y = 0.0;
 pose.orientation.z = 0.0;
 return pose;
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
 * @brief Find optimal thetaz that doesn't block the required press_face
 */
 double findOptimalThetaZ(int press_face) {
 std::vector<double> candidate_angles = {0.0, 90.0, 180.0, 270.0};
 
 for (double angle : candidate_angles) {
 std::set<int> blocked_faces = getBlockedFaces(angle);
 if (blocked_faces.find(press_face) == blocked_faces.end()) {
 RCLCPP_INFO(node_->get_logger(), "Found optimal thetaz=%.0f° for press_face=%d (blocked faces: %s)", 
 angle, press_face, setToString(blocked_faces).c_str());
 return angle;
 }
 }
 
 // If no angle works (shouldn't happen with valid press_face), default to 0
 RCLCPP_WARN(node_->get_logger(), "No optimal thetaz found for press_face=%d, defaulting to 0°", press_face);
 return 0.0;
 }
 
 /**
 * @brief Convert set to string for logging
 */
 std::string setToString(const std::set<int>& s) {
 std::string result = "{";
 bool first = true;
 for (int val : s) {
 if (!first) result += ",";
 result += std::to_string(val);
 first = false;
 }
 result += "}";
 return result;
 }
 
 /**
 * @brief Create pose with proper quaternion conversion from Euler angles
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
 Eigen::AngleAxisd rollAngle(thetax, Eigen::Vector3d::UnitX());
 Eigen::AngleAxisd pitchAngle(thetay, Eigen::Vector3d::UnitY());
 Eigen::AngleAxisd yawAngle(thetaz, Eigen::Vector3d::UnitZ());
 
 Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
 
 pose.orientation.x = quaternion.x();
 pose.orientation.y = quaternion.y();
 pose.orientation.z = quaternion.z();
 pose.orientation.w = quaternion.w();
 
 RCLCPP_DEBUG(node_->get_logger(), "Created pose: pos(%.3f,%.3f,%.3f) euler(%.1f,%.1f,%.1f) quat(%.3f,%.3f,%.3f,%.3f)",
 x, y, z, thetax_deg, thetay_deg, thetaz_deg,
 pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
 
 return pose;
 }

 /**
 * @brief Wait for MoveIt2 services to be available
 */
 bool waitForMoveItServices() {
 RCLCPP_INFO(node_->get_logger(), "Waiting for MoveIt2 planning services...");
 
 // Wait for move_group action server to be available
 const auto timeout = std::chrono::seconds(30);
 const auto start_time = std::chrono::steady_clock::now();
 
 while (rclcpp::ok()) {
 if (std::chrono::steady_clock::now() - start_time > timeout) {
 RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for MoveIt2 services");
 return false;
 }
 
 // Check if the move_group action servers are available
 auto client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
 node_, "move_action");
 
 if (client->wait_for_action_server(std::chrono::seconds(1))) {
 RCLCPP_INFO(node_->get_logger(), "MoveIt2 move_group action server is available");
 
 // Also check for planning scene service
 auto planning_scene_client = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
 "get_planning_scene");
 
 if (planning_scene_client->wait_for_service(std::chrono::seconds(1))) {
 RCLCPP_INFO(node_->get_logger(), "MoveIt2 planning scene service is available");
 return true;
 }
 }
 
 RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
 "Waiting for MoveIt2 services to become available...");
 rclcpp::sleep_for(std::chrono::milliseconds(500));
 }
 
 return false;
 }

 void transformSkillgraphToRobot(double x_sg, double y_sg, double z_sg, 
 double& x_robot, double& y_robot, double& z_robot, 
 double& rx, double& ry, double& rz) {
 // Transform block frame coordinates to RIGHT robot base frame coordinates
 // 
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
 double table_height = 0.15; // Higher table height to avoid robot collisions
 double block_height_offset = 0.0125; // Half block height (2.5cm / 2) to place on table surface
 
 // Transform from block frame to right robot base frame
 // Note: X direction is inverted between block frame and robot frame
 x_robot = block_frame_offset_x - (x_sg * block_size); // Inverted X direction
 y_robot = -(block_frame_offset_y + (y_sg * block_size)); // Negative Y for right robot arm
 z_robot = table_height + block_frame_offset_z + (z_sg * block_size) + block_height_offset; // Place on table surface
 
 // Default orientation (gripper pointing down)
 rx = 0.0;
 ry = 0.0; 
 rz = 0.0;
 
 RCLCPP_INFO(node_->get_logger(), "Block frame to right robot transform: (%.1f,%.1f,%.1f) -> (%.3f,%.3f,%.3f)",
 x_sg, y_sg, z_sg, x_robot, y_robot, z_robot);
 RCLCPP_INFO(node_->get_logger(), "Block height check: table_height=%.3f + z_offset=%.3f + block_half_height=%.3f = %.3f total",
 table_height, z_sg * block_size, block_height_offset, z_robot);
 }
 
 bool getBlockPickPose(int task_idx, double& x, double& y, double& z) {
 // Map task index to block name (task 0 -> b1, task 1 -> b2, etc.)
 std::string block_name = "b" + std::to_string(task_idx + 1);
 
 // Look up the block position in the environment setup
 if (env_setup_.isMember(block_name)) {
 const Json::Value& block_pos = env_setup_[block_name];
 if (block_pos.isMember("x") && block_pos.isMember("y") && block_pos.isMember("z")) {
 double x_sg = block_pos["x"].asDouble();
 double y_sg = block_pos["y"].asDouble();
 double z_sg = block_pos["z"].asDouble();
 
 // Transform from skillgraph to robot coordinates
 double rx, ry, rz;
 transformSkillgraphToRobot(x_sg, y_sg, z_sg, x, y, z, rx, ry, rz);
 
 RCLCPP_DEBUG(node_->get_logger(), "Found pick pose for block %s at (%.3f, %.3f, %.3f)",
 block_name.c_str(), x, y, z);
 return true;
 }
 }
 
 RCLCPP_WARN(node_->get_logger(), "Could not find block %s in environment setup", block_name.c_str());
 return false;
 }
 
 /**
 * @brief Get approach direction offset based on press_face for place operation
 */
 std::tuple<double, double, double> getPlaceApproachOffset(int press_face, double approach_distance = 0.15) {
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
 RCLCPP_WARN(node_->get_logger(), "Unknown press_face=%d, defaulting to +Z approach", press_face);
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
 RCLCPP_WARN(node_->get_logger(), "Unknown gripper_ori=%d, defaulting to (0,180)", gripper_ori);
 return {0.0, 180.0}; // Default orientation
 }
 }
 
 /**
 * @brief Calculate place pose with proper approach direction and orientation
 */
 geometry_msgs::msg::Pose createPlacePose(double x, double y, double z, 
 int press_face, int gripper_ori, double pick_thetaz) {
 // Get place orientation from gripper_ori
 auto [place_thetax, place_thetay] = getPlaceOrientation(gripper_ori);
 
 // thetaz stays the same as pick operation
 double place_thetaz = pick_thetaz;
 
 RCLCPP_INFO(node_->get_logger(), "Place orientation: press_face=%d, gripper_ori=%d -> thetax=%.1f°, thetay=%.1f°, thetaz=%.1f°",
 press_face, gripper_ori, place_thetax, place_thetay, place_thetaz);
 
 return createPoseWithOrientation(x, y, z, place_thetax, place_thetay, place_thetaz);
 }
 
 /**
 * @brief Calculate approach pose for place operation based on press_face
 */
 geometry_msgs::msg::Pose createPlaceApproachPose(const geometry_msgs::msg::Pose& place_pose, 
 int press_face, double approach_distance = 0.15) {
 auto [dx, dy, dz] = getPlaceApproachOffset(press_face, approach_distance);
 
 geometry_msgs::msg::Pose approach_pose = place_pose;
 approach_pose.position.x += dx;
 approach_pose.position.y += dy; 
 approach_pose.position.z += dz;
 
 RCLCPP_INFO(node_->get_logger(), "Place approach offset for press_face=%d: (%.3f, %.3f, %.3f)", 
 press_face, dx, dy, dz);
 
 return approach_pose;
 }
 
 int determineOptimalRobot(double x_target, double y_target, double z_target) {
 // Default to right arm for all positions since we're focusing on right arm only
 // and the coordinate transformation is specifically for right robot base frame
 return 2; // right_arm
 }
 
 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getRobotArm(int robot_id) {
 switch(robot_id) {
 case 0: return left_arm_group_;
 case 1: return center_arm_group_;
 case 2: return right_arm_group_;
 default: return right_arm_group_; // default to right_arm
 }
 }
 
 std::string getRobotName(int robot_id) {
 switch(robot_id) {
 case 0: return "left_arm";
 case 1: return "center_arm";
 case 2: return "right_arm";
 default: return "right_arm"; // default to right_arm
 }
 }
 
 int getRobotIdFromName(const std::string& robot_name) {
 if (robot_name == "left_arm") return 0;
 if (robot_name == "center_arm") return 1;
 if (robot_name == "right_arm") return 2;
 return 2; // default to right_arm
 }

private:
 rclcpp::Node::SharedPtr node_;
 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_group_;
 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_group_;
 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> center_arm_group_;
 std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
 std::shared_ptr<skillgraph::MagBlockSkillGraph> skillgraph_;
 
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
 
 // Planning scene client for simulation
 rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client_;
 moveit_msgs::msg::PlanningScene planning_scene_diff_;
 
 Json::Value env_setup_; // Environment setup configuration
 bool wait_for_confirmation_; // Control execution flow
};

int main(int argc, char** argv)
{
 rclcpp::init(argc, argv);
 auto node = rclcpp::Node::make_shared("magblock_assembly_executor");
 
 auto executor = std::make_shared<MagBlockAssemblyExecutor>(node);
 
 // Default task files
 std::string task_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/I.json";
 std::string env_setup_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_I.json";
 
 // Parse command line arguments
 if (argc > 1) {
 task_file = argv[1];
 }
 if (argc > 2) {
 env_setup_file = argv[2];
 }
 
 RCLCPP_INFO(node->get_logger(), "Starting MagBlock Assembly Executor with Planning Scene Simulation");
 RCLCPP_INFO(node->get_logger(), "Mode: Simulation via planning scene updates");
 RCLCPP_INFO(node->get_logger(), "Task file: %s", task_file.c_str());
 RCLCPP_INFO(node->get_logger(), "Environment file: %s", env_setup_file.c_str());
 
 // Wait for MoveIt to be ready
 rclcpp::sleep_for(std::chrono::seconds(3));
 
 try {
 // Execute assembly with simulation-based trajectory visualization
 if (!executor->executeAssemblyWithTrajectories(task_file, env_setup_file)) {
 RCLCPP_ERROR(node->get_logger(), "Failed to execute assembly task");
 return 1;
 }
 
 RCLCPP_INFO(node->get_logger(), "Assembly simulation completed successfully!");
 
 // Keep node alive for visualization
 rclcpp::spin(node);
 
 } catch (const std::exception& ex) {
 RCLCPP_ERROR(node->get_logger(), "Exception: %s", ex.what());
 return 1;
 }
 
 rclcpp::shutdown();
 return 0;
}




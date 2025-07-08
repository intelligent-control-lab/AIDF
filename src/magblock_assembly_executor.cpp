#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
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
        
        // Setup trajectory visualization publishers
        trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/move_group/display_planned_path", 10);
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/magblock_trajectory_markers", 10);
        
        // Add additional trajectory publisher for RViz
        trajectory_display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10);
        
        // Initialize MoveIt Visual Tools for interactive visualization
        moveit_visual_tools_left_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "world", "/rviz_visual_tools_left");
        moveit_visual_tools_right_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "world", "/rviz_visual_tools_right");
        moveit_visual_tools_center_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "world", "/rviz_visual_tools_center");
        
        // Initialize visual tools
        moveit_visual_tools_left_->loadRobotStatePub("/display_robot_state");
        moveit_visual_tools_right_->loadRobotStatePub("/display_robot_state");
        moveit_visual_tools_center_->loadRobotStatePub("/display_robot_state");
        
        // Clear existing markers
        moveit_visual_tools_left_->deleteAllMarkers();
        moveit_visual_tools_right_->deleteAllMarkers();
        moveit_visual_tools_center_->deleteAllMarkers();
        
        // Set up planning parameters
        setupPlanningParameters();
        
        // Initialize the MagBlock skillgraph
        std::string config_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/skillgraph.json";
        skillgraph_ = std::make_shared<skillgraph::MagBlockSkillGraph>(config_path);
        skillgraph_->initialize();
        
        // Control execution flow
        wait_for_confirmation_ = true;  // Set to false for automatic execution
        
        RCLCPP_INFO(node_->get_logger(), "MagBlock Assembly Executor initialized with skillgraph trajectory planning");
    }
    
    void setupPlanningParameters() {
        std::vector<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> arms = 
            {left_arm_group_, right_arm_group_, center_arm_group_};
        
        for (auto& arm : arms) {
            arm->setPlanningTime(15.0);  // More time for complex planning
            arm->setMaxVelocityScalingFactor(0.5);
            arm->setMaxAccelerationScalingFactor(0.3);
            arm->setNumPlanningAttempts(10);
            arm->setGoalTolerance(0.01);  // 1cm tolerance
            arm->setPlannerId("RRTConnect");  // Use RRT Connect for reliability
        }
        
        // Set pose reference frame to robot base for each arm
        left_arm_group_->setPoseReferenceFrame("left_base_link");
        right_arm_group_->setPoseReferenceFrame("right_base_link");
        center_arm_group_->setPoseReferenceFrame("center_base_link");
    }
    
    /**
     * @brief Interactive prompt for user confirmation with visual tools
     */
    void prompt(const std::string& message) {
        RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
        RCLCPP_INFO(node_->get_logger(), "Press 'Next' in the RvizVisualToolsGui window to continue");
        
        // Wait for user input in RViz
        moveit_visual_tools_center_->prompt(message);
    }
    
    /**
     * @brief Draw title in RViz
     */
    void draw_title(const std::string& title) {
        // Create identity transform for text position
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation() = Eigen::Vector3d(0, 0, 1.5);  // Position text above the scene
        
        moveit_visual_tools_center_->publishText(
            text_pose, 
            title, 
            rviz_visual_tools::WHITE, 
            rviz_visual_tools::XLARGE
        );
    }
    
    /**
     * @brief Draw trajectory tool path in RViz
     */
    void draw_trajectory_tool_path(const moveit_msgs::msg::RobotTrajectory& trajectory, 
                                  const std::string& robot_name) {
        auto visual_tools = getVisualTools(robot_name);
        if (visual_tools) {
            visual_tools->publishTrajectoryLine(trajectory, getJointModelGroup(robot_name));
        }
    }
    
    /**
     * @brief Get visual tools for specific robot
     */
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> getVisualTools(const std::string& robot_name) {
        if (robot_name == "left_arm") return moveit_visual_tools_left_;
        if (robot_name == "right_arm") return moveit_visual_tools_right_;
        if (robot_name == "center_arm") return moveit_visual_tools_center_;
        return moveit_visual_tools_center_;
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
     * @brief Plan complete trajectory for a pick and place operation with interactive visualization
     */
    bool planPickPlaceTrajectory(int robot_id, 
                                const geometry_msgs::msg::Pose& pick_pose,
                                const geometry_msgs::msg::Pose& place_pose,
                                const std::string& object_name,
                                int press_face,
                                std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
        
        auto arm = getRobotArm(robot_id);
        std::string robot_name = getRobotName(robot_id);
        auto visual_tools = getVisualTools(robot_name);
        
        RCLCPP_INFO(node_->get_logger(), "Planning pick-place trajectory for %s with object %s", 
                   robot_name.c_str(), object_name.c_str());
        
        trajectories.clear();
        
        // Print ALL target locations upfront for debugging, regardless of planning success
        geometry_msgs::msg::Pose approach_pose = pick_pose;
        approach_pose.position.z += 0.15;  // 15cm above pick
        
        geometry_msgs::msg::Pose lift_pose = pick_pose;
        lift_pose.position.z += 0.1;  // 10cm lift
        
        // Calculate transport and retract poses based on press_face approach direction
        geometry_msgs::msg::Pose transport_pose = createPlaceApproachPose(place_pose, press_face, 0.15);
        geometry_msgs::msg::Pose retract_pose = createPlaceApproachPose(place_pose, press_face, 0.1);
        
        RCLCPP_INFO(node_->get_logger(), "=== ALL TARGET LOCATIONS FOR %s (press_face=%d) ===", object_name.c_str(), press_face);
        RCLCPP_INFO(node_->get_logger(), "APPROACH TARGET: (%.3f, %.3f, %.3f)", 
                   approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "PICK TARGET: (%.3f, %.3f, %.3f)", 
                   pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "LIFT TARGET: (%.3f, %.3f, %.3f)", 
                   lift_pose.position.x, lift_pose.position.y, lift_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "TRANSPORT TARGET: (%.3f, %.3f, %.3f)", 
                   transport_pose.position.x, transport_pose.position.y, transport_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "PLACE TARGET: (%.3f, %.3f, %.3f)", 
                   place_pose.position.x, place_pose.position.y, place_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "RETRACT TARGET: (%.3f, %.3f, %.3f)", 
                   retract_pose.position.x, retract_pose.position.y, retract_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "=== END TARGET LOCATIONS ===");
        
        // Step 1: Plan approach to pick position (pre-grasp)
        
        // Set target pose
        arm->setPoseTarget(approach_pose);
        
        // Interactive planning for approach
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan approach trajectory");
        draw_title("Planning Approach to " + object_name);
        visual_tools->trigger();
        
        // Create a plan
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        auto approach_success = static_cast<bool>(arm->plan(approach_plan));
        
        if (approach_success) {
            trajectories.push_back(approach_plan.trajectory_);
            draw_trajectory_tool_path(approach_plan.trajectory_, robot_name);
            visual_tools->trigger();
            prompt("Press 'Next' in the RvizVisualToolsGui window to continue to pick planning");
        } else {
            draw_title("Approach Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan approach trajectory");
            return false;
        }
        
        // Step 2: Plan descent to pick position
        arm->setPoseTarget(pick_pose);
        
        RCLCPP_INFO(node_->get_logger(), "PICK TARGET: (%.3f, %.3f, %.3f)", 
                   pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
        
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan pick trajectory");
        draw_title("Planning Pick for " + object_name);
        visual_tools->trigger();
        
        moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
        auto pick_success = static_cast<bool>(arm->plan(pick_plan));
        
        if (pick_success) {
            trajectories.push_back(pick_plan.trajectory_);
            draw_trajectory_tool_path(pick_plan.trajectory_, robot_name);
            visual_tools->trigger();
            prompt("Press 'Next' in the RvizVisualToolsGui window to continue to lift planning");
        } else {
            draw_title("Pick Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan pick trajectory");
            return false;
        }
        
        // Step 3: Plan lift after pick
        lift_pose.position.z = pick_pose.position.z + 0.1;  // 10cm lift
        
        RCLCPP_INFO(node_->get_logger(), "LIFT TARGET: (%.3f, %.3f, %.3f)", 
                   lift_pose.position.x, lift_pose.position.y, lift_pose.position.z);
        
        arm->setPoseTarget(lift_pose);
        
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan lift trajectory");
        draw_title("Planning Lift for " + object_name);
        visual_tools->trigger();
        
        moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
        auto lift_success = static_cast<bool>(arm->plan(lift_plan));
        
        if (lift_success) {
            trajectories.push_back(lift_plan.trajectory_);
            draw_trajectory_tool_path(lift_plan.trajectory_, robot_name);
            visual_tools->trigger();
            prompt("Press 'Next' in the RvizVisualToolsGui window to continue to transport planning");
        } else {
            draw_title("Lift Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan lift trajectory");
            return false;
        }
        
        // Step 4: Plan transport to place position using press_face approach direction
        
        RCLCPP_INFO(node_->get_logger(), "TRANSPORT TARGET: (%.3f, %.3f, %.3f) using press_face=%d approach", 
                   transport_pose.position.x, transport_pose.position.y, transport_pose.position.z, press_face);
        
        arm->setPoseTarget(transport_pose);
        
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan transport trajectory");
        draw_title("Planning Transport for " + object_name);
        visual_tools->trigger();
        
        moveit::planning_interface::MoveGroupInterface::Plan transport_plan;
        auto transport_success = static_cast<bool>(arm->plan(transport_plan));
        
        if (transport_success) {
            trajectories.push_back(transport_plan.trajectory_);
            draw_trajectory_tool_path(transport_plan.trajectory_, robot_name);
            visual_tools->trigger();
            prompt("Press 'Next' in the RvizVisualToolsGui window to continue to place planning");
        } else {
            draw_title("Transport Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan transport trajectory");
            return false;
        }
        
        // Step 5: Plan descent to place position
        arm->setPoseTarget(place_pose);
        
        RCLCPP_INFO(node_->get_logger(), "PLACE TARGET: (%.3f, %.3f, %.3f)", 
                   place_pose.position.x, place_pose.position.y, place_pose.position.z);
        
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan place trajectory");
        draw_title("Planning Place for " + object_name);
        visual_tools->trigger();
        
        moveit::planning_interface::MoveGroupInterface::Plan place_plan;
        auto place_success = static_cast<bool>(arm->plan(place_plan));
        
        if (place_success) {
            trajectories.push_back(place_plan.trajectory_);
            draw_trajectory_tool_path(place_plan.trajectory_, robot_name);
            visual_tools->trigger();
            prompt("Press 'Next' in the RvizVisualToolsGui window to continue to retract planning");
        } else {
            draw_title("Place Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan place trajectory");
            return false;
        }
        
        // Step 6: Plan retract after place using press_face approach direction
        
        RCLCPP_INFO(node_->get_logger(), "RETRACT TARGET: (%.3f, %.3f, %.3f) using press_face=%d approach", 
                   retract_pose.position.x, retract_pose.position.y, retract_pose.position.z, press_face);
        
        arm->setPoseTarget(retract_pose);
        
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan retract trajectory");
        draw_title("Planning Retract for " + object_name);
        visual_tools->trigger();
        
        moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
        auto retract_success = static_cast<bool>(arm->plan(retract_plan));
        
        if (retract_success) {
            trajectories.push_back(retract_plan.trajectory_);
            draw_trajectory_tool_path(retract_plan.trajectory_, robot_name);
            visual_tools->trigger();
            draw_title("All Planning Complete for " + object_name);
            visual_tools->trigger();
        } else {
            draw_title("Retract Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan retract trajectory");
            return false;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Successfully planned %zu trajectory segments for %s", 
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
     * @brief Visualize planned trajectories in RViz
     */
    void visualizeTrajectories(const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
                              const std::string& robot_name,
                              const std::string& object_name) {
        
        if (trajectories.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No trajectories to visualize");
            return;
        }
        
        // Get the robot arm for this visualization
        auto arm = getRobotArm(getRobotIdFromName(robot_name));
        
        // Create display trajectory message
        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        display_trajectory.model_id = "three_arm_robot";  // Use the robot model name
        
        // Set the trajectory start state (current robot state)
        moveit_msgs::msg::RobotState robot_state;
        robot_state.joint_state.header.stamp = node_->now();
        robot_state.joint_state.header.frame_id = "world";
        
        // Get current joint values
        std::vector<double> joint_values = arm->getCurrentJointValues();
        robot_state.joint_state.name = arm->getJointNames();
        robot_state.joint_state.position = joint_values;
        
        display_trajectory.trajectory_start = robot_state;
        
        // Add all trajectory segments
        for (size_t i = 0; i < trajectories.size(); ++i) {
            display_trajectory.trajectory.push_back(trajectories[i]);
        }
        
        // Publish the complete trajectory on both topics
        trajectory_pub_->publish(display_trajectory);
        trajectory_display_pub_->publish(display_trajectory);
        
        // Create trajectory markers
        createTrajectoryMarkers(trajectories, robot_name, object_name);
        
        RCLCPP_INFO(node_->get_logger(), "Published complete trajectory visualization for %s handling %s", 
                   robot_name.c_str(), object_name.c_str());
    }
    
    /**
     * @brief Create trajectory markers for enhanced visualization
     */
    void createTrajectoryMarkers(const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
                                const std::string& robot_name,
                                const std::string& object_name) {
        
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;
        
        // Colors for different robots
        std::map<std::string, std::vector<float>> robot_colors = {
            {"left_arm", {1.0, 0.0, 0.0}},    // Red
            {"right_arm", {0.0, 1.0, 0.0}},   // Green  
            {"center_arm", {0.0, 0.0, 1.0}}   // Blue
        };
        
        auto color = robot_colors[robot_name];
        
        for (size_t traj_idx = 0; traj_idx < trajectories.size(); ++traj_idx) {
            const auto& trajectory = trajectories[traj_idx];
            
            // Create path markers for each trajectory segment
            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = "world";
            path_marker.header.stamp = node_->now();
            path_marker.ns = robot_name + "_" + object_name;
            path_marker.id = marker_id++;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;
            
            path_marker.scale.x = 0.01;  // Line width
            path_marker.color.r = color[0];
            path_marker.color.g = color[1];
            path_marker.color.b = color[2];
            path_marker.color.a = 0.8;
            
            // Add waypoints (this would need forward kinematics to get poses)
            // For now, add a simple visualization
            geometry_msgs::msg::Point start_point, end_point;
            start_point.x = 0.0; start_point.y = 0.0; start_point.z = 0.8;
            end_point.x = 0.5; end_point.y = 0.0; end_point.z = 0.8;
            
            path_marker.points.push_back(start_point);
            path_marker.points.push_back(end_point);
            
            marker_array.markers.push_back(path_marker);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    /**
     * @brief Execute planned trajectories with interactive visualization
     */
    bool executeTrajectories(int robot_id, 
                            const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
                            const std::string& object_name) {
        
        auto arm = getRobotArm(robot_id);
        std::string robot_name = getRobotName(robot_id);
        auto visual_tools = getVisualTools(robot_name);
        
        RCLCPP_INFO(node_->get_logger(), "Executing %zu trajectory segments for %s", 
                   trajectories.size(), object_name.c_str());
        
        std::vector<std::string> segment_names = {
            "Approach", "Pick", "Lift", "Transport", "Place", "Retract"
        };
        
        for (size_t i = 0; i < trajectories.size(); ++i) {
            std::string segment_name = (i < segment_names.size()) ? segment_names[i] : "Segment " + std::to_string(i+1);
            
            RCLCPP_INFO(node_->get_logger(), "Executing %s segment %zu/%zu", 
                       segment_name.c_str(), i+1, trajectories.size());
            
            // Interactive execution prompt
            prompt("Press 'Next' in the RvizVisualToolsGui window to execute " + segment_name);
            draw_title("Executing " + segment_name + " for " + object_name);
            visual_tools->trigger();
            
            // Execute the trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectories[i];
            
            auto result = arm->execute(plan);
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                draw_title(segment_name + " Execution Failed!");
                visual_tools->trigger();
                RCLCPP_ERROR(node_->get_logger(), "Failed to execute trajectory segment %zu (%s)", 
                           i, segment_name.c_str());
                return false;
            }
            
            // Show completion
            draw_title(segment_name + " Completed!");
            visual_tools->trigger();
            
            // Small pause between segments
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Final completion message
        draw_title("All Execution Complete for " + object_name);
        visual_tools->trigger();
        
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
            
            rclcpp::sleep_for(std::chrono::seconds(1));
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
                env_stream >> env_setup_;  // Store in member variable
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
        
        RCLCPP_INFO(node_->get_logger(), "Skillgraph parsed block coordinates: (%.3f, %.3f, %.3f), press_face: %d, gripper_ori: %d", 
                   x_blocks, y_blocks, z_blocks, press_face, gripper_ori);
        
        // Get optimal robot for this location using our improved robot selection logic
        int robot_id = determineOptimalRobot(x_blocks, y_blocks, z_blocks);
        std::string robot_name = getRobotName(robot_id);
        
        // Transform from skillgraph block coordinates to robot frame using our corrected transformation
        double x_robot, y_robot, z_robot, rx, ry, rz;
        transformSkillgraphToRobot(x_blocks, y_blocks, z_blocks, x_robot, y_robot, z_robot, rx, ry, rz);
        
        RCLCPP_INFO(node_->get_logger(), "Transformed to robot frame %s: (%.3f, %.3f, %.3f)", 
                   robot_name.c_str(), x_robot, y_robot, z_robot);
        
        // Calculate optimal pick orientation based on press_face constraints
        // Pick approach is always from above: thetax=0deg, thetay=180deg
        double pick_thetax = 0.0;   // Always approach from above
        double pick_thetay = 180.0; // Gripper pointing down
        double pick_thetaz = findOptimalThetaZ(press_face);
        
        RCLCPP_INFO(node_->get_logger(), "Pick orientation: thetax=%.1f°, thetay=%.1f°, thetaz=%.1f° (press_face=%d)", 
                   pick_thetax, pick_thetay, pick_thetaz, press_face);
        
        // Get pick location from environment setup
        geometry_msgs::msg::Pose pick_pose;
        double pick_x, pick_y, pick_z;
        if (getBlockPickPose(task_idx, pick_x, pick_y, pick_z)) {
            pick_pose = createPoseWithOrientation(pick_x, pick_y, pick_z, pick_thetax, pick_thetay, pick_thetaz);
            RCLCPP_INFO(node_->get_logger(), "PICK TARGET: Sending robot to pick location: (%.3f, %.3f, %.3f) with orientation (%.1f°, %.1f°, %.1f°)", 
                       pick_pose.position.x, pick_pose.position.y, pick_pose.position.z,
                       pick_thetax, pick_thetay, pick_thetaz);
        } else {
            // Fallback to default table pick location
            pick_pose = createPoseWithOrientation(x_robot, y_robot, 0.25, pick_thetax, pick_thetay, pick_thetaz);  // Reduced from 0.8 to 0.25 (0.8 - 0.55)
            RCLCPP_INFO(node_->get_logger(), "PICK TARGET (fallback): Sending robot to pick location: (%.3f, %.3f, %.3f) with orientation (%.1f°, %.1f°, %.1f°)", 
                       pick_pose.position.x, pick_pose.position.y, pick_pose.position.z,
                       pick_thetax, pick_thetay, pick_thetaz);
        }
        
        // Place location from skillgraph transformation with proper orientation based on gripper_ori and press_face
        geometry_msgs::msg::Pose place_pose = createPlacePose(x_robot, y_robot, z_robot, press_face, gripper_ori, pick_thetaz);
        RCLCPP_INFO(node_->get_logger(), "PLACE TARGET: Sending robot to place location: (%.3f, %.3f, %.3f) with press_face=%d, gripper_ori=%d", 
                   place_pose.position.x, place_pose.position.y, place_pose.position.z, press_face, gripper_ori);
        
        // Plan complete trajectory using MoveIt with skillgraph locations
        std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
        std::string object_name = "block_" + std::to_string(task_idx);
        
        if (!planPickPlaceTrajectory(robot_id, pick_pose, place_pose, object_name, press_face, trajectories)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan MoveIt trajectory for skillgraph task %d", task_idx);
            return false;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Interactive trajectory planning completed for skillgraph task");
        RCLCPP_INFO(node_->get_logger(), "Robot: %s, Pick: (%.3f,%.3f,%.3f), Place: (%.3f,%.3f,%.3f)", 
                   robot_name.c_str(), 
                   pick_pose.position.x, pick_pose.position.y, pick_pose.position.z,
                   place_pose.position.x, place_pose.position.y, place_pose.position.z);
        
        // Execute the planned trajectories with interactive visualization
        if (!executeTrajectories(robot_id, trajectories, object_name)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to execute trajectory for skillgraph task %d", task_idx);
            return false;
        }
        
        // Update planning scene with placed block from skillgraph coordinates
        addCollisionBlock(object_name + "_placed", x_robot, y_robot, z_robot + 0.025);
        
        return true;
    }
    
    
    /**
     * @brief Setup initial environment blocks from skillgraph
     */
    void setupSkillgraphEnvironment() {
        RCLCPP_INFO(node_->get_logger(), "Setting up initial environment from skillgraph...");
        
        // Get initial state from skillgraph
        skillgraph::State initial_state = skillgraph_->get_initial_state();
        
        // Add all objects from initial environment state
        for (const auto& object : initial_state.env_state.objects) {
            if (object && object->name.find("block") != std::string::npos) {
                // Add as collision object for MoveIt planning
                addCollisionBlock(object->name, object->x, object->y, object->z,
                                object->length, object->width, object->height);
                
                RCLCPP_INFO(node_->get_logger(), "Added skillgraph object: %s at (%.3f, %.3f, %.3f)",
                           object->name.c_str(), object->x, object->y, object->z);
            }
        }
        
        // Add table from skillgraph if available
        for (const auto& object : initial_state.env_state.objects) {
            if (object && object->name == "table") {
                addCollisionBlock("assembly_table", object->x, object->y, object->z,
                                object->length, object->width, object->height);
                RCLCPP_INFO(node_->get_logger(), "Added skillgraph table at (%.3f, %.3f, %.3f)",
                           object->x, object->y, object->z);
                break;
            }
        }
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
                addCollisionBlock(block_name, x_world, y_world, 0.25);  // Reduced table height (0.8 - 0.55)
            }
        }
    }
    
    void addCollisionBlock(const std::string& block_id, double x, double y, double z, 
                          double size_x = 0.025, double size_y = 0.025, double size_z = 0.025) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
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
        
        RCLCPP_INFO(node_->get_logger(), "Added collision block: %s at (%.3f, %.3f, %.3f)", 
                   block_id.c_str(), x, y, z);
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
            case 0:   return {2, 5, 3};
            case 90:  return {1, 4, 3};
            case 180: return {2, 5, 3};
            case 270: return {1, 4, 3};
            default:  return {2, 5, 3}; // Default fallback
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
        
        double block_size = 0.025;  // 2.5cm per block unit
        double block_frame_offset_x = 0.40;  // 40cm offset to right of right robot base
        double block_frame_offset_y = 0.11;  // 11cm offset in front of right robot base  
        double block_frame_offset_z = 0.0;   // Same height as right robot base
        double table_height = 0.195;         // Reduced table height (0.745 - 0.55 = 0.195)
        
        // Transform from block frame to right robot base frame
        // Note: X direction is inverted between block frame and robot frame
        x_robot = block_frame_offset_x - (x_sg * block_size);  // Inverted X direction
        y_robot = -(block_frame_offset_y + (y_sg * block_size));  // Negative Y for right robot arm
        z_robot = table_height + block_frame_offset_z + (z_sg * block_size);  // Reduced Z height
        
        // Default orientation (gripper pointing down)
        rx = 0.0;
        ry = 0.0; 
        rz = 0.0;
        
        RCLCPP_INFO(node_->get_logger(), "Block frame to right robot transform: (%.1f,%.1f,%.1f) -> (%.3f,%.3f,%.3f)",
                    x_sg, y_sg, z_sg, x_robot, y_robot, z_robot);
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
            case 0: return {0.0, 0.0, approach_distance};   // +Z approach
            case 1: return {-approach_distance, 0.0, 0.0};  // -X approach
            case 2: return {0.0, approach_distance, 0.0};   // +Y approach  
            case 3: return {0.0, 0.0, -approach_distance};  // -Z approach
            case 4: return {approach_distance, 0.0, 0.0};   // +X approach
            case 5: return {0.0, -approach_distance, 0.0};  // -Y approach
            default: 
                RCLCPP_WARN(node_->get_logger(), "Unknown press_face=%d, defaulting to +Z approach", press_face);
                return {0.0, 0.0, approach_distance};       // Default +Z approach
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
                return {0.0, 180.0};  // Default orientation
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
        return 2;  // right_arm
    }
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getRobotArm(int robot_id) {
        switch(robot_id) {
            case 0: return left_arm_group_;
            case 1: return center_arm_group_;
            case 2: return right_arm_group_;
            default: return right_arm_group_;  // default to right_arm
        }
    }
    
    std::string getRobotName(int robot_id) {
        switch(robot_id) {
            case 0: return "left_arm";
            case 1: return "center_arm";
            case 2: return "right_arm";
            default: return "right_arm";  // default to right_arm
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
    
    // Visual tools for interactive visualization
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_left_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_right_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_center_;
    
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_display_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    Json::Value env_setup_;  // Environment setup configuration
    bool wait_for_confirmation_;  // Control execution flow
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
    
    RCLCPP_INFO(node->get_logger(), "Starting MagBlock Assembly Executor with Trajectory Planning");
    RCLCPP_INFO(node->get_logger(), "Task file: %s", task_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Environment file: %s", env_setup_file.c_str());
    
    // Wait for MoveIt to be ready
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    try {
        // Execute assembly with complete trajectory planning
        if (!executor->executeAssemblyWithTrajectories(task_file, env_setup_file)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute assembly task");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Assembly execution completed successfully!");
        
        // Keep node alive for visualization
        rclcpp::spin(node);
        
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", ex.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}

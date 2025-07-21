#include "moveit_backend.hpp"
#include "Utils/Logger.hpp"
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <vector>
#include <mutex>
#include <algorithm>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace skillgraph {

// Static member definitions
std::vector<MoveitInstance*> MoveitInstance::active_instances_;
std::mutex MoveitInstance::instances_mutex_;

// Signal handler that can cleanup all active instances
void MoveitInstance::signalHandler(int signal) {
    const char* signal_name = "";
    switch(signal) {
        case SIGSEGV: signal_name = "SIGSEGV"; break;
        case SIGTERM: signal_name = "SIGTERM"; break;
        case SIGINT: signal_name = "SIGINT"; break;
        case SIGABRT: signal_name = "SIGABRT"; break;
        default: signal_name = "UNKNOWN"; break;
    }
    
    log(std::string("MoveitInstance: Received signal ") + signal_name + " - performing emergency cleanup", LogLevel::ERROR);
    emergencyCleanup();
    std::exit(signal);
}

void MoveitInstance::emergencyCleanup() {
    std::lock_guard<std::mutex> lock(instances_mutex_);
    log("MoveitInstance: Emergency cleanup starting for " + std::to_string(active_instances_.size()) + " instances", LogLevel::INFO);
    
    for (auto* instance : active_instances_) {
        if (instance) {
            instance->cleanupProcesses();
        }
    }
    active_instances_.clear();
    log("MoveitInstance: Emergency cleanup complete", LogLevel::INFO);
}

void MoveitInstance::cleanupProcesses() {
    // Simplified cleanup for ROS 2
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Shutting down MoveitInstance node");
    }
}

MoveitInstance::MoveitInstance(moveit::core::RobotStatePtr kinematic_state,
                               const std::string &joint_group_name,
                               planning_scene::PlanningScenePtr planning_scene)
    : kinematic_state_(kinematic_state), joint_group_name_(joint_group_name), planning_scene_(planning_scene)
{
    planning_scene_->getPlanningSceneMsg(original_scene_);
}

MoveitInstance::MoveitInstance(const std::string &move_group_name, const std::string &moveit_pkg_name)
{
    // Register this instance for emergency cleanup
    {
        std::lock_guard<std::mutex> lock(instances_mutex_);
        active_instances_.push_back(this);
        
        // Setup signal handlers only for the first instance
        if (active_instances_.size() == 1) {
            std::signal(SIGSEGV, signalHandler);
            std::signal(SIGTERM, signalHandler);
            std::signal(SIGINT, signalHandler);
            std::signal(SIGABRT, signalHandler);
            log("MoveitInstance: Signal handlers registered for emergency cleanup", LogLevel::INFO);
        }
    }
    
    // Initialize ROS 2 node
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("moveit_instance_node");
    
    log("MoveitInstance: ROS 2 node initialized successfully", LogLevel::INFO);

    // Initialize robot model loader with ROS 2 node
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    robot_model_ = robot_model_loader.getModel();
    
    if (!robot_model_) {
        throw std::runtime_error("Failed to load robot model");
    }
    
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    kinematic_state_->setToDefaultValues();

    // Create planning scene
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    
    // Initialize service client for planning scene
    planning_scene_diff_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    
    // Get original scene
    planning_scene_->getPlanningSceneMsg(original_scene_);
    
    // Initialize marker publisher
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
    
    joint_group_name_ = move_group_name;
    
    // Initialize move group interface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, move_group_name);
    
    // Configure planning parameters similar to working approach
    if (move_group_) {
        move_group_->setPlanningTime(15.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        move_group_->setNumPlanningAttempts(10);
        move_group_->setGoalTolerance(0.01);
        move_group_->setPlannerId("RRTConnect");
        
        // Set proper reference frame
        std::string base_frame = move_group_name.substr(0, move_group_name.find("_")) + "_base_link";
        move_group_->setPoseReferenceFrame(base_frame);
        
        log("MoveitInstance: Configured planning parameters for " + move_group_name, LogLevel::INFO);
    }
    
    planning_scene_diff_ = original_scene_;
    
    log("MoveitInstance: Initialization complete", LogLevel::INFO);
}

MoveitInstance::MoveitInstance(rclcpp::Node::SharedPtr node, const std::string &move_group_name, const std::string &moveit_pkg_name)
{
    // Register this instance for emergency cleanup
    {
        std::lock_guard<std::mutex> lock(instances_mutex_);
        active_instances_.push_back(this);
        
        // Setup signal handlers only for the first instance
        if (active_instances_.size() == 1) {
            std::signal(SIGSEGV, signalHandler);
            std::signal(SIGTERM, signalHandler);
            std::signal(SIGINT, signalHandler);
            std::signal(SIGABRT, signalHandler);
            log("MoveitInstance: Signal handlers registered for emergency cleanup", LogLevel::INFO);
        }
    }
    
    // Use the provided node
    node_ = node;
    
    log("MoveitInstance: Using provided ROS 2 node", LogLevel::INFO);

    // Initialize robot model loader with provided node
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    robot_model_ = robot_model_loader.getModel();
    
    if (!robot_model_) {
        throw std::runtime_error("Failed to load robot model");
    }
    
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    kinematic_state_->setToDefaultValues();

    // Create planning scene
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    
    // Initialize service client for planning scene
    planning_scene_diff_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    
    // Get original scene
    planning_scene_->getPlanningSceneMsg(original_scene_);
    
    // Initialize marker publisher
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
    
    joint_group_name_ = move_group_name;
    
    // Initialize move group interface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, move_group_name);
    
    // Configure planning parameters similar to working approach
    if (move_group_) {
        move_group_->setPlanningTime(15.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        move_group_->setNumPlanningAttempts(10);
        move_group_->setGoalTolerance(0.01);
        move_group_->setPlannerId("RRTConnect");
        
        // Set proper reference frame
        std::string base_frame = move_group_name.substr(0, move_group_name.find("_")) + "_base_link";
        move_group_->setPoseReferenceFrame(base_frame);
        
        log("MoveitInstance: Configured planning parameters for " + move_group_name, LogLevel::INFO);
    }
    
    planning_scene_diff_ = original_scene_;
    
    log("MoveitInstance: Initialization complete", LogLevel::INFO);
}

MoveitInstance::~MoveitInstance() {
    cleanupProcesses();
    
    // Remove from active instances
    {
        std::lock_guard<std::mutex> lock(instances_mutex_);
        auto it = std::find(active_instances_.begin(), active_instances_.end(), this);
        if (it != active_instances_.end()) {
            active_instances_.erase(it);
        }
    }
}

void MoveitInstance::setPadding(double padding) {
    // Set robot padding
    const std::vector<std::string> &links = robot_model_->getLinkModelNames();
    for (size_t i = 0; i < links.size(); ++i) {
        if (planning_scene_->getAllowedCollisionMatrix().hasEntry(links[i])) {
            // In ROS 2, the method might be different
            planning_scene_->getCollisionEnvNonConst()->setLinkPadding(links[i], padding);
        }
    }
    // Note: propogateRobotPadding() doesn't exist in ROS 2 MoveIt, padding is handled differently
}

bool MoveitInstance::checkCollision(const std::vector<skillgraph::RobotState> &poses, bool self, bool debug) {
    for (size_t j = 0; j < poses.size(); j++) {
        const auto &pose = poses[j];
        
        if (debug) {
            auto links = kinematic_state_->getJointModelGroup(pose.robot_name)->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                log("Link: " + link, LogLevel::DEBUG);
            }
        }
        
        // Set joint positions
        for (size_t i = 0; i < pose.joint_values.size(); i++) {
            if (debug) {
                auto links = kinematic_state_->getJointModelGroup(pose.robot_name)->getLinkModelNamesWithCollisionGeometry();
                for (const auto &link : links) {
                    log("Link: " + link, LogLevel::DEBUG);
                }
            }
        }
        
        // Set robot state
        std::vector<std::string> joint_names = kinematic_state_->getJointModelGroup(pose.robot_name)->getActiveJointModelNames();
        kinematic_state_->setJointGroupPositions(pose.robot_name, pose.joint_values);
        
        // Check collision
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        
        if (self) {
            collision_request.contacts = true;
            collision_request.max_contacts = 1000;
            planning_scene_->checkSelfCollision(collision_request, collision_result, *kinematic_state_);
        } else {
            planning_scene_->checkCollision(collision_request, collision_result, *kinematic_state_);
        }
        
        if (collision_result.collision) {
            return true;
        }
    }
    return false;
}

double MoveitInstance::computeDistance(const skillgraph::RobotState& a, const skillgraph::RobotState &b) const {
    double dist = 0;
    for (size_t i = 0; i < a.joint_values.size(); i++) {
        std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[i];
        const moveit::core::JointModel* joint_model = kinematic_state_->getJointModel(joint_name);
        dist += joint_model->getDistanceFactor() * std::abs(a.joint_values[i] - b.joint_values[i]);
    }
    return dist;
}

double MoveitInstance::computeDistance(const skillgraph::RobotState& a, const skillgraph::RobotState &b, int dim) const {
    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint_model = kinematic_state_->getJointModel(joint_name);
    return joint_model->getDistanceFactor() * std::abs(a.joint_values[dim] - b.joint_values[dim]);
}

skillgraph::RobotState MoveitInstance::interpolate(const skillgraph::RobotState &a, const skillgraph::RobotState&b, double t) const {
    skillgraph::RobotState result = a;
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    
    std::vector<double> values_a = a.joint_values;
    std::vector<double> values_b = b.joint_values;
    std::vector<double> values_result(values_a.size());
    
    joint_model_group->interpolate(values_a.data(), values_b.data(), t, values_result.data());
    result.joint_values = values_result;
    
    return result;
}

double MoveitInstance::interpolate(const skillgraph::RobotState &a, const skillgraph::RobotState&b, double t, int dim) const {
    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint_model = kinematic_state_->getJointModel(joint_name);
    
    std::vector<double> values(1);
    joint_model->interpolate(&a.joint_values[dim], &b.joint_values[dim], t, values.data());
    return values[0];
}

bool MoveitInstance::connect(const skillgraph::RobotState& a, const skillgraph::RobotState& b, double col_step_size, bool debug) {
    // Simple connection check - interpolate and check collisions
    std::vector<skillgraph::RobotState> path;
    
    double dist = computeDistance(a, b);
    int steps = static_cast<int>(std::ceil(dist / col_step_size));
    
    for (int i = 1; i < steps; i++) {
        double t = static_cast<double>(i) / steps;
        skillgraph::RobotState intermediate = interpolate(a, b, t);
        path.push_back(intermediate);
        
        if (debug) {
            auto links = kinematic_state_->getJointModelGroup(a.robot_name)->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                log("Link: " + link, LogLevel::DEBUG);
            }
        }
    }
    
    if (!path.empty() && checkCollision(path, false, debug)) {
        return false;
    }
    
    // Check final connection
    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    std::vector<double> values_a = a.joint_values;
    std::vector<double> values_b = b.joint_values;
    
    return joint_model_group->satisfiesPositionBounds(values_b.data());
}

bool MoveitInstance::steer(const skillgraph::RobotState& a, const skillgraph::RobotState& b, double max_dist, skillgraph::RobotState& result, double col_step_size) {
    double dist = computeDistance(a, b);
    if (dist <= max_dist) {
        result = b;
        return true;
    }
    
    double t = max_dist / dist;
    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    
    std::vector<double> values_a = a.joint_values;
    std::vector<double> values_b = b.joint_values;
    std::vector<double> values_result(values_a.size());
    
    joint_model_group->interpolate(values_a.data(), values_b.data(), t, values_result.data());
    
    result = a;
    result.joint_values = values_result;
    
    return true;
}

bool MoveitInstance::sample(skillgraph::RobotState &pose) {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(joint_group_name_);
    
    // ROS 2 MoveIt uses random_numbers::RandomNumberGenerator
    random_numbers::RandomNumberGenerator rng;
    kinematic_state_->setToRandomPositions(joint_model_group, rng);
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
    
    pose.joint_values = joint_values;
    pose.robot_name = joint_group_name_;
    return true;
}

// Simplified stub implementations for object manipulation
void MoveitInstance::addMoveableObject(const skillgraph::Object& obj) {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = node_->now();
    co.id = obj.name;
    
    // Add basic shape (simplified)
    shape_msgs::msg::SolidPrimitive primitive;
    if (obj.shape == Object::Shape::Box) {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = obj.length;
        primitive.dimensions[primitive.BOX_Y] = obj.width;
        primitive.dimensions[primitive.BOX_Z] = obj.height;
    } else if (obj.shape == Object::Shape::Cylinder) {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj.height;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = obj.radius;
    }
    
    co.primitives.push_back(primitive);
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = obj.x;
    pose.position.y = obj.y;
    pose.position.z = obj.z;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);
    
    co.operation = co.ADD;
    
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;
    
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::moveObject(const skillgraph::Object& obj) {
    // Simplified implementation
    removeObject(obj.name);
    addMoveableObject(obj);
}

void MoveitInstance::removeObject(const std::string& name) {
    moveit_msgs::msg::CollisionObject co;
    co.id = name;
    co.operation = co.REMOVE;
    
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;
    
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::moveRobot(int robot_id, const skillgraph::RobotState& pose) {
    // Simplified implementation
    std::vector<std::string> joint_names = kinematic_state_->getJointModelGroup(pose.robot_name)->getActiveJointModelNames();
    
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    
    for (size_t i = 0; i < joint_names.size() && i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

// Stub implementations for other required methods
void MoveitInstance::attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const skillgraph::RobotState &pose) {
    // Simplified stub
    log("AttachObjectToRobot called but not implemented", LogLevel::WARN);
}

void MoveitInstance::detachObjectFromRobot(const std::string& name, const skillgraph::RobotState &pose) {
    // Simplified stub
    log("DetachObjectFromRobot called but not implemented", LogLevel::WARN);
}

void MoveitInstance::setObjectColor(const std::string &name, double r, double g, double b, double a) {
    // Simplified stub
    log("SetObjectColor called but not implemented", LogLevel::WARN);
}

void MoveitInstance::updateScene() {
    if (planning_scene_diff_client_ && planning_scene_diff_client_->service_is_ready()) {
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = planning_scene_diff_;
        
        auto future = planning_scene_diff_client_->async_send_request(request);
        // Note: In final implementation, might want to wait for the result
    }
}

void MoveitInstance::resetScene(bool reset_sim) {
    planning_scene_->setPlanningSceneMsg(original_scene_);
    if (reset_sim && planning_scene_diff_client_ && planning_scene_diff_client_->service_is_ready()) {
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = original_scene_;
        
        auto future = planning_scene_diff_client_->async_send_request(request);
    }
}

void MoveitInstance::computeRelativeTransform(Object &obj, const RobotState &robot_state) {
    // Simplified stub
    log("ComputeRelativeTransform called but not implemented", LogLevel::WARN);
}

void MoveitInstance::computeWorldTransform(Object &obj, const RobotState &robot_state) {
    // Simplified stub
    log("ComputeWorldTransform called but not implemented", LogLevel::WARN);
}

bool MoveitInstance::setCollision(const std::string& obj_name, const std::string& link_name, bool allow) {
    // Simplified stub
    log("SetCollision called but not implemented", LogLevel::WARN);
    return true;
}

void MoveitInstance::printKnownObjects() const {
    // Simplified stub
    log("PrintKnownObjects called but not implemented", LogLevel::WARN);
}

void MoveitInstance::setState(const State &state) {
    // Simplified stub
    log("SetState called but not implemented", LogLevel::WARN);
}

// MoveitControl implementation
MoveitControl::MoveitControl(std::shared_ptr<MoveitInstance> instance, bool fake_move)
    : instance_(instance), fake_move_(fake_move) {
}

bool MoveitControl::move(TaskParamPtr post_condition, const RobotTrajectory &trajectory) {
    if (fake_move_) {
        log("MoveitControl::move called in fake mode", LogLevel::INFO);
        return true;
    }
    
    // Get the move group interface from the instance
    auto move_group = instance_->move_group_;
    if (!move_group) {
        log("MoveitControl::move - no move group interface available", LogLevel::ERROR);
        return false;
    }
    
    try {
        // Convert RobotTrajectory to MoveIt trajectory format
        moveit_msgs::msg::RobotTrajectory moveit_trajectory;
        
        // Set up trajectory header
        moveit_trajectory.joint_trajectory.header.stamp = instance_->node_->now();
        moveit_trajectory.joint_trajectory.header.frame_id = instance_->joint_group_name_ + "_base_link";
        
        // Get joint names from the robot model
        const moveit::core::JointModelGroup* joint_model_group =
            instance_->kinematic_state_->getJointModelGroup(instance_->joint_group_name_);
        moveit_trajectory.joint_trajectory.joint_names = joint_model_group->getActiveJointModelNames();
        
        // Convert trajectory points
        for (size_t i = 0; i < trajectory.trajectory.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            
            // Set positions
            point.positions = trajectory.trajectory[i].joint_values;
            
            // Set velocities and accelerations to zero (will be computed by controller)
            point.velocities.resize(trajectory.trajectory[i].joint_values.size(), 0.0);
            point.accelerations.resize(trajectory.trajectory[i].joint_values.size(), 0.0);
            
            // Set time from start
            if (i < trajectory.times.size()) {
                point.time_from_start = rclcpp::Duration::from_seconds(trajectory.times[i]);
            } else {
                point.time_from_start = rclcpp::Duration::from_seconds(i * 0.1); // Default 100ms between points
            }
            
            moveit_trajectory.joint_trajectory.points.push_back(point);
        }
        
        // If trajectory is empty, create a simple trajectory to current position
        if (moveit_trajectory.joint_trajectory.points.empty()) {
            log("Empty trajectory provided, creating single point trajectory", LogLevel::WARN);
            
            trajectory_msgs::msg::JointTrajectoryPoint point;
            std::vector<double> current_joint_values;
            move_group->getCurrentState()->copyJointGroupPositions(joint_model_group, current_joint_values);
            
            point.positions = current_joint_values;
            point.velocities.resize(current_joint_values.size(), 0.0);
            point.accelerations.resize(current_joint_values.size(), 0.0);
            point.time_from_start = rclcpp::Duration::from_seconds(1.0);
            
            moveit_trajectory.joint_trajectory.points.push_back(point);
        }
        
        log("MoveitControl::move - Executing trajectory with " + std::to_string(moveit_trajectory.joint_trajectory.points.size()) + " points", LogLevel::INFO);
        
        // Execute the trajectory
        auto result = move_group->execute(moveit_trajectory);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            log("MoveitControl::move - Trajectory execution successful", LogLevel::INFO);
            
            // Add a small delay to ensure motion completes
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            return true;
        } else {
            log("MoveitControl::move - Trajectory execution failed with error code: " + std::to_string(result.val), LogLevel::ERROR);
            return false;
        }
        
    } catch (const std::exception& e) {
        log("MoveitControl::move - Exception during trajectory execution: " + std::string(e.what()), LogLevel::ERROR);
        return false;
    }
}

// IK solver functionality for joint-space planning
bool MoveitInstance::solveIK(const std::string& robot_name, 
                            const geometry_msgs::msg::Pose& target_pose,
                            const std::vector<double>& seed_joints,
                            std::vector<double>& solution_joints) {
    try {
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model_->getJointModelGroup(robot_name);
        
        if (!joint_model_group) {
            log("Joint model group not found for " + robot_name, LogLevel::ERROR);
            return false;
        }
        
        // Create a robot state for IK solving
        moveit::core::RobotState robot_state(robot_model_);
        
        // Set seed state
        if (seed_joints.size() == joint_model_group->getActiveJointModelNames().size()) {
            robot_state.setJointGroupPositions(joint_model_group, seed_joints);
        } else {
            log("Seed joint size mismatch for " + robot_name + " (got " + 
                std::to_string(seed_joints.size()) + ", expected " + 
                std::to_string(joint_model_group->getActiveJointModelNames().size()) + 
                "), using default positions", LogLevel::WARN);
            robot_state.setToDefaultValues();
        }
        

        // TEMP - TESTING
        // const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform("right_end_effector_link");
        // std::cout << "Translation: " << end_effector_state.translation() << std::endl;
        // std::cout << "Rotation: " << end_effector_state.rotation() << std::endl;


        // Convert geometry_msgs::Pose to Eigen::Isometry3d
        Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
        target_transform.translation() = Eigen::Vector3d(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        );
        // target_transform.translation() = Eigen::Vector3d(
        //      -0.442,
        //      -0.022,
        //      1.217
        //  );
        
        target_transform.linear() = Eigen::Quaterniond(
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z
        ).toRotationMatrix();
        // target_transform.linear() = Eigen::Quaterniond(
        //     0.013,
        //     -0.707,
        //     0.707,
        //     -0.013
        // ).toRotationMatrix();

        // Apply gripper offset
        // double gripper_offset_z = 0.1628; // BASED ON ROBOTIQ 2F85 CLOSED LENGTH
        // Eigen::Isometry3d gripper_offset = Eigen::Isometry3d::Identity();
        // gripper_offset.translation() = Eigen::Vector3d(0.0, 0.0, -gripper_offset_z);
        // target_transform = target_transform * gripper_offset;

        // Get the tip link (end effector) for the robot arm
        std::string tip_link;
        std::string base_link;
        if (robot_name == "left_arm") {
            tip_link = "left_end_effector_link";
        } else if (robot_name == "right_arm") {
            tip_link = "right_end_effector_link";
            base_link = "right_base_link";
        } else if (robot_name == "center_arm") {
            tip_link = "center_end_effector_link";
        } else {
            // Fallback to the last link in the group
            const std::vector<std::string>& link_names = joint_model_group->getLinkModelNames();
            if (!link_names.empty()) {
                tip_link = link_names.back();
            } else {
                log("No links found for group " + robot_name, LogLevel::ERROR);
                return false;
            }
        }
        
        log("Using tip link: " + tip_link + " for IK solving", LogLevel::DEBUG);

        const Eigen::Isometry3d& ee_to_world = robot_state.getGlobalLinkTransform(base_link);
        Eigen::Isometry3d target_transform_world = ee_to_world * target_transform;
        std::cout << "Target transform in world frame: " << target_transform_world.translation() << std::endl;

        bool found_ik = robot_state.setFromIK(joint_model_group, target_transform_world, tip_link, 
                                             5.0, // increased timeout to 5 seconds
                                             moveit::core::GroupStateValidityCallbackFn(), 
                                             kinematics::KinematicsQueryOptions());
        
        if (found_ik) {
            // Get the joint values from the solution
            std::vector<double> joint_values;
            robot_state.copyJointGroupPositions(joint_model_group, joint_values);
            solution_joints = joint_values;

            std::cout << joint_values[0] << " " << joint_values[1] << " " << joint_values[2] 
            << " " << joint_values[3] << " " << joint_values[4] << " " << joint_values[5]
            << " " << joint_values[6] << std::endl;
            
            log("IK solution found for " + robot_name + " with " + std::to_string(solution_joints.size()) + " joints", LogLevel::DEBUG);
            return true;
        } else {
            log("IK solution not found for " + robot_name + " (target: [" + 
                std::to_string(target_pose.position.x) + ", " + 
                std::to_string(target_pose.position.y) + ", " + 
                std::to_string(target_pose.position.z) + "])", LogLevel::WARN);
            return false;
        }
        
    } catch (const std::exception& e) {
        log("Error in IK solving for " + robot_name + ": " + std::string(e.what()), LogLevel::ERROR);
        return false;
    }
}

bool MoveitInstance::getCurrentJointValues(const std::string& robot_name, 
                                         std::vector<double>& current_joints) {
    try {
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model_->getJointModelGroup(robot_name);
        
        if (!joint_model_group) {
            log("Joint model group not found for " + robot_name, LogLevel::ERROR);
            return false;
        }
        
        // Get current state from planning scene
        moveit::core::RobotState current_state = planning_scene_->getCurrentState();
        current_state.copyJointGroupPositions(joint_model_group, current_joints);
        
        log("Retrieved current joint values for " + robot_name + " with " + 
            std::to_string(current_joints.size()) + " joints", LogLevel::DEBUG);
        
        return true;
    } catch (const std::exception& e) {
        log("Error getting current joint values for " + robot_name + ": " + std::string(e.what()), LogLevel::ERROR);
        return false;
    }
}

std::vector<std::vector<double>> MoveitInstance::interpolateJointTrajectory(
    const std::vector<double>& start_joints,
    const std::vector<double>& end_joints,
    int num_steps) {
    
    std::vector<std::vector<double>> trajectory;
    
    if (start_joints.size() != end_joints.size()) {
        log("Joint vector size mismatch in interpolation", LogLevel::ERROR);
        return trajectory;
    }
    
    for (int i = 0; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        std::vector<double> interpolated_joints(start_joints.size());
        
        for (size_t j = 0; j < start_joints.size(); ++j) {
            interpolated_joints[j] = start_joints[j] + t * (end_joints[j] - start_joints[j]);
        }
        
        trajectory.push_back(interpolated_joints);
    }
    
    return trajectory;
}

void MoveitInstance::executeJointTrajectory(const std::string& robot_name,
                                          const std::vector<std::vector<double>>& joint_trajectory,
                                          double step_duration) {
    try {
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model_->getJointModelGroup(robot_name);
        
        if (!joint_model_group) {
            log("Joint model group not found for " + robot_name, LogLevel::ERROR);
            return;
        }
        
        // Get robot ID from robot name
        int robot_id = (robot_name == "left_arm") ? 0 : (robot_name == "center_arm") ? 1 : 2;
        
        // Execute trajectory by updating robot state at each step
        for (size_t i = 0; i < joint_trajectory.size(); ++i) {
            skillgraph::RobotState robot_state;
            robot_state.robot_name = robot_name;
            robot_state.robot_id = robot_id;
            robot_state.joint_values = joint_trajectory[i];
            
            // Update the robot position
            moveRobot(robot_id, robot_state);
            updateScene();
            
            // Sleep for step duration
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(step_duration * 1000)));
        }
        
        log("Joint trajectory execution completed for " + robot_name, LogLevel::INFO);
        
    } catch (const std::exception& e) {
        log("Error executing joint trajectory for " + robot_name + ": " + std::string(e.what()), LogLevel::ERROR);
    }
}
} // namespace skillgraph

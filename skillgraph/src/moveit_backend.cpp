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
        
    }
    
    planning_scene_diff_ = original_scene_;
    
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
        }
    }
    
    // Use the provided node
    node_ = node;
    
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
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = joint_group_name_;
    if (debug) {
        c_req.contacts = true;
        c_req.max_contacts = 10;
    }

    // set the robot state to the one we are checking
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    
    
    std::vector<double> all_joints;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrixNonConst();

    //print the acm entry names
    // std::vector<std::string> acm_names;
    // acm.getAllEntryNames(acm_names);
    // for (const auto &entry : acm_names) {
    //     std::cout << entry << " ";
    // }
    // std::cout << std::endl;

    int index = 0;
    for (int i = 0; i < num_robots_; i++) {
        std::string group = robot_names_[i];

        // find if this robot is in collision with the environment
        bool checking_i = false;
        RobotState pose;
        for (int j = 0; j < poses.size(); j++) {
            if (poses[j].robot_id == i) {
                checking_i = true;
                pose = poses[j];
                break;
            }
        }

        // set the acm for this robot to true if it is not checked for collision
        if (!checking_i) {
            auto links = kinematic_state_->getJointModelGroup(group)->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                acm.setEntry(link, true);
            }
            if (hand_names_.size() > i) {
                auto hand_links = kinematic_state_->getJointModelGroup(hand_names_[i])->getLinkModelNamesWithCollisionGeometry();
                for (const auto &link : hand_links) {
                    acm.setEntry(link, true);
                }
            }
            // insert the joint values for this robot
            std::vector<double> dummy_values(getRobotDOF(i), 0.0);
            all_joints.insert(all_joints.end(), dummy_values.begin(), dummy_values.end());
        }
        else {
            auto links = kinematic_state_->getJointModelGroup(group)->getLinkModelNamesWithCollisionGeometry();
            // copy the joint values for this robot
            all_joints.insert(all_joints.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        
        index += start_poses_[i].joint_values.size();
    }

    robot_state.setJointGroupPositions(joint_group_name_, all_joints);

    c_res.clear();
    if (self) {
        //robot_state.updateCollisionBodyTransforms();
        //planning_scene_->getCollisionEnv()->checkSelfCollision(c_req, c_res, robot_state, acm);
        planning_scene_->checkSelfCollision(c_req, c_res, robot_state, acm);  
    } else {
        planning_scene_->checkCollision(c_req, c_res, robot_state, acm);
    }
    num_collision_checks_++;

    if (debug) {
        std::cout << "Number of contacts: " << c_res.contacts.size() << std::endl;
        for (const auto &contact : c_res.contacts) {
            std::cout << "Contact between " << contact.first.first << " and " << contact.first.second << std::endl;
        }
    }

    return c_res.collision;
}

double MoveitInstance::computeDistance(const skillgraph::RobotState& a, const skillgraph::RobotState &b) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    double distance = robot_state_a.distance(robot_state_b);
    return distance;
}

double MoveitInstance::computeDistance(const skillgraph::RobotState& a, const skillgraph::RobotState &b, int dim) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    assert(dim <= a.joint_values.size() && dim <= b.joint_values.size());
    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint = kinematic_state_->getJointModel(joint_name);
    double a_d = a.joint_values[dim];
    double b_d = b.joint_values[dim];
    double distance = joint->distance(&a_d, &b_d);
    return distance;
}

skillgraph::RobotState MoveitInstance::interpolate(const skillgraph::RobotState &a, const skillgraph::RobotState&b, double t) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    skillgraph::RobotState result = a;
    std::cout << "ROBOT NAME BEING USED: " << a.robot_name << std::endl;
    if (a.robot_name.empty()) throw std::runtime_error("robot_name is empty in RobotState a");

    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    
    std::vector<double> values_a = a.joint_values;
    std::vector<double> values_b = b.joint_values;
    std::vector<double> values_result(values_a.size());
    
    joint_model_group->interpolate(values_a.data(), values_b.data(), t, values_result.data());
    result.joint_values = values_result;
    result.hand_values = a.hand_values; // Assuming hand values remain the same for interpolation
    
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
    //* check if a collision-free kinematic path exists from pose a to b for the robot (ignoring other robots)*/
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    
    // discretize and check for collision along the path
    double joint_distance = computeDistance(a, b);
    int num_steps = std::ceil(joint_distance / col_step_size);
    if (num_steps == 0) {
        log("Connecting two poses with zero distance", LogLevel::WARN);
        return false;
    }

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = a.robot_name;
    if (debug) {
        c_req.contacts = true;
        c_req.max_contacts = 10;
    }

    auto acm = planning_scene_->getAllowedCollisionMatrixNonConst();
    for (int i = 0; i < num_robots_; i++) {
        if (i != a.robot_id) {
            auto links = kinematic_state_->getJointModelGroup(robot_names_[i])->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                acm.setEntry(link, true);
            }
            if (hand_names_.size() > i) {
                links = kinematic_state_->getJointModelGroup(hand_names_[i])->getLinkModelNamesWithCollisionGeometry();
                for (const auto &link : links) {
                    acm.setEntry(link, true);
                }
            }
        }
    }

    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);

    for (int i = 0; i <= num_steps; i++) {
        c_res.clear();
        
        robot_state.setJointGroupPositions(a.robot_name, a.joint_values);
        robot_state_a.interpolate(robot_state_b, (double)i / num_steps, robot_state, joint_model_group);
        planning_scene_->checkCollision(c_req, c_res, robot_state, acm);
        num_collision_checks_++;
        if (c_res.collision) {
            if (debug) {
                for (const auto &contact : c_res.contacts) {
                    std::cout << "Collision detected at step " << i << " between ";
                    std::cout << contact.first.first << " and " << contact.first.second << std::endl;
                    RobotState c = a;
                    robot_state.copyJointGroupPositions(c.robot_name, c.joint_values);
                    moveRobot(c.robot_id, c);
                    updateScene();
                    log("updated scene when connect collides", LogLevel::DEBUG);
                }
            }
            return false;
        }
    }

    return true;
}

bool MoveitInstance::steer(const skillgraph::RobotState& a, const skillgraph::RobotState& b, double max_dist, skillgraph::RobotState& result, double col_step_size) {
    /* find a collision-free that steers the robot from a towards b for max_distance */
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    
    double joint_distance = computeDistance(a, b);
    if (joint_distance <= max_dist) {
        result = b;
        return true;
    }

    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();

    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    robot_state_a.interpolate(robot_state_b, max_dist / joint_distance, robot_state, joint_model_group);
    result.robot_id = a.robot_id;
    result.robot_name = a.robot_name;
    result.joint_values.resize(a.joint_values.size());
    robot_state.copyJointGroupPositions(a.robot_name, result.joint_values);

    if (connect(a, result, col_step_size)) {
        return true;
    }
    else {
        return false;
    }
}

bool MoveitInstance::sample(skillgraph::RobotState &pose) {
    /* sample a collision free pose for the robot (ignoring other robots)
    */

    // initialize the joint values vector
    std::string robot_name = robot_names_[pose.robot_id];
    pose.robot_name = robot_name;

    // get the bounds of the joint space for the robot
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(robot_name);
    const std::vector<const moveit::core::JointModel*> & joint_models = joint_model_group->getActiveJointModels();
    
    // boilerplate for checking collision
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = robot_name;

    bool in_collision = true;
    int attempt = 0;
    int max_attempts = 10;
    do {
        std::vector<double> joint_values;
        joint_values.reserve(joint_models.size());

        // sample each joint
        for (int i = 0; i < joint_models.size(); i++) {
            const auto &bounds = joint_models[i]->getVariableBounds();
            if (!bounds.empty()) {
                // assume the joint has only one variable
                std::uniform_real_distribution<double> distribution(bounds[0].min_position_, bounds[0].max_position_);
                joint_values.push_back(distribution(rng_));
            }
            else {
                // raise an error if the joint has no bounds
                throw std::runtime_error("Joint " + joint_models[i]->getName() + " has no bounds");
            }
        }

        // check collision
        c_res.clear();
        robot_state.setJointGroupPositions(robot_name, joint_values);
        planning_scene_->checkCollision(c_req, c_res, robot_state);
        num_collision_checks_++;

        in_collision = c_res.collision;
        if (!in_collision) {
            pose.joint_values = joint_values;
        }

        attempt ++;
    } while (in_collision && attempt < max_attempts);
    

    
    return !in_collision;
}

// Simplified stub implementations for object manipulation
void MoveitInstance::addMoveableObject(const skillgraph::Object& obj) {
    if (objects_.find(obj.name) != objects_.end()) {
        log("Object " + obj.name + " already exists in the scene", LogLevel::ERROR);
        return;
    }
    log("Adding object " + obj.name + " to the scene", LogLevel::DEBUG);
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = node_->now();
    co.id = obj.name;
    
    objects_[obj.name] = obj;

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
    pose.orientation.x = obj.qx;
    pose.orientation.y = obj.qy;
    pose.orientation.z = obj.qz;
    pose.orientation.w = obj.qw;
    co.primitive_poses.push_back(pose);
    
    co.operation = co.ADD;

    collision_object_map_[co.id] = co;
    
    moveit_msgs::msg::PlanningScene planning_scene;
    // planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;
    for (const auto& [id, object] : collision_object_map_) {
        planning_scene.world.collision_objects.push_back(object);
    }
    
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::moveObject(const skillgraph::Object& obj) {
    if (objects_.find(obj.name) == objects_.end()) {
        addMoveableObject(obj);
        return;
    }

    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = node_->now();
    co.id = obj.name;
    
    co.pose.position.x = obj.x;
    co.pose.position.y = obj.y;
    co.pose.position.z = obj.z ;
    co.pose.orientation.x = obj.qx;
    co.pose.orientation.y = obj.qy;
    co.pose.orientation.z = obj.qz;
    co.pose.orientation.w = obj.qw;

    co.operation = co.MOVE;
 
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::removeObject(const std::string& name) {
    // find the object in the scene
    auto it = objects_.find(name);
    if (it != objects_.end()) {
        // delete the object in objects list
        objects_.erase(it);
        log("Removing object " + name + " from the scene", LogLevel::DEBUG);
    }
    else {
        log("Object " + name + " not found in the scene", LogLevel::ERROR);
        return;
    }

    // remove the object from the scene
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
    moveit_msgs::msg::PlanningScene cur_scene;
    planning_scene_->getPlanningSceneMsg(cur_scene);
    
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    std::vector<std::string> joint_names = kinematic_state_->getJointModelGroup(pose.robot_name)->getActiveJointModelNames();
    for (size_t i = 0; i < joint_names.size() && i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    if (pose.hand_values.size() > 0 && hand_names_.size() > robot_id) {
        auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.hand_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
        }
    }
    planning_scene.robot_state.attached_collision_objects = cur_scene.robot_state.attached_collision_objects;
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const skillgraph::RobotState &pose) {
/*
    directly attach the object to the robot
    */
   if (objects_.find(name) == objects_.end()) {
       log("Object " + name + " not found in the scene", LogLevel::ERROR);
       return;
    }
    Object &obj = objects_[name];
    if (obj.state == Object::State::Attached) {
        log("Object " + name + " is already attached to a robot " + robot_names_[obj.robot_id] 
            + " cannot attach to robot " + std::to_string(robot_id), LogLevel::ERROR);
        return;
    }
    log("Attaching object " + name + " to robot " + robot_names_[robot_id] + " at link " + link_name, LogLevel::DEBUG);

    obj.state = Object::State::Attached;
    obj.robot_id = robot_id;
    std::string old_parent_link = obj.parent_link;
    obj.parent_link = link_name;

    moveit_msgs::msg::AttachedCollisionObject co;
    co.link_name = obj.parent_link;
    co.object.header.frame_id = obj.parent_link;
    co.object.header.stamp = node_->now();
    co.object.id = name;
    co.object.operation = co.object.ADD;


    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(co);
    planning_scene.robot_state.is_diff = true;
    
    auto joint_names = planning_scene_->getRobotModel()->getJointModelGroup(robot_names_[robot_id])->getActiveJointModelNames();
    for (int i = 0; i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    if (pose.hand_values.size() > 0 && hand_names_.size() > robot_id) {
        auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.hand_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
        }
    }
    
    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::detachObjectFromRobot(const std::string& name, const skillgraph::RobotState &pose) {
   if (objects_.find(name) == objects_.end()) {
        log("Object " + name + " not found in the scene", LogLevel::ERROR);
        return;
    }
    Object &obj = objects_[name];
    if (obj.state != Object::State::Attached) {
        log("Object " + name + " is not attached to any robot", LogLevel::ERROR);
        return;
    }

    log("Detaching object " + name + " from robot " + robot_names_[obj.robot_id], LogLevel::DEBUG);
    obj.state = Object::State::Static;
    obj.robot_id = -1;
    std::string old_parent_link = obj.parent_link;
    obj.parent_link = "base";

    moveit_msgs::msg::AttachedCollisionObject co_remove;
    co_remove.object.id = name;
    co_remove.link_name = old_parent_link;
    co_remove.object.operation = co_remove.object.REMOVE;

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    //planning_scene.world.collision_objects.push_back(co);
    planning_scene.robot_state.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(co_remove);
    auto joint_names = planning_scene_->getRobotModel()->getJointModelGroup(robot_names_[pose.robot_id])->getActiveJointModelNames();
    for (int i = 0; i < pose.joint_values.size(); i++) {
        planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
        planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
    }
    if (pose.hand_values.size() > 0 && hand_names_.size() > pose.robot_id) {
        auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[pose.robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.hand_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
        }
    }

    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::setObjectColor(const std::string &name, double r, double g, double b, double a) {
    if (objects_.find(name) == objects_.end()) {
        return;
    }

    moveit_msgs::msg::ObjectColor oc;
    oc.id = name;
    oc.color.r = r;
    oc.color.g = g;
    oc.color.b = b;
    oc.color.a = a;

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.object_colors.push_back(oc);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    if (planning_scene_diff_client_ && planning_scene_diff_client_->service_is_ready()) {
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = planning_scene;
        auto future = planning_scene_diff_client_->async_send_request(request);
        // Note: In final implementation, might want to wait for the result
    } else {
        log("Planning scene diff client is not ready", LogLevel::ERROR);
    }
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
    // Get the transform from world to end-effector
    moveit::core::RobotState tmp_robot_state = planning_scene_->getCurrentStateNonConst();
    int obj_robot_id = obj.robot_id;
    if((obj_robot_id < robot_names_.size() && obj_robot_id >= 0) == false) {
        log("Object " + obj.name + " has invalid robot id " + std::to_string(obj_robot_id), LogLevel::ERROR);
        return;
    }
    tmp_robot_state.setJointGroupPositions(robot_state.robot_name, robot_state.joint_values);
    tmp_robot_state.update();

    // Get the transform from world to end-effector
    const Eigen::Isometry3d& world_to_ee = tmp_robot_state.getGlobalLinkTransform(obj.parent_link);

    // Create Eigen transform from world object pose
    Eigen::Isometry3d world_obj_transform = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q(obj.qw, obj.qx, obj.qy, obj.qz);
    world_obj_transform.translate(Eigen::Vector3d(obj.x, obj.y, obj.z));
    world_obj_transform.rotate(q);

    // Compute relative transform (object in end-effector frame)
    Eigen::Isometry3d relative_transform = world_to_ee.inverse() * world_obj_transform;

    // Convert back to geometry_msgs::Pose
    Eigen::Vector3d pos = relative_transform.translation();
    Eigen::Quaterniond rot(relative_transform.rotation());
    obj.x_attach = pos.x();
    obj.y_attach = pos.y();
    obj.z_attach = pos.z();
    obj.qx_attach = rot.x();
    obj.qy_attach = rot.y();
    obj.qz_attach = rot.z();
    obj.qw_attach = rot.w();
}

void MoveitInstance::computeWorldTransform(Object &obj, const RobotState &robot_state) {
    // Get the robot state
    moveit::core::RobotState tmp_robot_state = planning_scene_->getCurrentStateNonConst();
    int obj_robot_id = obj.robot_id;
    if((obj_robot_id < robot_names_.size() && obj_robot_id >= 0) == false) {
        log("Object " + obj.name + " has invalid robot id " + std::to_string(obj_robot_id), LogLevel::ERROR);
        return;
    }
    
    // Set the joint positions from robot_state
    tmp_robot_state.setJointGroupPositions(robot_state.robot_name, robot_state.joint_values);
    tmp_robot_state.update();

    // Get the transform from world to end-effector
    const Eigen::Isometry3d& world_to_ee = tmp_robot_state.getGlobalLinkTransform(obj.parent_link);

    // Create Eigen transform for object in end-effector frame
    Eigen::Isometry3d ee_obj_transform = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q(obj.qw_attach, obj.qx_attach, obj.qy_attach, obj.qz_attach);
    ee_obj_transform.translate(Eigen::Vector3d(obj.x_attach, obj.y_attach, obj.z_attach));
    ee_obj_transform.rotate(q);

    // Compute world transform (object in world frame)
    Eigen::Isometry3d world_transform = world_to_ee * ee_obj_transform;

    // Convert back to object world coordinates
    Eigen::Vector3d pos = world_transform.translation();
    Eigen::Quaterniond rot(world_transform.rotation());
    obj.x = pos.x();
    obj.y = pos.y();
    obj.z = pos.z();
    obj.qx = rot.x();
    obj.qy = rot.y();
    obj.qz = rot.z(); 
    obj.qw = rot.w();
}

bool MoveitInstance::setCollision(const std::string& obj_name, const std::string& link_name, bool allow) {

    // Get the Allowed Collision Matrix (ACM)
    // Use the PlanningSceneMonitor to get the current planning scene

    moveit_msgs::msg::AllowedCollisionMatrix acm;
    planning_scene_->getAllowedCollisionMatrixNonConst().getMessage(acm);

    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), obj_name) == acm.entry_names.end()) {
        // object is not in the ACM, add a new row and column
        acm.entry_names.push_back(obj_name);

        // first visit all esiting rows and add a new column
        for (size_t i = 0; i < acm.entry_values.size(); i++) {
            if (acm.entry_names[i] == link_name) {
                acm.entry_values[i].enabled.push_back(allow);
            }
            // by default we disallow object to object collision
            else if (objects_.find(acm.entry_names[i]) != objects_.end()) {
                acm.entry_values[i].enabled.push_back(false);
            }
            else {
                acm.entry_values[i].enabled.push_back(false);
            }
        }
        // add a new row
        moveit_msgs::msg::AllowedCollisionEntry new_entry;
        for (size_t i = 0; i < acm.entry_names.size(); i++) {
            if (acm.entry_names[i] == link_name) {
                new_entry.enabled.push_back(allow);
            }
            else if (objects_.find(acm.entry_names[i]) != objects_.end()) {
                new_entry.enabled.push_back(false);
            }
            else {
                new_entry.enabled.push_back(false);
            }
        }
        acm.entry_values.push_back(new_entry);
    }
    else {
        // directly modify the entry
        for (size_t i = 0; i < acm.entry_names.size(); i++) {
            for (size_t j = 0; j < acm.entry_names.size(); j++) {
                if (acm.entry_names[i] == obj_name && acm.entry_names[j] == link_name) {
                    acm.entry_values[i].enabled[j] = allow;
                }
                if (acm.entry_names[i] == link_name && acm.entry_names[j] == obj_name) {
                    acm.entry_values[i].enabled[j] = allow;
                }
            }
        }
    }

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = acm;
    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;

    return true;
}

void MoveitInstance::printKnownObjects() const {
    planning_scene_->printKnownObjects();
}

void MoveitInstance::setState(const State &state) {
 auto & robot_states = state.robot_states;
    auto & env_states = state.env_state;

    // move the robot state

    moveit_msgs::msg::PlanningScene planning_scene;

    // set the robot state
    for (int i = 0; i < robot_states.size(); i++) {
        auto &pose = robot_states[i];
        auto joint_names = planning_scene_->getRobotModel()->getJointModelGroup(robot_names_[pose.robot_id])->getActiveJointModelNames();
        for (int i = 0; i < pose.joint_values.size(); i++) {
            planning_scene.robot_state.joint_state.name.push_back(joint_names[i]);
            planning_scene.robot_state.joint_state.position.push_back(pose.joint_values[i]);
        }
        if (pose.hand_values.size() > 0 && hand_names_.size() > pose.robot_id) {
            auto handjoint_names = planning_scene_->getRobotModel()->getJointModelGroup(hand_names_[pose.robot_id])->getActiveJointModelNames();
            for (int i = 0; i < pose.hand_values.size(); i++) {
                planning_scene.robot_state.joint_state.name.push_back(handjoint_names[i]);
                planning_scene.robot_state.joint_state.position.push_back(pose.hand_values[i]);
            }
        }
    }

    // Prepare visualization markers publisher for object labels
    // visualization_msgs::MarkerArray marker_array;
    // int marker_id = 0;

    // update the object state
    for (int i = 0; i < env_states.objects.size(); i++) {
        ObjPtr obj = env_states.objects[i];
        auto fid = objects_.find(obj->name);
        
        if (obj->state == Object::State::Attached) {
            // attach the object
            moveit_msgs::msg::AttachedCollisionObject co;
            co.link_name = obj->parent_link;
            co.object.header.frame_id = obj->parent_link;
            co.object.header.stamp = node_->now();
            co.object.id = obj->name;
            co.object.operation = co.object.ADD;

            shape_msgs::msg::SolidPrimitive primitive;
            if (obj->shape == Object::Shape::Box) {
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[primitive.BOX_X] = obj->length;
                primitive.dimensions[primitive.BOX_Y] = obj->width;
                primitive.dimensions[primitive.BOX_Z] = obj->height;
            }
            else if (obj->shape == Object::Shape::Cylinder) {
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj->length;
                primitive.dimensions[primitive.CYLINDER_RADIUS] = obj->radius;
            }
            // object world pose is obj.x, obj.y, obj.z, obj.qx, obj.qy, obj.qz, obj.qw
            // now need to compute this in the robot endeffector frame

            geometry_msgs::msg::Pose relative_pose;
            relative_pose.position.x = obj->x_attach;
            relative_pose.position.y = obj->y_attach;
            relative_pose.position.z = obj->z_attach;
            relative_pose.orientation.x = obj->qx_attach;
            relative_pose.orientation.y = obj->qy_attach;
            relative_pose.orientation.z = obj->qz_attach;
            relative_pose.orientation.w = obj->qw_attach;
            co.object.pose = relative_pose;

            co.object.primitives.push_back(primitive);
            planning_scene.robot_state.attached_collision_objects.push_back(co);
        }
        else {
            // add a new object
            moveit_msgs::msg::CollisionObject co;
            co.header.frame_id = obj->parent_link;
            co.header.stamp = node_->now();
            co.id = obj->name;

            shape_msgs::msg::SolidPrimitive primitive;
            if (obj->shape == Object::Shape::Box) {
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[primitive.BOX_X] = obj->length;
                primitive.dimensions[primitive.BOX_Y] = obj->width;
                primitive.dimensions[primitive.BOX_Z] = obj->height;
            }
            else if (obj->shape == Object::Shape::Cylinder) {
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj->length;
                primitive.dimensions[primitive.CYLINDER_RADIUS] = obj->radius;
            }
            geometry_msgs::msg::Pose world_pose;
            world_pose.position.x = obj->x;
            world_pose.position.y = obj->y;
            world_pose.position.z = obj->z ;
            world_pose.orientation.x = obj->qx;
            world_pose.orientation.y = obj->qy;
            world_pose.orientation.z = obj->qz;
            world_pose.orientation.w = obj->qw;
                
            co.primitives.push_back(primitive);
            co.primitive_poses.push_back(world_pose);
            co.operation = co.ADD;
            planning_scene.world.collision_objects.push_back(co);
        }
        // copy the object information to the scene
        objects_[obj->name] = *obj;

        // // Add text marker for this object
        // visualization_msgs::Marker text_marker;
        // text_marker.header.frame_id = obj->parent_link;
        // text_marker.header.stamp = node_->now();
        // text_marker.ns = "object_labels";
        // text_marker.id = marker_id++;
        // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // text_marker.action = visualization_msgs::Marker::ADD;
        
        // // Position the text slightly above the object
        // text_marker.pose.position.x = obj->x;
        // text_marker.pose.position.y = obj->y;
        // text_marker.pose.position.z = obj->z + obj->height/2 + 0.05; // Adjust based on object size
        // text_marker.pose.orientation.w = 1.0;
        
        // // Set text properties
        // text_marker.text = obj->name;
        // text_marker.scale.z = 0.02; // Text height
        // text_marker.color.r = 1.0;
        // text_marker.color.g = 1.0;
        // text_marker.color.b = 1.0;
        // text_marker.color.a = 1.0;
        // text_marker.lifetime = ros::Duration(); // Persistent until removed
        
        // marker_array.markers.push_back(text_marker);
        
    }

    // marker_pub_.publish(marker_array);

    //planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = false;
    planning_scene.robot_state.is_diff = false; 

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
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
                
        // Execute the trajectory
        auto result = move_group->execute(moveit_trajectory);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {            
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
        std::cout << "JOINT MODEL GROUP FOR " << robot_name << ": " 
                  << joint_model_group->getName() << std::endl;
        
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
            robot_state.setToDefaultValues(joint_model_group, "Home");
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
        
        target_transform.linear() = Eigen::Quaterniond(
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z
        ).toRotationMatrix();

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
            base_link = "left_base_link";
        } else if (robot_name == "right_arm") {
            tip_link = "right_end_effector_link";
            base_link = "right_base_link";
        } else if (robot_name == "center_arm") {
            tip_link = "center_end_effector_link";
            base_link = "center_base_link";
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

        return true;
    } catch (const std::exception& e) {
        log("Error getting current joint values for " + robot_name + ": " + std::string(e.what()), LogLevel::ERROR);
        return false;
    }
}

skillgraph::RobotTrajectory MoveitInstance::interpolateJointTrajectory(
    const std::vector<double>& start_joints,
    const std::vector<double>& end_joints,
    int num_steps,
    const std::string& robot_name,
    int act_id) {
    
    // Get robot ID from robot name
    int robot_id = (robot_name == "left_arm") ? 0 : (robot_name == "center_arm") ? 1 : 2;
    skillgraph::RobotTrajectory traj;
    traj.robot_id = robot_id;
    traj.cost = 0.0;
    
    if (start_joints.size() != end_joints.size()) {
        log("Joint vector size mismatch in interpolation", LogLevel::ERROR);
        return traj;
    }
    
    for (int i = 0; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        std::vector<double> interpolated_joints(start_joints.size());
        
        for (size_t j = 0; j < start_joints.size(); ++j) {
            interpolated_joints[j] = start_joints[j] + t * (end_joints[j] - start_joints[j]);
        }

        skillgraph::RobotState state;
        state.robot_id = robot_id;
        state.robot_name = robot_name;
        state.joint_values = interpolated_joints;
        state.hand_values = {};
        
        traj.trajectory.push_back(state);
        traj.times.push_back(t);
        traj.act_ids.push_back(act_id);
        
    }

    return traj;
}

void MoveitInstance::executeJointTrajectory(const std::vector<skillgraph::RobotTrajectory>& robot_trajectories,
                                          double step_duration) {
    try {
        for (const auto& traj : robot_trajectories) {
            const std::string& robot_name = traj.trajectory.front().robot_name;
            int robot_id = traj.robot_id;
            
            const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(robot_name);
        
            if (!joint_model_group) {
                log("Joint model group not found for " + robot_name, LogLevel::ERROR);
                return;
            }

            for (const auto& robot_state : traj.trajectory) {
                moveRobot(robot_id, robot_state);
                updateScene();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(step_duration * 1000)));
            }

        } 
    } catch (const std::exception& e) {
        log("Error executing joint trajectory: " + std::string(e.what()), LogLevel::ERROR);
    }
}

bool MoveitInstance::transformRobotToWorld(const std::string& robot_name, const geometry_msgs::msg::Pose& target_pose, geometry_msgs::msg::Pose& target_transform_world) {
    const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(robot_name);
    std::cout << "JOINT MODEL GROUP FOR " << robot_name << ": " 
                << joint_model_group->getName() << std::endl;
    
    if (!joint_model_group) {
        log("Joint model group not found for " + robot_name, LogLevel::ERROR);
        return false;
    }

    // Create a robot state
    moveit::core::RobotState robot_state(robot_model_);

    // Convert geometry_msgs::Pose to Eigen::Isometry3d
    Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
    target_transform.translation() = Eigen::Vector3d(
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z
    );
    
    target_transform.linear() = Eigen::Quaterniond(
        target_pose.orientation.w,
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z
    ).toRotationMatrix();

    std::string base_link;
    if (robot_name == "left_arm") {
        base_link = "left_base_link";
    } else if (robot_name == "right_arm") {
        base_link = "right_base_link";
    } else if (robot_name == "center_arm") {
        base_link = "center_base_link";
    } else {
        return false;
    }

    const Eigen::Isometry3d& ee_to_world = robot_state.getGlobalLinkTransform(base_link);
    Eigen::Isometry3d eigen_target_transform_world = ee_to_world * target_transform;

    target_transform_world.position.x = eigen_target_transform_world.translation().x();
    target_transform_world.position.y = eigen_target_transform_world.translation().y();
    target_transform_world.position.z = eigen_target_transform_world.translation().z();

    Eigen::Quaterniond quaternion(eigen_target_transform_world.rotation());
    target_transform_world.orientation.x = quaternion.x();
    target_transform_world.orientation.y = quaternion.y();
    target_transform_world.orientation.z = quaternion.z();
    target_transform_world.orientation.w = quaternion.w();

    return true;
}
} // namespace skillgraph
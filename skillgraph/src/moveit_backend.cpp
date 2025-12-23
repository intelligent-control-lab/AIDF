/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu 
    Peiqi Yu peiqiy@andrew.cmu.edu 
    Chaitanya Chawla cchawla@cs.cmu.edu 

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

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
    
    fprintf(stderr, "\nCaught signal %s (%d). Performing emergency cleanup...\n", signal_name, signal);
    
    // Emergency cleanup of all active instances
    emergencyCleanup();
    
    // Restore default handler and re-raise for core dump if needed
    std::signal(signal, SIG_DFL);
    std::raise(signal);
}

// Emergency cleanup function
void MoveitInstance::emergencyCleanup() {
    std::lock_guard<std::mutex> lock(instances_mutex_);
    
    fprintf(stderr, "Emergency cleanup: Found %zu active MoveitInstance(s)\n", active_instances_.size());
    
    for (MoveitInstance* instance : active_instances_) {
        if (instance) {
            fprintf(stderr, "Emergency cleanup: Cleaning up MoveitInstance...\n");
            instance->cleanupProcesses();
        }
    }
    
    // Additional system-wide cleanup
    // fprintf(stderr, "Emergency cleanup: Killing remaining ROS processes...\n");
    // int ret1 = std::system("pkill -TERM -f 'ros' 2>/dev/null || true");
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // int ret2 = std::system("pkill -KILL -f 'ros' 2>/dev/null || true");
    // (void)ret1; (void)ret2; // Suppress warnings
}

MoveitInstance::MoveitInstance(robot_state::RobotStatePtr kinematic_state,
                               const std::string &joint_group_name,
                               planning_scene::PlanningScenePtr planning_scene)
    : kinematic_state_(kinematic_state), joint_group_name_(joint_group_name), planning_scene_(planning_scene)
{
    planning_scene_->getPlanningSceneMsg(original_scene_);
    //planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);
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
    
    // launch the move_group node for the robot, based on the moveitConfigPkg, 
    // i.e. roslaunch moveitConfigPkg move_group.launch
    // launch it in the background (create a new process)
    
    namespace bp = boost::process;
    std::vector<std::string> args = {
        moveit_pkg_name,
        "demo.launch",
        "use_rviz:=true"
    };
    boost::asio::io_context io;
    bp::environment env = boost::this_process::environment();
    env["LIBGL_ALWAYS_SOFTWARE"] = "1";
    
    // Create the process in its own process group for easier cleanup
    move_group_process_ = bp::child("/opt/ros/noetic/bin/roslaunch", 
            bp::args(args), 
            env,
            bp::start_dir("/tmp"),
            io
        );
    io.run();

    // Wait a bit for the process to start
    log("MoveitInstance: Waiting for MoveIt package " + moveit_pkg_name + " process to start...", LogLevel::INFO);
    sleep(10);

    // Check if process is running
    if (!move_group_process_.running()) {
        throw std::runtime_error("Failed to start MoveIt move_group process");
    }
    log("MoveitInstance: MoveIt move_group process started successfully", LogLevel::INFO);

    // create moveit and ROS backend
    int argc = 0;
    char **argv = NULL;
    log("MoveitInstance: Initializing a ROS1 node lego_skillgraph", LogLevel::INFO);
    ros::init(argc, argv, "lego_skillgraph");
    nh_ = std::make_shared<ros::NodeHandle>();

    // load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    log("MoveitInstance: Robot model loaded successfully", LogLevel::INFO);

    // create move group interface
    // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_name);
    // joint_group_name_ = move_group_->getName();
    log("MoveitInstance: Move group interface created for group " + move_group_name, LogLevel::INFO);

    // create planning scene interface
    //std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_ = 
    //    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // create a copy of robot kinematic
    kinematic_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    kinematic_state_->setToDefaultValues();
    ros::Duration(0.3).sleep();
    // create planning scene
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    // create a planning scene diff client
    planning_scene_diff_client_ = nh_->serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client_.waitForExistence();

    planning_scene_->getPlanningSceneMsg(original_scene_);

    // marker_publisher
    marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    log("MoveitInstance: MoveIt instance initialized successfully", LogLevel::INFO);
}

MoveitInstance::~MoveitInstance() {
    // Remove this instance from the active instances list
    {
        std::lock_guard<std::mutex> lock(instances_mutex_);
        active_instances_.erase(
            std::remove(active_instances_.begin(), active_instances_.end(), this),
            active_instances_.end()
        );
    }
    
    // Perform cleanup
    cleanupProcesses();
}

void MoveitInstance::cleanupProcesses() {
    // Shutdown ROS node handle first to close connections
    if (nh_) {
        log("MoveitInstance: Shutting down ROS node handle", LogLevel::INFO);
        nh_->shutdown();
        nh_.reset();
    }

    // Ensure the spawned move_group process and all associated ROS processes are terminated
    if (move_group_process_.valid() && move_group_process_.running()) {
        log("MoveitInstance: Terminating move_group process and all ROS processes", LogLevel::INFO);
        
        // Get the process group ID to kill all child processes
        pid_t pgid = getpgid(move_group_process_.id());
        
        // First try graceful termination of the entire process group
        if (pgid > 0) {
            log("MoveitInstance: Sending SIGTERM to process group " + std::to_string(pgid), LogLevel::INFO);
            killpg(pgid, SIGTERM);
            
            // Wait a bit for graceful shutdown
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // If still running, force kill the process group
            if (move_group_process_.running()) {
                log("MoveitInstance: Force killing process group with SIGKILL", LogLevel::INFO);
                killpg(pgid, SIGKILL);
            }
        }
        
        // Terminate the main process if still running
        if (move_group_process_.running()) {
            move_group_process_.terminate();
        }
        
        // Wait for process to finish
        move_group_process_.wait();
        log("MoveitInstance: Move_group process terminated", LogLevel::INFO);
    }
}

void MoveitInstance::setPadding(double padding) {
    planning_scene_->getCollisionEnvNonConst()->setPadding(padding);
    planning_scene_->propogateRobotPadding();
}

bool MoveitInstance::checkCollision(const std::vector<RobotState> &poses, bool self, bool debug) {
    /* check if there is robot-robot or scene collision for a set of poses for some robots*/
    /* true if has collision, false if no collision*/
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

double MoveitInstance::computeDistance(const RobotState& a, const RobotState &b) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    double distance = robot_state_a.distance(robot_state_b);
    return distance;
}

double MoveitInstance::computeDistance(const RobotState& a, const RobotState &b, int dim) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    assert(dim <= a.joint_values.size() && dim <= b.joint_values.size());
    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint = kinematic_state_->getJointModel(joint_name);
    double a_d = a.joint_values[dim];
    double b_d = b.joint_values[dim];
    double distance = joint->distance(&a_d, &b_d);
    return distance;
}

RobotState MoveitInstance::interpolate(const RobotState &a, const RobotState&b, double t) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);

    moveit::core::RobotState res_state = planning_scene_->getCurrentStateNonConst();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    robot_state_a.interpolate(robot_state_b, t, res_state, joint_model_group);

    RobotState res = initRobotState(a.robot_id);
    res_state.copyJointGroupPositions(a.robot_name, res.joint_values);
    res.hand_values = a.hand_values;
    return res;
}

double MoveitInstance::interpolate(const RobotState &a, const RobotState&b, double t, int dim) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    assert(dim <= a.joint_values.size() && dim <= b.joint_values.size());

    std::string joint_name = kinematic_state_->getJointModelGroup(a.robot_name)->getActiveJointModelNames()[dim];
    const moveit::core::JointModel* joint = kinematic_state_->getJointModel(joint_name);
    double a_d = a.joint_values[dim];
    double b_d = b.joint_values[dim];
    double res;
    joint->interpolate(&a_d, &b_d, t, &res);
    return res;
}

bool MoveitInstance::connect(const RobotState& a, const RobotState& b, double col_step_size, bool debug) {
    /* check if a collision-free kinematic path exists from pose a to b for the robot (ignoring other robots)*/
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

bool MoveitInstance::steer(const RobotState& a, const RobotState& b, double max_dist, RobotState& result, double col_step_size) {
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

bool MoveitInstance::sample(RobotState &pose) {
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

void MoveitInstance::addMoveableObject(const Object& obj) {
    if (objects_.find(obj.name) != objects_.end()) {
        log("Object " + obj.name + " already exists in the scene", LogLevel::ERROR);
        return;
    }
    log("Adding object " + obj.name + " to the scene", LogLevel::DEBUG);
    moveit_msgs::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = ros::Time::now();
    co.id = obj.name;

    objects_[obj.name] = obj;

    shape_msgs::SolidPrimitive primitive;
    if (obj.shape == Object::Shape::Box) {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = obj.length;
        primitive.dimensions[primitive.BOX_Y] = obj.width;
        primitive.dimensions[primitive.BOX_Z] = obj.height;
    }
    else if (obj.shape == Object::Shape::Cylinder) {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj.length;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = obj.radius;
    }
    geometry_msgs::Pose world_pose;
    world_pose.position.x = obj.x;
    world_pose.position.y = obj.y;
    world_pose.position.z = obj.z ;
    world_pose.orientation.x = obj.qx;
    world_pose.orientation.y = obj.qy;
    world_pose.orientation.z = obj.qz;
    world_pose.orientation.w = obj.qw;
        
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(world_pose);
    co.operation = co.ADD;
 
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::setObjectColor(const std::string &name, double r, double g, double b, double a) {
    if (objects_.find(name) == objects_.end()) {
        return;
    }

    moveit_msgs::ObjectColor oc;
    oc.id = name;
    oc.color.r = r;
    oc.color.g = g;
    oc.color.b = b;
    oc.color.a = a;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.object_colors.push_back(oc);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client_.call(srv);
}

void MoveitInstance::moveObject(const Object& obj) {
    if (objects_.find(obj.name) == objects_.end()) {
        addMoveableObject(obj);
        return;
    }

    moveit_msgs::CollisionObject co;
    co.header.frame_id = obj.parent_link;
    co.header.stamp = ros::Time::now();
    co.id = obj.name;
    
    co.pose.position.x = obj.x;
    co.pose.position.y = obj.y;
    co.pose.position.z = obj.z ;
    co.pose.orientation.x = obj.qx;
    co.pose.orientation.y = obj.qy;
    co.pose.orientation.z = obj.qz;
    co.pose.orientation.w = obj.qw;

    co.operation = co.MOVE;
 
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::removeObject(const std::string &name)
{
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
    moveit_msgs::CollisionObject co;
    co.id = name;
    co.operation = co.REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::moveRobot(int robot_id, const RobotState& pose) {
    moveit_msgs::PlanningScene cur_scene;
    planning_scene_->getPlanningSceneMsg(cur_scene);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
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
    planning_scene.robot_state.attached_collision_objects = cur_scene.robot_state.attached_collision_objects;
    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotState& pose) {
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

    // calculate relative pose
    // obj.x_attach = 0.006;
    // obj.y_attach = 0.0;
    // obj.z_attach = 0.07;

    // update in moveit
    moveit_msgs::AttachedCollisionObject co;
    co.link_name = obj.parent_link;
    co.object.header.frame_id = obj.parent_link;
    co.object.header.stamp = ros::Time::now();
    co.object.id = name;
    co.object.operation = co.object.ADD;

    // geometry_msgs::Pose relative_pose;
    // relative_pose.position.x = obj.x_attach;
    // relative_pose.position.y = obj.y_attach;
    // relative_pose.position.z = obj.z_attach;
    // relative_pose.orientation.x = obj.qx_attach;
    // relative_pose.orientation.y = obj.qy_attach;
    // relative_pose.orientation.z = obj.qz_attach;
    // relative_pose.orientation.w = obj.qw_attach;
        
    // shape_msgs::SolidPrimitive primitive;
    // if (obj.shape == Object::Shape::Box) {
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[primitive.BOX_X] = obj.length;
    //     primitive.dimensions[primitive.BOX_Y] = obj.width;
    //     primitive.dimensions[primitive.BOX_Z] = obj.height;
    // } 

    // co.object.primitives.push_back(primitive);
    // co.object.primitive_poses.push_back(relative_pose);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    // if (old_parent_link == "base") {
    //     moveit_msgs::CollisionObject co_remove;
    //     co_remove.id = obj.name;
    //     co_remove.header.frame_id = old_parent_link;
    //     co_remove.operation = co_remove.REMOVE;
    //     planning_scene.world.collision_objects.push_back(co_remove);
    // } else {
    //     moveit_msgs::AttachedCollisionObject co_remove;
    //     co_remove.object.id = obj.name;
    //     co_remove.link_name = old_parent_link;
    //     co_remove.object.operation = co_remove.object.REMOVE;
    //     planning_scene.robot_state.attached_collision_objects.push_back(co_remove);
    // }
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

void MoveitInstance::detachObjectFromRobot(const std::string& name, const RobotState& pose) {
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

    moveit_msgs::AttachedCollisionObject co_remove;
    co_remove.object.id = name;
    co_remove.link_name = old_parent_link;
    co_remove.object.operation = co_remove.object.REMOVE;

    // moveit_msgs::CollisionObject co;
    // co.id = obj.name;
    // co.header.frame_id = "base";
    // co.operation = co.ADD;

    // geometry_msgs::Pose world_pose;
    // world_pose.position.x = obj.x;
    // world_pose.position.y = obj.y;
    // world_pose.position.z = obj.z;
    // world_pose.orientation.x = obj.qx;
    // world_pose.orientation.y = obj.qy;
    // world_pose.orientation.z = obj.qz;
    // world_pose.orientation.w = obj.qw;
        
    // shape_msgs::SolidPrimitive primitive;
    // if (obj.shape == Object::Shape::Box) {
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[primitive.BOX_X] = obj.length;
    //     primitive.dimensions[primitive.BOX_Y] = obj.width;
    //     primitive.dimensions[primitive.BOX_Z] = obj.height;
    // }

    // co.object.primitives.push_back(primitive);
    // co.object.primitive_poses.push_back(world_pose);

    moveit_msgs::PlanningScene planning_scene;
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

void MoveitInstance::updateScene() {
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene_diff_;
    bool success = planning_scene_diff_client_.call(srv);
    if (!success || !srv.response.success) {
        log("Failed to update the planning scene", LogLevel::WARN);
    }
}

void MoveitInstance::resetScene(bool reset_sim) {
    // for (const auto& pair : objects_) {
    //     if (pair.second.state == Object::State::Attached) {
    //         detachObjectFromRobot(pair.first, initRobotState(pair.second.robot_id));
    //         if (reset_sim) {
    //             updateScene();
    //         }
    //     }
    // }

    // std::vector<std::string> keys;
    // for (const auto& pair : objects_) {
    //    keys.push_back(pair.first);
    // }
    // for (const auto& key : keys) {
    //     removeObject(key);
    //     if (reset_sim) {
    //         updateScene();
    //     }
    // }

    planning_scene_->setPlanningSceneMsg(original_scene_);
    if (reset_sim) {
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = original_scene_;
        planning_scene_diff_client_.call(srv);
    }
    objects_.clear();
}

void MoveitInstance::setState(const State &state) {
    auto & robot_states = state.robot_states;
    auto & env_states = state.env_state;

    // move the robot state

    moveit_msgs::PlanningScene planning_scene;

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
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;

    // update the object state
    for (int i = 0; i < env_states.objects.size(); i++) {
        ObjPtr obj = env_states.objects[i];
        auto fid = objects_.find(obj->name);
        
        if (obj->state == Object::State::Attached) {
            // attach the object
            moveit_msgs::AttachedCollisionObject co;
            co.link_name = obj->parent_link;
            co.object.header.frame_id = obj->parent_link;
            co.object.header.stamp = ros::Time::now();
            co.object.id = obj->name;
            co.object.operation = co.object.ADD;

            shape_msgs::SolidPrimitive primitive;
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

            geometry_msgs::Pose relative_pose;
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
            moveit_msgs::CollisionObject co;
            co.header.frame_id = obj->parent_link;
            co.header.stamp = ros::Time::now();
            co.id = obj->name;

            shape_msgs::SolidPrimitive primitive;
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
            geometry_msgs::Pose world_pose;
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

        // Add text marker for this object
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = obj->parent_link;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "object_labels";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        // Position the text slightly above the object
        text_marker.pose.position.x = obj->x;
        text_marker.pose.position.y = obj->y;
        text_marker.pose.position.z = obj->z + obj->height/2 + 0.05; // Adjust based on object size
        text_marker.pose.orientation.w = 1.0;
        
        // Set text properties
        text_marker.text = obj->name;
        text_marker.scale.z = 0.02; // Text height
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.lifetime = ros::Duration(); // Persistent until removed
        
        marker_array.markers.push_back(text_marker);
        
    }

    marker_pub_.publish(marker_array);

    //planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = false;
    planning_scene.robot_state.is_diff = false; 

    planning_scene_->usePlanningSceneMsg(planning_scene);
    planning_scene_diff_ = planning_scene;
}

void MoveitInstance::printKnownObjects() const {
    planning_scene_->printKnownObjects();
}

bool MoveitInstance::setCollision(const std::string& obj_name, const std::string& link_name, bool allow) {

    // Get the Allowed Collision Matrix (ACM)
    // Use the PlanningSceneMonitor to get the current planning scene

    moveit_msgs::AllowedCollisionMatrix acm;
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
        moveit_msgs::AllowedCollisionEntry new_entry;
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

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = acm;
    planning_scene_->usePlanningSceneMsg(planning_scene);

    planning_scene_diff_ = planning_scene;

    return true;
}

void MoveitInstance::computeRelativeTransform(Object &obj, const RobotState &robot_state)
{
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

void MoveitInstance::computeWorldTransform(Object &obj, const RobotState &robot_state)
{
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

MoveitControl::MoveitControl(std::shared_ptr<MoveitInstance> instance, bool fake_move) 
        : instance_(instance), fake_move_(fake_move) {
    // initialize the moveit instance

}

bool MoveitControl::move(TaskParamPtr post_condition, const RobotTrajectory &trajectory) {
    if (fake_move_) {
        // interpolate the robot state to the target state according to the trajectory
        // iterate the trajectory
        if (!trajectory.trajectory.empty()) {
            for (const auto &robot_state : trajectory.trajectory) {
                instance_->moveRobot(robot_state.robot_id, robot_state);
                instance_->updateScene();
                ros::Duration(0.1).sleep();
        
            }
        }

        if (post_condition) {
            instance_->setState(post_condition->target_state);
            instance_->updateScene();
        }
        return true;
    }

    // check if the trajectory is valid
    if (trajectory.trajectory.empty()) {
        log("Trajectory is empty", LogLevel::ERROR);
        return false;
    }

#ifndef HAVE_YK_TASKS
    log("Real robot execution requested but HAVE_YK_TASKS is not defined", LogLevel::ERROR);
    return false;
#else
    const auto &states = trajectory.trajectory;
    const RobotState &start_state = states.front();
    const std::string &group_name = start_state.robot_name;
    const int robot_id = start_state.robot_id;

    for (const auto &state : states) {
        if (state.robot_id != robot_id) {
            log("Trajectory contains states for multiple robots, cannot execute", LogLevel::ERROR);
            return false;
        }
        if (state.robot_name != group_name) {
            log("Trajectory contains inconsistent joint groups, cannot execute", LogLevel::ERROR);
            return false;
        }
    }

    planning_scene::PlanningScenePtr planning_scene = instance_->getPlanningScene();
    if (!planning_scene) {
        log("Planning scene is unavailable for real robot execution", LogLevel::ERROR);
        return false;
    }

    robot_model::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
    if (!robot_model) {
        log("Robot model is unavailable for real robot execution", LogLevel::ERROR);
        return false;
    }

    const moveit::core::JointModelGroup *joint_model_group = robot_model->getJointModelGroup(group_name);
    if (!joint_model_group) {
        log("Joint model group " + group_name + " not found", LogLevel::ERROR);
        return false;
    }

    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    if (joint_names.size() != start_state.joint_values.size()) {
        log("Joint dimension mismatch for group " + group_name + 
                ": expected " + std::to_string(joint_names.size()) +
                ", got " + std::to_string(start_state.joint_values.size()),
            LogLevel::ERROR);
        return false;
    }

    moveit_msgs::ExecuteKnownTrajectory srv;
    srv.request.wait_for_execution = true;
    trajectory_msgs::JointTrajectory &joint_traj = srv.request.trajectory.joint_trajectory;
    joint_traj.header.stamp = ros::Time::now();
    joint_traj.joint_names = joint_names;

    constexpr double kDefaultTimeStep = 0.1;
    double base_time = (!trajectory.times.empty()) ? trajectory.times.front() : 0.0;
    double prev_time = 0.0;

    for (size_t i = 0; i < states.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = states[i].joint_values;
        point.velocities.assign(joint_names.size(), 0.0);
        point.accelerations.assign(joint_names.size(), 0.0);

        double time_from_start = 0.0;
        if (!trajectory.times.empty() && i < trajectory.times.size()) {
            time_from_start = trajectory.times[i] - base_time;
        } else if (!trajectory.times.empty()) {
            time_from_start = (trajectory.times.back() - base_time) +
                              kDefaultTimeStep * static_cast<double>(i - trajectory.times.size() + 1);
        } else {
            time_from_start = kDefaultTimeStep * static_cast<double>(i);
        }

        if (i == 0) {
            time_from_start = 0.0;
        }

        if (i > 0 && time_from_start <= prev_time) {
            time_from_start = prev_time + kDefaultTimeStep;
        }

        point.time_from_start = ros::Duration(time_from_start);
        joint_traj.points.push_back(point);
        prev_time = time_from_start;
    }

    std::string service_name = "/" + group_name + "/yk_execute_trajectory";
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(service_name);

    if (!client.waitForExistence(ros::Duration(2.0))) {
        log("Service " + service_name + " is not available", LogLevel::ERROR);
        return false;
    }

    log("Executing trajectory for robot " + group_name + " through service " + service_name, LogLevel::INFO);
    if (!client.call(srv)) {
        log("Failed to call service " + service_name, LogLevel::ERROR);
        return false;
    }

    int error_code = srv.response.error_code.val;
    if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        log("Trajectory execution failed with error code " + std::to_string(error_code), LogLevel::ERROR);
        return false;
    }

    log("Trajectory execution succeeded for robot " + group_name, LogLevel::INFO);

    if (post_condition) {
        instance_->setState(post_condition->target_state);
        instance_->updateScene();
    }

    return true;
#endif
}



}

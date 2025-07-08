#include "rrt_algorithm.hpp"
#include "moveit_backend.hpp" // For skillgraph::MoveitInstance

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_state/conversions.h>
#include "Utils/Logger.hpp" // For skillgraph::log

namespace skillgraph {

RRTConnect::RRTConnect(robot_model::RobotModelPtr robot_model,
                       std::shared_ptr<skillgraph::PlanInstance> instance)
    : robot_model_(robot_model), instance_(instance) {
    // If PlannerOptions were a member, you would initialize it here.
    // For example: options_.max_planning_time = 5.0;
}

bool RRTConnect::plan(const skillgraph::State &start, const skillgraph::State &goal,
                      skillgraph::RobotTrajectory &traj) {

    if (start.robot_states.empty() || goal.robot_states.empty()) {
        log("RRTConnect::plan: Start or goal robot states are empty.", LogLevel::ERROR);
        return false;
    }

    // Assuming planning for the first robot in the state
    // This could be parameterized or determined differently if needed.
    int robot_id_to_plan = start.robot_states[0].robot_id;
    if (robot_id_to_plan >= instance_->getNumberOfRobots()) {
        log("RRTConnect::plan: Invalid robot_id in start state.", LogLevel::ERROR);
        return false;
    }
    // Ensure the instance is a MoveitInstance
    auto moveit_instance = std::dynamic_pointer_cast<MoveitInstance>(instance_);

    /*
    written by Phillip
    // std::string group_name_to_plan = instance_->getRobotNames()[robot_id_to_plan];

    // // 1. Set the start state in the planning scene
    // instance_->setState(start);
    // instance_->updateScene(); // Apply the changes to MoveIt's planning scene

    // // 2. Prepare MotionPlanRequest
    // planning_interface::MotionPlanRequest req;
    // req.group_name = group_name_to_plan;
    // req.planner_id = "RRTConnectkConfigDefault"; // Or just "RRTConnect", check your MoveIt setup
    //                                  // Using RRTConnectkConfigDefault as often used.
    // // Using a hardcoded planning time. Ideally, this comes from PlannerOptions.
    // req.allowed_planning_time = 5.0; 
    // req.num_planning_attempts = 1; // Default

    // // Set start state from the updated planning scene
    // // The planning_scene_ in MoveitInstance should now reflect the 'start' state.
    // moveit::core::robotStateToRobotStateMsg(instance_->getPlanningScene()->getCurrentState(), req.start_state);
    // req.start_state.is_diff = false; // Ensure it's a full state

    // // 3. Set Goal Constraints
    // const skillgraph::RobotState& goal_robot_pose = goal.robot_states[0]; // Assuming goal for the same robot
    // robot_state::RobotState goal_state_moveit(robot_model_);
    // const robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name_to_plan);
    // if (!joint_model_group) {
    //     log("RRTConnect::plan: Could not get JointModelGroup: " + group_name_to_plan, LogLevel::ERROR);
    //     return false;
    // }
    // goal_state_moveit.setJointGroupPositions(joint_model_group, goal_robot_pose.joint_values);
    // // Potentially update attached objects for goal state if they differ and affect constraints
    // // For transit, usually, attached objects remain the same or are handled by the overall task.
    // // Here, we rely on the joint values for the goal.

    // req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state_moveit, joint_model_group));

    // // 4. Create Planning Pipeline
    // // Ensure MoveitInstance provides a valid NodeHandle (e.g., instance_->nh_)
    // planning_pipeline::PlanningPipelinePtr planning_pipeline = 
    //     std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, *(instance_->getNodeHandle()), 
    //                                                           "ompl", "request_adapters");
    //                                                         // "ompl" or "planning_plugin" based on your setup

    // // 5. Generate Plan
    // log("RRTConnect::plan: Planning for group " + group_name_to_plan, LogLevel::INFO);
    // planning_interface::MotionPlanResponse res;
    // planning_pipeline->generatePlan(instance_->planning_scene_, req, res);

    // // 6. Process Result
    // if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    //     log("RRTConnect::plan: Planning successful.", LogLevel::INFO);
        
    //     // Convert moveit_msgs::RobotTrajectory to skillgraph::RobotTrajectory
    //     traj.trajectory.clear();

    //     const moveit_msgs::RobotTrajectory& moveit_traj_msg = res.trajectory;
    //     const std::vector<std::string>& traj_joint_names = moveit_traj_msg.joint_trajectory.joint_names;

    //     // Ensure the target skillgraph::RobotState joint_values are ordered correctly.
    //     // We assume the order in moveit_traj_msg.joint_trajectory.points[i].positions
    //     // matches the active joints of the planning group 'group_name_to_plan'.

    //     for (const auto& point : moveit_traj_msg.joint_trajectory.points) {
    //         skillgraph::RobotState sk_rs = instance_->initRobotState(robot_id_to_plan); 
            
    //         if (point.positions.size() != sk_rs.joint_values.size()) {
    //              log("RRTConnect::plan: Mismatch in joint count between trajectory point and skillgraph RobotState. Traj joints: " + std::to_string(point.positions.size()) + ", Expected: " + std::to_string(sk_rs.joint_values.size()), LogLevel::WARN);
    //             // Fallback or error: For now, we'll try to copy if sizes match, otherwise skip hand_values or error out.
    //             // This indicates a potential mismatch in group definition or initRobotState.
    //             // For simplicity, if sizes match, copy. Otherwise, this part needs careful handling.
    //             if(point.positions.size() == sk_rs.joint_values.size()){
    //                 sk_rs.joint_values = point.positions;
    //             } else {
    //                 log("RRTConnect::plan: Critical joint count mismatch. Cannot convert trajectory.", LogLevel::ERROR);
    //                 return false;
    //             }
    //         } else {
    //              sk_rs.joint_values = point.positions;
    //         }

    //         // For transit tasks, hand_values usually don't change or are not part of the arm planning group.
    //         // Copy from the goal state's hand configuration.
    //         sk_rs.hand_values = goal_robot_pose.hand_values; 
                                                            
    //         traj.trajectory.push_back(sk_rs);
    //     }

    //     // Update instance to goal state (optional, depending on desired post-planning state)
    //     // moveit_instance->setState(goal);
    //     // moveit_instance->updateScene();
    //     return true;
    // } else {
    //     log("RRTConnect::plan: Planning failed. Error code: " + std::to_string(res.error_code_.val), LogLevel::ERROR);
    //     return false;
    // }

    */




    // written by Yijie for RRT-connect planning
    std::string group_name_to_plan = instance_->getRobotNames()[robot_id_to_plan];

    // set the state into Moveit
    instance_->setState(start);
    instance_->updateScene(); // Apply the changes to MoveIt's planning scene

    // Prepare MotionPlanRequest
    planning_interface::MotionPlanRequest req;
    req.group_name = group_name_to_plan;
    req.planner_id = "RRTConnectkConfigDefault"; // Or just "R
    req.allowed_planning_time = 5.0; // Using a hardcoded planning time. Ideally, this comes from PlannerOptions.
    req.num_planning_attempts = 1; // Default

    // Set start state from the updated planning scene
    // moveit::core::robotStateToRobotStateMsg(instance_->getPlanningScene()->getCurrentState(), req.start_state);
    moveit::core::robotStateToRobotStateMsg(moveit_instance->getPlanningScene()->getCurrentState(), req.start_state);
    req.start_state.is_diff = false; 

    // Set Goal Constraints
    const skillgraph::RobotState& goal_robot_pose = goal.robot_states[0]; //
    robot_state::RobotState goal_state_moveit(robot_model_);
    const robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name_to_plan);
    if (!joint_model_group) {
        log("RRTConnect::plan: Could not get JointModelGroup: " + group_name_to_plan, LogLevel::ERROR);
        return false;
    }
    goal_state_moveit.setJointGroupPositions(joint_model_group, goal_robot_pose.joint_values);
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state_moveit, joint_model_group));
    // Potentially update attached objects for goal state if they differ and affect constraints


    //pipeline
    planning_pipeline::PlanningPipelinePtr planning_pipeline = 
    // std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, *(instance_->getNodeHandle()), "ompl", "request_adapters");
    std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, *(moveit_instance->getNodeHandle()), "ompl", "request_adapters");

    planning_interface::MotionPlanResponse res;
    planning_pipeline->generatePlan(moveit_instance->getPlanningScene(), req, res);
    // planning_pipeline->generatePlan(instance_->getPlanningScene(), req, res);

    if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    log("RRTConnect::plan: Planning successful.", LogLevel::INFO);

    traj.trajectory.clear();

    // const moveit_msgs::RobotTrajectory& moveit_traj_msg = res.trajectory;
    moveit_msgs::RobotTrajectory moveit_traj_msg;
    if (res.trajectory_) {
        res.trajectory_->getRobotTrajectoryMsg(moveit_traj_msg);
    }



    for (const auto& point : moveit_traj_msg.joint_trajectory.points) {
        skillgraph::RobotState sk_rs = instance_->initRobotState(robot_id_to_plan); 
        
        if (point.positions.size() != sk_rs.joint_values.size()) {
            log("RRTConnect::plan: Mismatch in joint count between trajectory point and skillgraph RobotState.", LogLevel::WARN);
            if(point.positions.size() == sk_rs.joint_values.size()){
                sk_rs.joint_values = point.positions;
            } else {
                log("RRTConnect::plan: Critical joint count mismatch. Cannot convert trajectory.", LogLevel::ERROR);
                return false;
            }
        } else {
            sk_rs.joint_values = point.positions;
        }

        sk_rs.hand_values = goal_robot_pose.hand_values;

        traj.trajectory.push_back(sk_rs);
    }
        return true;
    } else {
        log("RRTConnect::plan: Planning failed. Error code: " + std::to_string(res.error_code_.val), LogLevel::ERROR);
        return false;
    }


//just a placeholder for now, as the actual planning logic is not implemented yet.
    // return false;
}

} // namespace skillgraph
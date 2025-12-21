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
    Changliu Liu cliu6@andrew.cmu.edu 
    Jiaoyang Li jiaoyanl@andrew.cmu.edu 
    Guanya Shi guanyas@andrew.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

/**
 * @file backend.cpp
 * @brief Implements the PlanInstance backend for robot planning in skillgraph.
 */

#include "backend.hpp"

namespace skillgraph {

/**
 * @brief Set the number of robots in the planning instance.
 * @param num_robots Number of robots.
 */
void PlanInstance::setNumberOfRobots(int num_robots) {
    num_robots_ = num_robots;
    start_poses_.resize(num_robots);
    goal_poses_.resize(num_robots);
    robot_dof_.resize(num_robots);
    hand_dof_.resize(num_robots, 0);
}

/**
 * @brief Set the start pose for a robot.
 * @param robot_id Robot ID.
 * @param pose Joint values for the start pose.
 */
void PlanInstance::setStartPose(int robot_id, const std::vector<double> &pose) {
    start_poses_[robot_id].robot_id = robot_id;
    start_poses_[robot_id].robot_name = robot_names_[robot_id];
    start_poses_[robot_id].joint_values = pose;
}

/**
 * @brief Set the goal pose for a robot.
 * @param robot_id Robot ID.
 * @param pose Joint values for the goal pose.
 */
void PlanInstance::setGoalPose(int robot_id, const std::vector<double> &pose) {
    goal_poses_[robot_id].robot_id = robot_id;
    goal_poses_[robot_id].robot_name = robot_names_[robot_id];
    goal_poses_[robot_id].joint_values = pose;
}

/**
 * @brief Set the degrees of freedom for a robot.
 * @param robot_id Robot ID.
 * @param dof Number of degrees of freedom.
 */
void PlanInstance::setRobotDOF(int robot_id, size_t dof) {
    if (robot_id >= robot_dof_.size()) {
        robot_dof_.resize(robot_id + 1);
    }
    robot_dof_[robot_id] = dof;
}

/**
 * @brief Set the hand degrees of freedom for a robot.
 * @param robot_id Robot ID.
 * @param dof Number of hand degrees of freedom.
 */
void PlanInstance::setHandDof(int robot_id, size_t dof) {
    if (robot_id >= hand_dof_.size()) {
        hand_dof_.resize(robot_id + 1);
    }
    hand_dof_[robot_id] = dof;
}


/**
 * @brief Get the degrees of freedom for a robot.
 * @param robot_id Robot ID.
 * @return Number of degrees of freedom.
 */
size_t PlanInstance::getRobotDOF(int robot_id) const {
    return robot_dof_[robot_id];
}

/**
 * @brief Get the hand degrees of freedom for a robot.
 * @param robot_id Robot ID.
 * @return Number of hand degrees of freedom.
 */
size_t PlanInstance::getHandDOF(int robot_id) const {
    return hand_dof_[robot_id];
}

/**
 * @brief Initialize a RobotState for a given robot.
 * @param robot_id Robot ID.
 * @return Initialized RobotState.
 */
skillgraph::RobotState PlanInstance::initRobotState(int robot_id) const {
    skillgraph::RobotState pose;
    pose.robot_id = robot_id;
    pose.robot_name = robot_names_[robot_id];
    pose.joint_values.resize(robot_dof_[robot_id]);
    pose.hand_values.resize(hand_dof_[robot_id], 0);
    return pose;
}

/**
 * @brief Get the maximum velocity for a robot.
 * @param robot_id Robot ID.
 * @return Maximum velocity.
 */
double PlanInstance::getVMax(int robot_id) {
    return v_max_;
}

/**
 * @brief Set the maximum velocity for all robots.
 * @param vmax Maximum velocity.
 */
void PlanInstance::setVmax(double vmax) {
    v_max_ = vmax;
}

/**
 * @brief Get and reset the number of collision checks performed.
 * @return Number of collision checks since last call.
 */
int PlanInstance::numCollisionChecks() {
    int ans = num_collision_checks_;
    num_collision_checks_ = 0;
    return ans;
}

}
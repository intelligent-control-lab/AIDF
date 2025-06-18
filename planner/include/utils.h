#pragma once

#include "planner.h"
#include "moveit_backend.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>

namespace planner {
    
// utils
bool convertSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
    const moveit_msgs::msg::RobotTrajectory &plan_traj,
    skillgraph::MRTrajectory &solution,
    bool reset_speed = true);


bool convertSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
    const moveit_msgs::msg::RobotTrajectory &plan_traj,
    int robot_id,
    skillgraph::RobotTrajectory &solution);

bool saveSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
  const moveit_msgs::msg::RobotTrajectory &plan_traj,
  const std::string &file_name);
  
bool saveSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
  const skillgraph::MRTrajectory &synced_traj,
  const std::string &file_name);

/* time is assumed to be uniform as dt */
bool loadSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
  const std::string &file_name,
  double dt,
  moveit_msgs::msg::RobotTrajectory &plan_traj);

/* time is supplied in the first column*/
bool loadSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
  const std::string &file_name,
  moveit_msgs::msg::RobotTrajectory &plan_traj);

bool validateSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
    const skillgraph::MRTrajectory &solution,
    double col_dt);

/* assuming uniform discretiziation, check for collisions*/
bool validateSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
       const skillgraph::MRTrajectory &solution);

void retimeSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
    const skillgraph::MRTrajectory &solution,
    skillgraph::MRTrajectory &retime_solution,
    double dt);

void rediscretizeSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
    const moveit_msgs::msg::RobotTrajectory &plan_traj,
    moveit_msgs::msg::RobotTrajectory &retime_traj,
    double new_dt);

void rediscretizeSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
        const skillgraph::MRTrajectory &solution,
        skillgraph::MRTrajectory &retime_solution,
        double new_dt);
void removeWait(std::shared_ptr<skillgraph::PlanInstance> instance,
    skillgraph::MRTrajectory &solution);
bool validateSolution(std::shared_ptr<skillgraph::PlanInstance> instance,
     const moveit_msgs::msg::RobotTrajectory &plan_traj);

bool optimizeTrajectory(std::shared_ptr<skillgraph::PlanInstance> instance,
        const moveit_msgs::msg::RobotTrajectory& input_trajectory,
        const std::string& group_name,
        moveit::core::RobotModelConstPtr robot_model,
        rclcpp::Node::SharedPtr node,
        moveit_msgs::msg::RobotTrajectory& smoothed_traj
        );

skillgraph::SmoothnessMetrics calculate_smoothness(const skillgraph::MRTrajectory &synced_plan, 
        std::shared_ptr<skillgraph::PlanInstance> instance);

}
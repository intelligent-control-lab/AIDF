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
 * @file rrt_algorithm.hpp
 * @brief Defines the RRTConnect planning algorithm and related options for skillgraph.
 */
#pragma once
#include "algorithms.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {

/**
 * @struct PlannerOptions
 * @brief Options for configuring the planner behavior.
 */
struct PlannerOptions {
    double max_planning_time = 1.0; /**< Maximum allowed planning time (seconds) */
    int max_planning_iterations = 10000; /**< Maximum number of planning iterations */
    bool terminate_on_first_sol = true; /**< Terminate on first solution found */
    bool pp_random_order = false; /**< Use random order for post-processing */
    std::string single_agent_planner = "RRT"; /**< Planner type for single agent */
    std::string log_fname = ""; /**< Log file name */
    skillgraph::MRTrajectory obstacles; /**< Obstacles for planning */

    // constructor
    PlannerOptions() = default;
    PlannerOptions(double max_planning_time, int max_planning_iterations)
    {
        this->max_planning_time = max_planning_time;
        this->max_planning_iterations = max_planning_iterations;
    }
};

/**
 * @class RRTConnect
 * @brief Implements the RRT-Connect planning algorithm using MoveIt.
 */
class RRTConnect : public PlanningAlgorithm {
    public:
        /**
         * @brief Construct an RRTConnect planner.
         * @param robot_model Robot model pointer.
         * @param instance Shared pointer to PlanInstance backend.
         */
        RRTConnect(robot_model::RobotModelPtr robot_model,
                   std::shared_ptr<skillgraph::PlanInstance> instance);
        /**
         * @brief Plan a trajectory from start to goal state.
         * @param start Start state.
         * @param goal Goal state.
         * @param traj Output planned trajectory.
         * @return True if planning succeeded, false otherwise.
         */
        virtual bool plan(const skillgraph::State &start, const skillgraph::State &goal,
                          skillgraph::RobotTrajectory &traj);
    private:
        robot_model::RobotModelPtr robot_model_; /**< Robot model pointer */
        std::shared_ptr<skillgraph::PlanInstance> instance_; /**< Backend instance */
    };

}
/**
 * @file rrt_algorithm.hpp
 * @brief Defines the RRTConnect planning algorithm and related options for skillgraph.
 */
#pragma once
#include "algorithms.hpp"
#include "moveit_backend.hpp"
#include <moveit/robot_model/robot_model.h>

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
        RRTConnect(moveit::core::RobotModelPtr robot_model,
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
        moveit::core::RobotModelPtr robot_model_; /**< Robot model pointer */
        std::shared_ptr<skillgraph::PlanInstance> instance_; /**< Backend instance */
    };

}
#pragma once
#include "algorithms.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {

struct PlannerOptions {
    double max_planning_time = 1.0;
    int max_planning_iterations = 10000;
    bool terminate_on_first_sol = true;
    bool pp_random_order = false;
    std::string single_agent_planner = "RRT";
    std::string log_fname = "";
    skillgraph::MRTrajectory obstacles;

    // constructor
    PlannerOptions() = default;
    PlannerOptions(double max_planning_time, int max_planning_iterations)
    {
        this->max_planning_time = max_planning_time;
        this->max_planning_iterations = max_planning_iterations;
    }
};

class RRTConnect : public PlanningAlgorithm {
    public:
        RRTConnect(robot_model::RobotModelPtr robot_model,
                   std::shared_ptr<skillgraph::PlanInstance> instance);
        
        virtual bool plan(const skillgraph::State &start, const skillgraph::State &goal,
                          skillgraph::RobotTrajectory &traj);
    private:
        robot_model::RobotModelPtr robot_model_;
        std::shared_ptr<skillgraph::PlanInstance> instance_;
    };

}
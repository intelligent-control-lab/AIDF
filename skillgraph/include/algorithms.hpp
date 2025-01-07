#pragma once
#include "moveit_env.hpp"
#include "robots_env.hpp"

namespace algo {
    class Algorithm {
        enum Type {
            RRTC = 0,
            PrioritizedPlan = 1,
            FTS_Pick = 2,
        };

        public:
            Algorithm() = default;
            Type type;
    };

    struct PlannerOptions {
        double max_planning_time = 1.0;
        int max_planning_iterations = 10000;
        bool terminate_on_first_sol = true;
        bool pp_random_order = false;
        std::string single_agent_planner = "RRT";
        std::string log_fname = "";
        robot::MRTrajectory obstacles;

        // constructor
        PlannerOptions() = default;
        PlannerOptions(double max_planning_time, int max_planning_iterations)
        {
            this->max_planning_time = max_planning_time;
            this->max_planning_iterations = max_planning_iterations;
        }
    };

    class RRTConnect : public Algorithm {
    public:
        RRTConnect(robot_model::RobotModelPtr robot_model);
        
        virtual bool plan(robot::RobotState &start,
                          robot::RobotState &goal,
                          std::shared_ptr<robot::MoveitInstance> instance,
                          robot::MRTrajectory &traj);
    private:
        robot_model::RobotModelPtr robot_model_;
        std::shared_ptr<robot::MoveitInstance> instance_;
    };

}
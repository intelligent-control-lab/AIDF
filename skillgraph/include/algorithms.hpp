#pragma once
#include "moveit_backend.hpp"
#include "backend.hpp"
#include "metrics.hpp"

namespace algo {
    class Algorithm {
        /*
        * Base Algorithm Class containing the type and name of the algorithm
        */
        enum Type {
            Planning = 0,
            Control = 1,
            Perception = 2,
        };

        public:
            Algorithm() = default;
            Type type;
            std::string name;
    };

    class SkillPerformingAlgorithm : public Algorithm {
        /*
        * SkillPerformingAlgorithm Class containing the implementation, properties, and a function API for performing a skill
        */
        public:
            SkillPerformingAlgorithm() = default;
            SkillPerformingAlgorithm(const std::string &name);
            
            // chooses an implementation
            Algorithm implementation;

            // properties
            metric::Evaluator pre_condition;
            metric::Evaluator post_condition;
            std::string skill_type; // corresponding skill

            // funnction
            std::function<std::any(const std::vector<std::any>&)> perform();
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

    class PlanningAlgorithm : public Algorithm {
        public:
            PlanningAlgorithm() = default;
    };

    class ControlAlgorithm : public Algorithm {
        public:
            ControlAlgorithm() = default;
    };

    class PerceptionAlgorithm: public Algorithm {
        public: 
            PerceptionAlgorithm() = default;
    };

    class RRTConnect : public PlanningAlgorithm {
    public:
        RRTConnect(robot_model::RobotModelPtr robot_model,
                   std::shared_ptr<env::PlanInstance> instance);
        
        virtual bool plan(const robot::RobotState &start, const robot::RobotState &goal,
                          robot::MRTrajectory &traj);
    private:
        robot_model::RobotModelPtr robot_model_;
        std::shared_ptr<env::PlanInstance> instance_;
    };

}
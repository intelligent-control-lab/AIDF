#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "robots_env.hpp"
#include "task_def.hpp"
#include "algorithms.hpp"

using namespace robot;

namespace skillgraph
{   
    struct Skill {
        enum Type {
            pick = 1,
            assemble = 2,
            support = 3,
            transfer = 4,
            reposition = 5,
            transit = 6,
        };

        Type type;
        task_def::ActPtr act_ptr;
        object::Object object;
        robot::Robot robot;
        algo::Algorithm algorithm;
    };

    class Skillgraph{
        private:
            std::vector<std::string> meta_skills;
            std::map<std::string, std::vector<std::string>> atmoic_skills;
            std::vector<std::string> robot_capabilities;

            std::map<std::string, object::Object> object_library;
            std::map<std::string, task_def::Activity> user_task;
            std::map<std::string, robot::RobotState> robot_state_form;
            std::map<std::string, task_def::EnvState> env_state_form;

        public:
            Skillgraph();
            ~Skillgraph(){}
            
            void print_skillgraph();

            void add_meta_skill(const std::string& meta_skill, const std::vector<std::string> &atomic_skill);
            void add_atomic_skill(const std::string& atomic_skill);
            void add_robot_capability(const std::string& robot_capability);

            std::vector<std::string> get_meta_skills();
            std::map<std::string, std::vector<std::string>> get_atomic_skills();
            std::vector<std::string> get_robot_capabilities();
            
            std::set<Skill> feasible_u(const task_def::State& state);

        protected:
            std::shared_ptr<PlanInstance> instance;
            // std::tuple<std::vector<std::string>, std::function<bool(const std::map<std::string, std::any>&)>> 
            //     feasible_target_states(const std::string& skill, const task_def::Object& obj, const task_def::State& state);
            // std::set<std::map> feasible_atomic_skills(const task_def::State& state);
            // std::function<bool(const std::map<std::string, std::any>&)> 
            //     pre_condition(const std::string& skill, const task_def::Object& obj, const task_def::State& state);
            // std::array<task_def::State> 
            //     trajectory_algorithm(const std::string& skill, const task_def::Object& obj, const task_def::State& initial_state, const task_def::State& target_state);

    };
}

#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "task_definition.hpp"

namespace skillgraph
{   
    class Skillgraph{
        private:
            std::vector<std::string> meta_skills;
            std::map<std::string, std::vector<std::string>> atmoic_skills;
            std::vector<std::string> robot_capabilities;

            std::map<std::string, std::map> object_library;
            std::map<std::string, std::map> user_task;
            std::map<std::string, std::map> robot_state_form;
            std::map<std::string, std::map> env_state_form;

        public:
            Skillgraph();
            ~Skillgraph(){}
            
            void print_skillgraph();

            void add_meta_skill(const std::string& meta_skill);
            void add_atomic_skill(const std::string& meta_skill, const std::string& atomic_skill);
            void add_robot_capability(const std::string& robot_capability);

            std::vector<std::string> get_meta_skills();
            std::map<std::string, std::vector<std::string>> get_atomic_skills();
            std::vector<std::string> get_robot_capabilities();
            
            std::set<std::map> feasible_u(const task_definition::State& state);

            std::tuple<std::vector<std::string>, std::function<bool(const std::map<std::string, std::any>&)>> 
                feasible_target_states(const std::string& skill, const task_definition::Object& obj, const task_definition::State& state);
            std::set<std::map> feasible_atomic_skills(const task_definition::State& state);
            std::function<bool(const std::map<std::string, std::any>&)> 
                pre_condition(const std::string& skill, const task_definition::Object& obj, const task_definition::State& state);
            std::array<task_definition::State> 
                trajectory_algorithm(const std::string& skill, const task_definition::Object& obj, const task_definition::State& initial_state, const task_definition::State& target_state);

    };
}

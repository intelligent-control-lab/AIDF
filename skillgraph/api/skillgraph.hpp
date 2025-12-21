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

#pragma once
#include "environment.hpp"
#include "tasks.hpp"
#include "objects.hpp"
#include "robots.hpp"
#include "algorithms.hpp"
#include "skills.hpp"


namespace skillgraph
{   
    enum TaskType {
        Lego = 1,
        NIST = 2,
    };

    enum BackEnd {
        MOVEIT = 1,
        MUJOCO = 2,
    };

    class SkillGraph{
        /*
        * SkillGraph Class containing that maps all the objects, robots, envs, skills, executors, and algorithms
        * Provides the API for the skill graph in the overall ontology
        */
        protected:
            // symbols defined by user
            std::map<std::string, Object> object_library; // maps of all objects
            std::map<std::string, Skill::Type> skill_types;  // maps from user-defined string to enum

            std::string robot_state_form; // choice of robot state representation
            std::string env_state_form; // choice of env state representation

            Json::Value root_config_; // user-provided Json config of this skill graph
            std::vector<RobotPtr> robots; // list of robots and capabilities
            std::map<std::string, std::vector<std::string>> robot_capabilities; // map of robot name, and its capabilities in string
            std::vector<Skill::Type> atmoic_skills; // list of atomic skills
            std::map<Skill::Type, std::vector<Skill::Type>> meta_skills; // map of all meta stkills
            std::map<Skill::Type, SkillPtr> skill_map_;  // map of all skills by name

            // task defintion for feasible_u
            TaskType task_type_; // task type
            State initial_state_;
            State target_state_;
            std::shared_ptr<AssemblySeq> task_seq_;

            // environemnt
            int num_robots_;
            std::shared_ptr<Environment> env_;

            // root config filename
            std::string config_fname;

            virtual void parse_skills(const Json::Value &root);
            virtual void parse_robots(const Json::Value &root);
            virtual void parse_env(const Json::Value &root);
            virtual void parse_tasks(const Json::Value &root);

        public:
            SkillGraph(const std::string &config_file);
            virtual ~SkillGraph() {};

            virtual void initialize();
            
            void print_skillgraph();

            void add_meta_skill(const std::string& meta_skill, const std::vector<std::string> &atomic_skill);
            void add_atomic_skill(const std::string& atomic_skill);
            void add_robot_capability(const std::string& robot_capability);

            std::vector<Skill::Type> get_meta_skills();
            std::map<Skill::Type, std::vector<Skill::Type>> get_atomic_skills();
            std::vector<Skill::Type> get_robot_capabilities();

            std::vector<std::string> get_robot_names() const;
            std::vector<std::string> get_hand_names() const;
            
            virtual bool at_target(const State& state) { throw std::runtime_error("At Target Not implemented");};
            virtual std::vector<SkillPtr> feasible_u(const State& state) { throw std::runtime_error("Feasible u Not implemented");};
            virtual bool get_next_state(const State& state, SkillPtr gs, State &next_state, double &cost) { throw std::runtime_error("get_next_state Not implemented");};
            virtual bool is_feasible(const State&state, Json::Value &skill_config, SkillPtr &gs)  { throw std::runtime_error("get_next_state Not implemented");};

            State get_initial_state() const { return initial_state_; }
            SkillPtr get_skill(const std::string &skill_name) const;
            RobotPtr get_robot(const std::string &robot_name) const;

            // std::tuple<std::vector<std::string>, std::function<bool(const std::map<std::string, std::any>&)>> 
            //     feasible_target_states(const std::string& skill, const Object& obj, const env::State& state);
            // std::set<std::map> feasible_atomic_skills(const env::State& state);
            // std::function<bool(const std::map<std::string, std::any>&)> 
            //     pre_condition(const std::string& skill, const Object& obj, const env::State& state);
            // std::array<env::State> 
            //     trajectory_algorithm(const std::string& skill, const Object& obj, const env::State& initial_state, const env::State& target_state);
    };

}

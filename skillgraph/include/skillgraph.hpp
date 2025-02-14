#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "environment.hpp"
#include "tasks.hpp"
#include "object.hpp"
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

    struct GroundedSkill : public Skill {
        /*
        Grounded Skill Class, containing detailed parameters for a symbolic skill
        */

        task::TaskParam task_param;
        env::Object object;
        robot::Robot robot;
        algo::Algorithm algorithm;
    };


    class SkillGraph{
        /*
        * SkillGraph Class containing that maps all the objects, robots, envs, skills, executors, and algorithms
        * Provides the API for the skill graph in the overall ontology
        */
        protected:
            // symbols defined by user
            std::map<std::string, env::Object> object_library; // maps of all objects
            std::map<std::string, Skill::Type> skill_types;             // maps from user-defined string to enum

            std::string robot_state_form; // choice of robot state representation
            std::string env_state_form; // choice of env state representation

            Json::Value root_config_; // user-provided Json config of this skill graph
            std::vector<robot::Robot> robots; // list of robots and capabilities
            std::vector<Skill::Type> atmoic_skills; // list of atomic skills
            std::map<Skill::Type, std::vector<Skill::Type>> meta_skills; // map of all meta stkills

            // task defintion for feasible_u
            TaskType task_type_; // task type
            env::State initial_state_;
            env::State target_state_;
            std::shared_ptr<task::AssemblySeq> task_seq_;

            // environemnt
            BackEnd backend_; // kinematic engine type
            std::shared_ptr<env::PlanInstance> instance; // robot environment (kinematic engine)

            virtual void init_task_seq(const Json::Value &root_config) {throw std::runtime_error("Init Task Seq Not implemented");}

        public:
            SkillGraph(const std::string &config_file);
            virtual ~SkillGraph() {};
            
            void print_skillgraph();

            void add_meta_skill(const std::string& meta_skill, const std::vector<std::string> &atomic_skill);
            void add_atomic_skill(const std::string& atomic_skill);
            void add_robot_capability(const std::string& robot_capability);

            std::vector<Skill::Type> get_meta_skills();
            std::map<Skill::Type, std::vector<Skill::Type>> get_atomic_skills();
            std::vector<Skill::Type> get_robot_capabilities();
            
            virtual std::set<GroundedSkill> feasible_u(const env::State& state) { throw std::runtime_error("Feasible u Not implemented");};

            // std::tuple<std::vector<std::string>, std::function<bool(const std::map<std::string, std::any>&)>> 
            //     feasible_target_states(const std::string& skill, const task::Object& obj, const env::State& state);
            // std::set<std::map> feasible_atomic_skills(const env::State& state);
            // std::function<bool(const std::map<std::string, std::any>&)> 
            //     pre_condition(const std::string& skill, const task::Object& obj, const env::State& state);
            // std::array<env::State> 
            //     trajectory_algorithm(const std::string& skill, const task::Object& obj, const env::State& initial_state, const env::State& target_state);
    };

    class LegoSkillGraph : public SkillGraph {
        /*
        * Temporary solution for LegoSkillGraph, a subclass of SkillGraph that is Lego specific
        */
        private:
            // lego specific

            ros::NodeHandle nh_;
            ros::ServiceClient set_state_client_;
            std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
            Json::Value task_json_;
        
            virtual void init_task_seq(const Json::Value &root_config) override;
        
        public:
            LegoSkillGraph(const std::string &config_file);
            virtual ~LegoSkillGraph() {};
    
            virtual std::set<GroundedSkill> feasible_u(const env::State &state);
    };

}

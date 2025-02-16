#pragma once
#include "skillgraph.hpp"
#include "lego/Lego.hpp"
#include "lego_tasks.hpp"

namespace skillgraph {
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
    
            virtual std::set<GroundedSkill> feasible_u(const skillgraph::State &state);
    };
}
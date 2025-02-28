#pragma once
#include "skillgraph.hpp"
#include "lego/Lego.hpp"
#include "lego_tasks.hpp"
#include "lego_objects.hpp"

namespace skillgraph {
    class LegoSkillGraph : public SkillGraph {
        /*
        * Temporary solution for LegoSkillGraph, a subclass of SkillGraph that is Lego specific
        */
        private:
            // lego specific

            std::shared_ptr<ros::NodeHandle> nh_;
            ros::ServiceClient set_state_client_;
            std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
            Json::Value task_json_;
        
            virtual void parse_env(const Json::Value &root_config) override;
            virtual void parse_tasks(const Json::Value &root_config) override;
        
            //read the location and size of lego block at start and return as a rectangular object
            LegoBrick getLegoStart(const std::string &brick_name);
            
            //read the location and size of lego block at target and return as a rectangular object
            LegoBrick getLegoTarget(int task_idx);

            // calculate the location of lego block when the robot is handing ovet the block at a certain task index
            LegoBrick getLegoHandover(int task_idx, const RobotState &start_pose);

        public:
            LegoSkillGraph(const std::string &config_file);
            virtual ~LegoSkillGraph() {};

            virtual bool at_target(const State& state) override;
            virtual std::vector<SkillPtr> feasible_u(const skillgraph::State &state) override;
            virtual bool get_next_state(const State& state, SkillPtr gs, State &next_state, double &cost) override;

    };
}
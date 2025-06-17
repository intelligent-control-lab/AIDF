#pragma once
#include "skillgraph.hpp"
#include "lego/Lego.hpp"
#include "lego_tasks.hpp"
#include "lego_objects.hpp"
#include "lego_algorithms.hpp"

namespace skillgraph {
    /**
     * @brief Lego-specific implementation of the SkillGraph.
     *
     * Temporary solution for LegoSkillGraph, a subclass of SkillGraph that is Lego specific.
     */
    class LegoSkillGraph : public SkillGraph {
        private:
            // lego specific

            /**
             * @brief ROS node handle.
             */
            std::shared_ptr<ros::NodeHandle> nh_;
            /**
             * @brief Service client for setting state.
             */
            ros::ServiceClient set_state_client_;
            /**
             * @brief Pointer to the Lego object.
             */
            std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
            /**
             * @brief Lego policy configuration.
             */
            LegoPolicyCfg lego_config_;
            /**
             * @brief JSON value for the task configuration.
             */
            Json::Value task_json_;
        
            /**
             * @brief Parse the environment configuration.
             * @param root_config Root JSON configuration.
             */
            virtual void parse_env(const Json::Value &root_config) override;
            /**
             * @brief Parse the tasks configuration.
             * @param root_config Root JSON configuration.
             */
            virtual void parse_tasks(const Json::Value &root_config) override;
        
            /**
             * @brief Read the location and size of a Lego block at start and return as a rectangular object.
             * @param brick_name Name of the brick.
             * @return LegoBrick object representing the start state.
             */
            LegoBrick getLegoStart(const std::string &brick_name);
            
            /**
             * @brief Read the location and size of a Lego block at target and return as a rectangular object.
             * @param task_idx Task index.
             * @return LegoBrick object representing the target state.
             */
            LegoBrick getLegoTarget(int task_idx);

            /**
             * @brief Calculate the location of a Lego block during handover at a certain task index.
             * @param task_idx Task index.
             * @param start_pose Robot start pose.
             * @return LegoBrick object representing the handover state.
             */
            LegoBrick getLegoHandover(int task_idx, const RobotState &start_pose);

        public:
            /**
             * @brief Constructor for LegoSkillGraph.
             * @param config_file Path to the configuration file.
             */
            LegoSkillGraph(const std::string &config_file);
            virtual ~LegoSkillGraph() {};

            /**
             * @brief Check if the state is at the target.
             * @param state The current state.
             * @return True if at target.
             */
            virtual bool at_target(const State& state) override;
            /**
             * @brief Get feasible skills for the given state.
             * @param state The current state.
             * @return Vector of feasible skills.
             */
            virtual std::vector<SkillPtr> feasible_u(const skillgraph::State &state) override;
            /**
             * @brief Get the next state given a skill.
             * @param state The current state.
             * @param gs The skill to apply.
             * @param next_state The resulting next state.
             * @param cost The cost of the transition.
             * @return True if successful.
             */
            virtual bool get_next_state(const State& state, SkillPtr gs, State &next_state, double &cost) override;
            /**
             * @brief Check if a skill is feasible in the given state.
             * @param state The current state.
             * @param skill_config JSON configuration for the skill.
             * @param gs Output parameter for the feasible skill.
             * @return True if the skill is feasible.
             */
            virtual bool is_feasible(const State&state, Json::Value &skill_config, SkillPtr &gs) override;

    };
}
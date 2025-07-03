#pragma once
#include "skillgraph.hpp"
#include "magblock_tasks.hpp"
#include "magblock_objects.hpp"
#include "magblock_algorithms.hpp"

namespace skillgraph {
    /**
     * @brief MagBlock-specific implementation of the SkillGraph.
     * 
     * Inherits from SkillGraph and provides magnetic block assembly functionality,
     * exactly mirroring the LEGO implementation approach.
     */
    class MagBlockSkillGraph : public SkillGraph {
        public:
            /**
             * @brief Constructor for MagBlockSkillGraph.
             * @param config_file Path to the configuration file.
             */
            MagBlockSkillGraph(const std::string &config_file);
            virtual ~MagBlockSkillGraph() = default;

            /**
             * @brief Parse the environment configuration.
             * @param root_config Root JSON configuration.
             */
            virtual void parse_env(const Json::Value &root_config) override;

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
            virtual std::vector<SkillPtr> feasible_u(const State &state) override;

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

        private:
            /**
             * @brief Determine which robot to use for a given location.
             * @param x World x coordinate.
             * @param y World y coordinate.
             * @param z World z coordinate.
             * @return Robot ID (0, 1, or 2).
             */
            int determineRobotForLocation(double x, double y, double z);
    };
}

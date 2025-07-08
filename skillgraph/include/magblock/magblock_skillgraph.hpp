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

            /**
             * @brief Transform block coordinates to robot coordinates.
             * @param robot_name Name of the robot.
             * @param x_blocks X coordinate in block frame.
             * @param y_blocks Y coordinate in block frame.
             * @param z_blocks Z coordinate in block frame.
             * @param x_robot Output X coordinate in robot frame.
             * @param y_robot Output Y coordinate in robot frame.
             * @param z_robot Output Z coordinate in robot frame.
             * @param rx_deg Output roll in degrees.
             * @param ry_deg Output pitch in degrees.
             * @param rz_deg Output yaw in degrees.
             * @return True if transformation successful.
             */
            bool blockToRobotFrame(const std::string& robot_name, 
                                 double x_blocks, double y_blocks, double z_blocks,
                                 double& x_robot, double& y_robot, double& z_robot,
                                 double& rx_deg, double& ry_deg, double& rz_deg);
            
            /**
             * @brief Load assembly task from JSON file.
             * @param task_file Path to assembly task JSON.
             * @param env_setup_file Path to environment setup JSON.
             * @return True if successful.
             */
            bool loadAssemblyTask(const std::string& task_file, const std::string& env_setup_file);
            
            /**
             * @brief Get the assembly sequence for trajectory planning.
             * @return Assembly sequence pointer.
             */
            std::shared_ptr<MagBlockAssemblySeq> getAssemblySequence() const;
            
            /**
             * @brief Set the assembly sequence.
             * @param assembly_seq Assembly sequence to set.
             */
            void setAssemblySequence(std::shared_ptr<MagBlockAssemblySeq> assembly_seq);
            
            /**
             * @brief Set environment configuration.
             * @param env_config Environment configuration JSON.
             */
            void setEnvironmentConfig(const Json::Value& env_config);
            
            /**
             * @brief Get optimal robot for given block coordinates.
             * @param x_blocks X coordinate in block frame.
             * @param y_blocks Y coordinate in block frame.
             * @param z_blocks Z coordinate in block frame.
             * @return Robot ID (0=left, 1=center, 2=right).
             */
            int getOptimalRobot(double x_blocks, double y_blocks, double z_blocks);
            
            /**
             * @brief Transform block coordinates to robot frame - renamed for clarity.
             * @param robot_name Name of the robot.
             * @param x_blocks X coordinate in block frame.
             * @param y_blocks Y coordinate in block frame.
             * @param z_blocks Z coordinate in block frame.
             * @param x_robot Output X coordinate in robot frame.
             * @param y_robot Output Y coordinate in robot frame.
             * @param z_robot Output Z coordinate in robot frame.
             * @param rx_deg Output roll in degrees.
             * @param ry_deg Output pitch in degrees.
             * @param rz_deg Output yaw in degrees.
             * @return True if transformation successful.
             */
            bool transformBlockToRobot(const std::string& robot_name, 
                                     double x_blocks, double y_blocks, double z_blocks,
                                     double& x_robot, double& y_robot, double& z_robot,
                                     double& rx_deg, double& ry_deg, double& rz_deg);
            
            /**
             * @brief Get pick pose for a task from skillgraph logic.
             * @param task Task to get pick pose for.
             * @param x Output x coordinate.
             * @param y Output y coordinate.
             * @param z Output z coordinate.
             * @return True if successful.
             */
            bool getPickPose(TaskPtr task, double& x, double& y, double& z);

        private:
            /**
             * @brief Determine which robot to use for a given location.
             * @param x World x coordinate.
             * @param y World y coordinate.
             * @param z World z coordinate.
             * @return Robot ID (0, 1, or 2).
             */
            int determineRobotForLocation(double x, double y, double z);

            // Robot transformation configurations
            struct RobotConfig {
                double x_origin_blocks;
                double y_origin_blocks;
                std::vector<double> default_orientation_deg;
                Eigen::Matrix2d block_to_robot_matrix;
            };
            std::map<std::string, RobotConfig> robot_configs_;
            double block_size_ = 0.025; // 2.5cm block size

            // Assembly task data
            Json::Value assembly_tasks_;
            Json::Value env_setup_;
            std::shared_ptr<MagBlockAssemblySeq> assembly_seq_;
            std::shared_ptr<MagBlockAssemblySeq> task_seq_;  // For compatibility
    };
}

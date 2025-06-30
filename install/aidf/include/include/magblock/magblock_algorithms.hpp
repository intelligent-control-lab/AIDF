#pragma once
#include "algorithms.hpp"
#include "magblock_objects.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {
    /**
     * @brief Configuration for the magnetic block assembly policy.
     */
    struct MagBlockPolicyCfg {
        // Approach distance for picking and placing
        double approach_distance = 0.1;
        // Grip width for the gripper
        double grip_width = 0.08;
        // Speed for robot motions
        double execution_speed = 0.5;
    };

    /**
     * @brief Generator for magnetic block grasp poses.
     */
    class MagBlockGraspGenerator {
    public:
        MagBlockGraspGenerator(std::shared_ptr<PlanInstance> backend,
                             const MagBlockPolicyCfg& config,
                             RobotPtr robot,
                             ObjPtr object)
            : backend_(backend), config_(config), robot_(robot), object_(object) {}

        /**
         * @brief Generate grasp poses for a given skill.
         * @param constraints JSON constraints for the skill.
         * @param type Type of the skill.
         * @param seq_id Sequence ID of the skill.
         * @param state Target state to update.
         * @return True if generation was successful.
         */
        bool generate(const Json::Value& constraints, Skill::Type type, int seq_id, State& state);

    protected:
        std::shared_ptr<PlanInstance> backend_;
        MagBlockPolicyCfg config_;
        RobotPtr robot_;
        ObjPtr object_;
    };

    /**
     * @brief Planning algorithms for magnetic block assembly.
     */
    class MagBlockPlan : public PlanningAlgorithm {
    public:
        MagBlockPlan(std::shared_ptr<PlanInstance> backend,
                    const MagBlockPolicyCfg& config,
                    RobotPtr robot,
                    ObjPtr object)
            : backend_(backend), config_(config), robot_(robot), object_(object) {}

        /**
         * @brief Plan a skill execution.
         * @param current_state Current state.
         * @param task_param Task parameters.
         * @param type Skill type.
         * @param traj Output trajectory.
         * @return True if planning was successful.
         */
        bool plan_skill(const State& current_state,
                       const TaskParam& task_param,
                       Skill::Type type,
                       RobotTrajectory& traj);

    protected:
        /**
         * @brief Plan a pick operation.
         */
        bool plan_pick(const State& current_state,
                      const TaskParam& task_param,
                      RobotTrajectory& traj);

        /**
         * @brief Plan a place operation.
         */
        bool plan_place(const State& current_state,
                       const TaskParam& task_param,
                       RobotTrajectory& traj);

    protected:
        std::shared_ptr<PlanInstance> backend_;
        MagBlockPolicyCfg config_;
        RobotPtr robot_;
        ObjPtr object_;
    };
}

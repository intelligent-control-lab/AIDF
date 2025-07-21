#pragma once

#include "skills.hpp"
#include "moveit_backend.hpp"
#include <jsoncpp/json/json.h>
#include <vector>
#include <memory>

namespace skillgraph {

/**
 * @brief MagBlock-specific skill executor.
 * 
 * Inherits from SkillExecutor and provides execution logic for MagBlock tasks,
 * mirroring the LEGO implementation approach with MoveIt2 integration.
 */
class MagBlockSkillExecutor : public SkillExecutor {
public:
    /**
     * @brief Constructor for MagBlockSkillExecutor.
     * @param type The skill type.
     * @param backend Shared pointer to the PlanInstance backend.
     */
    MagBlockSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend);

    /**
     * @brief Execute the skill on the current state.
     * @param current_state The current state to execute on.
     * @return True if execution was successful.
     */
    virtual bool execute(State &current_state) override;

private:
    /**
     * @brief Backend plan instance.
     */
    std::shared_ptr<PlanInstance> backend_;
    
    /**
     * @brief Moveit controller for robot motion.
     */
    std::shared_ptr<MoveitControl> controller_;
    
    /**
     * @brief Skill type
     */
    Skill::Type skill_type_;

    /**
     * @brief Get robot name from constraints or use default.
     * @return Robot name to use for execution.
     */
    std::string getRobotName() const;

    /**
     * @brief Execute pick skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @param constraints JSON constraints for the skill.
     * @return True if successful.
     */
    bool execute_pick_skill(State &current_state);
    
    /**
     * @brief Execute place skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @return True if successful.
     */
    bool execute_place_skill(State &current_state);
    
    /**
     * @brief Execute transit skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @return True if successful.
     */
    bool execute_transit_skill(State &current_state);
    
    /**
     * @brief Execute pick and place skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @return True if successful.
     */
    bool execute_pick_and_place_skill(State &current_state);

    // Helper to get MoveitInstance from backend
    std::shared_ptr<MoveitInstance> getMoveitInstance() {
        return std::dynamic_pointer_cast<MoveitInstance>(backend_);
    }
};

/**
 * @brief Shared pointer type for MagBlockSkillExecutor.
 */
typedef std::shared_ptr<MagBlockSkillExecutor> MagBlockSkillExecutorPtr;

} // namespace skillgraph

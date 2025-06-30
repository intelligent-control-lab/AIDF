#pragma once

#include "../api/skills.hpp"
#include "../api/backend.hpp"

namespace skillgraph {

/**
 * @brief MagBlock-specific skill executor.
 * 
 * Inherits from SkillExecutor and provides execution logic for MagBlock tasks.
 */
class MagBlockSkillExecutor : public SkillExecutor {
public:
    /**
     * @brief Constructor for MagBlockSkillExecutor.
     * @param type The skill type.
     */
    MagBlockSkillExecutor(Skill::Type type);

    /**
     * @brief Execute the skill on the current state.
     * @param current_state The current state to execute on.
     * @return True if execution was successful.
     */
    virtual bool execute(State &current_state) override;

private:
    /**
     * @brief Skill type
     */
    Skill::Type skill_type_;
};

} // namespace skillgraph

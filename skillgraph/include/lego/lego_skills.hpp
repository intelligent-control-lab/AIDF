#pragma once

#include "skills.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {
class LegoSkillExecutor : public SkillExecutor {
public:
    LegoSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend);

    virtual bool execute(State &current_state) override;

private:
    std::shared_ptr<PlanInstance> backend_;
    std::shared_ptr<MoveitControl> controller_;
};


typedef std::shared_ptr<LegoSkillExecutor> LegoSkillExecutorPtr;

}
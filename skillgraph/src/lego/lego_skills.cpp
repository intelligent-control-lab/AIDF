#include "lego/lego_skills.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {

LegoSkillExecutor::LegoSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend) 
    : SkillExecutor(type), backend_(backend) {
    
    auto moveit_backend = std::dynamic_pointer_cast<MoveitInstance>(backend_);
    controller_ = std::make_shared<MoveitControl>(moveit_backend, true);

}

bool LegoSkillExecutor::execute(State &current_state) {
    // To be implemented
    if (post_condition == nullptr) {
        log("Post condition is null", LogLevel::ERROR);
        return false;
    }

    bool success = controller_->move(post_condition);
    current_state.robot_states = post_condition->target_state.robot_states;
    current_state.env_state = post_condition->target_state.env_state;
    // sleep for 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return success;
}

}
#include "metrics.hpp"


namespace skillgraph {

    Metric::Metric(const std::string &name) : name(name) {}

    ConditionEvaluator::ConditionEvaluator(const std::string &name) : Metric(name) {}

    bool ConditionEvaluator::eval_condition() {
        if (!state_) {
            ROS_WARN("PDDL state not set.");
            return false;
        }

        if (pre_or_post == 0) {
            // Pre-condition: both robots must have empty hands
            return !state_->inhand_rob1 && !state_->inhand_rob2;
        } else if (pre_or_post == 1) {
            // Post-condition: at least one robot must have Lego in hand
            return (state_->inhand_rob1 || state_->inhand_rob2) && state_->assembly;
        }

        return true;  // 默认满足
    }

    ConstraintEvaluator::ConstraintEvaluator(const std::string &name) : Metric(name) {}

    bool ConstraintEvaluator::eval_condition() {
        // Implement the logic to evaluate the constraint
        return true; // Placeholder implementation
    }

} // namespace skillgraph
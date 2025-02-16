#pragma once
#include "Utils/Common.hpp"


namespace skillgraph {
    class Metric {
    public:
        Metric(const std::string &name);

    protected:
        std::string name;
    };

    struct ConditionEvaluator: public Metric {
        ConditionEvaluator(const std::string &name);

        bool eval_condition();
    };

    struct ConstraintEvaluator: public Metric {
        ConstraintEvaluator(const std::string &name);

        bool eval_condition();
    };
}
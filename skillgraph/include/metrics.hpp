#pragma once
#include "Utils/Common.hpp"


namespace metric {
    class Metric {
    public:
        Metric(const std::string &name);

    protected:
        std::string name;
    };

    struct Evaluator: public Metric {
        Evaluator(const std::string &name);

        bool eval_constraint();
        bool eval_condition();
    };
}
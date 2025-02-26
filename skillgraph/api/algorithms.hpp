#pragma once
#include "backend.hpp"
#include "metrics.hpp"

namespace skillgraph {
    class Algorithm {
        /*
        * Base Algorithm Class containing the type and name of the algorithm
        */
        enum Type {
            Planning = 0,
            Control = 1,
            Perception = 2,
        };

        public:
            Algorithm() = default;
            Type type;
            std::string name;
    };
    typedef std::shared_ptr<Algorithm> AlgorithmPtr;

    class SkillPerformingAlgorithm : public Algorithm {
        /*
        * SkillPerformingAlgorithm Class containing the implementation, properties, and a function API for performing a skill
        */
        public:
            SkillPerformingAlgorithm() = default;
            SkillPerformingAlgorithm(const std::string &name);
            
            // chooses an implementation
            Algorithm implementation;

            // properties
            skillgraph::ConditionEvaluator pre_condition;
            skillgraph::ConditionEvaluator post_condition;
            std::string skill_type; // corresponding skill

            // funnction
            std::function<std::any(const std::vector<std::any>&)> perform();
    };

    class PlanningAlgorithm : public Algorithm {
        public:
            PlanningAlgorithm() = default;
    };

    class ControlAlgorithm : public Algorithm {
        public:
            ControlAlgorithm() = default;
    };

    class PerceptionAlgorithm: public Algorithm {
        public: 
            PerceptionAlgorithm() = default;
    };

    

}
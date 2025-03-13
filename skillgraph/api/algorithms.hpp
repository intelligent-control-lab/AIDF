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

    class PlanningAlgorithm : public Algorithm {
        public:
            PlanningAlgorithm() = default;
    };
    typedef std::shared_ptr<PlanningAlgorithm> PlanningAlgorithmPtr;

    class ControlAlgorithm : public Algorithm {
        public:
            ControlAlgorithm() = default;
    };
    typedef std::shared_ptr<ControlAlgorithm> ControlAlgorithmPtr;

    class PerceptionAlgorithm: public Algorithm {
        public: 
            PerceptionAlgorithm() = default;
    };
    typedef std::shared_ptr<PerceptionAlgorithm> PerceptionAlgorithmPtr;

    

}
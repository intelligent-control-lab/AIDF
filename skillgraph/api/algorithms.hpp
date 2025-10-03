/**
 * @file algorithms.hpp
 * @brief Contains base classes for algorithms used in the skillgraph framework.
 */
#pragma once
#include "backend.hpp"
#include "metrics.hpp"

namespace skillgraph {
    /**
     * @class Algorithm
     * @brief Base Algorithm Class containing the type and name of the algorithm.
     */
    class Algorithm {
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

    /**
     * @class PlanningAlgorithm
     * @brief Derived class for planning algorithms.
     */
    class PlanningAlgorithm : public Algorithm {
        public:
            PlanningAlgorithm() = default;
    };
    typedef std::shared_ptr<PlanningAlgorithm> PlanningAlgorithmPtr;

    /**
     * @class ControlAlgorithm
     * @brief Derived class for control algorithms.
     */
    class ControlAlgorithm : public Algorithm {
        public:
            ControlAlgorithm() = default;
    };
    typedef std::shared_ptr<ControlAlgorithm> ControlAlgorithmPtr;

}

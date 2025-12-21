/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu 
    Peiqi Yu peiqiy@andrew.cmu.edu 
    Chaitanya Chawla cchawla@cs.cmu.edu 
    Changliu Liu cliu6@andrew.cmu.edu 
    Jiaoyang Li jiaoyanl@andrew.cmu.edu 
    Guanya Shi guanyas@andrew.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

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

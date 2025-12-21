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

#pragma once
#include "tasks.hpp"
#include "lego/Lego.hpp"

namespace skillgraph {

    /**
     * @brief Subclass for Lego Assembly Tasks specifically.
     *
     * Inherits from AssemblySeq and provides additional functionality for Lego assembly tasks.
     */
    class LegoAssemblySeq : public AssemblySeq {
        public:
            /**
             * @brief Constructor for LegoAssemblySeq.
             * @param lego_ptr Pointer to the Lego object.
             * @param task_json JSON string describing the task.
             */
            LegoAssemblySeq(lego_manipulation::lego::Lego::Ptr lego_ptr,
                            const std::string &task_json);
            virtual  ~LegoAssemblySeq() {};

            /**
             * @brief Print the assembly sequence.
             */
            virtual void print() override;
        
            /**
             * @brief Remove the brick sequence from the assembly task.
             */
            void remove_brick_seq();
    
        private:
            /**
             * @brief Pointer to the Lego object.
             */
            lego_manipulation::lego::Lego::Ptr lego_ptr_;
            /**
             * @brief JSON value describing the task.
             */
            Json::Value task_json_;
        };
}
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
 * @file tasks.cpp
 * @brief Implements task-related classes and methods for the skillgraph framework.
 */
#include "tasks.hpp"
#include "Utils/Logger.hpp"


namespace skillgraph {

/**
 * @brief Print the assembly sequence to stdout.
 */
void AssemblySeq::print() {
    std::cout << "Assembly Sequence: \n";
    for (int i = 0; i < num_tasks_; i++) {
        std::cout << task_seq_[i]->name << " " << task_seq_[i]->description << std::endl;
    }
    std::cout << std::endl;
}

/**
 * @brief Get the task at a specific index.
 * @param i Index of the task.
 * @return Task pointer at index i.
 */
TaskPtr AssemblySeq::get_task_at(int i) 
{
    if (i >= task_seq_.size()) {
        std::cerr << "Object index out of range" << std::endl;
    }

    return task_seq_[i];
}

    

}
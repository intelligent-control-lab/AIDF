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
    if (i < 0 || i >= task_seq_.size()) {
        std::cerr << "Object index out of range: " << i << " (size: " << task_seq_.size() << ")" << std::endl;
        return nullptr;
    }

    return task_seq_[i];
}

    

}
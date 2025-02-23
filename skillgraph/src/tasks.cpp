#include "tasks.hpp"
#include "Utils/Logger.hpp"


namespace skillgraph {

void AssemblySeq::print() {
    std::cout << "Assembly Sequence: \n";
    for (int i = 0; i < num_tasks_; i++) {
        std::cout << task_seq_[i]->name << " " << task_seq_[i]->description << std::endl;
    }
    std::cout << std::endl;
}

TaskPtr AssemblySeq::get_task_at(int i) 
{
    if (i >= task_seq_.size()) {
        std::cerr << "Object index out of range" << std::endl;
    }

    return task_seq_[i];
}

    

}
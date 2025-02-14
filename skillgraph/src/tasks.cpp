#include "tasks.hpp"
#include "Utils/Logger.hpp"

using namespace env;

namespace task {

void AssemblySeq::print() {
    std::cout << "Assembly Sequence: \n";
    for (int i = 0; i < num_tasks_; i++) {
        std::cout << task_seq_[i].name << " ";
    }
    std::cout << std::endl;
}

Task AssemblySeq::get_task_at(int i) 
{
    if (i >= task_seq_.size()) {
        std::cerr << "Object index out of range" << std::endl;
    }

    return task_seq_[i];
}

LegoAssemblySeq::LegoAssemblySeq(lego_manipulation::lego::Lego::Ptr lego_ptr,
                                const std::string &task_json_fname) {
    lego_ptr_ = lego_ptr;

    std::ifstream task_file(task_json_fname, std::ifstream::binary);
    task_file >> task_json_;
    num_tasks_ = task_json_.size();
    remove_brick_seq();


    for (int i = 0; i < num_tasks_; i++) {
        Json::Value node = task_json_[std::to_string(i+1)];
        std::string seq = "t" + std::to_string(i+1);
        LegoBrick obj_i(lego_ptr, node, seq);
    }
}


void LegoAssemblySeq::remove_brick_seq() {
    for (int i = 0; i < num_tasks_; i++) {
        int task_idx = i + 1;
        // calculate the ik for the target pose
        auto & cur_graph_node = task_json_[std::to_string(task_idx)];
        if (cur_graph_node.isMember("brick_seq")) {
            cur_graph_node.removeMember("brick_seq");
        }
    }
}

    

}
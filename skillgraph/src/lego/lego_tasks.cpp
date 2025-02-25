#include "lego/lego_tasks.hpp"

namespace skillgraph {

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
        int brick_id = node["brick_id"].asInt();
        int brick_x = node["x"].asInt();
        int brick_y = node["y"].asInt();
        int brick_z = node["z"].asInt() - 1;
        int ori = node["ori"].asInt();
        int manip_type = node["manip_type"].asInt();
        int support_x = node["support_x"].asInt();
        int support_y = node["support_y"].asInt();
        int support_z = node["support_z"].asInt() - 1;
        int support_ori = node["support_ori"].asInt();

        // Initialize a task
        TaskPtr task = std::make_shared<Task>();
        task->name = seq;
        task->description = "Pick and place brick " + std::to_string(brick_id) + " to " + std::to_string(brick_x) + " " + std::to_string(brick_y) + " " + std::to_string(brick_z);

        // set the goal condition for this task
        task->post_condition = std::make_shared<TaskParam>();

        task->post_condition->constraints_json = node;
        if (manip_type == 1) {
            task->post_condition->allowed_skill_type.push_back(Skill::Type::PickHandoverAndPlace);
        } else if (manip_type == 0) {
            if (support_x == -1) {
                task->post_condition->allowed_skill_type.push_back(Skill::Type::PickAndPlace);
            } else {
                task->post_condition->allowed_skill_type.push_back(Skill::Type::PickAndPlaceWithSupport);
            }
        }
        task_seq_.push_back(task);
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

void LegoAssemblySeq::print() {
    std::cout << "Assembly Sequence: \n";
    for (int i = 0; i < num_tasks_; i++) {
        std::cout << task_seq_[i]->name << " " << task_seq_[i]->description << std::endl;
    }
    std::cout << std::endl;
}

} // namespace skillgraph
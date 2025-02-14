#pragma once
#include "robots.hpp"
#include "object.hpp"
#include "algorithms.hpp"
#include "Utils/FileIO.hpp"
#include "environment.hpp"
#include "metrics.hpp"

namespace task
{ 
    struct TaskParam {
        /*
        * Task Parameters (for pre_conditions/post_condition of tasks, and inputs/outputs of algorithms)
        */
        robot::RobotState target_state; // target state for the robot
        env::EnvState env_state;
        algo::Algorithm generator;
        metric::Evaluator condition_check;
    };

    struct Task {
        /*
        * Task Class containing the name of the task as well as required parameters
        */
        std::string name;
        TaskParam pre_condition;
        TaskParam post_condition;
    };

    class AssemblySeq {
    /*
    * Class representing the sequence of tasks in an assembly, provided by user for now
    */
    public:
        AssemblySeq() = default;
        virtual ~AssemblySeq() {};
        virtual std::vector<Task> get_sequence() {return task_seq_;}

        virtual Task get_task_at(int i);

        virtual void print();
        int num_tasks() {return num_tasks_;}
    
    protected:
        int num_tasks_;
        std::vector<Task> task_seq_;
    };


    class LegoAssemblySeq : public AssemblySeq {
    /*
    * Subclass for Lego Assmelby Tasks specifically
    */
    public:
        LegoAssemblySeq(lego_manipulation::lego::Lego::Ptr lego_ptr,
                        const std::string &task_json);
        virtual  ~LegoAssemblySeq() {};
    
        void remove_brick_seq();

    private:
        lego_manipulation::lego::Lego::Ptr lego_ptr_;
        Json::Value task_json_;
    };

}
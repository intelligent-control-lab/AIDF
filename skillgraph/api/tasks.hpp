#pragma once
#include "robots.hpp"
#include "objects.hpp"
#include "algorithms.hpp"
#include "Utils/FileIO.hpp"
#include "environment.hpp"
#include "metrics.hpp"

namespace skillgraph
{ 
    struct Constraint {
        /*
        * Constraint Class containing the constraint type and parameters
        */
        std::string type;
        std::string param;
    };

    struct TaskParam {
        /*
        * Task Parameters (for pre_conditions/post_condition of tasks, and inputs/outputs of algorithms)
        */
        skillgraph::RobotState target_state; // target state for the robot
        skillgraph::EnvState env_state;
        skillgraph::Algorithm generator;
        skillgraph::ConstraintEvaluator condition_check;
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


}
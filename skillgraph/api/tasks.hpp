#pragma once
#include "robots.hpp"
#include "objects.hpp"
#include "algorithms.hpp"
#include "Utils/FileIO.hpp"
#include "environment.hpp"
#include "metrics.hpp"

namespace skillgraph
{ 
    class Constraint {
        /*
        * Constraint Class containing the constraint type and parameters
        */
    public:
        Constraint() = default;

        std::string type;
        std::string param;
    };

    class TaskParam {
        /*
        * Task Parameters (for pre_conditions/post_condition of tasks, and inputs/outputs of algorithms)
        */
    public:
        TaskParam() = default;

        std::vector<Constraint> constraints;
        RobotState target_state; // target state for the robot
        EnvState env_state;
        std::shared_ptr<Algorithm> generator;
        std::shared_ptr<ConstraintEvaluator> condition_check;
    };

    class Task {
        /*
        * Task Class containing the name of the task as well as required parameters
        */
    public:
        Task() = default;

        std::string name;
        std::string description;
        TaskParam pre_condition;
        TaskParam post_condition;
    };
    typedef std::shared_ptr<Task> TaskPtr;

    class AssemblySeq {
    /*
    * Class representing the sequence of tasks in an assembly, provided by user for now
    */
    public:
        AssemblySeq() = default;
        virtual ~AssemblySeq() {};
        virtual std::vector<TaskPtr> get_sequence() {return task_seq_;}

        virtual TaskPtr get_task_at(int i);

        virtual void print();
        int num_tasks() {return num_tasks_;}
    
    protected:
        int num_tasks_;
        std::vector<TaskPtr> task_seq_;
    };


}
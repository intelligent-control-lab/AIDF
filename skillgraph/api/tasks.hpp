#pragma once
#include "robots.hpp"
#include "objects.hpp"
#include "algorithms.hpp"
#include "Utils/FileIO.hpp"
#include "environment.hpp"
#include "metrics.hpp"
#include "skills.hpp"

namespace skillgraph
{
    class TaskParam {
        /*
        * Task Parameters (for pre_conditions/post_condition of tasks, and inputs/outputs of algorithms)
        */
    public:
        TaskParam() = default;

        Json::Value constraints_json;
        std::vector<Skill::Type> allowed_skill_type;

        RobotState target_state; // target state for the robot
        EnvState env_state;
        std::shared_ptr<Algorithm> generator;
        std::shared_ptr<ConstraintEvaluator> condition_check;
    };
    typedef std::shared_ptr<TaskParam> TaskParamPtr;

    class Task {
        /*
        * Task Class containing the name of the task as well as required parameters
        */
    public:
        Task() = default;

        std::string name;
        std::string description;
        TaskParamPtr pre_condition;
        TaskParamPtr post_condition;
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
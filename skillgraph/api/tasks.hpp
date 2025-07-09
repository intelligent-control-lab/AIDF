/**
 * @file tasks.hpp
 * @brief Defines task-related classes for the skillgraph framework.
 */
#pragma once
#include "robots.hpp"
#include "objects.hpp"
#include "algorithms.hpp"
#include "Utils/FileIO.hpp"
#include "environment.hpp"
#include "metrics.hpp"
#include "skills.hpp"


//pddl_state written by yijie
#include "symbolic_state.hpp"

namespace skillgraph
{
    /**
     * @class TaskParam
     * @brief Parameters for pre-conditions, post-conditions, and algorithm I/O.
     */
    class TaskParam {
        /*
        * Task Parameters (for pre_conditions/post_condition of tasks, and inputs/outputs of algorithms)
        */
    public:
        TaskParam() = default;

        Json::Value constraints_json; /**< JSON constraints for the task */
        std::vector<Skill::Type> allowed_skill_type; /**< Allowed skill types */

        State target_state; /**< Target state for the task */
        std::shared_ptr<Algorithm> generator; /**< Algorithm generator */
        std::shared_ptr<ConditionEvaluator> condition_check; /**< Condition evaluator */
        std::shared_ptr<ConstraintEvaluator> constraint_check; /**< Constraint evaluator */


        
    
    };

    /**
     * @class Task
     * @brief Represents a task with name, description, and conditions.
     */
    class Task {
        /*
        * Task Class containing the name of the task as well as required parameters
        */
    public:
        Task() = default;

        std::string name; /**< Name of the task */
        std::string description; /**< Description of the task */
        TaskParamPtr pre_condition; /**< Pre-condition parameters */
        TaskParamPtr post_condition; /**< Post-condition parameters */
    };
    typedef std::shared_ptr<Task> TaskPtr;

    /**
     * @class AssemblySeq
     * @brief Represents a sequence of tasks in an assembly.
     */
    class AssemblySeq {
    /*
    * Class representing the sequence of tasks in an assembly, provided by user for now
    */
    public:
        AssemblySeq() = default;
        virtual ~AssemblySeq() {};
        /**
         * @brief Get the sequence of tasks.
         * @return Vector of task pointers.
         */
        virtual std::vector<TaskPtr> get_sequence() {return task_seq_;}

        /**
         * @brief Get the task at a specific index.
         * @param i Index of the task.
         * @return Task pointer at index i.
         */
        virtual TaskPtr get_task_at(int i);

        /**
         * @brief Print the assembly sequence to stdout.
         */
        virtual void print();
        /**
         * @brief Get the number of tasks in the sequence.
         * @return Number of tasks.
         */
        int num_tasks() {return num_tasks_;}
    
    protected:
        int num_tasks_; /**< Number of tasks in the sequence */
        std::vector<TaskPtr> task_seq_; /**< Sequence of tasks */
    };


}
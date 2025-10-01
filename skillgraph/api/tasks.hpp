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
        * Task Parameters (for the configuration of tasks, such as grap pose param, allowed skills, approach pose, etc.)
        */
    public:
        // Constructors
        TaskParam() = default;
        TaskParam(const Json::Value &config) : config(config) {}
        TaskParam(const Json::Value &config, const std::vector<Skill::Type> &allowed_skill_type);

        Json::Value get(const std::string &key) const;

        bool has(const std::string &key) const;

        void set(const std::string &key, const Json::Value &value);

        void set(const Json::Value &keyValues);

        void add_allowed_skill_type(Skill::Type skill_type);

        void get_allowed_skill_type(std::vector<Skill::Type> &type);

        std::string to_string() const;

    private:
        Json::Value config; /**< JSON constraints for the task */
        std::vector<Skill::Type> allowed_skill_type; /**< Allowed skill types */
    };
    typedef std::shared_ptr<TaskParam> TaskParamPtr;

    /**
     * @class Task
     * @brief Represents a task with name, description, and conditions.
     */
    class Task {
        /*
        * Task Class containing the name of the task as well as required parameters
        */
    public:
        // Constructors
        Task() = default;
        Task(std::string name) : name(name) {}
        Task(std::string name, TaskParamPtr param) : name(name), param(param) {}
        Task(std::string name, Json::Value &config) : name(name) {
            param = std::make_shared<TaskParam>(config);
        }

        std::string name; /**< Name of the task */
        std::string description; /**< Description of the task */
        TaskParamPtr param; /**< Pre-condition for the task */
        State initial_state; /**< Initial state of the task, to be automatically generated or provided by user */
        State goal_state; /** Goal state of the task , to be automatically generated or provided by user */
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
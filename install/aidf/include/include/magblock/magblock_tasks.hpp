#pragma once
#include "tasks.hpp"

namespace skillgraph {
    /**
     * @brief Represents a sequence of magnetic block assembly tasks.
     */
    class MagBlockAssemblySeq : public AssemblySeq {
    public:
        MagBlockAssemblySeq() = default;
        virtual ~MagBlockAssemblySeq() = default;
        
        /**
         * @brief Parse the assembly sequence from a JSON file.
         * @param json_fname Path to the JSON file.
         * @return True if parsing was successful.
         */
        bool parse_from_json(const std::string &json_fname);

        /**
         * @brief Get the task JSON value.
         * @return The task JSON value.
         */
        Json::Value& get_task_json() { return task_json_; }

        /**
         * @brief Get the number of tasks.
         * @return Number of tasks in the sequence.
         */
        int num_tasks() const { return num_tasks_; }

    protected:
        int num_tasks_ = 0;
        Json::Value task_json_;
    };
}

#pragma once
#include "tasks.hpp"

namespace skillgraph {

    /**
     * @brief Subclass for Magnetic Block Assembly Tasks specifically.
     *
     * Inherits from AssemblySeq and provides additional functionality for magblock assembly tasks,
     * exactly mirroring the LEGO implementation approach.
     */
    class MagBlockAssemblySeq : public AssemblySeq {
        public:
            /**
             * @brief Default constructor for MagBlockAssemblySeq.
             */
            MagBlockAssemblySeq() = default;

            /**
             * @brief Constructor for MagBlockAssemblySeq.
             * @param task_json_fname Filename of the task JSON.
             */
            MagBlockAssemblySeq(const std::string &task_json_fname);
            virtual ~MagBlockAssemblySeq() {};

            /**
             * @brief Print the assembly sequence.
             */
            virtual void print() override;
        
        private:
            /**
             * @brief JSON value describing the task.
             */
            Json::Value task_json_;
        };
}

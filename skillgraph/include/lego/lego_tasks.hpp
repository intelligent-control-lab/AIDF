#pragma once
#include "tasks.hpp"
#include "lego/Lego.hpp"

namespace skillgraph {

    /**
     * @brief Subclass for Lego Assembly Tasks specifically.
     *
     * Inherits from AssemblySeq and provides additional functionality for Lego assembly tasks.
     */
    class LegoAssemblySeq : public AssemblySeq {
        public:
            /**
             * @brief Constructor for LegoAssemblySeq.
             * @param lego_ptr Pointer to the Lego object.
             * @param task_json JSON string describing the task.
             */
            LegoAssemblySeq(lego_manipulation::lego::Lego::Ptr lego_ptr,
                            const std::string &task_json);
            virtual  ~LegoAssemblySeq() {};

            /**
             * @brief Print the assembly sequence.
             */
            virtual void print() override;
        
            /**
             * @brief Remove the brick sequence from the assembly task.
             */
            void remove_brick_seq();
    
        private:
            /**
             * @brief Pointer to the Lego object.
             */
            lego_manipulation::lego::Lego::Ptr lego_ptr_;
            /**
             * @brief JSON value describing the task.
             */
            Json::Value task_json_;
        };
}
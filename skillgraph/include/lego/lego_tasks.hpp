#pragma once
#include "tasks.hpp"
#include "lego/Lego.hpp"

namespace skillgraph {

    class LegoAssemblySeq : public AssemblySeq {
        /*
        * Subclass for Lego Assmelby Tasks specifically
        */
        public:
            LegoAssemblySeq(lego_manipulation::lego::Lego::Ptr lego_ptr,
                            const std::string &task_json);
            virtual  ~LegoAssemblySeq() {};

            virtual void print() override;
        
            void remove_brick_seq();
    
        private:
            lego_manipulation::lego::Lego::Ptr lego_ptr_;
            Json::Value task_json_;
        };
}
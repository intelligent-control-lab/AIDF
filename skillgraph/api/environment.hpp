#pragma once
#include "Utils/Common.hpp"
#include "backend.hpp"
#include "data_structure.hpp"


namespace skillgraph {
    class Environmnet {
    /*
    * Environment class containing the environment specification and backend
    */
    public:
        Environmnet() = default;

        Json::Value spec_;
        std::shared_ptr<PlanInstance> backend_;
    };

     
   

}

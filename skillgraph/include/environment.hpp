#pragma once
#include "Utils/Common.hpp"
#include "backend.hpp"


namespace env {
    class Environmnet {
    public:
        Environmnet() = default;

        Json::Value spec_;
        std::shared_ptr<PlanInstance> backend_;
    };

     
    struct EnvState {
        std::vector<env::Object> objects;
    };

    struct State {
        /*
        * State Class containing the robot state and environment state
        */
        robot::RobotState robot_state;
        EnvState env_state;
    };

}

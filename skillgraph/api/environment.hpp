#pragma once
#include "Utils/Common.hpp"
#include "backend.hpp"
#include "data_structure.hpp"


namespace skillgraph {
    class Environment {
    /*
    * Environment class containing the environment specification and backend
    */
    public:
        enum Type {
            Lego = 1,
            NIST = 2,
        };

        Environment() = default;
        Environment(const std::string &name, const std::string &type, const Json::Value &spec);

        void setBackend(std::shared_ptr<PlanInstance> backend);
        std::shared_ptr<PlanInstance> getBackend() const { return backend_; }

        void setState(const EnvState &state);

        std::string name;
        Type type;

        Json::Value spec_; // environment specification
        EnvState env_state;
        std::shared_ptr<PlanInstance> backend_;
    };

     
   

}

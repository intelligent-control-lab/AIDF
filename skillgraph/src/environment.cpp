#include "environment.hpp"

namespace skillgraph
{

Environment::Environment(const std::string &name, const std::string &type, const Json::Value &spec)
    : name(name), spec_(spec)
{
    if (type == "Lego")
    {
        this->type = Type::Lego;
    }
    else if (type == "NIST")
    {
        this->type = Type::NIST;
    }
    else
    {
        throw std::runtime_error("Unknown environment type: " + type);
    }
}

void Environment::setBackend(std::shared_ptr<PlanInstance> backend)
{
    backend_ = backend;
}

void Environment::setInitialState(const EnvState &state)
{
    env_state = state;
}

}
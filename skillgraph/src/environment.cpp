/**
 * @file environment.cpp
 * @brief Implements the Environment class for the skillgraph framework.
 */
#include "environment.hpp"

namespace skillgraph
{

/**
 * @brief Construct an Environment with name, type, and specification.
 * @param name Name of the environment.
 * @param type Type of the environment (as string).
 * @param spec JSON specification of the environment.
 * @throws std::runtime_error if the environment type is unknown.
 */
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

/**
 * @brief Set the backend instance for the environment.
 * @param backend Shared pointer to PlanInstance backend.
 */
void Environment::setBackend(std::shared_ptr<PlanInstance> backend)
{
    backend_ = backend;
}

/**
 * @brief Set the environment state.
 * @param state The new environment state.
 */
void Environment::setState(const EnvState &state)
{
    env_state = state;
}

}
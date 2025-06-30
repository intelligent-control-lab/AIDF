/**
 * @file environment.hpp
 * @brief Defines the Environment class for the skillgraph framework.
 */
#pragma once
#include "Utils/Common.hpp"
#include "backend.hpp"
#include "data_structure.hpp"


namespace skillgraph {
    /**
     * @class Environment
     * @brief Contains the environment specification and backend for planning and simulation.
     */
    class Environment {
    /*
    * Environment class containing the environment specification and backend
    */
    public:
        /**
         * @enum Type
         * @brief Types of supported environments.
         */
        enum Type {
            Lego = 1,   /**< Lego environment */
            NIST = 2,   /**< NIST environment */
        };

        /**
         * @brief Default constructor.
         */
        Environment() = default;
        /**
         * @brief Construct an environment with name, type, and specification.
         * @param name Name of the environment.
         * @param type Type of the environment (as string).
         * @param spec JSON specification of the environment.
         */
        Environment(const std::string &name, const std::string &type, const Json::Value &spec);

        /**
         * @brief Set the backend instance for the environment.
         * @param backend Shared pointer to PlanInstance backend.
         */
        void setBackend(std::shared_ptr<PlanInstance> backend);
        /**
         * @brief Get the backend instance.
         * @return Shared pointer to PlanInstance backend.
         */
        std::shared_ptr<PlanInstance> getBackend() const { return backend_; }

        /**
         * @brief Set the environment state.
         * @param state The new environment state.
         */
        void setState(const EnvState &state);

        std::string name;      /**< Name of the environment */
        Type type;             /**< Type of the environment */

        Json::Value spec_;     /**< Environment specification (JSON) */
        EnvState env_state;    /**< Current environment state */
        std::shared_ptr<PlanInstance> backend_; /**< Backend instance */
    };

     
   

}

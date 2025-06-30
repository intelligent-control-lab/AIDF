/**
 * @file metrics.hpp
 * @brief Defines metrics and evaluators for the skillgraph framework.
 */
#pragma once
#include "Utils/Common.hpp"


namespace skillgraph {
    /**
     * @class Metric
     * @brief Base class for metrics used in evaluation.
     */
    class Metric {
    public:
        /**
         * @brief Construct a metric with a given name.
         * @param name Name of the metric.
         */
        Metric(const std::string &name);

    protected:
        std::string name; /**< Name of the metric */
    };

    /**
     * @struct ConditionEvaluator
     * @brief Evaluates a condition as a metric.
     */
    struct ConditionEvaluator: public Metric {
        /**
         * @brief Construct a condition evaluator with a given name.
         * @param name Name of the evaluator.
         */
        ConditionEvaluator(const std::string &name);

        /**
         * @brief Evaluate the condition.
         * @return True if the condition is met, false otherwise.
         */
        bool eval_condition();
    };

    /**
     * @struct ConstraintEvaluator
     * @brief Evaluates a constraint as a metric.
     */
    struct ConstraintEvaluator: public Metric {
        /**
         * @brief Construct a constraint evaluator with a given name.
         * @param name Name of the evaluator.
         */
        ConstraintEvaluator(const std::string &name);

        /**
         * @brief Evaluate the constraint.
         * @return True if the constraint is satisfied, false otherwise.
         */
        bool eval_condition();
    };

    /**
     * @struct SmoothnessMetrics
     * @brief Stores smoothness-related metrics for trajectories.
     */
    struct SmoothnessMetrics {
        double normalized_jerk_score;      /**< Normalized jerk score */
        double directional_consistency;    /**< Directional consistency */
    };
    
}
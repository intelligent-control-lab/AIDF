/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu 
    Peiqi Yu peiqiy@andrew.cmu.edu 
    Chaitanya Chawla cchawla@cs.cmu.edu 
    Changliu Liu cliu6@andrew.cmu.edu 
    Jiaoyang Li jiaoyanl@andrew.cmu.edu 
    Guanya Shi guanyas@andrew.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

#pragma once

#include "skills.hpp"
#include "data_structure.hpp"
#include "Utils/Logger.hpp"

#include <optional>
#include <string>
#include <mutex>
#include <functional>

#include <ros/ros.h>
#include <boost/process.hpp>

namespace skillgraph {

struct PerceptionSkillConfig;

/**
 * @brief ROS-based perception executor that interfaces with Python perception nodes.
 *
 * @tparam MsgT ROS message type emitted by the perception pipeline.
 */
template <typename MsgT>
class ROSPyPerceptionExecutor : public SkillExecutor {
public:
    using MessageType = MsgT;
    using MessageCallback = std::function<void(const MsgT &)>;
    using CompletionPredicate = std::function<bool(const std::optional<MsgT>&)>;

    ROSPyPerceptionExecutor(std::shared_ptr<ros::NodeHandle> nh,
                            Skill::Type skill_type,
                            const PerceptionSkillConfig &config,
                            CompletionPredicate completion_check = {},
                            MessageCallback external_callback = {});

    ~ROSPyPerceptionExecutor() override;

    bool execute(State &current_state) override;

    void start();
    void stop();

    bool isRunning() const;
    std::optional<MsgT> latestMessage() const;
    ros::Time lastUpdateTime() const;

    void setCallback(MessageCallback cb);
    void setCompletionPredicate(CompletionPredicate predicate);

private:
    using OptionalMessage = std::optional<MsgT>;

    void ensureSubscriber();
    void handleMessage(const typename MsgT::ConstPtr &msg);
    bool isComplete(const OptionalMessage &msg) const;
    double waitTimeout() const;

    std::shared_ptr<ros::NodeHandle> nh_;
    PerceptionSkillConfig config_;
    CompletionPredicate completion_check_;
    MessageCallback external_callback_;

    mutable std::mutex mutex_;
    OptionalMessage latest_msg_;
    ros::Time last_update_;

    ros::Subscriber subscriber_;
    bool subscriber_active_ = false;

    std::unique_ptr<boost::process::child> launch_process_;
};

// Implementation

template <typename MsgT>
ROSPyPerceptionExecutor<MsgT>::ROSPyPerceptionExecutor(std::shared_ptr<ros::NodeHandle> nh,
                                                       Skill::Type skill_type,
                                                       const PerceptionSkillConfig &config,
                                                       CompletionPredicate completion_check,
                                                       MessageCallback external_callback)
    : SkillExecutor(skill_type),
      nh_(std::move(nh)),
      config_(config),
      completion_check_(completion_check),
      external_callback_(std::move(external_callback)) {
    if (!completion_check_) {
        completion_check_ = [](const OptionalMessage &msg) { return msg.has_value(); };
    }
}

template <typename MsgT>
ROSPyPerceptionExecutor<MsgT>::~ROSPyPerceptionExecutor() {
    stop();
}

template <typename MsgT>
void ROSPyPerceptionExecutor<MsgT>::start() {
    if (!nh_) {
        throw std::runtime_error("ROSPyPerceptionExecutor requires a valid NodeHandle");
    }

    if (!config_.launch_command.empty() && !launch_process_) {
        namespace bp = boost::process;
        try {
            launch_process_ = std::make_unique<bp::child>(bp::search_path("bash"), "-lc", config_.launch_command);
        } catch (const std::exception &e) {
            log("Failed to launch perception process: " + config_.launch_command + " error: " + e.what(), LogLevel::ERROR);
            throw;
        }
    }

    ensureSubscriber();
}

template <typename MsgT>
void ROSPyPerceptionExecutor<MsgT>::stop() {
    if (subscriber_active_) {
        subscriber_.shutdown();
        subscriber_active_ = false;
    }

    if (launch_process_) {
        namespace bp = boost::process;
        if (launch_process_->running()) {
            launch_process_->terminate();
            launch_process_->wait();
        }
        launch_process_.reset();
    }
}

template <typename MsgT>
bool ROSPyPerceptionExecutor<MsgT>::execute(State &current_state) {
    (void)current_state;
    start();

    ros::Rate rate(50.0);
    const double timeout = waitTimeout();
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        OptionalMessage copy;
        {
            std::scoped_lock<std::mutex> lock(mutex_);
            copy = latest_msg_;
        }

        if (isComplete(copy)) {
            return true;
        }

        if (timeout > 0.0 && (ros::Time::now() - start_time).toSec() > timeout) {
            log("Perception skill timed out waiting for message on " + config_.topic, LogLevel::WARN);
            return false;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return false;
}

template <typename MsgT>
bool ROSPyPerceptionExecutor<MsgT>::isRunning() const {
    return subscriber_active_;
}

template <typename MsgT>
std::optional<MsgT> ROSPyPerceptionExecutor<MsgT>::latestMessage() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return latest_msg_;
}

template <typename MsgT>
ros::Time ROSPyPerceptionExecutor<MsgT>::lastUpdateTime() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return last_update_;
}

template <typename MsgT>
void ROSPyPerceptionExecutor<MsgT>::setCallback(MessageCallback cb) {
    std::scoped_lock<std::mutex> lock(mutex_);
    external_callback_ = std::move(cb);
}

template <typename MsgT>
void ROSPyPerceptionExecutor<MsgT>::setCompletionPredicate(CompletionPredicate predicate) {
    std::scoped_lock<std::mutex> lock(mutex_);
    completion_check_ = std::move(predicate);
    if (!completion_check_) {
        completion_check_ = [](const OptionalMessage &msg) { return msg.has_value(); };
    }
}

template <typename MsgT>
void ROSPyPerceptionExecutor<MsgT>::ensureSubscriber() {
    if (subscriber_active_) {
        return;
    }

    if (config_.topic.empty()) {
        throw std::runtime_error("Perception skill topic is empty");
    }

    subscriber_ = nh_->subscribe<MsgT>(config_.topic, 1, &ROSPyPerceptionExecutor<MsgT>::handleMessage, this);
    subscriber_active_ = true;
}

template <typename MsgT>
void ROSPyPerceptionExecutor<MsgT>::handleMessage(const typename MsgT::ConstPtr &msg) {
    {
        std::scoped_lock<std::mutex> lock(mutex_);
        latest_msg_ = *msg;
        last_update_ = ros::Time::now();
    }

    if (external_callback_) {
        external_callback_(*msg);
    }
}

template <typename MsgT>
bool ROSPyPerceptionExecutor<MsgT>::isComplete(const OptionalMessage &msg) const {
    return completion_check_ ? completion_check_(msg) : msg.has_value();
}

template <typename MsgT>
double ROSPyPerceptionExecutor<MsgT>::waitTimeout() const {
    return config_.wait_timeout;
}

} // namespace skillgraph


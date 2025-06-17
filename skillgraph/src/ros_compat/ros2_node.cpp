#include "../include/ros_compat/node.hpp"
#include "../include/ros_compat/service_client.hpp"
#include "../include/ros_compat/publisher.hpp"
#include "../include/ros_compat/subscriber.hpp"

#ifdef ROS2_BUILD
// ROS2 implementation

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace skillgraph {
namespace ros_compat {

/**
 * @brief ROS2 NodeHandle implementation
 */
class ROS2NodeHandle : public NodeHandle {
public:
    ROS2NodeHandle(const std::string& node_name) 
        : node_(rclcpp::Node::make_shared(node_name)) {}
    
    void shutdown() override {
        rclcpp::shutdown();
    }

protected:
    void* createServiceClientImpl(const std::string& service_name, const std::string& service_type) override {
        // This is a generic implementation - specific service types will be handled by templates
        return nullptr;
    }
    
    void* createPublisherImpl(const std::string& topic_name, const std::string& message_type, int queue_size) override {
        // This is a generic implementation - specific message types will be handled by templates  
        return nullptr;
    }
    
    void* createSubscriberImpl(const std::string& topic_name, const std::string& message_type, int queue_size, void* callback) override {
        // This is a generic implementation - specific message types will be handled by templates
        return nullptr;
    }

public:
    std::shared_ptr<rclcpp::Node> getNode() { return node_; }

private:
    std::shared_ptr<rclcpp::Node> node_;
};

// Template specializations for NodeHandle methods
template<typename ServiceType>
std::shared_ptr<ServiceClient<ServiceType>> NodeHandle::serviceClient(const std::string& service_name) {
    auto* ros2_nh = dynamic_cast<ROS2NodeHandle*>(this);
    if (ros2_nh) {
        return std::make_shared<ROS2ServiceClient<ServiceType>>(ros2_nh->getNode(), service_name);
    }
    return nullptr;
}

template<typename MessageType>
std::shared_ptr<Publisher<MessageType>> NodeHandle::advertise(const std::string& topic_name, int queue_size) {
    auto* ros2_nh = dynamic_cast<ROS2NodeHandle*>(this);
    if (ros2_nh) {
        return std::make_shared<ROS2Publisher<MessageType>>(ros2_nh->getNode(), topic_name, queue_size);
    }
    return nullptr;
}

template<typename MessageType>
std::shared_ptr<Subscriber<MessageType>> NodeHandle::subscribe(
    const std::string& topic_name,
    int queue_size,
    std::function<void(const std::shared_ptr<MessageType>&)> callback) {
    
    auto* ros2_nh = dynamic_cast<ROS2NodeHandle*>(this);
    if (ros2_nh) {
        return std::make_shared<ROS2Subscriber<MessageType>>(ros2_nh->getNode(), topic_name, queue_size, callback);
    }
    return nullptr;
}

// NodeFactory implementation for ROS2
std::shared_ptr<NodeHandle> NodeFactory::createNode(const std::string& node_name) {
    return std::make_shared<ROS2NodeHandle>(node_name);
}

void NodeFactory::init(int argc, char** argv, const std::string& node_name) {
    rclcpp::init(argc, argv);
}

bool NodeFactory::ok() {
    return rclcpp::ok();
}

int NodeFactory::detectRosVersion() {
    return 2; // ROS2
}

} // namespace ros_compat
} // namespace skillgraph

#endif // ROS2_BUILD

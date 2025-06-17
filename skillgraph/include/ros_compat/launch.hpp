#pragma once

#include <string>
#include <vector>
#include <memory>
#include <boost/process.hpp>

/**
 * @file launch.hpp
 * @brief ROS-agnostic launch system abstraction
 * 
 * Provides unified launch interface that works with both ROS1 and ROS2
 */

namespace skillgraph {
namespace ros_compat {

/**
 * @brief Process handle for launched ROS nodes
 */
class LaunchProcess {
public:
    virtual ~LaunchProcess() = default;
    
    /**
     * @brief Check if the process is running
     * @return True if running
     */
    virtual bool running() const = 0;
    
    /**
     * @brief Terminate the process
     */
    virtual void terminate() = 0;
    
    /**
     * @brief Wait for process to exit
     * @param timeout_sec Timeout in seconds (-1 for infinite)
     * @return True if process exited within timeout
     */
    virtual bool wait_for(double timeout_sec = -1.0) = 0;
    
    /**
     * @brief Kill the process forcefully
     */
    virtual void kill() = 0;
    
    /**
     * @brief Get process exit code (if exited)
     * @return Exit code
     */
    virtual int exit_code() const = 0;
};

/**
 * @brief ROS-agnostic launcher interface
 */
class Launcher {
public:
    virtual ~Launcher() = default;
    
    /**
     * @brief Launch a ROS package
     * @param package_name Name of the package
     * @param launch_file Name of the launch file
     * @param args Additional arguments
     * @return Process handle
     */
    virtual std::unique_ptr<LaunchProcess> launch(
        const std::string& package_name,
        const std::string& launch_file,
        const std::vector<std::string>& args = {}) = 0;
    
    /**
     * @brief Launch a node directly
     * @param package_name Name of the package
     * @param executable_name Name of the executable
     * @param node_name Name of the node instance
     * @param args Additional arguments
     * @return Process handle
     */
    virtual std::unique_ptr<LaunchProcess> launch_node(
        const std::string& package_name,
        const std::string& executable_name,
        const std::string& node_name = "",
        const std::vector<std::string>& args = {}) = 0;
};

/**
 * @brief Factory for creating ROS-compatible launchers
 */
class LauncherFactory {
public:
    /**
     * @brief Create a launcher based on the detected ROS version
     * @return Launcher instance
     */
    static std::unique_ptr<Launcher> createLauncher();
    
private:
    static int detectRosVersion();
    static std::string getRosDistro();
};

} // namespace ros_compat
} // namespace skillgraph

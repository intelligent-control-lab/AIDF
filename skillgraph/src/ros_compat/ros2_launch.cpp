#include "../include/ros_compat/launch.hpp"

#ifdef ROS2_BUILD
// ROS2 implementation

#include <boost/process.hpp>
#include <boost/asio.hpp>
#include <cstdlib>
#include <chrono>
#include <thread>

namespace skillgraph {
namespace ros_compat {

namespace bp = boost::process;

/**
 * @brief ROS2 process handle implementation
 */
class ROS2LaunchProcess : public LaunchProcess {
public:
    ROS2LaunchProcess(bp::child process) : process_(std::move(process)) {}
    
    bool running() const override {
        return const_cast<bp::child&>(process_).running();
    }
    
    void terminate() override {
        if (const_cast<bp::child&>(process_).running()) {
            process_.terminate();
        }
    }
    
    bool wait_for(double timeout_sec = -1.0) override {
        if (timeout_sec < 0) {
            process_.wait();
            return true;
        } else {
            auto status = process_.wait_for(std::chrono::duration<double>(timeout_sec));
            return status != std::future_status::timeout;
        }
    }
    
    void kill() override {
        if (const_cast<bp::child&>(process_).running()) {
            process_.terminate();
            
            // Give it a moment to terminate gracefully
            auto status = process_.wait_for(std::chrono::seconds(2));
            if (status == std::future_status::timeout) {
                // Force kill if it doesn't terminate
                system(("kill -9 " + std::to_string(process_.id())).c_str());
            }
        }
    }
    
    int exit_code() const override {
        return process_.exit_code();
    }

private:
    bp::child process_;
};

/**
 * @brief ROS2 launcher implementation
 */
class ROS2Launcher : public Launcher {
public:
    std::unique_ptr<LaunchProcess> launch(
        const std::string& package_name,
        const std::string& launch_file,
        const std::vector<std::string>& args = {}) override {
        
        std::vector<std::string> launch_args = {"launch", package_name, launch_file};
        launch_args.insert(launch_args.end(), args.begin(), args.end());
        
        boost::asio::io_context io;
        bp::environment env = boost::this_process::environment();
        env["LIBGL_ALWAYS_SOFTWARE"] = "1";  // Software rendering for compatibility
        
        std::string ros2_cmd = getRos2Path();
        
        bp::child process(ros2_cmd,
                         bp::args(launch_args),
                         env,
                         bp::start_dir("/tmp"),
                         io);
        
        io.run();
        
        return std::make_unique<ROS2LaunchProcess>(std::move(process));
    }
    
    std::unique_ptr<LaunchProcess> launch_node(
        const std::string& package_name,
        const std::string& executable_name,
        const std::string& node_name = "",
        const std::vector<std::string>& args = {}) override {
        
        std::vector<std::string> run_args = {"run", package_name, executable_name};
        
        if (!node_name.empty()) {
            run_args.push_back("--ros-args");
            run_args.push_back("-r");
            run_args.push_back("__node:=" + node_name);
        }
        
        run_args.insert(run_args.end(), args.begin(), args.end());
        
        boost::asio::io_context io;
        bp::environment env = boost::this_process::environment();
        
        std::string ros2_cmd = getRos2Path();
        
        bp::child process(ros2_cmd,
                         bp::args(run_args),
                         env,
                         bp::start_dir("/tmp"),
                         io);
        
        io.run();
        
        return std::make_unique<ROS2LaunchProcess>(std::move(process));
    }

private:
    std::string getRos2Path() {
        // Try to find ros2 in common locations
        const char* ament_prefix = std::getenv("AMENT_PREFIX_PATH");
        if (ament_prefix) {
            // Extract first path from AMENT_PREFIX_PATH
            std::string prefix(ament_prefix);
            size_t colon_pos = prefix.find(':');
            if (colon_pos != std::string::npos) {
                prefix = prefix.substr(0, colon_pos);
            }
            return prefix + "/bin/ros2";
        }
        
        std::string distro = getRosDistro();
        if (!distro.empty()) {
            return "/opt/ros/" + distro + "/bin/ros2";
        }
        
        return "ros2";  // Fallback to PATH
    }
    
    std::string getRosDistro() {
        const char* distro = std::getenv("ROS_DISTRO");
        return distro ? std::string(distro) : "humble";  // Default to humble
    }
};

// Factory implementation for ROS2
std::unique_ptr<Launcher> LauncherFactory::createLauncher() {
    return std::make_unique<ROS2Launcher>();
}

int LauncherFactory::detectRosVersion() {
    return 2; // ROS2
}

std::string LauncherFactory::getRosDistro() {
    const char* distro = std::getenv("ROS_DISTRO");
    return distro ? std::string(distro) : "humble";
}

} // namespace ros_compat
} // namespace skillgraph

#endif // ROS2_BUILD

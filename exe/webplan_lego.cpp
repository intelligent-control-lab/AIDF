/**
 * @file webplan_lego.cpp
 * @brief Entry point for running the Lego Skill Graph planner with web/file monitoring capabilities.
 *
 * This executable monitors a file for changes and triggers planning logic accordingly.
 * ROS processes are now managed by MoveitInstance in moveit_backend.cpp for natural shutdown.
 */

#include "skillgraph.hpp"
#include "lego/lego_skillgraph.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "planner.h"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <sys/stat.h>
#include <functional>

using namespace skillgraph;

// Control variable for main loop
std::atomic<bool> program_running(true);

/**
 * @class FileMonitor
 * @brief Monitors a file for changes and triggers a callback when the file is modified.
 *
 * Uses a polling mechanism to check file status at a specified interval.
 */
class FileMonitor {
    public:
        FileMonitor(const std::string& filepath, std::function<void()> callback, 
                   int poll_interval_ms = 100) 
            : filepath_(filepath), callback_(callback), 
              poll_interval_ms_(poll_interval_ms), running_(false) {}
        
        void start() {
            running_ = true;
            monitor_thread_ = std::thread([this]() { this->monitorFile(); });
        }
        
        void stop() {
            running_ = false;
            if (monitor_thread_.joinable()) {
                monitor_thread_.join();
            }
        }
        
        ~FileMonitor() {
            stop();
        }
        
    private:
        /**
         * @brief Monitors the file for changes in a separate thread.
         */
        void monitorFile() {
            struct stat prev_stat = {0};
            struct stat current_stat = {0};
            
            // Get initial file info
            if (stat(filepath_.c_str(), &prev_stat) != 0) {
                // File doesn't exist yet, wait for creation
                prev_stat.st_mtime = 0;
            }
            
            while (running_) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(poll_interval_ms_));
                
                if (stat(filepath_.c_str(), &current_stat) == 0) {
                    // Check if modification time changed
                    if (current_stat.st_mtime > prev_stat.st_mtime) {
                        callback_();
                        prev_stat = current_stat;
                    }
                }
            }
        }
        
        std::string filepath_;
        std::function<void()> callback_;
        int poll_interval_ms_;
        std::atomic<bool> running_;
        std::thread monitor_thread_;
    };

/**
 * @brief Main function for the web-enabled Lego Skill Graph planner executable.
 *
 * Registers signal handlers and sets up file monitoring for triggering planning logic.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code.
 */
int main(int argc, char* argv[]) {
    try {
        // Default paths using PathResolver
        std::string config_path = skillgraph::utils::PathResolver::getDefaultSkillgraphConfig();
        std::string web_msg_json_path = skillgraph::utils::PathResolver::getDefaultWebMessageConfig();

        // Parse command-line arguments
        if (argc > 2) {
            config_path = argv[1];
            web_msg_json_path = argv[2];
        } else {
            log("Using default config path: " + config_path, LogLevel::INFO);
            log("Using default web message JSON path: " + web_msg_json_path, LogLevel::INFO);
        }

        // Main Logic
        auto sg = std::make_shared<skillgraph::LegoSkillGraph>(config_path);
        sg->initialize();
        log("Lego Skill Graph Initialized", LogLevel::INFO);
        sg->print_skillgraph();

        State state = sg->get_initial_state();

        // Define a callback function to process the file when it changes
        auto processJsonFile = [&sg, &state, web_msg_json_path]() {
            std::ifstream file(web_msg_json_path);
            if (!file.is_open()) {
                std::cerr << "Failed to open data.json" << std::endl;
                return;
            }
            
            Json::Value web_json;
            file >> web_json;

            // Process the JSON data and use it with your skill graph
            // You might need a JSON library like nlohmann/json
            std::cout << "Processing new data from " << web_msg_json_path << std::endl;
            // web_json is something like 
            // {
            //     "skill": "PickAndPlace",
            //     "object": "b2_1",
            //     "robot": "1",
            //     "target_location": {
            //         "x": "1",
            //         "y": "2",
            //         "z": "3",
            //         "ori": ""
            //     },
            //     "command_id": 1741971091512
            // }
            
            skillgraph::SkillPtr skill;
            bool success = sg->is_feasible(state, web_json, skill);
            if (success) {
                std::cout << "Skill is feasible: " << skill->to_string() << std::endl;
                // Execute the skill
                success = skill->executor->execute(state);
                if (!success) {
                    std::string msg;
                    getPastLog(msg);
                    std::cerr << "error: not feasible, id: " << web_json["command_id"].asUInt64() << ", reason: " << "execution failed" << std::endl;
                }
            } else {
                std::string msg;
                getPastLog(msg);
                std::cerr << "error: not feasible, id: " << web_json["command_id"].asUInt64() << ", reason: " << msg << std::endl;
            }

        };

        // Setup file monitoring
        FileMonitor monitor(web_msg_json_path, processJsonFile);
        monitor.start();

        // Keep program running until any stop signal is received
        while (program_running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        monitor.stop();

        // Shutdown Logic (for natural exit)
        std::cout << "Main logic finished.  Shutting down..." << std::endl;


    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        program_running = false;
        return 1;
    }

    return 0;
}
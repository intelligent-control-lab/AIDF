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

#include "Utils/PathUtils.hpp"
#include "Utils/Logger.hpp"
#include <filesystem>
#include <cstdlib>
#include <iostream>

namespace skillgraph {
namespace utils {

std::string PathResolver::project_root_cache_;
bool PathResolver::root_found_ = false;

std::string PathResolver::getProjectRoot() {
    if (root_found_) {
        return project_root_cache_;
    }
    
    // Method 1: Check environment variable
    const char* env_root = std::getenv("AIDF_ROOT_DIR");
    if (env_root) {
        std::string env_path(env_root);
        if (std::filesystem::exists(env_path + "/config")) {
            project_root_cache_ = std::filesystem::absolute(env_path);
            root_found_ = true;
            log("PathResolver: Found project root via AIDF_ROOT_DIR: " + project_root_cache_, LogLevel::INFO);
            return project_root_cache_;
        }
    }
    
    // Method 2: Look relative to executable path
    try {
        std::string exe_path = std::filesystem::canonical("/proc/self/exe").parent_path();
        std::string found_root = findProjectRoot(exe_path);
        if (!found_root.empty()) {
            project_root_cache_ = found_root;
            root_found_ = true;
            log("PathResolver: Found project root relative to executable: " + project_root_cache_, LogLevel::INFO);
            return project_root_cache_;
        }
    } catch (const std::exception& e) {
        // Continue to next method
    }
    
    // Method 3: Check current working directory
    std::string cwd = std::filesystem::current_path();
    std::string found_root = findProjectRoot(cwd);
    if (!found_root.empty()) {
        project_root_cache_ = found_root;
        root_found_ = true;
        log("PathResolver: Found project root from current directory: " + project_root_cache_, LogLevel::INFO);
        return project_root_cache_;
    }
    
    // Method 4: Try common catkin workspace patterns
    std::vector<std::string> catkin_patterns = {
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/catkin_ws/src/AIDF",
        "/opt/ros/catkin_ws/src/AIDF",
        "/home/philip/catkin_ws/src/AIDF",
        "/home/mfi/repos/ros1_ws/src/philip/aidf"
    };
    
    for (const auto& pattern : catkin_patterns) {
        if (!pattern.empty() && std::filesystem::exists(pattern + "/config")) {
            project_root_cache_ = std::filesystem::absolute(pattern);
            root_found_ = true;
            log("PathResolver: Found project root via catkin pattern: " + project_root_cache_, LogLevel::WARN);
            return project_root_cache_;
        }
    }
    
    // Fallback: use current directory
    project_root_cache_ = std::filesystem::current_path();
    root_found_ = true;
    log("PathResolver: Using current directory as fallback project root: " + project_root_cache_, LogLevel::ERROR);
    return project_root_cache_;
}

std::string PathResolver::findProjectRoot(const std::string& startPath) {
    std::filesystem::path current(startPath);
    
    // Look for AIDF project markers (config directory, CMakeLists.txt, package.xml)
    while (current != current.root_path()) {
        std::filesystem::path config_dir = current / "config";
        std::filesystem::path cmake_file = current / "CMakeLists.txt";
        std::filesystem::path package_file = current / "package.xml";
        
        if (std::filesystem::exists(config_dir) && 
            std::filesystem::exists(cmake_file) && 
            std::filesystem::exists(package_file)) {
            
            // Additional check: look for lego_tasks subdirectory
            std::filesystem::path lego_tasks = config_dir / "lego_tasks";
            if (std::filesystem::exists(lego_tasks)) {
                return std::filesystem::absolute(current);
            }
        }
        
        current = current.parent_path();
    }
    
    return "";
}

std::string PathResolver::resolvePath(const std::string& relativePath) {
    std::string root = getProjectRoot();
    std::filesystem::path fullPath = std::filesystem::path(root) / relativePath;
    return std::filesystem::absolute(fullPath);
}

std::string PathResolver::getDefaultSkillgraphConfig() {
    return resolvePath("config/lego_tasks/skillgraph.json");
}

std::string PathResolver::getDefaultWebMessageConfig() {
    return resolvePath("config/web_message.json");
}

bool PathResolver::fileExists(const std::string& filePath) {
    return std::filesystem::exists(filePath);
}

} // namespace utils
} // namespace skillgraph

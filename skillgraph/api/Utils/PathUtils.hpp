#pragma once
#include <string>
#include <filesystem>

namespace skillgraph {
namespace utils {

/**
 * @brief Utility class for resolving project paths dynamically.
 */
class PathResolver {
public:
    /**
     * @brief Get the root directory of the AIDF project.
     * 
     * This function tries multiple methods to find the project root:
     * 1. Environment variable AIDF_ROOT_DIR
     * 2. Relative to executable path (looks for config/ directory)
     * 3. Current working directory if it contains config/
     * 4. Falls back to a default catkin workspace structure
     * 
     * @return std::string The absolute path to the project root directory.
     */
    static std::string getProjectRoot();
    
    /**
     * @brief Resolve a relative path to an absolute path based on project root.
     * @param relativePath The relative path from project root.
     * @return std::string The absolute path.
     */
    static std::string resolvePath(const std::string& relativePath);
    
    /**
     * @brief Get the default skillgraph configuration file path.
     * @return std::string The absolute path to skillgraph.json.
     */
    static std::string getDefaultSkillgraphConfig();
    
    /**
     * @brief Get the default web message configuration file path.
     * @return std::string The absolute path to web_message.json.
     */
    static std::string getDefaultWebMessageConfig();
    
    /**
     * @brief Check if a file exists.
     * @param filePath The file path to check.
     * @return bool True if file exists, false otherwise.
     */
    static bool fileExists(const std::string& filePath);
    
private:
    static std::string project_root_cache_;
    static bool root_found_;
    
    /**
     * @brief Find project root by looking for marker files/directories.
     * @param startPath The path to start searching from.
     * @return std::string The project root path, or empty if not found.
     */
    static std::string findProjectRoot(const std::string& startPath);
};

} // namespace utils
} // namespace skillgraph

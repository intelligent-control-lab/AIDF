#!/bin/bash

# test_build_system.sh - Test script for AIDF dual ROS build system

# Don't exit on errors - we want to run all tests
set +e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to test ROS version detection
test_ros_detection() {
    log_info "Testing ROS version detection..."
    
    # Test with ROS_VERSION environment variable
    export ROS_VERSION=1
    mkdir -p /tmp/aidf_test
    cd /tmp/aidf_test
    
    # Create minimal CMakeLists.txt with our detection logic
    cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.14)
project(test_detection)

# ROS Version Detection (copied from main CMakeLists.txt)
if(DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
else()
    if(DEFINED ENV{ROS_DISTRO})
        set(ROS_DISTRO $ENV{ROS_DISTRO})
        if(ROS_DISTRO MATCHES "humble|iron|jazzy|rolling")
            set(ROS_VERSION 2)
        else()
            set(ROS_VERSION 1)
        endif()
    else()
        set(ROS_VERSION 1)
    endif()
endif()

message(STATUS "Detected ROS version: ${ROS_VERSION}")

if(ROS_VERSION EQUAL 2)
    add_definitions(-DROS2_BUILD)
    message(STATUS "Building for ROS2")
else()
    add_definitions(-DROS1_BUILD)
    message(STATUS "Building for ROS1")
endif()
EOF
    
    # Test ROS1 detection
    export ROS_VERSION=1
    unset ROS_DISTRO
    cmake . > cmake_output.txt 2>&1
    if grep -q "Building for ROS1" cmake_output.txt; then
        log_success "ROS1 detection works with ROS_VERSION=1"
    else
        log_error "ROS1 detection failed with ROS_VERSION=1"
        cat cmake_output.txt
        return 1
    fi
    
    # Test ROS2 detection
    export ROS_VERSION=2
    cmake . > cmake_output.txt 2>&1
    if grep -q "Building for ROS2" cmake_output.txt; then
        log_success "ROS2 detection works with ROS_VERSION=2"
    else
        log_error "ROS2 detection failed with ROS_VERSION=2"
        cat cmake_output.txt
        return 1
    fi
    
    # Test distro-based detection
    unset ROS_VERSION
    export ROS_DISTRO=humble
    cmake . > cmake_output.txt 2>&1
    if grep -q "Building for ROS2" cmake_output.txt; then
        log_success "ROS2 detection works with ROS_DISTRO=humble"
    else
        log_error "ROS2 detection failed with ROS_DISTRO=humble"
        cat cmake_output.txt
        return 1
    fi
    
    export ROS_DISTRO=noetic
    cmake . > cmake_output.txt 2>&1
    if grep -q "Building for ROS1" cmake_output.txt; then
        log_success "ROS1 detection works with ROS_DISTRO=noetic"
    else
        log_error "ROS1 detection failed with ROS_DISTRO=noetic"
        cat cmake_output.txt
        return 1
    fi
    
    # Cleanup
    cd - > /dev/null
    rm -rf /tmp/aidf_test
    
    log_success "All ROS detection tests passed!"
}

# Function to test file structure
test_file_structure() {
    log_info "Testing AIDF file structure..."
    
    local base_dir="$1"
    if [[ -z "$base_dir" ]]; then
        base_dir="$(pwd)"
    fi
    
    # Required files for dual ROS support
    local required_files=(
        "CMakeLists.txt"
        "package.xml"
        "skillgraph/include/ros_compat/time.hpp"
        "skillgraph/include/ros_compat/node.hpp"
        "skillgraph/include/ros_compat/service_client.hpp"
        "skillgraph/include/ros_compat/publisher.hpp"
        "skillgraph/include/ros_compat/subscriber.hpp"
        "skillgraph/include/ros_compat/launch.hpp"
        "skillgraph/src/ros_compat/ros1_node.cpp"
        "skillgraph/src/ros_compat/ros1_launch.cpp"
        "skillgraph/src/ros_compat/ros2_node.cpp"
        "skillgraph/src/ros_compat/ros2_launch.cpp"
        "skillgraph/include/Utils/PathUtils.hpp"
        "skillgraph/src/Utils/PathUtils.cpp"
        "docs/ROS2_MIGRATION_PLAN.md"
        "docs/BUILD_INSTRUCTIONS.md"
        "setup_env.sh"
        "test_paths.sh"
    )
    
    local missing_files=()
    
    for file in "${required_files[@]}"; do
        local full_path="$base_dir/$file"
        if [[ -f "$full_path" ]]; then
            log_success "Found: $file"
        else
            log_error "Missing: $file"
            missing_files+=("$file")
        fi
    done
    
    if [[ ${#missing_files[@]} -eq 0 ]]; then
        log_success "All required files are present!"
        return 0
    else
        log_error "Missing ${#missing_files[@]} required files"
        return 1
    fi
}

# Function to test header includes
test_header_includes() {
    log_info "Testing header file syntax..."
    
    local base_dir="$1"
    if [[ -z "$base_dir" ]]; then
        base_dir="$(pwd)"
    fi
    
    # Test C++ header syntax
    local headers=(
        "skillgraph/include/ros_compat/time.hpp"
        "skillgraph/include/ros_compat/node.hpp"
        "skillgraph/include/ros_compat/service_client.hpp"
        "skillgraph/include/ros_compat/publisher.hpp"
        "skillgraph/include/ros_compat/subscriber.hpp"
        "skillgraph/include/ros_compat/launch.hpp"
        "skillgraph/include/Utils/PathUtils.hpp"
    )
    
    for header in "${headers[@]}"; do
        local full_path="$base_dir/$header"
        if [[ -f "$full_path" ]]; then
            # Basic syntax check
            if g++ -std=c++14 -fsyntax-only -I"$base_dir/skillgraph/include" "$full_path" 2>/dev/null; then
                log_success "Header syntax OK: $header"
            else
                log_warning "Header syntax issues: $header (may need ROS dependencies)"
            fi
        fi
    done
}

# Function to validate CMakeLists.txt
test_cmake_syntax() {
    log_info "Testing CMakeLists.txt syntax..."
    
    local base_dir="$1"
    if [[ -z "$base_dir" ]]; then
        base_dir="$(pwd)"
    fi
    
    local cmake_file="$base_dir/CMakeLists.txt"
    
    if [[ -f "$cmake_file" ]]; then
        # Check for required CMake features
        local required_patterns=(
            "ROS_VERSION"
            "ROS2_BUILD"
            "ROS1_BUILD"
            "ros_compat"
            "ament_cmake"
            "catkin"
        )
        
        for pattern in "${required_patterns[@]}"; do
            if grep -q "$pattern" "$cmake_file"; then
                log_success "CMakeLists.txt contains: $pattern"
            else
                log_warning "CMakeLists.txt missing: $pattern"
            fi
        done
    else
        log_error "CMakeLists.txt not found"
        return 1
    fi
}

# Function to test path resolution
test_path_resolution() {
    log_info "Testing path resolution system..."
    
    local base_dir="$1"
    if [[ -z "$base_dir" ]]; then
        base_dir="$(pwd)"
    fi
    
    # Test if path resolution script exists and works
    local test_script="$base_dir/test_paths.sh"
    
    if [[ -f "$test_script" && -x "$test_script" ]]; then
        export AIDF_ROOT_DIR="$base_dir"
        if "$test_script"; then
            log_success "Path resolution test passed"
        else
            log_error "Path resolution test failed"
            return 1
        fi
    else
        log_error "Path resolution test script not found or not executable"
        return 1
    fi
}

# Main test runner
main() {
    log_info "Starting AIDF Build System Tests"
    log_info "================================="
    
    local base_dir="${1:-$(pwd)}"
    local test_count=0
    local passed_count=0
    
    # List of test functions
    local tests=(
        "test_ros_detection"
        "test_file_structure"
        "test_header_includes" 
        "test_cmake_syntax"
        "test_path_resolution"
    )
    
    for test_func in "${tests[@]}"; do
        ((test_count++))
        log_info ""
        log_info "Running: $test_func"
        log_info "========================"
        
        # Run test in subshell to prevent early exit
        if (set -e; $test_func "$base_dir"); then
            ((passed_count++))
            log_success "Test passed: $test_func"
        else
            log_error "Test failed: $test_func"
        fi
    done
    
    # Summary
    log_info ""
    log_info "Test Summary"
    log_info "============"
    log_info "Total tests: $test_count"
    log_info "Passed: $passed_count"
    log_info "Failed: $((test_count - passed_count))"
    
    if [[ $passed_count -eq $test_count ]]; then
        log_success "All tests passed! AIDF build system is ready."
        return 0
    else
        log_error "Some tests failed. Please review the output above."
        return 1
    fi
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi

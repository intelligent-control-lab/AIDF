#!/bin/bash

# switch_ros_version.sh - Helper script to switch between ROS1 and ROS2 configurations

set -e

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

# Function to detect current ROS version
detect_ros_version() {
    if [[ -n "$ROS_VERSION" ]]; then
        echo "$ROS_VERSION"
    elif [[ -n "$ROS_DISTRO" ]]; then
        case "$ROS_DISTRO" in
            humble|iron|jazzy|rolling)
                echo "2"
                ;;
            noetic|melodic|kinetic)
                echo "1"
                ;;
            *)
                echo "1"  # Default to ROS1
                ;;
        esac
    else
        echo "1"  # Default to ROS1
    fi
}

# Function to switch to ROS1 configuration
switch_to_ros1() {
    log_info "Switching to ROS1 configuration..."
    
    if [[ -f "package_ros1.xml" ]]; then
        cp package_ros1.xml package.xml
        log_success "Switched to ROS1 package.xml"
    else
        log_error "package_ros1.xml not found!"
        return 1
    fi
    
    # Set environment
    export ROS_VERSION=1
    log_info "Set ROS_VERSION=1"
    
    if [[ -f "setup_env.sh" ]]; then
        source setup_env.sh
        log_success "Sourced setup_env.sh"
    fi
    
    log_success "Ready for ROS1 build with catkin_make"
}

# Function to switch to ROS2 configuration  
switch_to_ros2() {
    log_info "Switching to ROS2 configuration..."
    
    # The default package.xml is already ROS2
    if [[ -f "package.xml" ]] && grep -q "ament_cmake" package.xml; then
        log_success "package.xml is already configured for ROS2"
    else
        log_warning "package.xml may not be properly configured for ROS2"
        log_info "You may need to restore the default ROS2 package.xml"
    fi
    
    # Set environment
    export ROS_VERSION=2
    log_info "Set ROS_VERSION=2"
    
    if [[ -f "setup_env.sh" ]]; then
        source setup_env.sh
        log_success "Sourced setup_env.sh"
    fi
    
    log_success "Ready for ROS2 build with colcon build"
}

# Function to show current configuration
show_status() {
    local detected_version=$(detect_ros_version)
    
    log_info "AIDF ROS Configuration Status"
    log_info "============================="
    log_info "Detected ROS Version: $detected_version"
    log_info "ROS_DISTRO: ${ROS_DISTRO:-not set}"
    log_info "ROS_VERSION: ${ROS_VERSION:-not set}"
    log_info "AIDF_ROOT_DIR: ${AIDF_ROOT_DIR:-not set}"
    
    if [[ -f "package.xml" ]]; then
        if grep -q "ament_cmake" package.xml; then
            log_info "Current package.xml: ROS2 configuration"
        elif grep -q "catkin" package.xml; then
            log_info "Current package.xml: ROS1 configuration"
        else
            log_warning "Current package.xml: Unknown configuration"
        fi
    else
        log_error "package.xml not found!"
    fi
    
    echo ""
    log_info "Available commands:"
    log_info "  ./switch_ros_version.sh ros1    - Switch to ROS1"
    log_info "  ./switch_ros_version.sh ros2    - Switch to ROS2"
    log_info "  ./switch_ros_version.sh status  - Show this status"
}

# Main function
main() {
    local command="${1:-status}"
    
    case "$command" in
        "ros1"|"1")
            switch_to_ros1
            ;;
        "ros2"|"2")
            switch_to_ros2
            ;;
        "status"|"info")
            show_status
            ;;
        "help"|"-h"|"--help")
            echo "Usage: $0 [ros1|ros2|status]"
            echo ""
            echo "Commands:"
            echo "  ros1    - Switch to ROS1 configuration"
            echo "  ros2    - Switch to ROS2 configuration"
            echo "  status  - Show current configuration (default)"
            ;;
        *)
            log_error "Unknown command: $command"
            log_info "Use '$0 help' for usage information"
            show_status
            return 1
            ;;
    esac
}

# Run main function if script is executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi

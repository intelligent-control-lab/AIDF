# AIDF Dual ROS Build Instructions

This document provides instructions for building AIDF with both ROS1 and ROS2 support.

## Overview

AIDF now supports both ROS1 (Noetic) and ROS2 (Humble/Iron/Jazzy) through an abstraction layer. The build system automatically detects the ROS version and configures accordingly.

## Prerequisites

### For ROS1 Build
- Ubuntu 20.04 with ROS Noetic
- MoveIt for ROS1
- Required dependencies: `moveit_core`, `moveit_ros_planning`, `moveit_ros_planning_interface`, etc.

### For ROS2 Build  
- Ubuntu 20.04/22.04 with ROS2 Humble/Iron/Jazzy
- MoveIt2 for ROS2
- Required dependencies: `moveit_core`, `moveit_ros_planning`, `moveit_ros_planning_interface`, etc.

## Building

### Important: Package.xml Configuration

Due to limitations in dual ROS support, you need to use the appropriate package.xml for your ROS version:

- **ROS2 build**: The default `package.xml` is configured for ROS2
- **ROS1 build**: Copy `package_ros1.xml` to `package.xml` before building

```bash
# For ROS1 build
cp package_ros1.xml package.xml

# For ROS2 build  
# Use the default package.xml (already ROS2 configured)
```

### ROS1 Build (Traditional Catkin)

```bash
# Set up ROS1 environment
source /opt/ros/noetic/setup.bash

# Use ROS1 package.xml
cp package_ros1.xml package.xml

# Set AIDF root directory (important for path resolution)
export AIDF_ROOT_DIR=/path/to/your/catkin_ws/src/AIDF
source $AIDF_ROOT_DIR/setup_env.sh

# Navigate to catkin workspace
cd /path/to/your/catkin_ws

# Build with catkin
catkin_make

# Or with catkin tools
catkin build aidf
```

### ROS2 Build (Ament)

```bash
# Set up ROS2 environment  
source /opt/ros/humble/setup.bash  # or iron/jazzy

# Ensure ROS2 package.xml is used (default)
# The default package.xml is already configured for ROS2

# Set AIDF root directory (important for path resolution)
export AIDF_ROOT_DIR=/path/to/your/ros2_ws/src/AIDF
source $AIDF_ROOT_DIR/setup_env.sh

# Navigate to ROS2 workspace
cd /path/to/your/ros2_ws

# Build with colcon
colcon build --packages-select aidf

# Source the built workspace
source install/setup.bash
```

## ROS Version Detection

The build system automatically detects ROS version using:

1. `$ROS_VERSION` environment variable (if set)
2. `$ROS_DISTRO` environment variable:
   - `humble`, `iron`, `jazzy`, `rolling` → ROS2
   - `noetic`, `melodic`, etc. → ROS1
3. Default: ROS1 (for backward compatibility)

You can force a specific version:
```bash
export ROS_VERSION=2  # Force ROS2 build
export ROS_VERSION=1  # Force ROS1 build
```

## Build Verification

### Test Path Resolution
```bash
# Run path resolution test
./test_paths.sh
```

### Test Executables

#### ROS1
```bash
# After catkin_make
rosrun aidf plan_lego
rosrun aidf webplan_lego
```

#### ROS2  
```bash
# After colcon build
ros2 run aidf plan_lego
ros2 run aidf webplan_lego
```

## Build Flags

During build, you'll see output indicating the detected ROS version:
```
-- Detected ROS version: 1
-- Building for ROS1
```
or
```
-- Detected ROS version: 2
-- Building for ROS2
```

## Common Issues

### Issue: Wrong ROS version detected
**Solution:** Explicitly set `ROS_VERSION` environment variable

### Issue: Missing dependencies
**Solution:** Ensure all MoveIt packages are installed for your ROS version:

```bash
# ROS1
sudo apt install ros-noetic-moveit

# ROS2  
sudo apt install ros-humble-moveit  # or iron/jazzy
```

### Issue: Path resolution failures
**Solution:** Ensure `AIDF_ROOT_DIR` is set correctly and `setup_env.sh` is sourced

### Issue: "more than one build type" warning in ROS2
**Solution:** This is expected behavior due to dual compatibility - the warning can be ignored

### Issue: Interface generation errors in ROS2
**Solution:** Ensure `package.xml` has the correct ROS2 configuration:
```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Issue: Package.xml format errors
**Solution:** Make sure you're using the correct package.xml for your ROS version:
- ROS2: Use default `package.xml`
- ROS1: Copy `package_ros1.xml` to `package.xml`

## Architecture Notes

- **ROS Abstraction Layer**: Located in `skillgraph/include/ros_compat/` and `skillgraph/src/ros_compat/`
- **Conditional Compilation**: Uses `ROS1_BUILD` and `ROS2_BUILD` macros
- **MoveIt Backend**: Automatically uses correct MoveIt version
- **Launch System**: Abstracted to work with both `roslaunch` (ROS1) and `ros2 launch` (ROS2)

## Development

When developing new ROS-dependent code:

1. Use the abstraction layer interfaces in `ros_compat/`
2. Add conditional compilation where necessary using `#ifdef ROS2_BUILD`
3. Test with both ROS versions
4. Update this documentation for any new requirements

## Troubleshooting

Enable verbose build output:
```bash
# ROS1
catkin_make VERBOSE=1

# ROS2
colcon build --packages-select aidf --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

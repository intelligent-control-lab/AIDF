/*
***********************************************************************************************************************************************************************
Copyright notice for IP Docket # 2022-013, The CFS Library in C++ for Robot Arm Trajectory Planning 2020 Carnegie Mellon University. 
All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written permission.

Rui Chen      ruic3@andrew.cmu.edu
Ruixuan Liu   ruixuanl@andrew.cmu.edu
Weiye Zhao    weiyezha@andrew.cmu.edu
Changliu Liu  cliu6@andrew.cmu.edu

A023793. Sponsor is provided a non-exclusive, world-wide license to the Licensed Technology in the field of robotic painting and arcwelding.

This notice must appear in all copies of this file and its derivatives.
***********************************************************************************************************************************************************************
*/
#pragma once

#if BUILD_TYPE == BUILD_TYPE_DEBUG
    #define DEBUG_PRINT
#else
    #undef DEBUG_PRINT
#endif

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <cstdio>
#include <ctime>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <iterator>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <math.h>
#include <cmath>
#include <iomanip>
#include <map>
#include <unordered_map>
#include <random>
#include <utility>
#include <memory>
#include <chrono>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <queue>
#include <any>

// ROS2 includes  
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
// Note: gazebo_msgs for ROS2 are deferred - only needed for lego functionality
// #include <gazebo_msgs/srv/set_entity_state.hpp>

using namespace std::chrono;

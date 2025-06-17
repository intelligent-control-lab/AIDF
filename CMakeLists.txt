cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(aidf VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              ROS Version Detection                         */
# /* -------------------------------------------------------------------------- */
# Detect ROS version for dual compatibility
if(DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
else()
    # Try to detect from ROS_DISTRO
    if(DEFINED ENV{ROS_DISTRO})
        set(ROS_DISTRO $ENV{ROS_DISTRO})
        if(ROS_DISTRO MATCHES "humble|iron|jazzy|rolling")
            set(ROS_VERSION 2)
        else()
            set(ROS_VERSION 1)
        endif()
    else()
        # Default to ROS1 for backward compatibility
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

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */
# Ensure C++20 is enabled
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_OS_UNIX)
# Check if profiling is needed
option(ENABLE_PROFILING "Enable profiling with gprof" OFF)

if (ENABLE_PROFILING)
    # Add the -pg flag to the compiler and linker flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
endif()


# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)

# Conditional ROS dependency finding
if(ROS_VERSION EQUAL 2)
    # ROS2 dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(moveit_core REQUIRED)
    find_package(moveit_ros_planning REQUIRED)
    find_package(moveit_ros_planning_interface REQUIRED)
    find_package(moveit_visual_tools REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    #find_package(rosidl_default_generators REQUIRED)
    #find_package(rosidl_default_runtime REQUIRED)

    ## ROS2 service generation
    #set(srv_files
    #    "srv/StabilityScore.srv"
    #)

    #rosidl_generate_interfaces(${PROJECT_NAME}
    #    ${srv_files}
    #    DEPENDENCIES geometry_msgs std_msgs
    #    ADD_LINTER_TESTS
    #)
else()
    # ROS1 dependencies
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      rviz_visual_tools
      moveit_visual_tools
      tf2_geometry_msgs
      message_generation
      actionlib
      geometry_msgs
      cv_bridge
      image_transport
    )

    # ROS1 service generation
    add_service_files(
      FILES
      StabilityScore.srv
    )

    generate_messages(
      DEPENDENCIES
      std_msgs
      geometry_msgs
      sensor_msgs
    )
endif()

find_package(yk_msgs QUIET)

# /* -------------------------------------------------------------------------- */
# /*                               Package Configuration                        */
# /* -------------------------------------------------------------------------- */
if(ROS_VERSION EQUAL 2)
    # ROS2 package configuration will be handled by ament
else()
    # ROS1 catkin package configuration
    catkin_package(
      LIBRARIES
      INCLUDE_DIRS
      CATKIN_DEPENDS
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        tf2_geometry_msgs
        message_runtime
        yk_msgs
      DEPENDS
        EIGEN3
    )
endif()

  # /* -------------------------------------------------------------------------- */
  # /*                               Include Directories                          */
  # /* -------------------------------------------------------------------------- */

# Conditional include directories
if(ROS_VERSION EQUAL 2)
    include_directories(
      SYSTEM
      ${OpenCV_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIR}
      ${EIGEN3_INCLUDE_DIRS}
    )
    # ROS2 include directories will be handled by ament_target_dependencies
else()
    include_directories(
      SYSTEM
      ${OpenCV_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIR}
      ${EIGEN3_INCLUDE_DIRS}
    )
endif()

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

# Conditionally include source files based on ROS version
if(ROS_VERSION EQUAL 2)
    # ROS2 build - exclude lego files that have ROS1 dependencies
    file(GLOB src_files
         skillgraph/src/*.cpp
         skillgraph/src/Utils/*.cpp
         skillgraph/src/ros_compat/*.cpp
         planner/src/*.cpp)
    message(STATUS "ROS2 build: Excluding lego/ directory from build")
else()
    # ROS1 build - include all files
    file(GLOB src_files
         skillgraph/src/*.cpp
         skillgraph/src/lego/*.cpp
         skillgraph/src/Utils/*.cpp
         skillgraph/src/ros_compat/*.cpp
         planner/src/*.cpp)
    message(STATUS "ROS1 build: Including all source files")
endif()

# Build core shared library under a unique name to avoid clashes
add_library(${PROJECT_NAME}_lib SHARED ${src_files})

# Conditionally define a macro if yk_tasks is found
if(yk_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE HAVE_YK_TASKS)
else()
  message(WARNING "yk_tasks not found, excluding related code")
endif()


# Conditional linking based on ROS version
if(ROS_VERSION EQUAL 2)
    target_link_libraries(${PROJECT_NAME}_lib
      Eigen3::Eigen
      ${OMPL_LIBRARIES}
      ${Boost_LIBRARIES}
      ${OpenCV_LIBRARIES}
      yaml-cpp
      jsoncpp
    )

    # ROS2 ament dependencies
    ament_target_dependencies(${PROJECT_NAME}_lib
      rclcpp
      std_msgs
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      moveit_visual_tools
      tf2_geometry_msgs
      geometry_msgs
      cv_bridge
      image_transport
    )
else()
    target_link_libraries(${PROJECT_NAME}_lib
      Eigen3::Eigen
      ${catkin_LIBRARIES}
      ${OMPL_LIBRARIES}
      ${Boost_LIBRARIES}
      ${OpenCV_LIBRARIES}
      yaml-cpp
      jsoncpp
    )
endif()

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib OpenMP::OpenMP_CXX)
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/skillgraph/api>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/skillgraph/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)


# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */

add_executable(plan_lego
  exe/plan_lego.cpp
)

if(ROS_VERSION EQUAL 2)
    target_link_libraries(plan_lego
      ${PROJECT_NAME}_lib
    )

    ament_target_dependencies(plan_lego
      rclcpp
      std_msgs
      moveit_core
      moveit_ros_planning_interface
    )
else()
    target_link_libraries(plan_lego
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
endif()

target_include_directories(plan_lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(ROS_VERSION EQUAL 1)
    install(TARGETS plan_lego
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

add_executable(webplan_lego
  exe/webplan_lego.cpp
)

if(ROS_VERSION EQUAL 2)
    target_link_libraries(webplan_lego
      ${PROJECT_NAME}_lib
    )

    ament_target_dependencies(webplan_lego
      rclcpp
      std_msgs
      moveit_core
      moveit_ros_planning_interface
    )
else()
    target_link_libraries(webplan_lego
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
endif()

target_include_directories(webplan_lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(ROS_VERSION EQUAL 1)
    install(TARGETS webplan_lego
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()



# /* -------------------------------------------------------------------------- */
# /*                            Final ROS2 Configuration                        */
# /* -------------------------------------------------------------------------- */
if(ROS_VERSION EQUAL 2)
    # ROS2 installations
    install(TARGETS ${PROJECT_NAME}_lib
        EXPORT export_${PROJECT_NAME}_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install(TARGETS plan_lego webplan_lego
        DESTINATION lib/${PROJECT_NAME}_lib
    )

    install(DIRECTORY skillgraph/api skillgraph/include planner/include
        DESTINATION include
    )

    install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}_lib
    )

    # Export dependencies
    ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
    ament_export_dependencies(
        rclcpp
        std_msgs
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
        tf2_geometry_msgs
        cv_bridge
        image_transport
        #rosidl_default_runtime
    )


    ament_package()
endif()
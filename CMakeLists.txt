cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(aidf VERSION 1.0.0 LANGUAGES CXX)

# This is a ROS 2-only build
message(STATUS "Building for ROS2")

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */
# Ensure C++20 is enabled
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_OS_UNIX)
add_definitions(-DAIDF_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config")
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
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# /* -------------------------------------------------------------------------- */
# /*                               Include Directories                          */
# /* -------------------------------------------------------------------------- */

include_directories(
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add project directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/skillgraph
  ${CMAKE_CURRENT_SOURCE_DIR}/skillgraph/api
  ${CMAKE_CURRENT_SOURCE_DIR}/skillgraph/include
  ${CMAKE_CURRENT_SOURCE_DIR}/planner/include
)

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

# ROS2 build - exclude lego files that have ROS1 dependencies
file(GLOB src_files
     skillgraph/src/*.cpp
     skillgraph/src/Utils/*.cpp
     skillgraph/src/magblock/*.cpp
     planner/src/*.cpp)

# Build core shared library
add_library(${PROJECT_NAME}_lib SHARED ${src_files})

# Conditionally define a macro if yk_tasks is found
if(yk_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE HAVE_YK_TASKS)
else()
  message(WARNING "yk_tasks not found, excluding related code")
endif()

# ROS2 linking
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
  std_srvs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2_geometry_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

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

# TODO: Re-enable Lego executables after completing ROS 2 migration
# add_executable(plan_lego
#   exe/plan_lego.cpp
# )

# target_link_libraries(plan_lego
#   ${PROJECT_NAME}_lib
# )

# ament_target_dependencies(plan_lego
#   rclcpp
#   std_msgs
#   moveit_core
#   moveit_ros_planning_interface
# )

# target_include_directories(plan_lego PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   PRIVATE src)

# add_executable(webplan_lego
#   exe/webplan_lego.cpp
# )

# target_link_libraries(webplan_lego
#   ${PROJECT_NAME}_lib
# )

# ament_target_dependencies(webplan_lego
#   rclcpp
#   std_msgs
#   moveit_core
#   moveit_ros_planning_interface
# )

# target_include_directories(webplan_lego PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   PRIVATE src)



# /* -------------------------------------------------------------------------- */
# /*                            ROS2 Configuration                              */
# /* -------------------------------------------------------------------------- */

# ROS2 installations
install(TARGETS ${PROJECT_NAME}_lib
    EXPORT export_${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# TODO: Re-enable when Lego executables are restored
# install(TARGETS plan_lego webplan_lego
#     DESTINATION lib/${PROJECT_NAME}_lib
# )

install(DIRECTORY skillgraph/api skillgraph/include planner/include
    DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)


install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
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
)

# /* -------------------------------------------------------------------------- */
# /*                              Test Executables                               */
# /* -------------------------------------------------------------------------- */

# MagBlock executables
add_executable(plan_magblock exe/plan_magblock.cpp)
target_link_libraries(plan_magblock ${PROJECT_NAME}_lib yaml-cpp)
ament_target_dependencies(plan_magblock
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)


# MagBlock Assembly Executor executable (DISABLED - use magblock_assembly_test instead)
# add_executable(magblock_assembly_executor src/magblock_assembly_executor.cpp)
# target_link_libraries(magblock_assembly_executor ${PROJECT_NAME}_lib yaml-cpp)
# ament_target_dependencies(magblock_assembly_executor
#   rclcpp
#   moveit_core
#   moveit_ros_planning_interface
#   moveit_visual_tools
#   geometry_msgs
# )

# MagBlock Assembly Test executable
add_executable(magblock_assembly_test exe/magblock_assembly_test.cpp)
target_link_libraries(magblock_assembly_test ${PROJECT_NAME}_lib yaml-cpp)
ament_target_dependencies(magblock_assembly_test
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)

# Install MagBlock executables
install(TARGETS plan_magblock magblock_assembly_test
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

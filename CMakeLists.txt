cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(aidf VERSION 1.0.0 LANGUAGES CXX)

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

find_package(yk_msgs QUIET)

add_service_files(
  FILES
  StabilityScore.srv
  # SetPose.srv
  # GetPose.srv
  # SetJoints.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# /* -------------------------------------------------------------------------- */
# /*                               Catkin Package                               */
# /* -------------------------------------------------------------------------- */
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

  # /* -------------------------------------------------------------------------- */
  # /*                               Include Directories                          */
  # /* -------------------------------------------------------------------------- */

include_directories(
  SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  )

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */
file(GLOB src_files skillgraph/src/*.cpp skillgraph/src/lego/*.cpp
          skillgraph/src/Utils/*.cpp planner/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})


# Conditionally define a macro if yk_tasks is found
if(yk_msgs_FOUND)
  message(STATUS "yk_tasks found, defining HAVE_YK_TASKS")
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_YK_TASKS)
else()
  message(WARNING "yk_tasks not found, excluding related code")
endif()


target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  jsoncpp
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/skillgraph/api>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/skillgraph/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)


# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */
 
add_executable(plan_lego
  exe/plan_lego.cpp
)

target_link_libraries(plan_lego
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_include_directories(plan_lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

target_link_libraries(plan_lego
  ${catkin_LIBRARIES}
)

install(TARGETS plan_lego
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(webplan_lego
  exe/webplan_lego.cpp
)

target_link_libraries(webplan_lego
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_include_directories(webplan_lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

target_link_libraries(webplan_lego
  ${catkin_LIBRARIES}
)

install(TARGETS webplan_lego
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




add_executable(${PROJECT_NAME}_integrate_task_planning_chaitanya
  skillgraph/src/lego/lego_integrate_task_planning_chaitanya.cpp
)

add_dependencies(${PROJECT_NAME}_integrate_task_planning_chaitanya
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_integrate_task_planning_chaitanya
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
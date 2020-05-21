cmake_minimum_required(VERSION 2.4.6)
project(tw11)

# Find and get all the information about the rospy package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    markers_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    message_generation )

## Generate the services
add_service_files(
  FILES
  StartCharging.srv
)

## Generate actions messages interfaces
add_action_files(
  DIRECTORY action_defs
  FILES Move2Pos.action
  FILES Recharge.action
  FILES Rotate2Angle.action
  FILES Stop.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

# Declare the project as a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime roscpp actionlib_msgs
)

# Include directories for C++
include_directories(include ${catkin_INCLUDE_DIRS})

# C++
add_executable(battery_manager
               src/battery_manager.cpp
               src/LocalFrameWorldFrameTransformations.cpp)
target_link_libraries(battery_manager ${catkin_LIBRARIES})
add_dependencies(sim_control tw11_gencpp)

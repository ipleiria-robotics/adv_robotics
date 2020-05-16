cmake_minimum_required(VERSION 2.8.3)
project(lw2_sim)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs std_msgs message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  StartCharging.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp geometry_msgs sensor_msgs
#    LIBRARIES lib
#    DEPENDS
)

add_executable(sim_control
               src/sim_control.cpp
               src/LocalFrameWorldFrameTransformations.cpp)
target_link_libraries(sim_control ${catkin_LIBRARIES})
add_dependencies(sim_control lw2_sim_gencpp)

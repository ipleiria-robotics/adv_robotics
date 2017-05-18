cmake_minimum_required(VERSION 2.8.3)
project(factory_gazebo)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp gazebo_ros
    LIBRARIES lib
)

#add_executable(publisher src/publisher.cc)
#target_link_libraries(publisher ${catkin_LIBRARIES})

#find_package(Boost REQUIRED COMPONENTS)
#include_directories(${Boost_INCLUDE_DIRS})

# Plugin to control the forklift from ROS
add_library(forklift_plugin src/forklift_plugin.cpp)
target_link_libraries(forklift_plugin ${catkin_LIBRARIES} ${gazebo_LIBRARIES})

# Plugin for the "boxes" to change according to their position and orientation
add_library(tbox_plugin src/tbox_plugin.cpp)
target_link_libraries(tbox_plugin ${catkin_LIBRARIES} ${gazebo_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(gazebo_factory)

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
#add_library(forklift_plugin src/forklift_plugin.cpp)
#target_link_libraries(forklift_plugin ${catkin_LIBRARIES} ${gazebo_LIBRARIES} ${Boost_LIBRARIES})

add_library(tbox_plugin src/tbox_plugin.cpp)
target_link_libraries(tbox_plugin ${catkin_LIBRARIES} ${gazebo_LIBRARIES})

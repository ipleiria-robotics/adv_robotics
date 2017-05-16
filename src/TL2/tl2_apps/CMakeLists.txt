cmake_minimum_required(VERSION 2.8.3)
project(tl2_apps)

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
  message_generation roscpp geometry_msgs std_msgs robot_state_publisher cv_bridge
)

# Add OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp robot_state_publisher geometry_msgs sensor_msgs
    LIBRARIES lib
    DEPENDS opencv cv_bridge image_transport
)

add_executable(tl2_teleop src/tl2_teleop.cpp)
target_link_libraries(tl2_teleop ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(ta2_localization src/ta2_localization.cpp)
#target_link_libraries(ta2_localization ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(ta2_navigation src/ta2_navigation.cpp)
#target_link_libraries(ta2_navigation ${catkin_LIBRARIES} ${OpenCV_LIBS})

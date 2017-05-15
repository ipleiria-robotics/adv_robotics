cmake_minimum_required(VERSION 2.4.6)
project(tp10)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

# Find and get all the information about the roscpp package
find_package(catkin REQUIRED COMPONENTS roscpp markers_msgs)

# Declare the project as a catkin package
catkin_package()

# Find OpenCV
find_package(OpenCV)

# Add header folders
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
 
# Define an executable target called tp1
add_executable(tp10
               TP10.cpp
               main.cpp
               ExtendedKalmanFilter.cpp
               utils.cpp
               LocalFrameWorldFrameTransformations.cpp)

# Link the target against the libraries used by roscpp
target_link_libraries(tp10 ${catkin_LIBRARIES} ${OpenCV_LIBS})
# Make sure markers_msgs is compiled, as we need it
add_dependencies(tp10 markers_msgs_generate_messages_cpp)

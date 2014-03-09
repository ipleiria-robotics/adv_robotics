cmake_minimum_required(VERSION 2.4.6)
project(tp01)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

# Find and get all the information about the roscpp package
find_package(catkin REQUIRED COMPONENTS roscpp)

# Declare the project as a catkin package
catkin_package()

# Add header folders
include_directories(include ${roscpp_INCLUDE_DIRS})
 
# Define an executable target called tp1
add_executable(tp01 main.cpp)

# Link the target against the libraries used by roscpp
target_link_libraries(tp01 ${roscpp_LIBRARIES})

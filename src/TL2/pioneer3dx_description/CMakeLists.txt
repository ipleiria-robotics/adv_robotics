cmake_minimum_required(VERSION 2.8.3)
project(pioneer3dx_description)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
)

cmake_minimum_required(VERSION 2.4.6)
project(markers_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

# Find and get all the information about the roscpp package
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

# Add message files
add_message_files(
  FILES
  Markers.msg
)

# Generate messages
generate_messages(DEPENDENCIES std_msgs)

# Declare the project as a catkin package
catkin_package(CATKIN_DEPENDS message_runtime )


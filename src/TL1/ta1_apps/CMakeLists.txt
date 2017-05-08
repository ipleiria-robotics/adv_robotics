cmake_minimum_required(VERSION 2.4.6)
project(ta1_apps)

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
find_package(OpenCV REQUIRED)

# Add header folders
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
 
# Define an executable target
add_executable(ta1_localization
               ta1_localization.cpp
               LocalFrameWorldFrameTransformations.cpp)
# Link the target against the libraries used by roscpp
target_link_libraries(ta1_localization ${catkin_LIBRARIES} ${OpenCV_LIBS})
# Make sure markers_msgs is compiled, as we need it
add_dependencies(ta1_localization markers_msgs_generate_messages_cpp)

# Define an executable target
add_executable(ta1_map
               ta1_map.cpp
               LocalFrameWorldFrameTransformations.cpp)
# Link the target against the libraries used by roscpp
target_link_libraries(ta1_map ${catkin_LIBRARIES} ${OpenCV_LIBS})
# Make sure markers_msgs is compiled, as we need it
add_dependencies(ta1_map markers_msgs_generate_messages_cpp)

# Define an executable target
add_executable(ta1_navastar
               ta1_navastar.cpp
               LocalFrameWorldFrameTransformations.cpp)
# Link the target against the libraries used by roscpp
target_link_libraries(ta1_navastar ${catkin_LIBRARIES} ${OpenCV_LIBS})
# Make sure markers_msgs is compiled, as we need it
add_dependencies(ta1_navastar markers_msgs_generate_messages_cpp)


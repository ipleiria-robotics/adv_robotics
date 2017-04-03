cmake_minimum_required(VERSION 2.4.6)
project(tp07)

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

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add header folders
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
 
# Executable 1
add_executable(tp07_ex1 ex1.cpp)
target_link_libraries(tp07_ex1 ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Executable 2
add_executable(tp07_ex2 ex2.cpp)
target_link_libraries(tp07_ex2 ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Executable 3
add_executable(tp07_ex3 ex3.cpp)
target_link_libraries(tp07_ex3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Executable 4
add_executable(tp07_ex4 ex4.cpp)
target_link_libraries(tp07_ex4 ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Executable 5
add_executable(tp07_ex5 ex5.cpp)
target_link_libraries(tp07_ex5 ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Executable 6
add_executable(tp07_ex6 ex6.cpp)
target_link_libraries(tp07_ex6 ${catkin_LIBRARIES} ${OpenCV_LIBS})

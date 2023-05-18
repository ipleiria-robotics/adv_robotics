cmake_minimum_required(VERSION 3.5)
project(ar_cpp_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ar_utils REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install include files
install(DIRECTORY include/ DESTINATION include)

# Create Cpp executable for the battery_manager
add_executable(battery_manager
               src/battery_manager.cpp
               src/LocalFrameWorldFrameTransformations.cpp)
target_include_directories(battery_manager PUBLIC include)
ament_target_dependencies(battery_manager rclcpp nav_msgs sensor_msgs geometry_msgs tf2_geometry_msgs ar_utils)
# Install Cpp executable
install(TARGETS battery_manager DESTINATION lib/${PROJECT_NAME})

# Create Cpp executable for the sim_control
add_executable(sim_control
               src/sim_control.cpp
               src/LocalFrameWorldFrameTransformations.cpp)
target_include_directories(sim_control PUBLIC include)
ament_target_dependencies(sim_control rclcpp tf2 tf2_ros tf2_geometry_msgs ar_utils)
# Install Cpp executable
install(TARGETS sim_control DESTINATION lib/${PROJECT_NAME})

ament_package()

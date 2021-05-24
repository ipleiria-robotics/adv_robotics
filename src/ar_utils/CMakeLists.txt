cmake_minimum_required(VERSION 3.5)
project(ar_utils)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Message generation
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartCharging.srv"
  "action/Move2Pos.action"
  "action/PlaySound.action"
  "action/Recharge.action"
  "action/Rotate2Angle.action"
  "action/Stop.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs
 )

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

# Create Cpp executable
add_executable(battery_manager
               src/battery_manager.cpp
               src/LocalFrameWorldFrameTransformations.cpp)
target_include_directories(battery_manager PUBLIC include)
ament_target_dependencies(battery_manager rclcpp)
# Link against the generated interfaces
rosidl_target_interfaces(battery_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Install Cpp executable
install(TARGETS battery_manager DESTINATION lib/${PROJECT_NAME})

# Exports
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(ar_utils)

ament_export_include_directories(include)
  


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/py_node.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()

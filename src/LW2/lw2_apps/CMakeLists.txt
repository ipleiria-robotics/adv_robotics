cmake_minimum_required(VERSION 2.4.6)
project(lw2_apps)

# Find and get all the information about the rospy package
find_package(catkin REQUIRED COMPONENTS rospy markers_msgs)

# Declare the project as a catkin package
catkin_package()

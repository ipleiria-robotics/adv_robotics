#!/bin/sh

echo "Installing ROS Indigo on Ubuntu 14.04 Trusty"
## This commands were taken from http://www.ros.org/wiki/hydro/Installation/Ubuntu
## This script must run using sudo

# Check if this script is being run as root
if [ `whoami` != "root" ]
then
  echo "This script must be ran as root. Use: sudo ./install_ROS.sh"
  exit 1
fi

# Add the ROS sources to apt
echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

# Add ROS keys
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -

# Update the APT sources
apt-get update

# Perform full ROS installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
apt-get -y install ros-indigo-desktop-full
# Install additional ros packages
apt-get -y install ros-indigo-hector-quadrotor ros-indigo-turtlebot-simulator ros-indigo-ros-control ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-gazebo-ros-control

rosdep init


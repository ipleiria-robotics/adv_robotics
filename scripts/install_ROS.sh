#!/bin/sh

echo "Installing ROS noetic on Ubuntu $(lsb_release -sc)"
## This commands were taken from http://www.ros.org/wiki/noetic/Installation/Ubuntu
## This script must run using sudo

# Check if this script is being run as root
if [ `whoami` != "root" ]
then
  echo "This script must be ran as root. Use: sudo ./install_ROS.sh"
  exit 1
fi

# Add the ROS sources to apt
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Add ROS keys
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -

# Update the APT sources
apt update

# Perform full ROS installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
apt -y install ros-noetic-desktop ros-noetic-stage-ros
# Install additional ros packages
# ros-noetic-turtlebot-simulator
#apt -y install ros-noetic-hector-sensors-description ros-noetic-hector-models ros-noetic-ros-control ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs
apt -y install ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-laser-filters
#apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
#rosdep init




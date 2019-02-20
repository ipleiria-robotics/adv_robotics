#!/bin/sh

echo "Installing ROS Melodic on Ubuntu $(lsb_release -sc)"
## This commands were taken from http://www.ros.org/wiki/melodic/Installation/Ubuntu
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
#The following is not working currently, so use the older method
#apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -

# Update the APT sources
apt update

# Perform full ROS installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
apt -y install ros-melodic-desktop ros-melodic-stage-ros
# Install additional ros packages
# ros-melodic-turtlebot-simulator
apt -y install ros-melodic-hector-sensors-description ros-melodic-hector-models ros-melodic-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs
apt -y install ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-laser-filters
apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep init




#!/bin/sh

echo "Installing ROS hydro on Ubuntu 12.04 Precise"
## This commands were taken from http://www.ros.org/wiki/hydro/Installation/Ubuntu
## This script must run using sudo

# Check if this script is being run as root
if [ `whoami` != "root" ]
then
  echo "This script must be ran as root. Use: sudo ./install_ROS.sh"
  exit 1
fi

# Add the ROS sources to apt
echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list

# Add ROS keys
wget http://packages.ros.org/ros.key -O - | apt-key add -

# Update the APT sources
apt-get update

# Perform full ROS installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
apt-get -y install ros-hydro-desktop-full
# Install additional ros packages
apt-get -y install ros-hydro-hector-quadrotor ros-hydro-turtlebot-simulator
# ros-fuerte-p2os ros-fuerte-erratic-robot ros-fuerte-pr2-teleop-app


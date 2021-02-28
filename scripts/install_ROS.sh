#!/bin/sh

echo "Installing ROS2 Foxy Fitzroy on Ubuntu $(lsb_release -sc)"
## This script must run using sudo
## Most information came from https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

# Check if this script is being run as root
if [ `whoami` != "root" ]
then
  echo "This script must be ran as root. Use: sudo ./install_ROS.sh"
  exit 1
fi

# Add ROS2 keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Add the ROS2 sources to apt
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Update the APT sources
apt update

# Perform ROS2 desktop installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
sudo apt -y install ros-foxy-desktop
# Install additional ros packages
##apt -y install ros-melodic-hector-sensors-description ros-melodic-hector-models ros-melodic-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs
##apt -y install ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-laser-filters
##apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep init




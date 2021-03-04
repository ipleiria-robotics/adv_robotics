#!/bin/sh

<<<<<<< HEAD
echo "Installing ROS2 Foxy Fitzroy on Ubuntu $(lsb_release -sc)"
=======
echo "Installing ROS noetic on Ubuntu $(lsb_release -sc)"
## This commands were taken from http://www.ros.org/wiki/noetic/Installation/Ubuntu
>>>>>>> 9bbb30263b13a580d2426fb96a8979a3f368ba27
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

<<<<<<< HEAD
# Add the ROS2 sources to apt
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list
=======
# Add ROS keys
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
>>>>>>> 9bbb30263b13a580d2426fb96a8979a3f368ba27

# Update the APT sources
apt update

# Perform ROS2 desktop installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
<<<<<<< HEAD
sudo apt -y install ros-foxy-desktop ros-foxy-plotjuggler-ros
# Install additional ros packages
##apt -y install ros-melodic-hector-sensors-description ros-melodic-hector-models ros-melodic-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs
##apt -y install ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-laser-filters
##apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep init
=======
apt -y install ros-noetic-desktop ros-noetic-stage-ros
# Install additional ros packages
# ros-noetic-turtlebot-simulator
#apt -y install ros-noetic-hector-sensors-description ros-noetic-hector-models ros-noetic-ros-control ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs
apt -y install ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-laser-filters
#apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
#rosdep init
>>>>>>> 9bbb30263b13a580d2426fb96a8979a3f368ba27




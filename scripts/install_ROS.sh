#!/bin/sh

echo "Installing ROS 2 Humble Hawksbill on Ubuntu $(lsb_release -sc)"
## This script must run using sudo
## Most information came from https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Check if this script is being run as root
if [ `whoami` != "root" ]
then
  echo "This script must be ran as root. Use: sudo ./install_ROS.sh"
  exit 1
fi

apt -y install software-properties-common
add-apt-repository universe

# Add ROS2 keys
apt update && sudo apt -y install curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS2 sources to apt
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update the APT sources and do any upgrades
apt update
apt -y upgrade

# Perform ROS2 desktop installation
# In systems with low memory, installing everything simultaneously can lead to 
#problems. To prevent this, we should first install texlive, then proceed with ROS.
apt -y install ros-humble-desktop ros-humble-plotjuggler-ros ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager ros-humble-nav2-bringup ros-humble-ament-cmake-nose
# Install additional ros packages
apt -y install ros-humble-py-trees-ros ros-humble-py-trees-ros-interfaces ros-humble-rqt-tf-tree ros-humble-simple-launch
##apt -y install ros-melodic-hector-sensors-description ros-melodic-hector-models ros-melodic-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs
##apt -y install ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-laser-filters
##apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Relevant ROS-related packages
apt -y install python3-colcon-common-extensions python3-catkin-pkg-modules python3-rospkg-modules python3-rosdep
apt -y install ros-dev-tools

rosdep init




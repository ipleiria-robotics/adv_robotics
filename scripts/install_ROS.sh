#!/bin/sh

echo "Installing ROS 2 Jazzy Jalisco on Ubuntu $(lsb_release -sc)"
## This script must run using sudo
## Most information came from https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

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
apt -y install ros-jazzy-desktop ros-jazzy-plotjuggler-ros ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-bringup
# ros-jazzy-ament-cmake-nose

# Install additional ros packages
apt -y install ros-jazzy-py-trees-ros ros-jazzy-py-trees-ros-interfaces ros-jazzy-py-trees-ros-viewer
apt -y install ros-jazzy-rqt-tf-tree ros-jazzy-simple-launch ros-jazzy-tf-transformations

# Relevant ROS-related packages
apt -y install python3-colcon-common-extensions python3-catkin-pkg-modules python3-rospkg-modules python3-rosdep
apt -y install ros-dev-tools

rosdep init




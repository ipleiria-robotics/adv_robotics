#!/bin/sh

echo "Updating Gazebo7 on Ubuntu $(lsb_release -sc)"
## This script must NOT be run using sudo

# Check if this script is being run as root
if [ `whoami` = "root" ]
then
  echo "This script cannot ran as root."
  exit 1
fi

# Make sure the missing packages are installed
sudo apt-get update
sudo apt-get -y install ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

# Get OSRF repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
# Update gazebo packages
sudo apt-get -y dist-upgrade

# Make from scratch the home build
cd ~/ros
rm -rf devel build
catkin_make

echo "DONE!"

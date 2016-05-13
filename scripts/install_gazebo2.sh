#!/bin/sh

echo "Installing Gazebo2 on Ubuntu $(lsb_release -sc)"
## This commands were taken from http://www.ros.org/wiki/hydro/Installation/Ubuntu
## This script must run using sudo

# Check if this script is being run as root
if [ `whoami` != "root" ]
then
  echo "This script must be ran as root. Use: sudo ./install_gazebo2.sh"
  exit 1
fi

apt-get update
apt-get -y remove ros-indigo-gazebo7-ros gazebo7-doc libgdal-doc libgts-doc libhdf4-doc hdf4-tools
apt-get -y install ppa-purge
ppa-purge -p ubuntu-stable -s packages.osrfoundation.org gazebo
rm /etc/apt/sources.list.d/gazebo-stable.list
apt-get update

apt-get -y install ros-indigo-hector-quadrotor ros-indigo-turtlebot-simulator ros-indigo-ros-control ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-gazebo-ros-control ros-indigo-desktop-full

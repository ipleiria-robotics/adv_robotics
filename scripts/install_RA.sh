#!/bin/bash

# Install and set all the needed packages for the Advanced Robotic class
# This script assumes you have an Ubuntu based installation (Mint, Ubuntu, Kubuntu, or any other *buntu).
# You should have at least 3Gb of space free in your installation folder.

# Software needed for the class (other than ROS)
echo "--> Installing software needed for the Advanced Robotics class"
echo "--> This installation will download ~1GB of software, and should take a while"
echo "--> Type the root password if asked, sit back, and relax..."
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install qtcreator screen mesa-utils nano firefox vlc vlc-plugin-pulse browser-plugin-vlc xterm
sudo apt-get -y install texlive-latex-base gimp unrar git kwrite libfltk1.1-dev kde-workspace-randr
sudo apt-get -y remove kate

# ROS installation
echo "Downloading the install ROS script"
wget https://raw.githubusercontent.com/ipleiria-robotics/adv_robotics/master/scripts/install_ROS.sh
echo "Installing ROS (you might need to type the root password)..."
sudo sh install_ROS.sh
rosdep update

# Gazeo installation
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install ros-indigo-gazebo7-ros gazebo7-doc libgdal-doc libgts-doc libhdf4-doc hdf4-tools

# Set the user environment to use ROS
echo "Setting ROS environment variables..." 
#if [ ! -f ~/.bashrc ]
#then
#  echo ". /opt/ros/indigo/setup.bash" >> ~/.bashrc
#fi
echo "" >> ~/.bashrc
echo "# ROS Environment variables" >> ~/.bashrc
echo ". /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo ". $HOME/ros/devel/setup.bash" >> ~/.bashrc
git clone https://github.com/ipleiria-robotics/adv_robotics ~/ros
source "/opt/ros/indigo/setup.bash"
catkin_make -C ~/ros

# Some updates might need a restart, so lets do it just to be safe
echo
echo "NOTE: You should restart for changes to take effect!"



#!/bin/bash

# Install and set all the needed packages for the Advanced Robotic class
# This script assumes you have an Ubuntu based installation (Mint, Ubuntu, Kubuntu, or any other *buntu).
# You should have at least 3Gb of space free in your installation folder.

# Software needed for the class (other than ROS)
echo "--> Installing software needed for the Advanced Robotics class"
echo "--> This installation will download ~1GB of software and install ~3Gb, and should take a while"
echo "--> Type the root password if asked, sit back, and relax..."

# IDE (see https://code.visualstudio.com/docs/setup/linux)
sudo apt update
sudo apt -y install curl
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt -y install apt-transport-https
sudo apt -y install code

# Other needed software
sudo apt -y upgrade
sudo apt -y install screen mesa-utils nano firefox vlc browser-plugin-vlc xterm
sudo apt -y install python3-pip python3-catkin-pkg-modules python3-empy python3-rospkg-modules python3-flake8 python3-pep8 python3-numpy python3-opencv python3-matplotlib python3-scipy
sudo apt -y install gimp unrar git gdb open-vm-tools open-vm-tools-desktop
#sudo apt kde-workspace-randr kwrite texlive-latex-base vlc-plugin-pulse 
#sudo apt -y remove kate
# libfltk1.1-dev

# Gazebo installation
#sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#sudo apt-get update
#sudo apt-get -y install ros-kinetic-gazebo9-ros gazebo9-doc ros-kinetic-gazebo9-ros-control
#libgdal-doc libgts-doc libhdf4-doc hdf4-tools
#sudo apt-get -y install gazebo7 gazebo7-plugin-base gazebo7-common libgazebo7

# ROS installation
echo "Downloading the install ROS script"
wget https://raw.githubusercontent.com/ipleiria-robotics/adv_robotics/master/scripts/install_ROS.sh
echo "Installing ROS (you might need to type the root password)..."
sudo sh install_ROS.sh
rosdep update

# Set up variables and Clone our gir repository, if not already cloned
if [ ! -d "$HOME/ros" ]; then
  echo "Setting ROS environment variables..." 
  echo "" >> $HOME/.bashrc
  echo "# ROS Environment variables" >> $HOME/.bashrc
  echo ". /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
  echo ". $HOME/ros/devel/setup.bash" >> $HOME/.bashrc
  echo "ROS_PYTHON_VERSION=3" >> $HOME/.bashrc
  git clone https://github.com/ipleiria-robotics/adv_robotics $HOME/ros
else
  echo "$HOME/ros already exists, proceeding..." 
fi
source "/opt/ros/melodic/setup.bash"

export ROS_PYTHON_VERSION=3
catkin_make -C $HOME/ros
source "$HOME/ros/devel/setup.bash"

# Qt
#echo "Downloading update QtCreator script"
#wget https://raw.githubusercontent.com/ipleiria-robotics/adv_robotics/master/scripts/update_qtcreator.sh
#sudo sh update_qtcreator.sh


# Some updates might need a restart, so lets do it just to be safe
echo
echo "Done! You can now proceed with the Advanced Robotics classes..."



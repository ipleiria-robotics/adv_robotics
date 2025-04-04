#!/bin/bash

# Install and set all the needed packages for the Advanced Robotic class
# This script assumes you have an Ubuntu based installation (Mint, Ubuntu, Kubuntu, or any other *buntu).
# You should have at least 3Gb of space free in your installation folder.

# Software needed for the class (other than ROS)
echo "--> Installing software needed for the Advanced Robotics class"
echo "--> This installation will download ~1GB of software and install ~3Gb, and should take a while"
echo "--> Type the root password if asked, sit back, and relax..."

sudo apt update
sudo apt -y upgrade

# Other needed software
sudo apt -y install python3-pip python3-empy python3-flake8 python3-pep8 python3-numpy python3-opencv python3-matplotlib
sudo apt -y install python3-scipy python3-argcomplete python3-skimage python3-ruamel.yaml python3-pykdl
sudo apt -y install git gimp unrar vlc firefox screen kdiff3 curl synaptic alsa-utils espeak-ng mbrola-us1 unzip
#sudo apt -y install  mesa-utils libgirepository1.0-dev
#sudo apt -y install    gdb open-vm-tools open-vm-tools-desktop python3-virtualenv cc
#sudo apt kde-workspace-randr kwrite texlive-latex-base vlc-plugin-pulse 
#sudo apt -y remove kate

# Stage related
sudo apt -y install libfltk1.3-dev

# Visual Studio Code IDE (see https://code.visualstudio.com/docs/setup/linux)
snap install --classic code
#echo "code code/add-microsoft-repo boolean true" | sudo debconf-set-selections
#sudo apt update
#sudo apt install code

# Transformations using pip and a virtual environment
#pip3 install pytransform3d
#mkdir -p ~/.venvs
#python3 -m venv ~/.venvs/advrob --system-site-packages
#. $HOME/.venvs/advrob/bin/python -m pip install pytransform3d

# Gazebo installation
#sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#sudo apt-get update
#sudo apt-get -y install ros-kinetic-gazebo9-ros gazebo9-doc ros-kinetic-gazebo9-ros-control
#libgdal-doc libgts-doc libhdf4-doc hdf4-tools
#sudo apt-get -y install gazebo7 gazebo7-plugin-base gazebo7-common libgazebo7

# ROS installation
echo "Downloading the install ROS script"
wget https://github.com/ipleiria-robotics/adv_robotics/raw/master/scripts/install_ROS.sh
echo "Installing ROS (you might need to type the root password)..."
sudo sh install_ROS.sh
rosdep update

# Set up variables and Clone our gir repository, if not already cloned
if [ ! -d "$HOME/ros" ]; then
  if ! grep -q ROS "$HOME/.bashrc"; then
    echo "Setting ROS environment variables..." 
    echo "" >> $HOME/.bashrc
    echo "# ROS Environment variables" >> $HOME/.bashrc
    echo ". /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
    echo ". $HOME/ros/install/setup.bash" >> $HOME/.bashrc
    #echo "# Activate our virtual environment" >> $HOME/.bashrc
    #echo ". $HOME/.venvs/advrob/bin/activate" >> $HOME/.bashrc
    #echo "# Make sure ROS2 as access to our virtual environment modules (TODO: get the python version automatically)" >> $HOME/.bashrc
    #echo "export PYTHONPATH=\"${PYTHONPATH}:${HOME}/.venvs/advrob/lib/python3.12/site-packages/\"" >> $HOME/.bashrc
  fi
  git clone --recurse-submodules https://github.com/ipleiria-robotics/adv_robotics $HOME/ros
else
  echo "$HOME/ros already exists, proceeding..." 
fi

# Build our workspace
source "/opt/ros/jazzy/setup.bash"
# Activate Python virtual environment
#source "$HOME/.venvs/advrob/bin/activate"
cd $HOME/ros
colcon build --symlink-install
source "$HOME/ros/install/setup.bash"

# Qt
#echo "Downloading update QtCreator script"
#wget https://raw.githubusercontent.com/ipleiria-robotics/adv_robotics/master/scripts/update_qtcreator.sh
#sudo sh update_qtcreator.sh


# Some updates might need a restart, so lets do it just to be safe
echo
echo "If no errors appeared above, you are done! Restart your PC and proceed with the Advanced Robotics classes..."



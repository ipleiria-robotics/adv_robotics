#!/bin/bash

# We need to update qtcreator, since the version included in Kubuntu has some problems
echo "--> Downloading updated Qt"
wget http://download.qt.io/official_releases/qt/5.5/5.5.1/qt-opensource-linux-x86-5.5.1.run
echo "--> Removing old QtCreator"
sudo apt-get -y remove qtcreator qtcreator-doc qtcreator-plugin-remotelinux qtcreator-plugin-valgrind qtcreator-plugin-qnx qtcreator-plugin-cmake
echo "--> Updating GDB"
#sudo apt-get -y install gdb
chmod +x qt-opensource-linux-x64-5.5.1.run
echo "--> Updating qt. Accept all default options in the window"
sudo ./qt-opensource-linux-x64-5.5.1.run
sudo ln -s /opt/Qt5.5.1/Tools/QtCreator/bin/qtcreator /usr/local/bin/

# Done
echo
echo "QtCreator Installed"



Advanced Robotics
============

Repository for the Advanced Robotics class, taught at the School of Tecnhology and Management, Polytechnic of Leiria, Portugal.

Content currently being updated for the 2nd semester of 2023/2024 with ROS2 Humble and Ubuntu 22.04. Also running and tested on WSL2. For previous years, check the corresponding tags.

Created and maintained by Hugo Costelha <hugo.costelha@ipleiria.pt>

Folder Contents
===============

 - scripts: folder with installation scripts. Download scripts install_RA.sh and install_ROS.sh and run "bash install_RA.sh". If you are on WSL2, run "bash install_RA_WSL.sh" instead;
 - src: folder with the files used in the laboratory tutorial works;
 
Getting the tutorials
=====================

If you installed the class software using the provided scripts, this step is already done and you can skip to the next one, otherwise use the following command:

    >> git clone --recurse-submodules https://github.com/ipleiria-robotics/adv_robotics ~/ros

If you already had a folder named "ros" in your home folder, this will fail. You should delete the previous folder and run the command again.

Updating the tutorials
=====================

If you have already set up the installation and tutorials, you can always update your workspace with the latest content by using the following command:

    >> cd ~/ros
    >> git pull
  
Setting up your software environment
====================================

Follow the instructions [here](https://github.com/ipleiria-robotics/adv_robotics/blob/master/docs/software_install-PT.md), for Portuguese, and [here](https://github.com/ipleiria-robotics/adv_robotics/blob/master/docs/software_install-EN.md), for English, to setup your work environment.
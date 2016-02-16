adv-robotics
============

Repository for the Advanced Robotics class, taught at School of Tecnhology and Management, Polytechnic Institute of Leiria, Portugal.

Content currently being updated for the 2nd semester of 2015/2016 with ROS Indigo.

Folder Contents
===============

 - Scripts: folder with installation scripts. Download scripts install_RA.sh and run "sh install_RA.sh";
 - src: folder with the files used in the laboratory tutorials;
 - worlds: folder with stage and gazebo world models;
 
Getting the tutorials
=====================

If you installed the class software using the provided scripts, this step is already done and you can skip to the next one, otherwise use the following command:

    >> git clone https://github.com/ipleiria-robotics/adv_robotics ~/ros

If you already had a folder named "ros" in your home folder, this will fail. You should delete the previous folder and run the command again.

Updating the tutorials
=====================

If you have already set up the installation and tutorials, you can always update your workspace with the latest content by using the following command:

    >> cd ~/ros
    >> git pull
  
Make sure you have the following packages installed: ros-indigo-ros-control, ros-indigo-effort-controllers, ros-indigo-joint-state-controller

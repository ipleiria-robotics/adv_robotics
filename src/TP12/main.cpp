/*
Copyright (c) 2013, Hugo Costelha
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the Player Project nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Headers to read and write from the terminal
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <fstream>

// ROS API
//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h> // Velocity messages

// Coordinate transformations
#include "TP12.hpp"

/**
 * Main function
 * Navigate randomly using distance sensor data and Particle Filter-based
 * localization.
 */
int main(int argc, char** argv)
{
  // Init ROS
  ros::init(argc, argv, "tp12");

  // Initiate TP12. Recall that the arguments are:
  // map_resolution, map_length, safety_border, delta_save,
  // max_lin_vel, max_ang_vel
  // (check TP12.hpp for more details).
  TP12 tp12(0.05, 20.0, 2.0, 100, 1.0, deg2rad(90.0), "/robot_0");

  // Infinite loop (will call the callbacks whenever information is available,
  // until ros::shutdown() is called.
  ros::spin();

  return 1;
}

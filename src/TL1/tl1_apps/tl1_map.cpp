/*
Copyright (c) 2016, Hugo Costelha
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

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Other local functions
#include "utils.hpp"
#include <unistd.h>

// ROS API
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h> // Pose messages


geometry_msgs::Pose2D pose;

void poseCallback(const geometry_msgs::Pose2D& msg)
{
  // Store updated values
  pose.x = msg.x;
  pose.y = msg.y;
  pose.theta = msg.theta;

  std::cout << "Received pose: "
            << pose.x << " m, "
            << pose.y << " m, "
            << pose.theta << " rad" << std::endl;
}

/**
 * Main function
 * Implemente mapping algorithm.
 */
int main(int argc, char** argv)
{
  // Put your code here

  ///
  /// Pose subscriber and messages
  ///

  // Init ROS
  ros::init(argc, argv, "tl01-map");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle

  // Pose subscriber
  ros::Subscriber sub_pose = nh.subscribe("/robot_0/pose", 1, poseCallback);

  ros::Rate cycle(10.0); // Rate when no key is being pressed
  while(ros::ok())
  {
    // Get data from the robot and print it if available
    ros::spinOnce();

    // Proceed at desired framerate
    cycle.sleep();
  }
  return 1;
}

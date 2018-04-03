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

/**
 * Main function
 * Controls the robot using the keyboard keys and outputs posture and velocity
 * related information.
 */
int main(int argc, char** argv)
{
  // Delete this and put your code here.

  // A matrix, 4 x 2
  cv::Mat_<double> A(4, 2, 0.0);

  // b matrix, 4 x 1 (vector)
  cv::Mat_<double> b(4, 1, 0.0);

  // Store some test values in A
  A(0,0) = 1;
  A(0,1) = 2;
  A(1,0) = -1;
  A(1,1) = 2;
  A(2,0) = 1;
  A(2,1) = -2;
  A(3,0) = 2;
  A(3,1) = 2.5;

  // Store some test values in b
  b(0) = 1;
  b(1) = 2;
  b(2) = -1;
  b(3) = 1.5;

  // Compute r = inv(A'*A)*A'*b, where inv(X) = X^(-1)
  cv::Mat_<double> r = (A.t()*A).inv()*A.t()*b;

  // Debug code
  showMatValues(A, " - Matrix A:");
  showMatValues(b, " - Vector b:");
  showMatValues(r, " - Vector r:");

  ///
  /// Localization estimate publishing
  ///

  // Init ROS
  ros::init(argc, argv, "tl01-localization");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle

  // Localization publisher and messages
  geometry_msgs::Pose2D pose;
  ros::Publisher pose_pub;

  // Setup pose publisher
  pose_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_0/pose", 1);

  // Fill pose values (from the localization code)
  pose.x = 1.0; // Change with your own values
  pose.y = 2.0; // Change with your own values
  pose.theta = 0.3; // Change with your own values

  sleep(5);

  pose_pub.publish(pose);

  sleep(5);
  return 1;
}

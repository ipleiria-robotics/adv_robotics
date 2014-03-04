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

// ROS API
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Velocity messages
#include <geometry_msgs/Pose2D.h> // Velocity messages
#include <nav_msgs/Odometry.h> // Odometry messages
#include <sensor_msgs/LaserScan.h> // Laser sensor messages
#include <tf/tf.h> // Geometry transformations

#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees

// The robot will not move with speeds faster than these, so we better limit out
//values
#define MAX_LIN_VEL 0.5 // [m/s]
#define MAX_ANG_VEL 1.14 // 90º/s (in rad/s)

geometry_msgs::Pose2D true_pose;
double true_lin_vel, true_ang_vel;
bool odom_updated = false, laser_updated = false;
double closest_front_obstacle, closest_left_obstacle, closest_right_obstacle;

double clipValue(double value, double min, double max)
{
  if( value > max )
    return max;
  else if( value < min )
    return min;
  else return value;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
  // Store updated values
  true_pose.x = msg.pose.pose.position.x;
  true_pose.y = msg.pose.pose.position.y;
  true_pose.theta = tf::getYaw(msg.pose.pose.orientation);

  true_lin_vel = msg.twist.twist.linear.x;
  true_ang_vel = msg.twist.twist.angular.z;

  odom_updated = true;
}


void laserCallback(const sensor_msgs::LaserScan& msg)
{
  /// Update distance to closest obstacles
  ////////////////////////////////////////////////////////////////////////////
  // WRITE YOUR CODE HERE
  // closest_right_obstacle = ...
  // closest_left_obstacle = ...
  // closest_left_obstacle = ...


  // WRITE YOUR CODE ABOVE
  ////////////////////////////////////////////////////////////////////////////

  laser_updated = true;
  return;
}

/**
 * Main function
 * Controls the robot using the keyboard keys and outputs posture and velocity
 * related information.
 */
int main(int argc, char** argv)
{
  //
  // Create robot related objects
  //
  // Linear and angular velocities for the robot (initially stopped)
  double lin_vel=0, ang_vel=0;

  // Init ROS
  ros::init(argc, argv, "tp2");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle
  ros::Publisher vel_pub; // Velocity commands publisher
  geometry_msgs::Twist vel_cmd; // Velocity commands

  std::cout << "Random navigation with obstacle avoidance" << std::endl
            << "---------------------------" << std::endl;

  /// Setup subscribers
  // Odometry
  ros::Subscriber sub_odom = nh.subscribe("odom", 1, odomCallback);
  // Laser scans
  ros::Subscriber sub_laser = nh.subscribe("base_scan", 1, laserCallback);

  // Setup publisher
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Infinite loop
  ros::Rate cycle(10.0); // Rate when no key is being pressed
  while(ros::ok())
  {
    // Get data from the robot and print it if available
    ros::spinOnce();

    // Only change navigation controls if laser was updated
    if( laser_updated == false )
      continue;

    // show pose estimated from odometry
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
              << "Robot estimated pose = "
              << true_pose.x << " [m], " << true_pose.y << " [m], "
              << RAD2DEG(true_pose.theta) << " [º]\n";

    // Show estimated velocity
    std::cout << "Robot estimated velocity = "
              << true_lin_vel << " [m/s], "
              << RAD2DEG(true_ang_vel) << " [º/s]\n";

    ////////////////////////////////////////////////////////////////////////////
    // WRITE YOUR CODE BELOW
    // Change lin_vel and ang_vel so as to avoid obstacles
    // lin_vel = ...
    // ang_vel = ...


    // WRITE YOUR CODE ABOVE
    ////////////////////////////////////////////////////////////////////////////

    // Show desired velocity
    std::cout << "Robot desired velocity = "
              << lin_vel << " [m/s], "
              << RAD2DEG(lin_vel) << " [º/s]" << std::endl;

    // Send velocity commands
    vel_cmd.angular.z = ang_vel;
    vel_cmd.linear.x = lin_vel;
    vel_pub.publish(vel_cmd);

    // Proceed at desired framerate
    cycle.sleep();
  }

  // If we are quitting, stop the robot
  vel_cmd.angular.z = 0;
  vel_cmd.linear.x = 0;
  vel_pub.publish(vel_cmd);

  return 1;
}

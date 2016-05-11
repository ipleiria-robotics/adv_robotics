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
#include <tf/tf.h> // Geometry transformations
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Coordinate transformations
#include "LocalFrameWorldFrameTransformations.hpp"
#include "utils.hpp"

#define DEBUG_NAVIGATION_NODE 0

// The robot will not move with speeds faster than these, so we better limit out
//values
#define MAX_LIN_VEL 0.5 // [m/s]
#define MAX_ANG_VEL 1.14 // 90º/s (in rad/s)

// Robot pose from localization
pose_2d robot_pose;
bool pose_updated = false;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // Store updated values
  robot_pose.x = msg.pose.pose.position.x;
  robot_pose.y = msg.pose.pose.position.y;
  robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);

#if DEBUG_NAVIGATION_NODE
  // Print odometry information
  std::cout << "Robot odometry posture: (X,Y,Theta) = "
        << robot_pose.x << ", "
        << robot_pose.y << ", "
        << rad2deg(robot_pose.theta) << ")\n" << std::endl;
#endif
  pose_updated = true;
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
  // Navigation variables
  double velocity_at_target = 0;//0.5; // Desired velocity at next target
  uint curr_target = 0; // Current target location index
  double Kp_lin_vel = 1.0; // Proportional gain for the linear vel. control
  double Kp_ang_vel = 3.0; // Propostional gain for the angular vel. control
  double min_distance = 0.1; // Minimum acceptance distance to target
  double max_angle_to_target = deg2rad(30.0);
  std::string robot_name = "/robot_0";

  // Init ROS
  ros::init(argc, argv, "navigation_node");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle
  ros::Publisher vel_pub; // Velocity commands publisher
  geometry_msgs::Twist vel_cmd; // Velocity commands

  // Setup pose subscriber
  ros::Subscriber sub_pose = nh.subscribe(robot_name + "/odom_combined", 1, poseCallback);

  // Setup publisher
  vel_pub = nh.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 1);

  // Vector of points to be followed:
  uint  num_targets = 7;
  point_2d targets[] = {{ 2.0,  2.0}, // # 1
                        { 6.0,  2.0}, // # 2
                        { 6.0,  7.0}, // # 3
                        {-2.0,  7.0}, // # 4
                        {-7.0,  0.0}, // # 5
                        {-5.0, -7.0}, // # 6
                        { 0.0, -7.0}};// # 7

  // Let's do it...
  std::cout << "Robot motion control\n"
            << "---------------------------" << std::endl;

  // Wait until we have at least one localization update
  while( pose_updated == false )
    ros::spinOnce();

  // Determine the target closest to the robot. We will start with that one.
  // We do that by computing the distance from the robot to all desired target
  // locations, and selecting the one with the lowest distance.
  double lowest_sq_distance = INFINITY;
  for(uint i = 0; i < num_targets; i++)
  {
    double new_sq_distance = pow(robot_pose.x-targets[i].x,2) +
                             pow(robot_pose.y-targets[i].y,2);
    if( new_sq_distance < lowest_sq_distance)
    {
      lowest_sq_distance = new_sq_distance;
      curr_target = i;
    }
  }

  // Infinite loop
  // Go through each point continuously using a simple P controller
  ros::Rate cycle(10.0); // Rate when no key is being pressed
  while(ros::ok())
  {
    // Get data from the robot and print it if available
    ros::spinOnce();

    // Only change navigation controls if markers were detected or the odometry
    //updated
    if( pose_updated == false )
      continue;

    // Compute the squared distance to the target
    double distance = pow(robot_pose.x-targets[curr_target].x,2) +
                      pow(robot_pose.y-targets[curr_target].y,2);
    // If the distance is small enough, proceed to the next target
    if( distance < min_distance )
    {
      curr_target = (curr_target + 1)%num_targets;
#if DEBUG_NAVIGATION_NODE
      std::cout << "Going for target " << curr_target+1 << std::endl;
#endif
      continue;
    }

    // The angular velocity will be proportional to the angle of the target
    // as seen by the robot.
    point_2d target_local_pos;
    world2Local( robot_pose, targets[curr_target], &target_local_pos );
    double angle_to_target = atan2(target_local_pos.y,target_local_pos.x);
    ang_vel = Kp_ang_vel * angle_to_target;

    //  We will not update the linear velocity if the robot is not facing the
    // target enough. If it is, then the linear velocity will be proportional
    // to the distance, increased with the target velocity. We actually use
    // the squared distance just for performance reasons.
    if( fabs(angle_to_target) < max_angle_to_target )
      lin_vel = Kp_lin_vel * distance + velocity_at_target;

     // Limit maximum velocities
    lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
    ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

    // Send velocity commands
    vel_cmd.angular.z = 1.0*ang_vel;
    vel_cmd.linear.x = 1.0*lin_vel;
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

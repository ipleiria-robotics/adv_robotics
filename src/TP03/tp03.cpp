/*
Copyright (c) 2014, Hugo Costelha
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
#include <tf/tf.h> // Geometry transformations
#include <markers_msgs/Markers.h>

// Coordinate transformations
#include "LocalFrameWorldFrameTransformations.hpp"

#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees

// The robot will not move with speeds faster than these, so we better limit out
//values
#define MAX_LIN_VEL 0.5 // [m/s]
#define MAX_ANG_VEL 1.14 // 90º/s (in rad/s)

// Robot posture from odometry
geometry_msgs::Pose2D robot_odometry_pose;
// Computed posture from localization
geometry_msgs::Pose2D robot_localized_pose;
double true_lin_vel, true_ang_vel;
bool odom_updated = false, localization_from_markers_updated = false;

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
  robot_odometry_pose.x = msg.pose.pose.position.x;
  robot_odometry_pose.y = msg.pose.pose.position.y;
  robot_odometry_pose.theta = tf::getYaw(msg.pose.pose.orientation);

  true_lin_vel = msg.twist.twist.linear.x;
  true_ang_vel = msg.twist.twist.angular.z;

  // Print odometry information
  std::cout << "Robot odometry posture: (X,Y,Theta) = "
        << robot_odometry_pose.x << ", "
        << robot_odometry_pose.y << ", "
        << RAD2DEG(robot_odometry_pose.theta) << ")\n" << std::endl;

  odom_updated = true;
}

#define X_MAX_POS 3.32 // [m]
#define Y_MAX_POS 2.28 // [m]

void markersCallback(const markers_msgs::Markers& msg)
{
  localization_from_markers_updated = false;

  // Store the positions of the landmarks
  point_2d_t beacons_wpos[4] = {{-X_MAX_POS, -Y_MAX_POS}, // 1
                                {-X_MAX_POS,  Y_MAX_POS}, // 2
                                { X_MAX_POS,  Y_MAX_POS}, // 3
                                { X_MAX_POS, -Y_MAX_POS}};// 4

  // If we have less then 3 beacons, return an error
  if( msg.num_markers < 3 )
  {
    std::cerr << "Unable to localize !" << std::endl;
    return;
  }

  // Display found beacons information
  std::cout << "Found " << msg.num_markers << " beacons.\n";
  for( uint i = 0; i < msg.num_markers; i++ )
  {
    std::cout << "Beacon " << msg.id[i] << ": (range,bearing) = ("
              << msg.range[i] << ", "
              << msg.bearing[i] << ")\n";
  }

  // Compute the robot localization based on the first two beacons found
  point_2d_t b1w = beacons_wpos[msg.id[0]-1]; // Beacon 1 world position
  point_2d_t b2w = beacons_wpos[msg.id[1]-1]; // Beacon 2 world position
  point_2d_t b3w = beacons_wpos[msg.id[2]-1]; // Beacon 3 world position

  // Fill variables
  double d1s = msg.range[0]*msg.range[0];
  double d2s = msg.range[1]*msg.range[1];
  double d3s = msg.range[2]*msg.range[2];
  double x1 = b1w.x;
  double y1 = b1w.y;
  double x2 = b2w.x;
  double y2 = b2w.y;
  double x3 = b3w.x;
  double y3 = b3w.y;

  // Compute A
  //double A = (d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)/(2*(y2-y1));
  // Compute B
  //double B = (x1-x2)/(y2-y1);
  // Compute C
  //double C = (d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)/(2*(y3-y2));
  // Compute D
  //double D = (x2-x3)/(y3-y2);

  // Compute the robot position
  //robot_localized_pos->x = (A - C) / (D - B);
  //robot_localized_pos->y = A + B * robot_localized_pos->x;
  robot_localized_pose.x = 0.5*
    ((y3-y2)*(d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)-(y2-y1)*(d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3))/
          ((y2-y1)*(x2-x3)-(y3-y2)*(x1-x2));
  robot_localized_pose.y = 0.5*
    ((x2-x3)*(d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)-(x1-x2)*(d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3))/
          ((y2-y1)*(x2-x3)-(y3-y2)*(x1-x2));

  // Estimate the angle
  double alpha0 = atan2(y1-robot_localized_pose.y, x1-robot_localized_pose.x);
  robot_localized_pose.theta = alpha0-msg.bearing[0];
  // Limit the angle between -PI and PI
  if( robot_localized_pose.theta > M_PI )
    robot_localized_pose.theta -= 2*M_PI;
  else if( robot_localized_pose.theta < -M_PI )
    robot_localized_pose.theta += 2*M_PI;

  std::cout << "Robot estimated posture: (X,Y,Theta) = "
            << robot_localized_pose.x << ", "
            << robot_localized_pose.y << ", "
            << RAD2DEG(robot_localized_pose.theta) << ")\n" << std::endl;

  localization_from_markers_updated = true;
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
  // Navigation variables
  double velocity_at_target = 0;//0.5; // Desired velocity at next target
  uint curr_target = 0; // Current target location index
  double Kp_lin_vel = 1.0; // Proportional gain for the linear vel. control
  double Kp_ang_vel = 3.0; // Propostional gain for the angular vel. control
  double min_distance = 0.1; // Minimum acceptance distance to target
  double max_angle_to_target = DEG2RAD(30.0);
  std::string robot_name = "/robot_0";
  geometry_msgs::Pose2D robot_pose;

  // Init ROS
  ros::init(argc, argv, "tp03");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle
  ros::Publisher vel_pub; // Velocity commands publisher
  geometry_msgs::Twist vel_cmd; // Velocity commands

  /// Setup subscribers
  // Odometry
  ros::Subscriber sub_odom = nh.subscribe(robot_name + "/odom", 1, odomCallback);
  // Markers detected
  ros::Subscriber sub_markers = nh.subscribe(robot_name + "/markers", 1, markersCallback);

  // Setup publisher
  vel_pub = nh.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 1);

  // Vector of points to be followed:
  uint  num_targets = 6;
  point_2d_t targets[] = {{-2.6, -1.7}, // # 1
                          {-2.6,  1.7}, // # 2
                          { 0.0,  0.4}, // # 3
                          { 2.6,  1.7}, // # 4
                          { 2.6, -1.7}, // # 5
                          { 0.0, -0.4}}; // # 6

  // Let's do it...
  std::cout << "Robot motion control\n"
            << "---------------------------" << std::endl;

  // Choose which pose to use, from odometry or estimated from the markers.
  // Setting this variable to true means it will use odometry.
  bool use_odometry = true;

  // Wait until we have at least one localization update
  while( ( (use_odometry && odom_updated) ||
           ( (use_odometry == false) && localization_from_markers_updated) )
         == false )
  {
    ros::spinOnce();
  }

  if( use_odometry )
  {
    odom_updated = false;
    robot_pose = robot_odometry_pose;
  } else
  {
    localization_from_markers_updated = false;
    robot_pose = robot_localized_pose;
  }

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
    if( ( use_odometry && (odom_updated == false) ) ||
        ( (use_odometry == false) && (localization_from_markers_updated == false) ) )
    {
      vel_pub.publish(vel_cmd);
      continue;
    }

    if( use_odometry )
    {
      odom_updated = false;
      robot_pose = robot_odometry_pose; // Use posture from odometry
    } else
    {
      localization_from_markers_updated = false;
      robot_pose = robot_localized_pose; // Use posture from localization
    }

    // Compute the squared distance to the target
    double distance = pow(robot_pose.x-targets[curr_target].x,2) +
                      pow(robot_pose.y-targets[curr_target].y,2);
    // If the distance is small enough, proceed to he next target
    if( distance < min_distance )
    {
      curr_target = (curr_target + 1)%num_targets;
      std::cout << "Going for target " << curr_target+1 << std::endl;
      continue;
    }

    // The angular velocity will be proportional to the angle of the target
    // as seen by the robot.
    point_2d_t target_local_pos;
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

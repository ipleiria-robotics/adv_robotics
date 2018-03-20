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
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Velocity messages
#include <geometry_msgs/Pose2D.h> // Velocity messages
#include <nav_msgs/Odometry.h> // Odometry messages
#include <tf/tf.h> // Geometry transformations

// OpenCV header files
#include <cv.hpp>

// Coordinate transformations
#include "LocalFrameWorldFrameTransformations.hpp"

// Radians <-> Degrees convertion tools
#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees

// The robot will not move with speeds faster than these, so we better limit out
//values
#define MAX_LIN_VEL 1.0 // [m/s]
#define MAX_ANG_VEL 1.14 // 90º/s (in rad/s)

// Map related constants
#define MAP_RESOLUTION 0.05 // [m/px]
#define SCALE_FACTOR 0.5 // Amount of map scaling to apply

// Save robot poses to a file
std::ofstream outfile;

// Pose and laser information
geometry_msgs::Pose2D odo_robot_pose;
double odo_lin_vel, odo_ang_vel;
bool odom_updated = false;

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
  odo_robot_pose.x = msg.pose.pose.position.x;
  odo_robot_pose.y = msg.pose.pose.position.y;
  odo_robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);

  odo_lin_vel = msg.twist.twist.linear.x;
  odo_ang_vel = msg.twist.twist.angular.z;

  odom_updated = true;
}

/**
 * Main function
 * Controls the robot using the keyboard keys and outputs posture and velocity
 * related information.
 */
int main(int argc, char** argv)
{
  // Open file to store robot poses
  outfile.open("Postures.txt");

  // Scale to apply to the map
  double scale = SCALE_FACTOR/MAP_RESOLUTION;

  // Target point
  point_2d_t target = {2.6, 1.7}; // [m]

  // Potential repulsive factor
  double kr = 1.0;
  // Potential atractive factor
  double ka = 1.0;

  // Read map
  std::string map_file_path(getenv("HOME"));
  map_file_path += "/ros/src/TP05/mapa.png";
  cv::Mat_<uchar> org_map = cv::imread(map_file_path, CV_LOAD_IMAGE_GRAYSCALE);
  if( org_map.data == 0 )
  {
      std::cerr << "Error reading map!" << std::endl;
      return -1;
  }

  // This algorithm is very slow, so we will resize the input map image
  cv::Mat_<uchar> resized_map;
  resize(org_map, resized_map, cv::Size(), SCALE_FACTOR, SCALE_FACTOR);

  // Show original map
  cv::namedWindow("Mapa original (reduzido)", 0);
  cv::imshow("Mapa original (reduzido)", resized_map);

  //
  // Create repulsive potential map
  //

  // Repulsive potential map will be the same size as the original map
  cv::Mat_<double> rep_pot(resized_map.size());
  rep_pot.setTo(0);

  // Go trough each map point and fill according the the distance to all
  // obstacles
  double max_rep_value = 0;
  for( int xr = 0; xr < resized_map.size().width; xr++ )
  {
    for( int yr = 0; yr < resized_map.size().height; yr++ )
    {
      double rep_value = 0;
      for( int xo = 0; xo < rep_pot.size().width; xo++ )
      {
        for( int yo = 0; yo < rep_pot.size().height; yo++ )
        {
          if( (xr == xo) && (yr == yo) )
            rep_value += kr * (255.0-resized_map(yo,xo))/(255.0 * (1.0) / pow(scale,2));
          else
          {
            //////////////////////////////////////////////////////////////////
            // Compute the repulsive potential to add (when not in obstacle)
            //////////////////////////////////////////////////////////////////
            // rep_value += ??

            //////////////////////////////////////////////////////////////////
          }
        }
      }
      rep_pot(yr,xr) = rep_value;
      if( rep_value > max_rep_value )
        max_rep_value = rep_value;
    }
  }
  std::cout << "Max repulsive value: " << max_rep_value << std::endl;

  // Show and store repulsive potential map (for debug purposes only)
  cv::Mat_<uchar> dbg_rep_pot;
  rep_pot.convertTo(dbg_rep_pot, CV_8UC1, 255/max_rep_value);
  cv::namedWindow("Potential repulsivo", 0);
  cv::imshow("Potential repulsivo", dbg_rep_pot);
  cv::imwrite("mapa_pot_rep.png", dbg_rep_pot);

  //
  // Create atractive potential map
  //
  cv::Point2i target_pxl(cvRound(target.x*scale+resized_map.size().width/2.0),
                         cvRound(-target.y*scale+resized_map.size().height/2.0));
  cv::Mat_<double> atractive_pot(resized_map.size());
  double max_atr_value = 0;
  for( int xr = 0; xr < atractive_pot.size().width; xr++ )
    for( int yr = 0; yr < atractive_pot.size().height; yr++ )
    {
      //////////////////////////////////////////////////////////////////
      // Compute here the atractive potential to add
      //////////////////////////////////////////////////////////////////
      // atractive_pot(yr,xr) = ??

      //////////////////////////////////////////////////////////////////

      if ( atractive_pot(yr,xr) > max_atr_value )
        max_atr_value = atractive_pot(yr,xr);
    }
  std::cout << "Max atractive value: " << max_atr_value << std::endl;

  // Show and store atractive potential map (for debug purposes only)
  cv::Mat_<uchar> dbg_atr_pot;
  atractive_pot.convertTo(dbg_atr_pot, CV_8UC1, 255.0/max_atr_value);
  cv::namedWindow("Potential atractivo", 0);
  cv::imshow("Potential atractivo", dbg_atr_pot);
  cv::imwrite("mapa_pot_atr.png", dbg_atr_pot);

  //
  // Compute resulting potential
  //
  cv::Mat_<double> resulting_pot(resized_map.size());
  resulting_pot = rep_pot + atractive_pot;

  // Show and store resulting potential map (for debug purposes only)
  cv::Mat_<uchar> dbg_res_pot;
  resulting_pot.convertTo(dbg_res_pot, CV_8UC1, 255.0/(max_atr_value+max_rep_value));
  cv::namedWindow("Potential resultante", 0);
  cv::imshow("Potential resultante", dbg_res_pot);
  cv::imwrite("mapa_pot_res.png", dbg_res_pot);
  cv::waitKey(10); // Wait 10 seconds

  //
  // Navigate using the generated potential maps
  //
  // Create robot related objects
  double tolerance = 0.15; // [m]
  double lin_vel = 0.0, ang_vel=0.0;
  double Kp_lin_vel = 1.0; // Proportional gain for the linear vel. control
  double Kp_ang_vel = 3.0; // Propostional gain for the angular vel. control
  double velocity_at_target = 0.2;//0.5; // Desired velocity at next target
  double max_angle_to_target = DEG2RAD(5.0);

  // Init ROS
  ros::init(argc, argv, "tp5");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle
  ros::Publisher vel_pub; // Velocity commands publisher
  geometry_msgs::Twist vel_cmd; // Velocity commands

  std::cout << "Navigating using potential fields\n"
            << "---------------------------" << std::endl;

  /// Setup subscribers
  // Odometry
  ros::Subscriber sub_odom = nh.subscribe("/robot_0/odom", 1, odomCallback);

  // Setup publisher for linear and angular velocity
  vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);

  ros::Rate cycle(10.0); // Cycle rate

  // Wait until we get the robot pose (from odometry)
  while( odom_updated == false )
  {
    ros::spinOnce();
    cycle.sleep();
  }

  // Fill variables with some initial data
  target_pxl.x = cvRound(odo_robot_pose.x*scale+resized_map.size().width/2.0);
  target_pxl.y = cvRound(-odo_robot_pose.y*scale+resized_map.size().height/2.0);
  point_2d_t current_target;
  current_target.x = (target_pxl.x - resized_map.size().width/2.0)/scale;
  current_target.y = (-target_pxl.y + resized_map.size().height/2.0)/scale;

  // Infinite loop
  while(ros::ok())
  {
    // Get data from the robot and print it if available
    ros::spinOnce();

    // Only change navigation controls if laser was updated
    if( odom_updated == false )
      continue;

    // show pose estimated from odometry
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
              << "Robot estimated pose = "
              << odo_robot_pose.x << " [m], " << odo_robot_pose.y << " [m], "
              << RAD2DEG(odo_robot_pose.theta) << " [º]\n";

    // Show estimated velocity
    std::cout << "Robot estimated velocity = "
              << odo_lin_vel << " [m/s], "
              << RAD2DEG(odo_ang_vel) << " [º/s]\n";

    //
    // Head for the nearest cell with lower value
    //

    // If we are near the current target, select the next one
    if( ( fabs(odo_robot_pose.x - current_target.x) < tolerance ) &&
        ( fabs(odo_robot_pose.y - current_target.y) < tolerance ) )
    {
      //  Select near cell with lower potential
      double current_potential = resulting_pot(target_pxl.y,target_pxl.x);
      cv::Point2i next_target(target_pxl.x, target_pxl.y);
      for( int x = target_pxl.x-1; x <= target_pxl.x+1; x++ )
      {
        for( int y = target_pxl.y-1; y <= target_pxl.y+1; y++ )
        {
          // Do not check points outside the map
          if( (x < 0) || (x >= resulting_pot.size().width) ||
              (y < 0) || (y >= resulting_pot.size().height) )
            continue;
          // Is this a point with lower potential?
          if( resulting_pot(y,x) < current_potential )
          {
            current_potential = resulting_pot(y,x);
            next_target.x = x;
            next_target.y = y;
          }
        }
      }
      // Is this the last point
      if( (target_pxl.x == next_target.x) && (target_pxl.y == next_target.y) )
        break;
      else
        target_pxl = next_target;
      // Convert the target to world coordinates
      current_target.x = (target_pxl.x - resized_map.size().width/2.0)/scale;
      current_target.y = (-target_pxl.y + resized_map.size().height/2.0)/scale;
    }

    // Navigate the robot to the next target
    // The angular velocity will be proportional to the angle of the target
    // as seen by the robot.
    point_2d_t target_local_pos;
    world2Local( odo_robot_pose, current_target, &target_local_pos );
    double angle_to_target = atan2(target_local_pos.y, target_local_pos.x);
    ang_vel = Kp_ang_vel * angle_to_target;

    //  We will not update the linear velocity if the robot is not facing the
    // target enough. If it is, then the linear velocity will be proportional
    // to the distance, increased with the target velocity. We actually use
    // the squared distance just for performance reasons.
    // Compute the squared distance to the target
    double distance = pow(odo_robot_pose.x-current_target.x,2) +
                      pow(odo_robot_pose.y-current_target.y,2);
    if( fabs(angle_to_target) < max_angle_to_target )
      lin_vel = Kp_lin_vel * distance + velocity_at_target;

    // Limit maximum velocities
    // (not needed here)
    lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
    ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

    // Show desired velocity
    std::cout << "Robot desired velocity = "
              << lin_vel << " [m/s], "
              << RAD2DEG(lin_vel) << " [º/s]" << std::endl;

    // Send velocity commands
    vel_cmd.angular.z = ang_vel;
    vel_cmd.linear.x = lin_vel;
    vel_pub.publish(vel_cmd);

    // Terminate loop if Escape key is pressed
    if( cv::waitKey(10) == 27 )
      break;

    // Proceed at desired framerate
    cycle.sleep();
  }

  // If we are quitting, stop the robot
  vel_cmd.angular.z = 0;
  vel_cmd.linear.x = 0;
  vel_pub.publish(vel_cmd);

  // Close file
  outfile.close();

  std::cout << "Press any key to exit!" << std::endl;
  cv::waitKey(0);

  return 1;
}

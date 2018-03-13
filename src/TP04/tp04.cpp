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
#include <sensor_msgs/LaserScan.h> // Laser sensor messages
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
#define MAP_RESOLUTION 0.05 // [m]
#define MAP_HEIGHT 6.0 // [m]
#define MAP_WIDTH 8.0 // [m]
#define MIN_CELL_VALUE 0
#define MAX_CELL_VALUE 255
#define CELL_DELTA_UP 1 // Change value per increment update
#define CELL_DELTA_DOWN 5 // Change value per decrement update
#define DELTA_SAVE 100 // Number of iterations between each save of the map

// Map fo the environment (to be created/updated)
cv::Mat map;

// Control iterations between map file updates
int iteration = 0;

// Save robot poses to a file
std::ofstream outfile;

// Pose and laser information
geometry_msgs::Pose2D robot_pose;
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
  robot_pose.x = msg.pose.pose.position.x;
  robot_pose.y = msg.pose.pose.position.y;
  robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);

  true_lin_vel = msg.twist.twist.linear.x;
  true_ang_vel = msg.twist.twist.angular.z;

  odom_updated = true;
}


void laserCallback(const sensor_msgs::LaserScan& msg)
{
  double angle;
  unsigned int i;

  // Store robot position in map grid coordinates
  unsigned int robot_col = 0; // CHANGE ME
  unsigned int robot_row = 0; // CHANGE ME

  angle = msg.angle_min;
  i=0;
  // Go through all the LRF measurements
  while(angle <= msg.angle_max)
  {
    if( msg.ranges[i] > msg.range_min )
    {
      ///////////////////////////////////////////////////////////////////////
      // Update map with each sensor reading.
      //
      // Important variables (already defined and available):
      //  - (robot_pose.x, robot_pose.y, robot_pose.theta) ==> Robot pose in the
      // world frame;
      //  - msg.ranges[i] ==> LRF i reading (distance from the LRF to the beacon
      // detected by the beacon at the i-th angle);
      //
      ///////////////////////////////////////////////////////////////////////
      // INSERT YOUR CODE BELOW THIS POINT
      ///////////////////////////////////////////////////////////////////////

      // Create obstacle position in sensor coordinates from msg.ranges[i]


      // Transform obstacle position to robot coordinates using local2world


      // Get obtained value in world coordinates using again local2world


      // Get obtained value in the map grid (columm and row) - must be unsigned
      // int.


      // Update map using the line iterator for free space

      // Update map using the line iterator for occupied space, if applicable

      ///////////////////////////////////////////////////////////////////////
      // INSERT YOUR CODE ABOVE THIS POINT
      ///////////////////////////////////////////////////////////////////////
    }

    // Proceed to next laser measure
    angle += msg.angle_increment;
    i++;
  }

  // Show map
  cv::imshow("Mapa", map);

  // Save the map every DELTA_SAVE iterations
  iteration++;
  if( iteration == DELTA_SAVE )
  {
    cv::imwrite("mapa.png", map);
    iteration = 0;
  }

  /// Update distance to closest obstacles
  // Right obstacle
  angle = -1.571; // DEG2RAD(-90)
  i = round((angle - msg.angle_min)/msg.angle_increment);
  closest_right_obstacle = msg.range_max;
  while( angle < -1.309 ) // DEG2RAD(-90+15)
  {
    if( (msg.ranges[i] < msg.range_max) &&
        (msg.ranges[i] > msg.range_min) &&
        (msg.ranges[i] < closest_right_obstacle) )
      closest_right_obstacle = msg.ranges[i];
    i++;
    angle += msg.angle_increment;
  }

  // Front obstacle
  angle = -0.785; // DEG2RAD(-45)
  i = round((angle - msg.angle_min)/msg.angle_increment);
  closest_front_obstacle = msg.range_max;
  while( angle < 0.785 ) // DEG2RAD(45)
  {
    if( (msg.ranges[i] < msg.range_max) &&
        (msg.ranges[i] > msg.range_min) &&
        (msg.ranges[i] < closest_front_obstacle) )
      closest_front_obstacle = msg.ranges[i];
    i++;
    angle += msg.angle_increment;
  }

  // Left obstacle
  angle = 1.309; // DEG2RAD(90-15)
  i = round((angle - msg.angle_min)/msg.angle_increment);
  closest_left_obstacle = msg.range_max;
  while( angle < 1.571 ) // DEG2RAD(90)
  {
    if( (msg.ranges[i] < msg.range_max) &&
        (msg.ranges[i] > msg.range_min) &&
        (msg.ranges[i] < closest_left_obstacle) )
      closest_left_obstacle = msg.ranges[i];
    i++;
    angle += msg.angle_increment;
  }

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
  // Open file to store robot poses
  outfile.open("Postures.txt");

  // Create a window to show the map
  cv::namedWindow("Mapa", 0);

  //
  // Create map related variables
  //
  map.create(ceil(MAP_HEIGHT/MAP_RESOLUTION),
             ceil(MAP_WIDTH/MAP_RESOLUTION),
             CV_8UC1);
  // Set the initial map cell values to "undefined"
  map.setTo(127);

  //
  // Create robot related objects
  //
  // Linear and angular velocities for the robot (initially stopped)
  double lin_vel=0, ang_vel=0;
  double last_ang_vel = DEG2RAD(10);
  // Navigation variables
  bool avoid, new_rotation = false;
  double stop_front_dist, min_front_dist;
  // Random number generator (used for direction choice)
  CvRNG rnggstate = cvRNG(0xffffffff);

  // Init ROS
  ros::init(argc, argv, "tp04");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle
  ros::Publisher vel_pub; // Velocity commands publisher
  geometry_msgs::Twist vel_cmd; // Velocity commands

  std::cout << "Random navigation with obstacle avoidance and map generation\n"
            << "---------------------------" << std::endl;

  // Get parameters
  ros::NodeHandle n_private("~");
  n_private.param("min_front_dist", min_front_dist, 1.0);
  n_private.param("stop_front_dist", stop_front_dist, 0.6);

  /// Setup subscribers
  // Odometry
  ros::Subscriber sub_odom = nh.subscribe("/robot_0/odom", 1, odomCallback);
  // Laser scans
  ros::Subscriber sub_laser = nh.subscribe("/robot_0/base_scan", 1, laserCallback);

  /// Setup publisher
  vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);

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
              << robot_pose.x << " [m], " << robot_pose.y << " [m], "
              << RAD2DEG(robot_pose.theta) << " [º]\n";

    // Show estimated velocity
    std::cout << "Robot estimated velocity = "
              << true_lin_vel << " [m/s], "
              << RAD2DEG(true_ang_vel) << " [º/s]\n";

    // Check for obstacles near the front  of the robot
    avoid = false;
    if( closest_front_obstacle < min_front_dist )
    {
      if( closest_front_obstacle < stop_front_dist )
      {
        avoid = true;
        lin_vel = -0.100;
      } else
      {
        avoid = true;
        lin_vel = 0;
      }
    } else
    {
      lin_vel = 0.5;
      ang_vel = 0;
      new_rotation = false;
    }

    // Rotate to avoid obstacles
    if(avoid)
    {
      if( new_rotation == false )
      {
        float rnd_point = cvRandReal(&rnggstate );
        if( rnd_point >= 0.9 )
        {
          last_ang_vel = -last_ang_vel;
        }
      }
      ang_vel = last_ang_vel;
      new_rotation = true;
    }

    // Limit maximum velocities
    // (not needed here)
//    lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
//    ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

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

  // Store map
  cv::imwrite("mapa.png", map);

  // Close file
  outfile.close();

  return 1;
}

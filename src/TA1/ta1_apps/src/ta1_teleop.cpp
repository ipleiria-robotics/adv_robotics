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
#include <termios.h>
#include <stdio.h>

// ROS API
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Velocity messages
#include <geometry_msgs/Pose2D.h> // Velocity messages
#include <nav_msgs/Odometry.h> // Odometry messages
#include <tf/tf.h> // Geometry transformations
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// The robot will not move with speeds faster than these, so we better limit out
//values
#define MAX_LIN_VEL 0.8 // [m/s]
#define MAX_ANG_VEL 1.14 // 90º/s (in rad/s)

geometry_msgs::Pose2D odo_pose;
double true_lin_vel, true_ang_vel;
bool odom_updated = false;

// Forklift information
bool forklift_up = false,
     forklift_down = false;
#define FORKLIFT_DOWN 0.0   // Down position
#define FORKLIFT_UP   0.07  // Up position

bool save_next_frame = false;

struct termios org_tios;

/**
 * Restore the terminal to its original settings
 */
void restoreTerminal()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &org_tios);
}

/**
 * Checks if there a key was pressed, and returns a positive number if true.
 */
int checkForKey()
{
  struct timeval tv = { 0L, 0L };
  fd_set fds;
  FD_SET(0, &fds);
  return select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
}

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
  odo_pose.x = msg.pose.pose.position.x;
  odo_pose.y = msg.pose.pose.position.y;
  odo_pose.theta = tf::getYaw(msg.pose.pose.orientation);

  true_lin_vel = msg.twist.twist.linear.x;
  true_ang_vel = msg.twist.twist.angular.z;

  odom_updated = true;
}

/*
void forkliftCallback(const control_msgs::JointControllerStatePtr& msg)
{
  // If the forklift is moving, then it is neither down or up
  if( fabs(msg->error) > 0.02 )
  {
    forklift_up = false;
    forklift_down = false;
  } else
  {
    // The forklift is stopped, lets check in which position
    if( fabs(msg->process_value-FORKLIFT_DOWN) < 0.01 )
    {
      // It's down
      if( forklift_down == false )
      {
        std::cout << "Forklift is down" << std::endl;
        forklift_up = false;
        forklift_down = true;
      }
    } else if ( fabs(msg->process_value-FORKLIFT_UP) < 0.01 )
    {
      // It's up
      if( forklift_up == false )
      {
        forklift_up = true;
        forklift_down = false;
        std::cout << "Forklift is up" << std::endl;
      }
    }
  }
}
*/
/*
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception accessing camera: %s", e.what());
    return;
  }

  // Show image
  cv::imshow("camera", cv_ptr->image);

  // Save image if needed
  if( save_next_frame )
  {
      cv::imwrite("frame.png", cv_ptr->image);
      save_next_frame = false;
      ROS_INFO("Image saved...");
  }

  if( cv::waitKey(5) == 27 )
    ros::shutdown(); // Shutdown if ESC key was pressed
}
*/
/**
 * Main function
 * Controls the robot using the keyboard keys and outputs posture and velocity
 * related information.
 */
int main(int argc, char** argv)
{
  // Set terminal settings for capturing the keyboard
  struct termios new_tios;
  tcgetattr( STDIN_FILENO, &org_tios );
  tcgetattr( STDIN_FILENO, &new_tios );
  atexit(restoreTerminal); // Restore on exit
  new_tios.c_iflag &= ~(IGNCR | INLCR);
  new_tios.c_iflag |= ICRNL;
  new_tios.c_oflag |= ONLCR;
  new_tios.c_lflag &= ~ICANON;
  new_tios.c_lflag &= ~(ECHO|ECHOE|ECHOK|ECHONL); // No echo
  tcsetattr( STDIN_FILENO, TCSANOW, &new_tios );

  //
  // Create robot related objects
  //
  // Linear and angular velocities for the robot (initially stopped)
  double lin_vel=0, ang_vel=0;
  double l_scale, a_scale;
  // Velocity increase for each keystroke
  double delta_lin_vel=0.1, delta_ang_vel = 5.0/180.0*3.14;

  // Init ROS
  ros::init(argc, argv, "robot_keyboard_teleop");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle
  ros::Publisher vel_pub; // Velocity commands publisher
  geometry_msgs::Twist vel_cmd; // Velocity commands
  std::string robot_name = "/robot";
  std_msgs::Float64 forklift_pos_cmd; // Forklift position command

  // Output usage information
  std::cout << "Reading from keyboard\n"
            << "Use i, j, k, l and space to move front, left, back, right and stop, respectively\n"
            << "Use e, d to move the forklift up and down\n"
            << "Use s to save an image\n"
            << "Press q to quit.\n"
            << "---------------------------" << std::endl;

  // Get parameters
  ros::NodeHandle n_private("~");
  n_private.param("scale_angular", a_scale, 1.0);
  n_private.param("scale_linear", l_scale, 1.0);

  // Setup subscriber
  ros::Subscriber sub_p = nh.subscribe(robot_name+"/odom", 1, odomCallback);

  // Setup publisher for speed control
  vel_pub = nh.advertise<geometry_msgs::Twist>(robot_name+"/cmd_vel", 1);

  // Setup subscriber for the forlift status
//  ros::Subscriber sub_f = nh.subscribe(robot_name+"/forklift_position_controller/state", 1, forkliftCallback);

  // Setup publishers for the forklift commands
//  ros::Publisher forklift_pub = nh.advertise<std_msgs::Float64>(robot_name+"/forklift_position_controller/command", 1);

  // Setup camera subscriber
//  ros::Subscriber sub_cam = nh.subscribe(robot_name+"/camera_front/image_raw", 1, cameraCallback);

  // Infinite loop
  ros::Rate cycle(10.0); // Rate when no key is being pressed
  bool key_pressed;
  while(ros::ok())
  {
    key_pressed = false;

    // Get data from the robot and print it if available
    ros::spinOnce();
    // show pose estimated from odometry
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
              << "Robot estimated pose = "
              << odo_pose.x << " [m], " << odo_pose.y << " [m], "
              << odo_pose.theta*180.0/M_PI << " [º]\n";

    // Show estimated velocity
    std::cout << "Robot estimated velocity = "
              << true_lin_vel << " [m/s], "
              << true_ang_vel*180.0/M_PI << " [º/s]\n";

    // Read from the keyboard
    if(checkForKey())
    {
      int nChar;
      nChar = getchar_unlocked();
      if( nChar == 'q')
        break;
      // else
      switch(nChar)
      {
      case 'i':
        // Increase linear velocity
        lin_vel += delta_lin_vel;
        key_pressed = true;
        break;
      case 'k':
        // Decrease linear velocity
        lin_vel -= delta_lin_vel;
        key_pressed = true;
        break;
      case 'j':
        // Increase angular velocity
        ang_vel += delta_ang_vel;
        key_pressed = true;
        break;
      case 'l':
        // Decrease angular velocity
        ang_vel -= delta_ang_vel;
        key_pressed = true;
        break;
      case ' ':
        // Stop robot
        lin_vel = 0;
        ang_vel = 0;
        break;
/*      case 'e':
        // Send forklit up
        forklift_pos_cmd.data = FORKLIFT_UP;
        forklift_pub.publish(forklift_pos_cmd);
        break;
      case 'd':
        // Send forklit up
        forklift_pos_cmd.data = FORKLIFT_DOWN;
        forklift_pub.publish(forklift_pos_cmd);
        break;
*/      case 's':
        // Save next frame
        save_next_frame = true;
        break;
      }
      // Limit maximum velocities
      lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
      ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);
    } else
    {
      // Decelerate automatically if no key was pressed
      lin_vel *= 0.9;
      ang_vel *= 0.9;
    }

    // Show desired velocity
    std::cout << "Robot desired velocity = "
              << lin_vel << " [m/s], "
              << lin_vel*180.0/M_PI << " [º/s]" << std::endl;

    // Send velocity commands
    vel_cmd.angular.z = a_scale*ang_vel;
    vel_cmd.linear.x = l_scale*lin_vel;
    vel_pub.publish(vel_cmd);

    if( key_pressed == false )
      cycle.sleep(); // Limit the amount of messages when no key is pressed

    // Move cursor back up n lines (and erase them)
    for(uint n=1; n <= 3; n++)
      std::cout << "\033[1A" << "\033[2K";
  }

  // If we are quitting, stop the robot
  vel_cmd.angular.z = 0;
  vel_cmd.linear.x = 0;
  vel_pub.publish(vel_cmd);

  restoreTerminal();
  return 1;
}

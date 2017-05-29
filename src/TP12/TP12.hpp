 
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

#ifndef TP12_HPP
#define TP12_HPP

// Headers to read and write from the terminal and/or file
#include <fstream>


// OpenCV headers
#include <cv.hpp>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h> // Velocity messages
#include <nav_msgs/Odometry.h> // Odometry messages
#include <sensor_msgs/LaserScan.h> // Laser sensor messages
#include <markers_msgs/Markers.h> // Markers messages

#include "utils.hpp"
#include "ExtendedKalmanFilter.hpp"

class TP12
{
public:
  /**
   * Constructor
   * map_resolution - map resolution in [m/px].
   * map_length - length and width of the map [m].
   * map_border - free space border to add around the map [m].
   * delta_save - number of iterations between map saves to disk.
   * max_lin_vel - maximum allowed linear velocity for the robot [m/s].
   * max_ang_vel - maximum allowed angular velocity for the robot [m/s].
   * robotname - name associated with the robot.
   */
   TP12(const double map_resolution, const double map_length,
       const double map_border, const uint delta_save,
       const double max_lin_vel, const double max_ang_vel,
       std::string robotname);

   /**
    * Destructor
    */
    ~TP12();

private:

   /**
     * Show debug information graphically and store it in a file.
     */
   void showDebugInformation(std::string header="", bool erase_landmarks=true);

   /**
     * Output debug information to a file.
     * header - header to write before outputing the information.
     */
   void outputDebugInfoToFile(std::string header="");

   /**
     * Callback used to store the real pose given by the simulator (used only
     * for debug purposes.
     */
   void realPoseCallback(const nav_msgs::Odometry& msg);

   /**
     * Callback used to process odometry messages.
     */
   void odomCallback(const nav_msgs::Odometry& msg);

   /**
     * Callback used to process detected markers messages.
     */
   void markersCallback(const markers_msgs::Markers& msg);

   /**
     * Callback used to process laser messages, used for navigation.
     */
   void laserCallback(const sensor_msgs::LaserScan& msg);

   // Map related constants
   double _MAP_RESOLUTION; ///< [m/px]
   double _MAP_LENGTH; ///< Width and height of the map [m]
   double _MAP_BORDER; ///< Border around the map [m]
   uint _DELTA_SAVE; ///< Number of iterations between each save of the map

   // Create robot related variables
   // Linear and angular velocities for the robot
   double _lin_vel, _ang_vel;
   // Navigation variables
   bool _avoid, _rotating;
   bool _rotate_left;
   double _stop_front_dist, _min_front_dist;
   geometry_msgs::Twist _vel_cmd; // Velocity commands
   cv::RNG _rng; // Random number generator state for random navigation

   // The robot will not move with speeds faster than these, so we better limit out
   //values
   const double _MAX_LIN_VEL; ///< [m/s]
   const double _MAX_ANG_VEL; ///< 90º/s (in rad/s)

   // Pose related information to be retrieved from ROS
   geometry_msgs::Pose2D _odo_robot_pose; ///< Robot pose from odometry
   double _odo_lin_vel, _odo_ang_vel; ///< Velocity computed from odometry
   bool _odom_updated; ///< True if the odometry was updated
   bool _odom_first_update; ///< True if the odometry was never updated

   /// Save robot poses to a file
   std::ofstream _outfile;

   /// Real robot pose from the simulator (for debug purposes only)
   geometry_msgs::Pose2D _real_pose;

   /// Map global variables
   cv::Mat _org_map;
   /// Color map for debugging purposes
   cv::Mat _map;
   /// Control iterations between map file save
   uint _iteration;

   // Store the positions of the landmarks [m]
   point_2d _markers_true_wpos[8];

   // ROS variables/objects
   ros::NodeHandle _nh; // ROS Node handle

   // ROS subscribers
   ros::Subscriber _sub_odom; ///< Subscribe to odometry messages (EKF predict)
   ros::Subscriber _sub_real_pose; ///< Subscribe to real pose messages (debug)
   ros::Subscriber _sub_laser; ///< Subscribe to laser messages (navigation)
   ros::Subscriber _sub_markers; ///< Subscribe to markers messages (EKF update)

   // ROS publishers
   ros::Publisher _vel_pub; // Velocity commands publisher

   // Extended Kalman filter related variables
   ExtendedKalmanFilter _ekf; ///< EKF
   double _last_step_time; // Last time we performed the predict step
};


#endif // TP12_HPP

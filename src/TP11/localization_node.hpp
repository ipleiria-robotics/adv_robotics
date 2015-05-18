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

#ifndef LOCALIZATION_NODE_HPP
#define LOCALIZATION_NODE_HPP

// Headers to read and write from the terminal and/or file
#include <fstream>

// OpenCV headers
#include <cv.hpp>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h> // Velocity messages
#include <nav_msgs/Odometry.h> // Odometry messages
#include <sensor_msgs/LaserScan.h> // Laser sensor messages
#include <tf/tf.h> // Geometry transformations
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "utils.hpp"
#include "ExtendedKalmanFilter.hpp"

class LocalizationNode
{
public:
  // Create a shorthand for writing our PointCloud variables
  typedef cv::Mat_<double> PointCloud; // Will hold x, y

  /**
   * Constructor
   * map_resolution - map resolution in [m/px].
   * map_length - length and width of the map [m].
   * map_border - free space border to add around the map [m].
   * delta_save - number of iterations between map saves to disk.
   * max_icp_iterations - maximum number of ICP iterations
   * min_icp_delta_error - minimum error difference to terminate ICP [m]
   * icp_rate - maximum rate at wich to run ICP [Hz]
   */
   LocalizationNode(const double map_resolution, cv::Mat map,
                    const double map_border, const uint delta_save,
                    const uint max_icp_iterations,
                    const double min_icp_delta_error, const double icp_rate);

   /**
    * Destructor
    */
    ~LocalizationNode();

   /**
     * Show debug information graphically and store it in a file.
     */
   void showDebugInformation();


private:

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
     * Callback used to process laser messages, used for navigation.
     */
   void laserCallback(const sensor_msgs::LaserScan& msg);

   /**
     * Create pose message with the EKF result and publish it
     * stamp - timestamp for the message to be published
     */
   void createAndPublishPose(const ros::Time &stamp);

   // Map related constants
   double _MAP_RESOLUTION; ///< [m/px]
   double _MAP_BORDER; ///< Border around the map [m]
   uint _DELTA_SAVE; ///< Number of iterations between each save of the map

   // Pose related information to be retrieved from ROS
   pose_2d _odo_robot_pose; ///< Robot pose from odometry
   bool _odom_updated; ///< True if the odometry was updated
   bool _odom_first_update; ///< True if the odometry was never updated

   /// Save robot poses to a file
   std::ofstream _outfile, _icpfile;

   /// Real robot pose from the simulator (for debug purposes only)
   geometry_msgs::Pose2D _real_pose;

   /// Map global variables
   cv::Mat _org_map;
   /// Color map for debugging purposes
   cv::Mat _debug_img;
   cv::Mat _debug_icp;
   /// Control iterations between map file save
   uint _iteration;

   /// Cloud of points taken from the original map in [m]
   PointCloud _cloud_map;
   cv::flann::GenericIndex<cvflann::L2_Simple<double> > *_flann_index;
   uint _MAX_ICP_ITERATIONS;
   double _MIN_ICP_DELTA_ERROR;
   double _MIN_ICP_DELTATIME; ///< Minimum time between 2 consecutive ICP calls
   double _last_icp_call_time; ///< Last time ICP was started

   // ROS variables/objects
   ros::NodeHandle _nh; // ROS Node handle

   // ROS subscribers
   ros::Subscriber _sub_odom; ///< Subscribe to odometry messages (EKF predict)
   ros::Subscriber _sub_real_pose; ///< Subscribe to real pose messages (debug)
   ros::Subscriber _sub_laser; ///< Subscribe to laser messages (navigation)
//   ros::Subscriber _sub_markers; ///< Subscribe to markers messages (EKF update)
   ros::Subscriber _sub_pcl; ///< Subscribe to PointCloud messages (EKF update)

   // ROS publishers
   ros::Publisher _ekf_pose_pub; // EKF result publisher
   geometry_msgs::PoseWithCovarianceStamped _pose_msg; // Pose to publish

   // Extended Kalman filter related variables
   ExtendedKalmanFilter _ekf; ///< EKF
};


#endif // LOCALIZATION_NODE_HPP

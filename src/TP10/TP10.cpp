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

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h> // Geometry transformations

// OpenCV header files
#include <cv.hpp>

#include "TP10.hpp"
#include "utils.hpp"
#include "LocalFrameWorldFrameTransformations.hpp"

TP10::TP10(const double map_resolution, const double map_length,
         const double map_border, const uint delta_save,
         const double max_lin_vel, const double max_ang_vel,
         std::string robotname):
  _MAP_RESOLUTION(map_resolution), _MAP_LENGTH(map_length),
  _MAP_BORDER(map_border), _DELTA_SAVE(delta_save),
  _lin_vel(0.0), _ang_vel(0.0), _avoid(false), _rotating(false),
  _rotate_left(false), _MAX_LIN_VEL(max_lin_vel), _MAX_ANG_VEL(max_ang_vel),
  _odom_updated(false), _odom_first_update(true), _iteration(0)
{
  // Specify markers positions
  _markers_wpos[0].x = -8.0; _markers_wpos[0].y =  4.0;
  _markers_wpos[1].x = -8.0; _markers_wpos[1].y = -4.0;
  _markers_wpos[2].x = -3.0; _markers_wpos[2].y = -8.0;
  _markers_wpos[3].x =  3.0; _markers_wpos[3].y = -8.0;
  _markers_wpos[4].x =  8.0; _markers_wpos[4].y = -4.0;
  _markers_wpos[5].x =  8.0; _markers_wpos[5].y =  4.0;
  _markers_wpos[6].x =  3.0; _markers_wpos[6].y =  8.0;
  _markers_wpos[7].x = -3.0; _markers_wpos[7].y =  8.0;

  // Open file to store robot poses
  _outfile.open("Output.txt");

  // Write file header
  _outfile << "Estimated and real pose of the robot\n\n"
           << "[T]: Odometry [X Y Theta] Real [X Y Theta] Particle [X Y Theta]\n\n";

  //
  // Update/create map related variables
  //
  //  We will create an image with the map wich contains the original map plus
  // a border around it, so as to allow estimates outside of the original
  // map.

  // Read original map and resize it to our resolution
  std::string map_file_path(getenv("HOME"));
  map_file_path += "/ros/worlds/stage/cave.png";
  cv::Mat org_full_size_map = cv::imread(map_file_path, CV_LOAD_IMAGE_COLOR);
  if( org_full_size_map.data == 0 )
    std::cerr << "Error reading map, program will shutdown!" << std::endl;
  
  _org_map.create(ceil(_MAP_LENGTH/_MAP_RESOLUTION),
                  ceil(_MAP_LENGTH/_MAP_RESOLUTION),
                  CV_8UC3);
  cv::resize(org_full_size_map, _org_map,
             cv::Size(ceil(_MAP_LENGTH/_MAP_RESOLUTION),
                      ceil(_MAP_LENGTH/_MAP_RESOLUTION)),
             0, 0, cv::INTER_AREA );

  //  This will be our actual map, with the border around it.
  _map.create(ceil((_MAP_LENGTH+_MAP_BORDER)/_MAP_RESOLUTION),
              ceil((_MAP_LENGTH+_MAP_BORDER)/_MAP_RESOLUTION),
              CV_8UC3);
  cv::copyMakeBorder(_org_map, _map,
                     floor((_map.size().height-_org_map.size().height)/2),
                     ceil((_map.size().height-_org_map.size().height)/2),
                     floor((_map.size().width-_org_map.size().width)/2),
                     ceil((_map.size().width-_org_map.size().width)/2),
                     cv::BORDER_CONSTANT,
                     cv::Scalar(255,255,255));
  // Create window for the map
  cv::namedWindow("Debug", 0);

  // Initialize random number generator
  _rng(time(NULL));

  /// Initilize Kalman filter related variables
  // The state holds X, Y and Theta estimate of the robot, all of type double.
  _ekf._state.create(3, 1);
  // Covariance matrix associated with the robot movement
  _ekf._V = (cv::Mat_<double>(3,3) << 0.10*0.10,      0.00,      0.00,
                                      0.00,      0.03*0.03,      0.00,
                                      0.00,           0.00, 0.20*0.20);
  // Process dynamics jacobian regarding the state
  // Only elements (1,3) and (2,3) need to be updated, so we set the correct
  //values for the other ones now.
  _ekf._Fx = (cv::Mat_<double>(3,3) << 1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0);
  // Process dynamics jacobian regarding the inputs
  // Only elements (0,0), (0,1), (1,0) and (1,1) need to be updated, so we set
  // the correct the other ones now.
  _ekf._Fv = (cv::Mat_<double>(3,3) << 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 1.0);
  // Process covariance (confidence associated with current state)
  // (It is called Σ in the theoretic material)
  _ekf._P = cv::Mat::zeros(3, 3, CV_64F);

  // Get parameters
  ros::NodeHandle n_private("~");
  n_private.param("min_front_dist", _min_front_dist, 1.0);
  n_private.param("stop_front_dist", _stop_front_dist, 0.6);

  std::cout << "Random navigation with obstacle avoidance and EKF-based"
            << " localization\n---------------------------" << std::endl;

  /// Setup subscribers
  _sub_real_pose = _nh.subscribe(robotname + "/base_pose_ground_truth", 1,
                                 &TP10::realPoseCallback, this);
  _sub_odom = _nh.subscribe(robotname + "/odom", 10, &TP10::odomCallback, this);
  _sub_laser = _nh.subscribe(robotname + "/base_scan", 1, &TP10::laserCallback, this);
  _sub_markers = _nh.subscribe(robotname + "/markers", 1, &TP10::markersCallback, this);

  /// Setup publisher for linear and angular velocity
  _vel_pub = _nh.advertise<geometry_msgs::Twist>(robotname + "/cmd_vel", 1);

  /// Wait until we get an initial robot pose (from odometry)
  ros::Rate cycle(10.0); // Cycle rate
  while( _odom_updated == false )
  {
    ros::spinOnce();
    cycle.sleep();
  }

  /// Stop the robot (if not stopped already)
  _vel_cmd.linear.x = _lin_vel;
  _vel_cmd.angular.z = _ang_vel;
  _vel_pub.publish(_vel_cmd);
}


TP10::~TP10()
{
  // If we are quitting, stop the robot
  _vel_cmd.angular.z = 0;
  _vel_cmd.linear.x = 0;
  _vel_pub.publish(_vel_cmd);

  // Close file
  _outfile.close();

  // Store final map
  cv::imwrite("mapa.png", _map);
}


void TP10::realPoseCallback(const nav_msgs::Odometry& msg)
{
  // Store real, error-free pose values given by the simulator (for debugging
  // puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
  _real_pose.x = msg.pose.pose.position.x;
  _real_pose.y = msg.pose.pose.position.y;
  _real_pose.theta = tf::getYaw(msg.pose.pose.orientation);
}



void TP10::odomCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::Pose2D old_pose = _odo_robot_pose;

  // Store updated pose values
  _odo_robot_pose.x = msg.pose.pose.position.x;
  _odo_robot_pose.y = msg.pose.pose.position.y;
  _odo_robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);
  // Store velocity computed from odometry
  _odo_lin_vel = msg.twist.twist.linear.x;
  _odo_ang_vel = msg.twist.twist.angular.z;
  // Store timestamp and update flag
  _odom_updated = true;

  // Only perform the prediction update step if we have alread received an
  // odometry message in the past.
  if( !_odom_first_update )
  {
    /// Perform Step 1 of the particle filter
    /// --> Update particles with robot movement:
    // We need to obtain the robot movement in the robot coordinates
    geometry_msgs::Pose2D local_pose;
    world2Local(old_pose, _odo_robot_pose, &local_pose);
    _ekf.predictStep(local_pose.x, local_pose.y, local_pose.theta);
  } else
  {
    // Set initial state value to the odometry information
    _ekf._state(0,0) = _odo_robot_pose.x;
    _ekf._state(1,0) = _odo_robot_pose.y;
    _ekf._state(2,0) = _odo_robot_pose.theta;
    /// @TODO: use ROS given covariance to initialize P
  }


  // First update is done
  _odom_first_update = false;
  _last_predict_time = msg.header.stamp.toSec();

  // Show updated particles
  showDebugInformation();
}


void TP10::markersCallback(const markers_msgs::Markers& msg)
{
  // This callback only makes sense if we have more than one marker
  if( msg.num_markers < 1 )
   return;

  /// Before using the markers information, we need to update the estimated
  /// pose considering the amount of time that elapsed since the last update
  /// from odometry.
  /// We will use the linear and angular velocity of the robot for that.

  /// Step 1 - Predict state with robot movement:
  if( (_odom_first_update == false) &&
      (msg.header.stamp.toSec() - _last_predict_time >= 0.0001) )
  {
    double dt = msg.header.stamp.toSec() - _last_predict_time;
    double distance = _odo_lin_vel*dt; // Estimated travelled distance
    double dtheta = _odo_ang_vel*dt; // Rotation performed
    _ekf.predictStep(distance, 0.0, dtheta);
    _last_predict_time = msg.header.stamp.toSec();

    // Store updated odometry pose
    _odo_robot_pose.x += distance*cos(_odo_robot_pose.theta);
    _odo_robot_pose.y += distance*sin(_odo_robot_pose.theta);
    _odo_robot_pose.theta += dtheta;

  }

  /// Step 2 - Perform the update step based on the detected markers
  _ekf.updateStep(msg, _markers_wpos);

  // Show debug information
  showDebugInformation();

  // Save map from time to time
  _iteration++;
  if( _iteration == _DELTA_SAVE )
  {
    cv::imwrite("mapa.png", _map);
    _iteration = 0;
  }

}


void TP10::laserCallback(const sensor_msgs::LaserScan& msg)
{
  double angle, max_angle;
  unsigned int i;

  /// Update distance to closest front obstacles
  angle = deg2rad(-45);
  i = round((angle - msg.angle_min)/msg.angle_increment);
  double closest_front_obstacle = msg.range_max;
  max_angle = deg2rad(45);
  while( angle < max_angle ) // DEG2RAD(45)
  {
    if( (msg.ranges[i] < msg.range_max) &&
        (msg.ranges[i] > msg.range_min) &&
        (msg.ranges[i] < closest_front_obstacle) )
      closest_front_obstacle = msg.ranges[i];
    i++;
    angle += msg.angle_increment;
  }

  /// Perform navigation base on the detected obstacle
  bool avoid = false;
  if( closest_front_obstacle < _min_front_dist )
  {
    if( closest_front_obstacle < _stop_front_dist )
    {
      avoid = true;
      _lin_vel = -0.100;
    } else
    {
      avoid = true;
      _lin_vel = 0;
    }
  } else
  {
    _lin_vel = 0.5;
    _ang_vel = 0.0;
    _rotating = false;
  }

  if(avoid)
  {
    _lin_vel = 0.0;
    if( _rotating == false )
    {
      // Favor rotation in the same direction as the last rotation
      if( _rng.uniform(0.0, 1.0) < 0.1 )
        _rotate_left = !_rotate_left;

      if( _rotate_left == true )
        _ang_vel = deg2rad(30.0); // Rotate left
      else
        _ang_vel = deg2rad(-30.0); // Rotate right

      _rotating = true;
    }
  }

  // Limit maximum velocities
  // (not needed here)
  _lin_vel = clipValue(_lin_vel, -_MAX_LIN_VEL, _MAX_LIN_VEL);
  _ang_vel = clipValue(_ang_vel, -_MAX_ANG_VEL, _MAX_ANG_VEL);

  // Send velocity commands
  _vel_cmd.angular.z = _ang_vel;
  _vel_cmd.linear.x = _lin_vel;
  _vel_pub.publish(_vel_cmd);

  return;
}


void TP10::showDebugInformation()
{
  /// Write data to the file
  _outfile << std::setiosflags(std::ios::fixed) << std::setprecision(3)
           << _last_predict_time << ": "
           << _odo_robot_pose.x << " " << _odo_robot_pose.y << " "
           << _odo_robot_pose.theta << " "
           << _real_pose.x << " " << _real_pose.y
           << " " << _real_pose.theta <<" "
           << _ekf._state(0,0) << " "
           << _ekf._state(1,0) << " "
           << _ekf._state(2,0) << " "
           << std::endl;

  /// Draw postures in image
  // Real (in green)
  drawPostures(_map, _real_pose.x, _real_pose.y, _real_pose.theta,
               _MAP_RESOLUTION, cv::Scalar(0,255,0));
  // Estimated by odometry (in red)
  drawPostures(_map, _odo_robot_pose.x, _odo_robot_pose.y, _odo_robot_pose.theta,
               _MAP_RESOLUTION, cv::Scalar(0,0,255));
  // Estimated by kalman (in blue)
  drawPostures(_map,
               _ekf._state(0,0), // x
               _ekf._state(1,0), // y
               _ekf._state(2,0), // theta
               _MAP_RESOLUTION, cv::Scalar(255,0,0));
  // Show map with postures
  imshow("Debug", _map);
  // Quit if escape key is pressed
  if( cv::waitKey(5) == 27 )
    ros::shutdown();
}


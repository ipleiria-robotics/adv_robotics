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

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h> // Geometry transformations

// OpenCV header files
#include <cv.hpp>

#include "TP12.hpp"
#include "utils.hpp"
#include "LocalFrameWorldFrameTransformations.hpp"

TP12::TP12(const double map_resolution, const double map_length,
         const double map_border, const uint delta_save,
         const double max_lin_vel, const double max_ang_vel,
         std::string robotname):
  _MAP_RESOLUTION(map_resolution), _MAP_LENGTH(map_length),
  _MAP_BORDER(map_border), _DELTA_SAVE(delta_save),
  _lin_vel(0.0), _ang_vel(0.0), _avoid(false), _rotating(false),
  _rotate_left(false), _MAX_LIN_VEL(max_lin_vel), _MAX_ANG_VEL(max_ang_vel),
  _odom_updated(false), _odom_first_update(true), _iteration(0)
{
  // Specify markers positions (for debug purposes only)
  _markers_true_wpos[0].x = -8.0; _markers_true_wpos[0].y =  4.0;
  _markers_true_wpos[1].x = -8.0; _markers_true_wpos[1].y = -4.0;
  _markers_true_wpos[2].x = -3.0; _markers_true_wpos[2].y = -8.0;
  _markers_true_wpos[3].x =  3.0; _markers_true_wpos[3].y = -8.0;
  _markers_true_wpos[4].x =  8.0; _markers_true_wpos[4].y = -4.0;
  _markers_true_wpos[5].x =  8.0; _markers_true_wpos[5].y =  4.0;
  _markers_true_wpos[6].x =  3.0; _markers_true_wpos[6].y =  8.0;
  _markers_true_wpos[7].x = -3.0; _markers_true_wpos[7].y =  8.0;

  // Open file to store robot poses
  _outfile.open("Output.txt");

  // Write file header
  _outfile << "Estimated and real pose of the robot and landmarks\n\n"
           << "[T] | Real [X Y Theta] | Odometry [X Y Theta] | Kalman [X Y Theta] | Landmarks\n\n";

  //
  // Update/create map related variables
  //
  //  We will create an image with the map wich contains the original map plus
  // a border around it, so as to allow estimates outside of the original
  // map.

  //  Create map related variables for debugging purposes only
  _map.create(ceil(_MAP_LENGTH/_MAP_RESOLUTION),
              ceil(_MAP_LENGTH/_MAP_RESOLUTION),
              CV_8UC3);
  _map.setTo(cv::Scalar(255,255,255)); // Initial full white
  // Create window for the map
  cv::namedWindow("Debug", 0);

  // Initialize random number generator
  _rng(time(NULL));

  /// Initilize EKF related variables
  // Initially we do not have any landmark, so the state holds only x, y and
  // theta estimate of the robot, all of type double, and initially set to 0.
  _ekf._state.create(3, 1);
  _ekf._state.setTo(cv::Scalar(0));
  // Initial state covariance (confidence associated with current state)
  // (It is called Σ in the theoretic material)
  _ekf._P = (cv::Mat_<double>(3,3) << 0.1, 0.0, 0.0,
                                      0.0, 0.1, 0.0,
                                      0.0, 0.0, 0.1);
  // Covariance matrix associated with the robot movement
  _ekf._V = (cv::Mat_<double>(3,3) << 0.10*0.10,      0.00,      0.00,
                                      0.00, 0.03*0.03,      0.00,
                                      0.00,      0.00, 0.20*0.20);
  // Process dynamics jacobian regarding the state
  // Only elements (0,2) and (1,2) need to be updated, so we set the correct
  //values for the other ones now.
  _ekf._Jfxv = (cv::Mat_<double>(3,3) << 1.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0,
                                         0.0, 0.0, 1.0);
  // Process dynamics jacobian regarding the inputs
  // Only elements (0,0), (0,1), (1,0) and (1,1) need to be updated, so we set
  // the correct the other ones now.
  _ekf._Jfuv = (cv::Mat_<double>(3,3) << 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0);
  // Observation (sensor) noise covariance.
  // This could be updated in runtime to take into account that observations
  // of landmarks further away from the robot have larger noise, as done in the
  // EKF projetc.
  _ekf._W = (cv::Mat_<double>(2,2) << 0.1, 0.0,
                                      0.0, 0.1);

  // These matrices will be fully updated on runtime, so no need to set
  //values for them now.
  _ekf._v.create(2, 1); // Inovation factor (error)
  _ekf._s.create(2, 2); // Innovation covariance
  //_ekf._K; // Kalman gain (Size will change over time)
  // Vector for observation values
  _ekf._z.create(2, 1);
  // Vector for observation model (expected observation values)
  _ekf._h.create(2, 1);
  // Matrix for the observation model jacobian
  _ekf._Jh.create(2, 3+_ekf._num_landmarks);

  // Jacobian of the landmark world position observation with respect to the
  // robot pose. Only the last columm will need update in runtime.
  _ekf._Jhnxv = (cv::Mat_<double>(2,3) <<  1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0);
  // Jacobian of the landmark world position observation with respect to the
  // robot observation values. Will be fully updated in runtime
  _ekf._Jhnz.create(2, 2);


  // Get parameters
  ros::NodeHandle n_private("~");
  n_private.param("min_front_dist", _min_front_dist, 1.0);
  n_private.param("stop_front_dist", _stop_front_dist, 0.6);

  std::cout << "Random navigation with obstacle avoidance and EKF-based"
            << " localization\n---------------------------" << std::endl;

  /// Setup subscribers
  _sub_real_pose = _nh.subscribe(robotname + "/base_pose_ground_truth", 1,
                                 &TP12::realPoseCallback, this);
  _sub_odom = _nh.subscribe(robotname + "/odom", 1, &TP12::odomCallback, this);
  _sub_laser = _nh.subscribe(robotname + "/base_scan", 1, &TP12::laserCallback, this);
  _sub_markers = _nh.subscribe(robotname + "/markers", 1, &TP12::markersCallback, this);

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


TP12::~TP12()
{
  // If we are quitting, stop the robot
  _vel_cmd.angular.z = 0;
  _vel_cmd.linear.x = 0;
  _vel_pub.publish(_vel_cmd);

  // Close file
  _outfile.close();

  // One last debug information (without erasing the landmarks)
  showDebugInformation("", false);

  // Store final map
  cv::imwrite("mapa.png", _map);
}


void TP12::realPoseCallback(const nav_msgs::Odometry& msg)
{
  // Store real, error-free pose values given by the simulator (for debugging
  // puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
  _real_pose.x = msg.pose.pose.position.x;
  _real_pose.y = msg.pose.pose.position.y;
  _real_pose.theta = tf::getYaw(msg.pose.pose.orientation);
}



void TP12::odomCallback(const nav_msgs::Odometry& msg)
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
//    _ekf._state(0,0) = _odo_robot_pose.x;
//    _ekf._state(1,0) = _odo_robot_pose.y;
//    _ekf._state(2,0) = _odo_robot_pose.theta;
//    /// @TODO: use ROS given covariance to initialize P

    // Uncomment the lines above if you want to set the initial pose
    // to the coordinates given by ROS. Currently we ignore that information
    // and consider the robot to start at (0,0,0).
    _ekf._state(0,0) = 0.0;
    _ekf._state(1,0) = 0.0;
    _ekf._state(2,0) = 0.0;
    // First update is done
    _odom_first_update = false;
  }


  _last_step_time = msg.header.stamp.toSec();

  // Show updated particles
  showDebugInformation("Step 1:");
}


void TP12::markersCallback(const markers_msgs::Markers& msg)
{
  // This callback only makes sense if we have more than one marker
  if( msg.num_markers < 1 )
  {
    std::cerr << "Got markers message with no markers!" << std::endl;
    return;
  }

  // Do not perform this step if the information obtained is old
  if( msg.header.stamp.toSec() < _last_step_time )
  {
    std::cout << "Got markers messages with old timestamp" << std::endl;
    return;
  }

  /// Before using the markers information, we need to update the estimated
  /// pose considering the amount of time that elapsed since the last update
  /// from odometry.
  /// We will use the linear and angular velocity of the robot for that.

  /// Step 1 - Predict state with robot movement:
  if( (_odom_first_update == false) &&
      (msg.header.stamp.toSec() > _last_step_time) )
  {
    double dt = msg.header.stamp.toSec() - _last_step_time;
    double distance = _odo_lin_vel*dt; // Estimated travelled distance
    double dtheta = _odo_ang_vel*dt; // Rotation performed
    _ekf.predictStep(distance, 0.0, dtheta);
    _last_step_time = msg.header.stamp.toSec();

    // Store updated odometry pose
    _odo_robot_pose.x += distance*cos(_odo_robot_pose.theta);
    _odo_robot_pose.y += distance*sin(_odo_robot_pose.theta);
    _odo_robot_pose.theta += dtheta;

    // Output debug information to a file (do not update image yet)
    outputDebugInfoToFile("Step 2.1:");
  }

  /// Step 2 - Perform the update step based on the detected markers
  _ekf.updateStep(msg);

  // Show debug information
  showDebugInformation("Step 2.2:");

}


void TP12::laserCallback(const sensor_msgs::LaserScan& msg)
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


void TP12::outputDebugInfoToFile(std::string header)
{
  /// Write data to the file
  if( header.length() > 0 )
    _outfile << header << "\n";
  _outfile << std::setiosflags(std::ios::fixed) << std::setprecision(3)
           << _last_step_time << " | "
           << _real_pose.x << " " << _real_pose.y
           << " " << _real_pose.theta << " | "
           << _odo_robot_pose.x << " " << _odo_robot_pose.y << " "
           << _odo_robot_pose.theta << " | "
           << _ekf._state(0,0) << " " << _ekf._state(1,0)
           << " " << _ekf._state(2,0);
  for(uint n=0; n < _ekf._num_landmarks; n++)
    _outfile << std::setiosflags(std::ios::fixed) << std::setprecision(3)
             << " | " << n << ": " << _ekf._state(3+2*n,0)
             << " " << _ekf._state(3+2*n+1,0);
  _outfile << std::endl;
}


void TP12::showDebugInformation(std::string header, bool erase_landmarks)
{
  /// Write data to the file
  outputDebugInfoToFile(header);

  /// Draw information on image
  // Real (in green)
  drawPos(_map, _real_pose.x, _real_pose.y, _real_pose.theta,
          _MAP_RESOLUTION, cv::Scalar(0,255,0));
  // Estimated by odometry (in red)
  drawPos(_map, _odo_robot_pose.x, _odo_robot_pose.y, _odo_robot_pose.theta,
          _MAP_RESOLUTION, cv::Scalar(0,0,255));
  // Estimated by EKF (in blue)
  drawPos(_map, _ekf._state(0,0), _ekf._state(1,0),
          _ekf._state(2,0),
          _MAP_RESOLUTION, cv::Scalar(255,0,0));
  //  Draw landmarks true positions
  for(uint n=0; n < 8; n++)
    drawPos(_map, _markers_true_wpos[n].x, _markers_true_wpos[n].y, 0,
            _MAP_RESOLUTION, cv::Scalar(0,255,0), false, 5);
  // Draw landmarks estimates
  for(uint n=0; n < _ekf._num_landmarks; n++)
    drawPos(_map, _ekf._state(3+2*n,0),
            _ekf._state(4+2*n,0), 0, _MAP_RESOLUTION,
            cv::Scalar(255,0,0), false, 5);

  // Show map with postures
  imshow("Debug", _map);

  // Quit if escape key is pressed
  if( cv::waitKey(5) == 27 )
    ros::shutdown();

  // Save map from time to time
  _iteration++;
  if( _iteration == _DELTA_SAVE )
  {
    cv::imwrite("mapa.png", _map);
    _iteration = 0;
  }

  // Erase landmarks estimates (to keep the map viewable)
  if( erase_landmarks )
    for(uint n=0; n < _ekf._num_landmarks; n++)
      drawPos(_map, _ekf._state(3+2*n,0),
              _ekf._state(4+2*n,0), 0, _MAP_RESOLUTION,
              cv::Scalar(255,255,255), true, 5);

}


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
#include <markers_msgs/Markers.h> // Markers messages
#include <tf/tf.h> // Geometry transformations

// OpenCV header files
#include <cv.h>
#include <highgui.h>

// Coordinate transformations
#include "LocalFrameWorldFrameTransformations.hpp"
#include "utils.hpp"

// Specify if the particle filter steps should run
#define STEP_PREDICTION
#define STEP_UPDATE
#define STEP_RESAMPLE


// Radians <-> Degrees convertion tools
#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees

// The robot will not move with speeds faster than these, so we better limit out
//values
#define MAX_LIN_VEL 1.0 // [m/s]
#define MAX_ANG_VEL 1.57 // 90º/s (in rad/s)

// Map related constants
#define MAP_RESOLUTION 0.032 // [m/px]
#define MAP_LENGTH 16.0 // Width and height of the map [m]
#define SAFETY_BORDER 0.4 // Border around the map [m]
#define DELTA_SAVE 100 // Number of iterations between each save of the map

// Particle filter constants
#define NUM_PARTICLES 200 // Number of particles used in the particle filter
#define LANDMARK_RANGE 8.0 // Maximum distance to detectable landmarks [m]
#define LANDMARK_FOV 180.0 // Landmarks are detectable if in front of robot [º]
#define DISTANCE_ERROR_GAIN 0.5 // Gain factor when computing the weights from distance
#define ANGLE_ERROR_GAIN 5.0 // Gain factor when computing the weights from the angle
#define SIGMA1 0.1 // Distance standard deviation error [m]
#define SIGMA2 DEG2RAD(0.2) // Theta standard deviation error [rad]

// Save robot poses to a file
std::ofstream outfile;

// Pose and laser information
geometry_msgs::Pose2D odo_robot_pose; // Robot posed from odometry
double odo_lin_vel, odo_ang_vel; // Velocity computed from odometry
bool odom_updated = false; // True if the odometry was updated
bool odom_first_update = true; // True if the odometry was never updated

// Real robot pose from the simulator (for debug purposes only)
geometry_msgs::Pose2D real_pose;


// Particle filter global variables
// Vector of NUM_PARTICLES particles, where each particle corresponds to a
// possible robot pose (X,Y,Theta).
geometry_msgs::Pose2D posture_estimate;
cv::Mat particles_x(NUM_PARTICLES, 1, CV_64F);
cv::Mat particles_y(NUM_PARTICLES, 1, CV_64F);
cv::Mat particles_theta(NUM_PARTICLES, 1, CV_64F);
// Particles weight
cv::Mat particles_weight(NUM_PARTICLES, 1, CV_64F, 1.0/NUM_PARTICLES);
// Random number genereator state (for the particle filter and sensor distance
//based random navigation)
cv::RNG rng(time(NULL));
double last_step1_time; // Last time we performed the Particle Filter step 1
// The old particles vectors will be needed in the resampling algorithm
cv::Mat old_particles_x(NUM_PARTICLES, 1, CV_64F);
cv::Mat old_particles_y(NUM_PARTICLES, 1, CV_64F);
cv::Mat old_particles_theta(NUM_PARTICLES, 1, CV_64F);
double max_weight; // Weight of the best particle

// Map global variables
cv::Mat org_map(ceil(MAP_LENGTH/MAP_RESOLUTION),
                ceil(MAP_LENGTH/MAP_RESOLUTION),
                CV_8UC1);
// Color map for debugging purposes
cv::Mat map(ceil(MAP_LENGTH/MAP_RESOLUTION),
            ceil(MAP_LENGTH/MAP_RESOLUTION),
            CV_8UC3);
cv::Mat tmp_map;
// Control iterations between map file save
int iteration = 0;

//
// Create robot related variables
//
// Linear and angular velocities for the robot (initially stopped)
double lin_vel = 0.0, ang_vel=0.0;
bool avoid = false, rotating = false;
bool rotate_left = false;
double stop_front_dist, min_front_dist;
ros::Publisher vel_pub; // Velocity commands publisher
geometry_msgs::Twist vel_cmd; // Velocity commands

// Store the positions of the landmarks [m]
point_2d_t markers_wpos[8] = {{-8.0,  4.0}, // 1
                              {-8.0, -4.0}, // 2
                              {-3.0, -8.0}, // 3
                              { 3.0, -8.0}, // 4
                              { 8.0, -4.0}, // 5
                              { 8.0,  4.0}, // 6
                              { 3.0,  8.0}, // 7
                              {-3.0,  8.0},}; // 8

void showDebugInformation()
{
  //////////////////////////////////////////////////////////////////////////
  // Debug steps start here
  //
  // We draw all the debug information (particles, etc.) before the
  // resampling step, since the particles weight will be changed, and some
  // particles will be lost in the process.
  //////////////////////////////////////////////////////////////////////////
  /// Write data to the file
  outfile << std::setiosflags(std::ios::fixed) << std::setprecision(3)
          << last_step1_time << ": "
          << odo_robot_pose.x << " " << odo_robot_pose.y << " "
                                     << odo_robot_pose.theta << " "
          << real_pose.x << " " << real_pose.y << " " << real_pose.theta <<" "
          << posture_estimate.x << " " << posture_estimate.y << " "
                                        << posture_estimate.theta << " "
          << "\n";

  // Draw postures in image
  //map.setTo(cvScalar(255,255,255)); // Erase previous debug information
  // Real (in green)
  drawPostures(map, real_pose.x, real_pose.y, real_pose.theta,
               MAP_RESOLUTION, cv::Scalar(0,255,0));
  // Estimated by odometry (in magenta)
  drawPostures(map, odo_robot_pose.x, odo_robot_pose.y, odo_robot_pose.theta,
               MAP_RESOLUTION, cv::Scalar(255,0,255));
  // Estimated by particle filter (in blue)
  drawPostures(map,
               posture_estimate.x,
               posture_estimate.y,
               posture_estimate.theta,
               MAP_RESOLUTION, cv::Scalar(255,0,0));

  map.copyTo(tmp_map);
  // Draw all particles
  // The intensity color (highest is red, lowest is yellow) varies with the
  // particle weight.
  for(int j = 0; j < NUM_PARTICLES; j++)
    drawPostures(tmp_map,
                 particles_x.at<double>(j,0),
                 particles_y.at<double>(j,0),
                 particles_theta.at<double>(j,0),
                 MAP_RESOLUTION,
                 cv::Scalar(0,cvRound(255*(1.0-particles_weight.at<double>(j,0)/max_weight)),255));
  // Redraw estimate by particle filter (in blue), to stay on top
  drawPostures(map,
               posture_estimate.x,
               posture_estimate.y,
               posture_estimate.theta,
               MAP_RESOLUTION, cv::Scalar(255,0,0));

  // Show map with postures
  cv::imshow("Debug", tmp_map);
  // Quit if escape key is pressed
  if( cv::waitKey(5) == 27 )
    ros::shutdown();

  //////////////////////////////////////////////////////////////////////////
  // Debug steps end here
  //////////////////////////////////////////////////////////////////////////
}

void ParticleFilterStep1(double distance, double dtheta)
{
#ifdef STEP_PREDICTION
  //
  // Δd = Δd + Δd*normal_noise(sigma1)
  // Δθ = Δθ + Δθ*normal_noise(sigma2)
  //
  // | x(k+1|x(k),u(k)) | = | x(k) +  Δx(k) |
  // | y(k+1|x(k),u(k)) | = | y(k) +  Δy(k) |
  // | θ(k+1|x(k),u(k)) | = | θ(k) +  Δθ(k) |
  //  where
  // Δx(k) = Δd*cos(θ(k))
  // Δy(k) = Δd*sin(θ(k))
  // Δθ(k) = Δθ(k)
  //

  // This could be vectorized to improved speed and avoid the for loop!
  for(int n = 0; n < NUM_PARTICLES; n++)
  {
    distance *= 1.0 + rng.gaussian(SIGMA1);
    dtheta *= 1.0 + rng.gaussian(SIGMA2);
    double delta_X = distance*cos(particles_theta.at<double>(n,0));
    double delta_Y = distance*sin(particles_theta.at<double>(n,0));
    double delta_Theta = dtheta*(1+dtheta*rng.gaussian(SIGMA2));

    particles_x.at<double>(n,0) += delta_X;
    particles_y.at<double>(n,0) += delta_Y;
    particles_theta.at<double>(n,0) += delta_Theta;

    //  Check if any of particles is outside the map our in an occupied cell.
    // If so, move that particle to the previous position (the orientation
    // is maintained).
    if( ((particles_x.at<double>(n,0) >  MAP_LENGTH/2.0) ||
         (particles_x.at<double>(n,0) < -MAP_LENGTH/2.0) ||
         (particles_y.at<double>(n,0) >  MAP_LENGTH/2.0) ||
         (particles_y.at<double>(n,0) < -MAP_LENGTH/2.0) ) ||
        (org_map.at<uchar>(
           cvRound(org_map.size().height/2.0
                   - particles_y.at<double>(n,0)/MAP_RESOLUTION),
           cvRound(org_map.size().width/2.0
                   + particles_x.at<double>(n,0)/MAP_RESOLUTION)) < 127) )
    {
      // Undo this particle update
      particles_x.at<double>(n,0) -= delta_X;
      particles_y.at<double>(n,0) -= delta_Y;
    }
    // Normalize the orientation
    particles_theta.at<double>(n,0) = normalize(particles_theta.at<double>(n,0));
  }
#endif
}


void realPoseCallback(const nav_msgs::Odometry& msg)
{
  // Store real, error-free pose values given by the simulator (for debugging
  // puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
  real_pose.x = msg.pose.pose.position.x;
  real_pose.y = msg.pose.pose.position.y;
  real_pose.theta = tf::getYaw(msg.pose.pose.orientation);
}


void odomCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::Pose2D old_pose = odo_robot_pose;

  // Store updated pose values
  odo_robot_pose.x = msg.pose.pose.position.x;
  odo_robot_pose.y = msg.pose.pose.position.y;
  odo_robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);
  // Store velocity computed from odometry
  odo_lin_vel = msg.twist.twist.linear.x;
  odo_ang_vel = msg.twist.twist.angular.z;
  // Store timestamp and update flag
  odom_updated = true;

  // Only perform the particle filter update step if we have alread received an
  // odometry message in the past.
  if( !odom_first_update )
  {
    /// Perform Step 1 of the particle filter
    /// --> Update particles with robot movement:
    // We need to obtain the robot movement in the robot coordinates
    geometry_msgs::Pose2D local_pose;
    world2Local(old_pose, odo_robot_pose, &local_pose);
    ParticleFilterStep1(local_pose.x, local_pose.theta);
  }

  // First update is done
  odom_first_update = false;
  last_step1_time = msg.header.stamp.toSec();

  // Show updated particles
  showDebugInformation();
}


void markersCallback(const markers_msgs::Markers& msg)
{
  // This callback only makes sense if we have more than one marker
  if( msg.num_markers < 1 )
   return;

  /// Before using the markers information, we need to update the estimated
  /// pose considering the amount of time that elapsed since the last update
  /// from odometry.
  /// We will use the linear and angular velocity of the robot for that.

  /// Step 1 - Update particles with robot movement:
  if( odom_first_update == false )
  {
    double dt = msg.header.stamp.toSec() - last_step1_time;
    double distance = odo_lin_vel*dt; // Distance travelled
    double dtheta = odo_ang_vel*dt; // Rotation performed
    ParticleFilterStep1(distance, dtheta);
    last_step1_time = msg.header.stamp.toSec();

    // Store updated odometry pose
    odo_robot_pose.x += distance*cos(odo_robot_pose.theta);
    odo_robot_pose.y += distance*sin(odo_robot_pose.theta);
    odo_robot_pose.theta += dtheta;

  }

  /// Step 2 - Update the particle weights given the sensor model and map
  /// knowledge
#ifdef STEP_UPDATE
  // For now we only perform this step if there was any marker detected.
  // We could use the expected and not detected beacons for each particle,
  // but it woul become too expensive.
  //
  // The weight for each particle will be:
  //   w(j) = mean(1/(1+sqrt((x_e(i)-x_r(i))^2+(y_e(i)-y_r(i))^2)*DISTANCE_ERROR_GAIN))
  // where x_e(i)/y_e(i) and x_r(i)/y_r(i) are the expected and obtained
  // x/y world coordinates of the detected marker respectively. The GAIN are
  // constant gains wich can be tuned to value smaller detection errors.
  //

  // Reset normalization factor
  double norm_factor = 0;
  for(int j = 0; j < NUM_PARTICLES; j++)
  {
    // Compute the weight for each particle
    // For each obtained beacon value
    particles_weight.at<double>(j,0) = 0;
    for( uint n=0; n < msg.num_markers; n++ )
    {
      // Obtain beacon position in world coordinates
      geometry_msgs::Pose2D particle;
      particle.x = particles_x.at<double>(j,0);
      particle.y = particles_y.at<double>(j,0);
      particle.theta = particles_theta.at<double>(j,0);

      point_2d marker_lpos = {msg.range[n]*cos(msg.bearing[n]),
                              msg.range[n]*sin(msg.bearing[n])};
      point_2d marker_wpos;
      local2World( particle, marker_lpos, &marker_wpos );

      particles_weight.at<double>(j,0) +=
          1.0/(1+sqrt(pow(markers_wpos[msg.id[n]-1].x-marker_wpos.x,2)
                      +pow(markers_wpos[msg.id[n]-1].y-marker_wpos.y,2)*DISTANCE_ERROR_GAIN));
    }
    // Perform the mean. We summed all elements above and now divide by
    // the number of elements summed.
    particles_weight.at<double>(j,0) /= msg.num_markers;
    // Update the normalization factor
    norm_factor += particles_weight.at<double>(j,0);
  }

  // Normalize the weight
  max_weight = 0.0;
  for(int j = 0; j < particles_x.size().height; j++)
  {
    particles_weight.at<double>(j,0) /= norm_factor;
    // Store the particle with the best weight has our posture estimate
    if( particles_weight.at<double>(j,0) > max_weight )
    {
      posture_estimate.x = particles_x.at<double>(j,0);
      posture_estimate.y = particles_y.at<double>(j,0);
      posture_estimate.theta = particles_theta.at<double>(j,0);
      // This max_factor is just used for debug, so that we have more
      // different colors between particles.
      max_weight = particles_weight.at<double>(j,0);
    }

  }
#endif

  // Show debug information
  showDebugInformation();

  /// Step 3 - Resample
#ifdef STEP_RESAMPLE
  // The resampling step is the exact implementation of the algorithm
  // described in the theoretical classes, i.e., the "Importance resampling
  // algorithm".

  // Save the current values
  // The old particles will be needed in the resampling algorithm
  cv::Mat old_particles_weight(NUM_PARTICLES, 1, CV_64F, 1.0/NUM_PARTICLES);

  particles_x.copyTo(old_particles_x);
  particles_y.copyTo(old_particles_y);
  particles_theta.copyTo(old_particles_theta);
  particles_weight.copyTo(old_particles_weight);

  double delta = rng.uniform(0.0, 1.0/NUM_PARTICLES);
  double c = old_particles_weight.at<double>(0,0);
  int i = 0;
  for(int j = 0; j < NUM_PARTICLES; j++)
  {
    double u = delta + j/(1.0*NUM_PARTICLES);
    while( u > c )
    {
      i++;
      c += old_particles_weight.at<double>(i,0);
    }
    particles_x.at<double>(j,0) = old_particles_x.at<double>(i,0);
    particles_y.at<double>(j,0) = old_particles_y.at<double>(i,0);
    particles_theta.at<double>(j,0) = old_particles_theta.at<double>(i,0);
    // The weight is indicative only, it will be recomputed on marker detection
    particles_weight.at<double>(j,0) = old_particles_weight.at<double>(i,0);
  }
#endif
  ///
  /// Particle filter steps end here
  ///

  // Save map from time to time
  iteration++;
  if( iteration == DELTA_SAVE )
  {
    cv::imwrite("mapa.png", map);
    iteration = 0;
  }

}


void laserCallback(const sensor_msgs::LaserScan& msg)
{
  double angle;
  unsigned int i;

  /// Update distance to closest front obstacles
  angle = -0.785; // DEG2RAD(-45)
  i = round((angle - msg.angle_min)/msg.angle_increment);
  double closest_front_obstacle = msg.range_max;
  while( angle < 0.785 ) // DEG2RAD(45)
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
    ang_vel = 0.0;
    rotating = false;
  }

  if(avoid)
  {
    lin_vel = 0.0;
    if( rotating == false )
    {
      // Favor rotation in the same direction as the last rotation
      if( rng.uniform(0.0, 1.0) < 0.1 )
        rotate_left = !rotate_left;

      if( rotate_left == true )
        ang_vel = DEG2RAD(30.0); // Rotate left
      else
        ang_vel = DEG2RAD(-30.0); // Rotate right

      rotating = true;
    }
  }

  // Limit maximum velocities
  // (not needed here)
  lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
  ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

  // Send velocity commands
  vel_cmd.angular.z = ang_vel;
  vel_cmd.linear.x = lin_vel;
  vel_pub.publish(vel_cmd);

  return;
}


/**
 * Main function
 * Navigate randomly using distance sensor data and Particle Filter-based
 * localization.
 */
int main(int argc, char** argv)
{
  std::string robot_name = "/robot_0";

  // Open file to store robot poses
  outfile.open("Output.txt");

  // Write file header
  outfile << "Estimated and real pose of the robot\n\n"
          << "[T]: Odometry [X Y Theta] Real [X Y Theta] Particle [X Y Theta]\n\n";

  //
  // Create map related variables
  //
  //  We will create an image with the map wich contains the original map plus
  // a border around it, so as to allow estimates outside of the original
  // map.
  cv::Mat cave_map = cv::imread("cave2.png", CV_LOAD_IMAGE_COLOR);
  if( cave_map.data == 0 )
  {
      std::cerr << "Error reading map!" << std::endl;
      return -1;
  }

  // Read original map and resize it to our resolution
  cv::resize(cave_map, map, map.size(), 0, 0, cv::INTER_AREA );

  // Read original map and resize it to our resolution
  //  We need the original map, black and white, so that it is faster to find
  // occupied cells
  cv::resize(cv::imread("cave2.png", CV_LOAD_IMAGE_GRAYSCALE),
             org_map,
             org_map.size(),
             0, 0, cv::INTER_AREA );

  // We need a temporary map
  tmp_map = map.clone();

  // Create window for the map
  cv::namedWindow("Debug", 0);

  //
  // Particle filter related variables with their initial values
  //

  // This variable will hold our final estimation at each iteration
  geometry_msgs::Pose2D pose_estimate;

  // Initialize particles with uniform random distribution across all space
  cv::randu( particles_x,
            -MAP_LENGTH/2.0+SAFETY_BORDER,
             MAP_LENGTH/2.0-SAFETY_BORDER); // X
  cv::randu( particles_y,
        -MAP_LENGTH/2.0+SAFETY_BORDER,
         MAP_LENGTH/2.0-SAFETY_BORDER); // Y
  cv::randu( particles_theta, -CV_PI, CV_PI ); // Theta

  // Re-add particles that are outside of the map or inside an obstacle, until
  // it is in a free space inside the map.
  for(int n = 0; n < NUM_PARTICLES; n++)
  {
    while( ( (particles_x.at<double>(n,0) >  MAP_LENGTH/2.0) ||
             (particles_x.at<double>(n,0) < -MAP_LENGTH/2.0) ||
             (particles_y.at<double>(n,0) >  MAP_LENGTH/2.0) ||
             (particles_y.at<double>(n,0) < -MAP_LENGTH/2.0) ) ||
           (org_map.at<uchar>(
            cvRound(org_map.size().height/2.0
                    - particles_y.at<double>(n,0)/MAP_RESOLUTION),
            cvRound(org_map.size().width/2.0
                    + particles_x.at<double>(n,0)/MAP_RESOLUTION)) < 127) )
    {
      particles_x.at<double>(n,0) = rng.uniform(-MAP_LENGTH/2.0+SAFETY_BORDER, MAP_LENGTH/2.0-SAFETY_BORDER);
      particles_y.at<double>(n,0) = rng.uniform(-MAP_LENGTH/2.0+SAFETY_BORDER, MAP_LENGTH/2.0-SAFETY_BORDER);
    }
  }

  // Init ROS
  ros::init(argc, argv, "tp7");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle

  // Get parameters
  ros::NodeHandle n_private("~");
  n_private.param("min_front_dist", min_front_dist, 1.0);
  n_private.param("stop_front_dist", stop_front_dist, 0.6);

  std::cout << "Random navigation with obstacle avoidance and particle filter"
            << " based localization\n---------------------------" << std::endl;

  /// Setup subscribers
  // Odometry
  ros::Subscriber sub_odom = nh.subscribe(robot_name + "/odom", 10, odomCallback);
  // Real, error-free robot pose (for debug purposes only)
  ros::Subscriber sub_real_pose = nh.subscribe(robot_name + "/base_pose_ground_truth", 1, realPoseCallback);
  // Laser scans
  ros::Subscriber sub_laser = nh.subscribe(robot_name + "/base_scan", 1, laserCallback);
  // Markers detected
  ros::Subscriber sub_markers = nh.subscribe(robot_name + "/markers", 1, markersCallback);

  // Setup publisher for linear and angular velocity
  vel_pub = nh.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 1);

  ros::Rate cycle(10.0); // Cycle rate

  // Wait until we get the robot pose (from odometry)
  while( odom_updated == false )
  {
    ros::spinOnce();
    cycle.sleep();
  }

  // Stop the robot (if not stopped already)
  vel_cmd.angular.z = 0;
  vel_cmd.linear.x = 0;
  vel_pub.publish(vel_cmd);

  // Infinite loop (will call the callbacks whenever information is available,
  // until ros::shutdown() is called.
  ros::spin();

  // If we are quitting, stop the robot
  vel_cmd.angular.z = 0;
  vel_cmd.linear.x = 0;
  vel_pub.publish(vel_cmd);

  // Close file
  outfile.close();

  // Store final map
  cv::imwrite("mapa.png", map);

  return 1;
}

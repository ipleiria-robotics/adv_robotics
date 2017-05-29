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
#include <flann/flann.hpp>

#include "localization_node.hpp"
#include "utils.hpp"
#include "LocalFrameWorldFrameTransformations.hpp"

#define USE_ODOM 1
#define FULL_ICP_DEBUG 0
#define BASIC_ICP_DEBUG 1

LocalizationNode::LocalizationNode(
    const double map_resolution, cv::Mat map,
    const double map_border, const uint delta_save,
    const uint max_icp_iterations, const double min_icp_delta_error,
    const double icp_rate):
  _MAP_RESOLUTION(map_resolution), _MAP_BORDER(map_border),
  _DELTA_SAVE(delta_save), _odom_updated(false), _odom_first_update(true),
  _org_map(map), _iteration(0), _flann_index(0),
  _MAX_ICP_ITERATIONS(max_icp_iterations),
  _MIN_ICP_DELTA_ERROR(min_icp_delta_error), _MIN_ICP_DELTATIME(1.0/icp_rate),
  _last_icp_call_time(-2.0/icp_rate)
{

  // Open file to store robot poses
  _outfile.open("Output.txt");

#if FULL_ICP_DEBUG
  // Open file to store robot poses
  _icpfile.open("ICP.txt");
#endif

  // Write file header
  _outfile << "Estimated and real pose of the robot\n\n"
           << "[T] | Odometry [X Y Theta] | Real [X Y Theta] | EKF [X Y Theta]\n\n";

  //
  // Update/create map related variables
  //
  //  We will create an image with the map wich contains the original map plus
  // a border around it, so as to allow estimates outside of the original
  // map.

  //  This will be our actual map, with the border around it.
  _debug_img.create(ceil((_org_map.size().height +_MAP_BORDER/_MAP_RESOLUTION)),
                    ceil((_org_map.size().width +_MAP_BORDER/_MAP_RESOLUTION)),
                    CV_8UC3);
  // Fill with white
  _debug_img.setTo(cv::Scalar(255,255,255));
  // Copy original map to our debug image (centered)
  cv::Point start_pt(floor((_debug_img.size().width-_org_map.size().width)/2),
                     floor((_debug_img.size().height-_org_map.size().height)/2));
  cv::cvtColor(_org_map, _debug_img(cv::Range(start_pt.y,start_pt.y+_org_map.size().height),
                                    cv::Range(start_pt.x,start_pt.x+_org_map.size().width)),
               CV_GRAY2RGB);
  // Create window for the map
  cv::namedWindow("Debug", 0);
#if BASIC_ICP_DEBUG
  _debug_icp.create(ceil((_org_map.size().height +_MAP_BORDER/_MAP_RESOLUTION)),
                    ceil((_org_map.size().width +_MAP_BORDER/_MAP_RESOLUTION)),
                    CV_8UC3);
  // Fill with white
  _debug_icp.setTo(cv::Scalar(255,255,255));
  // Create window for the ICP output
  cv::namedWindow("Debug ICP", 0);
#endif

  /// Initilize Cloud based on map
  // Get total number of points in original map (will be the size of the cloud).
  // We compute this value as img_width_img_height-num_non_zero_points, since
  // the walls are black in our original map.
  uint num_map_points = _org_map.size().width*_org_map.size().height
                        -cv::countNonZero(_org_map);
  // Set correct number of points for the map cloud
  _cloud_map = cv::Mat_<double>(num_map_points,2);
  //_cloud_map.resize(num_map_points);
  // Now go through the image and store all the wall points in the cloud
  uint i=0;
  for(int y = 0; y < _org_map.rows; y++)
  {
    for(int x = 0; x < _org_map.cols; x++)
    {
      // If this pixel is 0, then it is a wall point. Add it to the cloud.
      if( _org_map.at<uchar>(y, x) == 0 )
      {
        _cloud_map(i,0) =  (x-_org_map.size().width/2)*_MAP_RESOLUTION;
        _cloud_map(i,1) = -(y-_org_map.size().height/2)*_MAP_RESOLUTION;
        i++;
      }
    }
  }
  // Sanity check. The number of read points must be equal to the number of
  // expected points
  if( i != num_map_points )
  {
    ROS_ERROR("Expected %d points but got %d points\n", num_map_points, i );
    ros::shutdown();
  }
  // Build flann object for nearest neighbours search
  _flann_index = new cv::flann::GenericIndex<cvflann::L2_Simple<double> >(_cloud_map, cvflann::KDTreeIndexParams(2));

  /// Initilize Kalman filter related variables
  // The state holds X, Y and Theta estimate of the robot, all of type double.
  _ekf._state.create(3, 1);
  _ekf._state.setTo(cv::Scalar(0));
  // Covariance matrix associated with the robot movement
  _ekf._V = (cv::Mat_<double>(3,3) << 0.10*0.10,      0.00,      0.00,
                                           0.00, 0.03*0.03,      0.00,
                                           0.00,      0.00, 0.20*0.20);
  // Process dynamics jacobian regarding the state
  // Only elements (1,3) and (2,3) need to be updated, so we set the correct
  //values for the other ones now.
  _ekf._Fx = (cv::Mat_<double>(3,3) << 1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0);
  // Process dynamics jacobian regarding the inputs
  // Only elements (0,0), (0,1), (1,0) and (1,1) need to be updated, so we set
  // the correct the other ones now.
  _ekf._Fu = (cv::Mat_<double>(3,3) << 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 1.0);
  // Process covariance (confidence associated with current state)
  // Consider medium confidence in the initial state
  // (It is called Σ in the theoretic material)
  _ekf._P = (cv::Mat_<double>(3,3) << 0.5, 0.0, 0.0,
                                      0.0, 0.5, 0.0,
                                      0.0, 0.0, 0.5);

  // Sensors measure jacobiann
  // In this case its is the identity matrix
  _ekf._H = cv::Mat_<double>::eye(cv::Size(3,3));

  // Sensors covariance (confidence associated with sensor values)
//  _ekf._W = (cv::Mat_<double>(3,3) << 0.15*0.15,      0.00,      0.00,
//                                           0.00, 0.10*0.10,      0.00,
//                                           0.00,      0.00, 0.25*0.25);

  std::cout << "Random navigation with obstacle avoidance and EKF-based"
            << " localization using ICP \n-----------------------" << std::endl;

  /// Setup subscribers
  std::string robot_name = "/robot_0";
  _sub_real_pose = _nh.subscribe(robot_name+"/base_pose_ground_truth", 1,
                                 &LocalizationNode::realPoseCallback, this);
  _sub_odom = _nh.subscribe(robot_name+"/odom", 1, &LocalizationNode::odomCallback, this);
  _sub_laser = _nh.subscribe(robot_name+"/base_scan", 1,
                             &LocalizationNode::laserCallback, this);

  /// Setup publisher for pose
  _ekf_pose_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(robot_name+"/odom_combined", 2);

  /// Wait until we get an initial robot pose (from odometry)
  ros::Rate cycle(10.0); // Cycle rate
  while( _odom_updated == false )
  {
    ros::spinOnce();
    cycle.sleep();
  }
}


LocalizationNode::~LocalizationNode()
{
  // Close file
  _outfile.close();
#if FULL_ICP_DEBUG
  _icpfile.close();
#endif

  // Store final map
  cv::imwrite("mapa.png", _debug_img);

  // Delete alocated memory
  delete _flann_index;
}


void LocalizationNode::realPoseCallback(const nav_msgs::Odometry& msg)
{
  // Store real, error-free pose values given by the simulator (for debugging
  // puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
  _real_pose.x = msg.pose.pose.position.x;
  _real_pose.y = msg.pose.pose.position.y;
  _real_pose.theta = tf::getYaw(msg.pose.pose.orientation);
}



void LocalizationNode::odomCallback(const nav_msgs::Odometry& msg)
{
#if USE_ODOM
  pose_2d old_pose = _odo_robot_pose;
#endif

  // Store updated pose values
  _odo_robot_pose.x = msg.pose.pose.position.x;
  _odo_robot_pose.y = msg.pose.pose.position.y;
  _odo_robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);
  // Store timestamp and update flag
  _odom_updated = true;

  // Only perform the prediction update step if we have alread received an
  // odometry message in the past.
  if( !_odom_first_update )
  {
    /// Perform Step 1 of the particle filter
    /// --> Update particles with robot movement:
    // We need to obtain the robot movement in the robot coordinates
#if USE_ODOM
    pose_2d local_pose;
    world2Local(old_pose, _odo_robot_pose, &local_pose);
    _ekf.predictStep(local_pose);
#else
    pose_2d local_pose = {0.0,0.0,0.0};
    _ekf.predictStep(local_pose);
#endif
  } else
  {
#if USE_ODOM
    // Set initial state value to the odometry information
    _ekf._state(0,0) = _odo_robot_pose.x;
    _ekf._state(1,0) = _odo_robot_pose.y;
    _ekf._state(2,0) = _odo_robot_pose.theta;
    /// @TODO: use ROS given covariance to initialize P
#endif
    // First update is done
    _odom_first_update = false;
  }

#if USE_ODOM
  // Publish computed pose value
  createAndPublishPose(msg.header.stamp);

  // Show updated particles
  showDebugInformation();
#endif
}


void LocalizationNode::laserCallback(const sensor_msgs::LaserScan& msg)
{
  int i;
  double last_error = INFINITY;

  // Only proceed if we got at least one odometry update
  if( _odom_first_update == true )
    return;

  // Do the ICP algorithm only if enough time has elapsed
  double current_time = ros::Time::now().toSec();
  if( current_time - _last_icp_call_time >= _MIN_ICP_DELTATIME )
  {
    _last_icp_call_time = current_time;

#if BASIC_ICP_DEBUG
    // Clear debug window
    _debug_icp.setTo(cv::Scalar(255,255,255));

    // Show original cloud
    for( i=0; i < _cloud_map.rows; i++ )
    {
      // Get point in map grid coordinates
      unsigned int col = cvRound((_debug_icp.size().width/2+_cloud_map(i,0)/_MAP_RESOLUTION));
      unsigned int row = cvRound((_debug_icp.size().height/2-_cloud_map(i,1)/_MAP_RESOLUTION));

      cv::circle(_debug_icp, cv::Point(col, row),
                 1, cv::Scalar(0,0,0), -1);
    }
#endif
    // Build sensed point cloud
    double angle = msg.angle_min;
    i=0;
    PointCloud sensed_cloud(0,2);
    PointCloud new_point(1,2);
    while(angle <= msg.angle_max)
    {
      if( (msg.ranges[i] > msg.range_min) && (msg.ranges[i] < msg.range_max) )
      {
        // Store values in polar coordinates, as given by the sensor
        new_point(0,0) = msg.ranges[i];
        new_point(0,1) = angle;
        sensed_cloud.push_back(new_point);
      }
      angle += msg.angle_increment;
      i++;
    }
    // Convert to local cartesian coordinates from polar coordinates using
    // x = r.cos(angle)
    // y = r.sin(angle)
    cv::polarToCart(sensed_cloud.col(0), sensed_cloud.col(1),
                    sensed_cloud.col(0), sensed_cloud.col(1));

#if FULL_ICP_DEBUG
    std::cout << "--------------------------------\n";
    // Output ICP data to a file
    std::ofstream testfile;
    testfile.open("data_for_ICP.txt");
    testfile << "-- Data file to test ICP elsewhere:\n"
             << "-- Robot posture:\n"
             << _ekf._state(2,0) << " " << _ekf._state(2,1) << " "
             << _ekf._state(2,1) << "\n";
    testfile << "-- Original point cloud:\n";
    for(int i=0; i < _cloud_map.rows; i++)
      testfile << _cloud_map(i,0) << " " << _cloud_map(i,1) << "\n";
    testfile << "-- Sensed point cloud:\n";
    for(int i=0; i < sensed_cloud.rows; i++)
      testfile << sensed_cloud(i,0) << " " << sensed_cloud(i,1) << "\n";
    testfile.close();
#endif

    // This will hold the total transformations (rotation and translation)
    cv::Mat_<double> totalRotation = (cv::Mat_<double>(2,2) << cos(_ekf._state(2,0)), -sin(_ekf._state(2,0)),
                                                               sin(_ekf._state(2,0)),  cos(_ekf._state(2,0))),
        totalTranslation = (cv::Mat_<double>(2,1) << _ekf._state(0,0),
                                                     _ekf._state(1,0));
    // Store initial transformations (rotation and translation)
    cv::Mat_<double> rotation = totalRotation.clone(),
                     translation = totalTranslation.clone();

    /// ICP
    for( uint num_icp_iterations = 0; num_icp_iterations < _MAX_ICP_ITERATIONS;
         num_icp_iterations++ )
    {
#if FULL_ICP_DEBUG
      std::cout << "--------------------> ICP iteration " << num_icp_iterations << "\n";
#endif
      // Update sensed cloud with last transformation
      // We do sensed_cloud*rotation_matrix.t(), since the elements are row-wise.
      sensed_cloud = sensed_cloud*rotation.t();
      // Translate
      sensed_cloud.col(0) += translation(0,0);
      sensed_cloud.col(1) += translation(1,0);

#if BASIC_ICP_DEBUG
      // Show sensed cloud
      for( int i=0; i < sensed_cloud.rows; i++ )
      {
        // Get point in map grid coordinates
        unsigned int col = cvRound((_debug_icp.size().width/2+sensed_cloud(i,0)/_MAP_RESOLUTION));
        unsigned int row = cvRound((_debug_icp.size().height/2-sensed_cloud(i,1)/_MAP_RESOLUTION));

        cv::circle(_debug_icp, cv::Point(col, row),
                   1, cv::Scalar(255,0,0), -1);
      }
#endif

      // Compute correspondence using OpenCV nearest neighbours functions
      cv::Mat_<int> indices(sensed_cloud.rows, 1);
      PointCloud dists(sensed_cloud.rows, 1);
      _flann_index->knnSearch(sensed_cloud, indices, dists, 1, cvflann::SearchParams(32) );

      // Build vector with only the correspondant pairs
      PointCloud correspondences(sensed_cloud.size());
      for (int i=0; i<indices.rows; ++i)
      {
        correspondences(i,0) = _cloud_map(indices(i,0),0);
        correspondences(i,1) = _cloud_map(indices(i,0),1);
#if BASIC_ICP_DEBUG
        // Debug correspondences
        cv::Point pt1(cvRound((_debug_icp.size().width/2+correspondences(i,0)/_MAP_RESOLUTION)),
                      cvRound((_debug_icp.size().height/2-correspondences(i,1)/_MAP_RESOLUTION)));
        cv::Point pt2(cvRound((_debug_icp.size().width/2+sensed_cloud(i,0)/_MAP_RESOLUTION)),
                      cvRound((_debug_icp.size().height/2-sensed_cloud(i,1)/_MAP_RESOLUTION)));
        cv::line(_debug_icp, pt1, pt2, cv::Scalar(40,150,255));
#endif
      }

      // Compute the translation as the difference between the center of mass of
      // each set of points in the correspondence
      PointCloud m_sensed_cloud, m_correspondences;
      // Compute mean
      cv::reduce(sensed_cloud, m_sensed_cloud, 0, CV_REDUCE_AVG );
      cv::reduce(correspondences, m_correspondences, 0, CV_REDUCE_AVG );
#if FULL_ICP_DEBUG
      std::cout << "Sensed centroid: " << m_sensed_cloud(0,0) << " "
                << m_sensed_cloud(0,1) << "\n"
                << "Data centroid: " << m_correspondences(0,0) << " "
                << m_correspondences(0,1) << std::endl;
#endif
#if BASIC_ICP_DEBUG
      // Debug centers
      drawCross(_debug_icp, m_sensed_cloud(0,0), m_sensed_cloud(0,1),
                _MAP_RESOLUTION, cv::Scalar(255,0,0));
      drawCross(_debug_icp, m_correspondences(0,0), m_correspondences(0,1),
                _MAP_RESOLUTION, cv::Scalar(0,0,0));
#endif

      // Compute the rotation by:
      //  1. Compute the inercia W
      //  2. Perform SVD as W = USV'
      //  3. Compute ratation as R = VU'
      PointCloud W(2,2), U(2,2), S(2,2), Vt(2,2);
      W.setTo(cv::Scalar(0));
      for(int i=0; i < correspondences.rows; i++)
      {
        W += (sensed_cloud.row(i)-m_sensed_cloud).t()*(correspondences.row(i)-m_correspondences);
      }
      cv::SVD::compute(W, S, U, Vt);//, cv::SVD::MODIFY_A);
#if FULL_ICP_DEBUG
      std::cout << "W matrix:\n";
      showMatValues(&W);
      std::cout << "U matrix:\n";
      showMatValues(&U);
      std::cout << "S matrix:\n";
      showMatValues(&S);
      std::cout << "V matrix transposed:\n";
      showMatValues(&Vt);
#endif
      // For the SVD approach to work always, we use the approach detailed in
      // D.W. Eggert, A. Lorusso, R.B. Fisher, "Estimating 3-D rigid body
      // transformations: a comparison of four major algorithms", Machine Vision
      // and Applications, pp 272-290, Springer-Verlag, 1997.
      S = PointCloud::eye(2,2);
      S(1,1) = cv::determinant(U*Vt);
      rotation = Vt.t()*S*U.t();

      // Compute translation
      translation = m_correspondences.t() - rotation*m_sensed_cloud.t();

      // Update total transformation
      totalRotation = rotation*totalRotation;
      totalTranslation = rotation*totalTranslation+translation;

#if FULL_ICP_DEBUG
      std::cout << " Current rotation:\n";
      showMatValues(&rotation);
      std::cout << " Current translation:\n";
      showMatValues(&translation);

      std::cout << " Current total rotation:\n";
      showMatValues(&totalRotation);
      std::cout << " Current total translation:\n";
      showMatValues(&totalTranslation);
      std::cout << "Total rotation angle [degrees]: "
                << rad2deg(atan2(rotation(1,0),rotation(0,0))) << "\n";
#endif


#if BASIC_ICP_DEBUG
      // Show ICP
      cv::imshow("Debug ICP", _debug_icp);
      cv::waitKey(5);
#endif
      // Compute RMS error (before last computed transformation)
      PointCloud sqr_dists(dists.rows, 1);
      cv::pow(dists,2, sqr_dists);
      cv::Scalar result = cv::sum(sqr_dists/dists.rows);
      double curr_error = sqrt(result.val[0]);
#if FULL_ICP_DEBUG
      std::cout << "Current error: " << curr_error << std::endl;
      _icpfile << curr_error << std::endl;
#endif
      if( curr_error <= last_error  )
      {
        if( last_error - curr_error < _MIN_ICP_DELTA_ERROR )
        {
          last_error = curr_error;
          break;
        }
      }
#if FULL_ICP_DEBUG
      else
        std::cerr << "Got error increase!!!!" << std::endl;
#endif
      last_error = curr_error;
    }
#if BASIC_ICP_DEBUG
    std::cout << "Final ICP pose (x, y, theta): " << totalTranslation(0,0)
              << " " << totalTranslation(1,0)
              << " " << rad2deg(atan2(totalRotation(1,0),totalRotation(0,0))) << std::endl;
    _icpfile << "----------------------------" << std::endl;
#endif

    /// Step 2 of the kalman filter - Perform the update step based on the
    /// information retrieved from ICP
    // We will consider the error computed above for the 2nd step
    cv::Mat_<double> W = (cv::Mat_<double>(3,3) << last_error, 0.0, 0.0,
                                                   0.0, last_error, 0.0,
                                                   0.0, 0.0, last_error);
    pose_2d z;
    z.x = totalTranslation(0,0);
    z.y = totalTranslation(1,0);
    z.theta = atan2(totalRotation(1,0),totalRotation(0,0));
    _ekf.updateStep(z, W);

    // Publish computed pose value
    createAndPublishPose(ros::Time::now());

    // Show debug information
    showDebugInformation();

    // Save map from time to time
    _iteration++;
    if( _iteration == _DELTA_SAVE )
    {
      cv::imwrite("mapa.png", _debug_img);
      _iteration = 0;
    }
  }

  return;
}


void LocalizationNode::createAndPublishPose(const ros::Time &stamp)
{
  // Update and send pose message
  _pose_msg.header.stamp = stamp;
  _pose_msg.pose.pose.position.x = _ekf._state(0,0);
  _pose_msg.pose.pose.position.y = _ekf._state(1,0);
  _pose_msg.pose.pose.position.z = 0.0;
  _pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_ekf._state(2,0));

  // Fill covariance matrix. We will consider 0 for all the values we are not
  // computing, i.e., Z, RX and RY, and correlated values.
  // We will fill only the upper diagional, the other are updated as a copy.
  float covariance[36] =
  { //   X            Y          Z    RX   RY      RZ
    _ekf._P(0,0), _ekf._P(0,1), 0.0, 0.0, 0.0, _ekf._P(0,2), // X
    _ekf._P(1,0), _ekf._P(1,1), 0.0, 0.0, 0.0, _ekf._P(1,2), // Y
             0.0,          0.0, 0.0, 0.0, 0.0,          0.0, // Z
             0.0,          0.0, 0.0, 0.0, 0.0,          0.0, // RX
             0.0,          0.0, 0.0, 0.0, 0.0,          0.0, // RY
    _ekf._P(2,0), _ekf._P(2,1), 0.0, 0.0, 0.0, _ekf._P(2,2), // RZ

  };
  for( uint i=0; i<36; i++)
    _pose_msg.pose.covariance[i] = covariance[i];

  // Publish message
  _ekf_pose_pub.publish(_pose_msg);
}


void LocalizationNode::showDebugInformation()
{
  /// Write data to the file
  _outfile << std::setiosflags(std::ios::fixed) << std::setprecision(3)
           << _pose_msg.header.stamp << " | "
           << _odo_robot_pose.x << " " << _odo_robot_pose.y << " "
           << _odo_robot_pose.theta << " | "
           << _real_pose.x << " " << _real_pose.y
           << " " << _real_pose.theta << " | "
           << _ekf._state(0,0) << " "
           << _ekf._state(1,0) << " "
           << _ekf._state(2,0) << " "
           << std::endl;

  /// Draw postures in image
  // Real (in green)
  drawPos(_debug_img, _real_pose.x, _real_pose.y, _real_pose.theta,
          _MAP_RESOLUTION, cv::Scalar(0,255,0));
  // Estimated by odometry (in red)
  drawPos(_debug_img, _odo_robot_pose.x, _odo_robot_pose.y, _odo_robot_pose.theta,
          _MAP_RESOLUTION, cv::Scalar(0,0,255));
  // Estimated by kalman (in blue)
  drawPos(_debug_img,
          _ekf._state(0,0), // x
          _ekf._state(1,0), // y
          _ekf._state(2,0), // theta
          _MAP_RESOLUTION, cv::Scalar(255,0,0));
  // Show map with postures
  imshow("Debug", _debug_img);
  // Quit if escape key is pressed
  if( cv::waitKey(5) == 27 )
    ros::shutdown();
}

/**
 * Main function
 * Navigate randomly using distance sensor data and Particle Filter-based
 * localization.
 */
int main(int argc, char** argv)
{
  // Init ROS
  ros::init(argc, argv, "localization_node");
  
  std::string map_filename(getenv("HOME"));
  map_filename += "/ros/worlds/stage/cave2_walls_only.png";
  cv::Mat map = cv::imread(map_filename, CV_LOAD_IMAGE_GRAYSCALE);
  if( !map.data )
  {
    std::cerr << "Map " << map_filename << " not found!" << std::endl;
    return 1;
  }

  // Initiate LocalizationNode. Recall that the arguments are:
  // map_resolution, map, safety_border, delta_save, max_icp_iterations,
  // min_icp_delta_error, icp_rate
  // (check localizatio_node.hpp for more details).
  LocalizationNode localization_node(0.032, map, 2.0, 100, 10, 0.01, 1.0);

  // Infinite loop (will call the callbacks whenever information is available,
  // until ros::shutdown() is called.
  ros::spin();

  return -1;
}

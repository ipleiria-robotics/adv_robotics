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

#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include <cv.hpp>
#include <markers_msgs/Markers.h> // Markers messages

#include "utils.hpp"

class ExtendedKalmanFilter
{
public:
  /**
   * Constructor with no intializations
   */
  ExtendedKalmanFilter();

  /**
   * Destructor
   */
  ~ExtendedKalmanFilter();

  /**
   * Run the EKF predict step.
   * delta_x - amount of x displacement (robot coordinates).
   * delta_y - amount of y displacement (robot coordinates).
   * delta_theta, - amount of rotation (robot coordinates).
   */
  void predictStep(const double &delta_xr, const double &delta_yr,
                   const double &delta_thetar);

  /**
   * Run the update step.
   * const markers_msgs::Markers& msg - detected markers information;
   * const point_2d *_markers_wpos - markers world position
   */
  void updateStep(const markers_msgs::Markers& msg, const point_2d *_markers_wpos);

  cv::Mat_<double> _state; ///< EKF state.
  cv::Mat_<double> _P; ///< EKF state covariance.
  cv::Mat_<double> _V; ///< EKF dynamics covariance matrix
  cv::Mat_<double> _Fx; ///< EKF dynamics matrix (prediction) jacobian regarding the state
  cv::Mat_<double> _Fv; ///< EKF dynamics matrix (prediction) jacobian regarding the inputs
};

#endif // EXTENDED_KALMAN_FILTER_HPP

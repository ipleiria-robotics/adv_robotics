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

#include "ExtendedKalmanFilter.hpp"
#include "utils.hpp"

#include <list>

//// Specify if the particle filter steps should run
#define STEP_PREDICTION
#define STEP_UPDATE


ExtendedKalmanFilter::ExtendedKalmanFilter():
  _num_landmarks(0)
{

}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}


void ExtendedKalmanFilter::predictStep(const double &delta_xr,
                                       const double &delta_yr,
                                       const double &delta_thetar)
{
#ifdef STEP_PREDICTION
  // Temporary variables
  double sin_thetar = sin(_state.at<double>(2,0)),
         cos_thetar = cos(_state.at<double>(2,0));

  /// Step 1 - Prediction:

  // --> State update based on previous state and movement made
  // x(k+1|k) = f(x(k|k),u(k),k) <=>
  // <=> | xr(k+1|k) | = | xr(k) + Δx(k) |
  //     | yr(k+1|k) | = | yr(k) + Δy(k) |
  //     | θr(k+1|k) | = | θr(k) + Δθ(k) |
  // where
  //    Δx = Δxr*cos(θr(k))-Δyr*sin(θr(k))
  //    Δy = Δxr*sin(θr(k))+Δyr*cos(θr(k))
  // but where the movements are extracted directly from the odometry.
  double dx =  delta_xr*cos_thetar - delta_yr*sin_thetar;
  double dy =  delta_xr*sin_thetar + delta_yr*cos_thetar;
  double dtheta = delta_thetar;

  // The remainder of state states (landmark positions) stays the same, since
  // the landmakrs are fixed and do not move with the robot.
  _state.at<double>(0,0) += dx;
  _state.at<double>(1,0) += dy;
  _state.at<double>(2,0) = normalize(_state.at<double>(2,0)+dtheta);

  // --> Update state covariance given prediction
  //  P(k+1|k) = Jfx(k)P(k|k)Jfx(k)' + Jfu(k)V(k)Jfu(k)'
  //    where (disregarding (k))
  //        Jfx = | Jfxv 0 |,
  //              |  0   I |
  //        Jfu = | Jfuv |
  //              |  0   |
  //        V(k) is constant (initialized at the beggining),
  // with  I being an identity matrix with with/height equal to the number
  // of detected landmarks so far, and
  //        Jfxv = | 1 0 -Δxr.sin(θr)-Δyr.cos(θr) | = | 1 0 -Δy(k) |
  //               | 0 1  Δxr.cos(θr)-Δyr.sin(θr) |   | 0 1  Δx(k) |
  //               | 0 0             1            |   | 0 0    1   |
  //        Jfuv = | cos(θr) -sin(θr) 0 |
  //               | sin(θr)  cos(θr) 0 |
  //               |  0          0    1 |
  //
  // Given the sparse nature of the matrices, the process covariance matrix
  // can be updated in submatrices as follows.
  // Considering P = | Pv   Pvm |, computing P can be written as:
  //                 | Pvm' Pm  |
  //     P(k+1|k) = | Jfxv.Pv.Jfxv'+Jfxu.V.Jfxu' Jfxv.Pvm |
  //                |         (Jfxv.Pvm)'            Pm   |
  // As expected, a change in the robot pose thus no influence the
  //correlation between the various landmark estimates.

  // Note that the constant members of these matrices were already set previously,
  // so we will just update the 1other valuees.
  _Jfxv.at<double>(0,2) = -dy;
  _Jfxv.at<double>(1,2) =  dx;
  _Jfuv.at<double>(0,0) =  cos_thetar;
  _Jfuv.at<double>(0,1) = -sin_thetar;
  _Jfuv.at<double>(1,0) =  sin_thetar;
  _Jfuv.at<double>(1,1) =  cos_thetar;
  // NOTE: .t() is used to compute the transpose
  _P(cv::Range(0,3),cv::Range(0,3)) =
      _Jfxv*_P(cv::Range(0,3),cv::Range(0,3))*_Jfxv.t() + _Jfuv*_V*_Jfuv.t();
  if( _num_landmarks > 0 )
  {
    _P(cv::Range(0,3),cv::Range(3,3+2*_num_landmarks)) =
        _Jfxv*_P(cv::Range(0,3),cv::Range(3,3+2*_num_landmarks));
    _P(cv::Range(3,3+2*_num_landmarks),cv::Range(0,3)) =
        _P(cv::Range(0,3),cv::Range(3,3+2*_num_landmarks)).t();
  }
#endif
}


void ExtendedKalmanFilter::updateStep(const markers_msgs::Markers& msg)
{
  if( msg.num_markers < 1 )
    return;

#ifdef STEP_UPDATE
  // Do the update step or the augmentation step for each detected landmark
  for( uint n=0; n < msg.num_markers; n++ )
  {
    // Store the sensor values in z
    _z.at<double>(0,0) = msg.range[n];
    _z.at<double>(1,0) = msg.bearing[n];

    // If this is the first time we are seeing this marker, we need to
    // augment our state vector and state covariance with its information.
    // In this case we do not perform the update state, we only do the
    // augmentation.
    if( _marker_id_to_idx.find(msg.id[n]) == _marker_id_to_idx.end())
    {
      // This is a new marker, lets add it
      _marker_id_to_idx[msg.id[n]] = _num_landmarks;
      std::cout << "Found new landmark with id " << msg.id[n]
                << " (stored with index "
                << _marker_id_to_idx[msg.id[n]] << ")." << std::endl;
      // Store the index of this landmark (it starts at index 0)
      _num_landmarks++;

      // Perform state augmentation
      //
      // Given the previous state and the landmark observation
      //  Xaug = | X  |
      //         | xl |
      //         | yl |
      // The associated covariance is given by
      //  Paug = |  Pv  Pvm 0 |
      //         | Pvm'  Pm 0 |
      //         |  0    0  R |
      // where R is the covariance associated with the sensor noise.
      //
      // The expected landmark world position coordinates can be written as:
      //   hn = | xr + d*cos(θr+b) |
      //        | yr + d*sin(θr+b) |
      // where d and b are the observed distance and bearing.
      //
      // As such, the augmented state is basically the previous state plus
      // the x and y world coordinates of the new detected landmark, which
      // can be written as:
      //  X = |  X |
      //      | hn |
      // In this case the state covariance update can be approximated by:
      //  P = Jhn.Paug.Jhn'
      // where Jhn is the jacobian of the observation model with respect to
      // the augmented state:
      //  Jhn = |  Iv    0   0  |
      //        |   0   Im   0  |
      //        | Jhnxv  0 Jhnz |
      // Here Iv is an identity matrix with the number of columms and rows
      // equal to the number of robot pose variables, and Im is an identity
      // matrix with the number of columms and rows equal to the number of
      // previoulsy nown landmark positions variables.
      // Jhxxv and Jhnz are the jacobians of hn with respect to the robot
      // pose state variables and the new landmark observation variables,
      // respectively:
      //
      //  Jhnxv = | 1 0 -d.sin(θr+b) |
      //          | 0 1  d.cos(θr+b) |
      //  Jhnz = | cos(θr+b) -d.sin(θr+b) |
      //         | sin(θr+b)  d.cos(θr+b) |
      //
      // Finnally, computing the new covariance can be written as
      //  P = |    Pv      Pvm               Pv.Jhnxv'           |
      //      |    Pvm'     Pm              Pvm'.Jhnxv'          |
      //      | Jhnxv.Pv' Jhnxv.Pvm Jhnxv.Pv.Jhnxv'+Jhnz.R.Jhnz' |
      // We can thus have a faster computation of these values.

      // Compute the expected landmark world position
      double cs = cos(_state.at<double>(2,0) + _z.at<double>(1,0));
      double sn = sin(_state.at<double>(2,0) + _z.at<double>(1,0));
      _h.at<double>(0,0) = _state.at<double>(0,0) + _z.at<double>(0,0)*cs;
      _h.at<double>(1,0) = _state.at<double>(1,0) + _z.at<double>(0,0)*sn;

      // Compute the jacobians
      // NOTE: We could improve performance here by avoinding repeating some
      // of the computations, but I leave it this way to improve readability.
      _Jhnxv.at<double>(0,2) = -_z.at<double>(0,0)*sn;
      _Jhnxv.at<double>(1,2) =  _z.at<double>(0,0)*cs;
      _Jhnz.at<double>(0,0)  =  cs;
      _Jhnz.at<double>(0,1)  = -_z.at<double>(0,0)*sn;
      _Jhnz.at<double>(1,0)  =  sn;
      _Jhnz.at<double>(1,1)  =  _z.at<double>(0,0)*cs;

      // Finnally update the state and its covariance.
      // Note that we will update the final row and then make the last
      // columm equal to that row transposed.
      _state.push_back(_h); // Add vector h to the end of the state vector
      cv::Mat oldP = _P.clone(); // Store old P value

      // Increase _P (and _tempP, and _tempI)
      _P = cv::Mat(3+2*_num_landmarks, 3+2*_num_landmarks, CV_64F);

      // This is the part of the covariance P which does not change with new
      // landmarks detected (robot  pose and previous landmarks).
      // The I multiplication is here to overcome an OpenCV bug
      _P(cv::Range(0,3+2*(_num_landmarks-1)),
         cv::Range(0,3+2*(_num_landmarks-1))) =
          oldP(cv::Range(0,3+2*(_num_landmarks-1)),
               cv::Range(0,3+2*(_num_landmarks-1))) *
          cv::Mat::eye(3+2*(_num_landmarks-1),3+2*(_num_landmarks-1),CV_64F);

      // Update last line with 1st submatrix, Jhnxv.Pv'
      _P(cv::Range(3+2*(_num_landmarks-1),3+2*_num_landmarks),
         cv::Range(0,3)) = _Jhnxv * oldP(cv::Range(0,3),cv::Range(0,3));

      // Update last line with Jhnxv.Pvm (if there were previous landmarks).
      if (_num_landmarks > 1 )
        _P(cv::Range(3+2*(_num_landmarks-1),3+2*_num_landmarks),
           cv::Range(3,3+2*(_num_landmarks-1))) =
            _Jhnxv * oldP(cv::Range(0,3),cv::Range(3,3+2*(_num_landmarks-1)));
      // Copy the two computed submatrices from the "last line" transposed
      // to the "last colummm".
      _P(cv::Range(0,3+2*(_num_landmarks-1)),
         cv::Range(3+2*(_num_landmarks-1),3+2*_num_landmarks)) =
        _P(cv::Range(3+2*(_num_landmarks-1),3+2*_num_landmarks),
           cv::Range(0,3+2*(_num_landmarks-1))).t();

      // Compute the bottom-right submatrix with Jhnxv.Pv.Jhnxv'+Jhnz.W.Jhnz'
      _P(cv::Range(3+2*(_num_landmarks-1),3+2*_num_landmarks),
         cv::Range(3+2*(_num_landmarks-1),3+2*_num_landmarks)) =
          _Jhnxv*oldP(cv::Range(0,3),cv::Range(0,3))*_Jhnxv.t()
          + _Jhnz*_W*_Jhnz.t();

      // Increase the matrix used for the observation model jacobian,
      // needed when processing observations of previously detected
      // landmarks.
      _Jh = cv::Mat(2, 3+2*_num_landmarks, CV_64F);

      // Proceed to the next landmark (skip the update step for this landmark)
      continue;
    }

    // The expected sensor reading can be obtained from the robot estimated
    // pose and the detected landmark position in world coordinates. Here j
    // denotes the landmarl id.
    //
    //  y_j(k+1) = | xl_j(k+1) |
    //             | yl_j(k+1) |
    //
    // zhat = h_j(k+1) = |     sqrt((xl_j-xr(k+1))²+(yl_j-yr(k+1))²)      |
    //                   | atan2(yl_j-yr(k+1), (xl_j-xr(k+1))) - θxr(k+1) |
    //
    // The Jacobian with respect to the state variables is given as follows:
    //  Jh_j(k+1) =
    // |-(xl_j-xr)/d  -(yl_j-yr)/d   0 ...  (xl_j-xr)/d  (yl_j-yr)/d  0 ... |
    // | (yl_j-yr)/d² -(xl_j-xr)/d² -1 ... -(yl_j-yr)/d² (xl_j-xr)/d² 0 ... |
    // where d = sqrt((xl_j-xr(k+1))²+(yl_j-yr(k+1))²).
    // Note that it considers not just the robot pose variables, but also
    // the landmaks position variables.
    //
    // Given these, one can update the state using the EKF as follows:
    //  v = z - hj <-- Inovation
    //  s = Jh.P.Jh' + W <-- Inovation confidence
    //  K = Pa.Jh'.s⁻¹ <-- Kalman gain
    //
    //  X = X + K.v
    //  P = P - K.S.K'

    uint landmark_idx = _marker_id_to_idx[msg.id[n]];
    // Get beacon estimated world posture from our state estimate.
    point_2d marker_wpos = {_state.at<double>(3+landmark_idx*2,0),
                            _state.at<double>(4+landmark_idx*2,0)};

    // Compute expected distance measure from the robot to the marker.
    double d = sqrt(pow(marker_wpos.x-_state.at<double>(0,0),2)+
                    pow(marker_wpos.y-_state.at<double>(1,0),2));

    // Compute expected observation (distance and bearing)
    _h.at<double>(0,0) = d;
    _h.at<double>(1,0) = atan2(marker_wpos.y-_state.at<double>(1,0),
                               marker_wpos.x-_state.at<double>(0,0))
                          - _state.at<double>(2,0);

    // Compute the observation Jacobian
    double dlx = _state.at<double>(3+landmark_idx*2,0) -
                 _state.at<double>(0,0),
           dly = _state.at<double>(4+landmark_idx*2,0) -
                 _state.at<double>(1,0),
           d_sq = d*d;
    _Jh.setTo(cv::Scalar(0));
    _Jh.at<double>(0,0) = -dlx/d;
    _Jh.at<double>(0,1) = -dly/d;
    _Jh.at<double>(0,3+landmark_idx*2) = dlx/d;
    _Jh.at<double>(0,4+landmark_idx*2) = dly/d;
    _Jh.at<double>(1,0) =  dly/d_sq;
    _Jh.at<double>(1,1) = -dlx/d_sq;
    _Jh.at<double>(1,2) = -1;
    _Jh.at<double>(1,3+landmark_idx*2) = -dly/d_sq;
    _Jh.at<double>(1,4+landmark_idx*2) =  dlx/d_sq;

    // Compute the confidence associated with this measure
    // The expected size of s is (2, 2, CV_64F)
    _s = _Jh*_P*_Jh.t() + _W;

    // Compute the Kalman gain.
    // Use Cholesky decompoision to compute the inverse of s, since s is a
    // covariance matrix, it is positive definite. In that case Cholsesky
    // decomposition is much faster then the default LU or SVD decomposition.
    // The expected size of K is (3+num_landmarks*2, 2, CV_64F)
    // Note that s.inv() is the inverse of s.
    _K = _P*_Jh.t()*_s.inv(cv::DECOMP_CHOLESKY);

    // Compute the error (inovation) of the observation
    _v = _z - _h;
    // Make sure the bearing error is between -PI and PI
    _v.at<double>(1,0) = normalize(_v.at<double>(1,0));

    // Finnally update the state and the state covariance matrix
    _state += _K*_v;
    // Make sure theta is between -PI and PI
    _state.at<double>(2,0) = normalize(_state.at<double>(2,0));
    _P += -_K*_s*_K.t();
  }

#endif
}

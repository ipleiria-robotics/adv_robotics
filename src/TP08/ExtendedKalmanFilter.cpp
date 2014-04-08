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

#include "ExtendedKalmanFilter.hpp"
#include "utils.hpp"

//// Specify if the particle filter steps should run
//#define STEP_PREDICTION
//#define STEP_UPDATE


ExtendedKalmanFilter::ExtendedKalmanFilter()
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

  // --> State covariance
  //  P(k+1|k) = Fx(k)P(k|k)Fx(k)' + Fv(k)V(k)Fv(k)'
  //
  // with Fx(k) = | 1 0 -Δy |
  //              | 0 1  Δx |,
  //              | 0 0  1  |
  //
  // Fv(k) = | cos(θr(k)) -sin(θr(k)) 0 |
  //         | sin(θr(k))  cos(θr(k)) 0 |,
  //         |     0           0      1 |
  //
  // V(k) constant (initialized at the beggining)
  //
  // and
  //    Δx = Δxr*cos(θr(k))-Δyr*sin(θr(k))
  //    Δy = Δxr*sin(θr(k))+Δyr*cos(θr(k))

  double dx =  delta_xr*cos_thetar - delta_yr*sin_thetar;
  double dy =  delta_xr*sin_thetar + delta_yr*cos_thetar;
  double dtheta = delta_thetar;

  // Note that the constant members of Fx(k) were already set previously, so
  // we will just update the 1st and 2nd positions of the last column.
  _Fx.at<double>(0,2) = -dy;
  _Fx.at<double>(1,2) =  dx;
  // Note that the constant members of Fv(k) were already set previously, so
  // we will just update the 1st and 2nd positions of the first 2 columns.
  _Fv.at<double>(0,0) =  cos_thetar;
  _Fv.at<double>(0,1) = -sin_thetar;
  _Fv.at<double>(1,0) =  sin_thetar;
  _Fv.at<double>(1,1) =  cos_thetar;
  // Update the state covariance
  _P = _Fx*_P*_Fx.t() + _Fv*_V*_Fv.t();

  // --> State
  // x(k+1|k) = f(x(k|k),u(k),k) <=>
  // <=> | xr(k+1|k) | = | xr(k) + Δx(k) |
  //     | yr(k+1|k) | = | yr(k) + Δy(k) |
  //     | θr(k+1|k) | = | θr(k) + Δθ(k) |
  // but where the movements are extracted directly from the odometry (see
  // above).
  _state.at<double>(0,0) += dx;
  _state.at<double>(1,0) += dy;
  _state.at<double>(2,0) = normalize(_state.at<double>(2,0)+dtheta);
#endif
}


void ExtendedKalmanFilter::updateStep(const markers_msgs::Markers& msg,
                                      const point_2d *markers_wpos)
{
#ifdef STEP_UPDATE
  // Temporary variables:
  double xr = _state.at<double>(0,0),
         yr = _state.at<double>(1,0),
         thetar = _state.at<double>(2,0);
  double sin_thetar = sin(thetar),
         cos_thetar = cos(thetar);


  // Before doing the actual update process, let us build the y(k+1)
  // (measures) vector, the h(k+1) (expected measures) vector and
  // corresponding H(k+1) matrix.
  // For each landmark j of the l possibly detected landmarks we have:
  //
  //  y_j(k+1) = | xl_j(k+1) |
  //             | yl_j(k+1) |
  //
  //  h_j(k+1) =
  // | (xl_j-xr(k+1))*cos(θr(k+1))+(yl_j-yr(k+1))*sin(θr(k+1)) |
  // |-(xl_j-xr(k+1))*sin(θr(k+1))+(yl_j-yr(k+1))*cos(θr(k+1)) |
  //
  //  H_j(k+1) =
  // |-cos(θr(k+1)) -sin(θr(k+1)) -(xl_j-xr(k+1))*sin(θr(k+1))+(yl_j-yr(k+1))*cos(θr(k+1)) |
  // | sin(θr(k+1)) -cos(θr(k+1)) -(xl_j-xr(k+1))*cos(θr(k+1))-(yl_j-yr(k+1))*sin(θr(k+1)) |
  //
  cv::Mat y(msg.num_markers*2, 1, CV_64F);
  cv::Mat h(msg.num_markers*2, 1, CV_64F);
  cv::Mat H(msg.num_markers*2, 3, CV_64F);
  cv::Mat W = cv::Mat::zeros(msg.num_markers*2,msg.num_markers*2, CV_64F);
  for( uint n=0; n < msg.num_markers; n++ )
  {
    // Store y
    y.at<double>(n*2,0) = msg.range[n]*cos(msg.bearing[n]); // Local position x of landmark n
    y.at<double>(n*2+1,0) =  msg.range[n]*sin(msg.bearing[n]); // Local position y of landmark n
    // Store h
    // NOTE: We could have used the local2world, but then we needed to store h differently.
    h.at<double>(n*2,0) = (markers_wpos[msg.id[n]-1].x-xr)*cos_thetar
                         +(markers_wpos[msg.id[n]-1].y-yr)*sin_thetar;
    h.at<double>(n*2+1,0) = -(markers_wpos[msg.id[n]-1].x-xr)*sin_thetar
                            +(markers_wpos[msg.id[n]-1].y-yr)*cos_thetar;
    // Store H
    H.at<double>(n*2,0) = -cos_thetar;
    H.at<double>(n*2,1) = -sin_thetar;
    H.at<double>(n*2,2) = -(markers_wpos[msg.id[n]-1].x-xr)*sin_thetar
                          +(markers_wpos[msg.id[n]-1].y-yr)*cos_thetar;
    H.at<double>(n*2+1,0) =  sin_thetar;
    H.at<double>(n*2+1,1) = -cos_thetar;
    H.at<double>(n*2+1,2) = -(markers_wpos[msg.id[n]-1].x-xr)*cos_thetar
                            -(markers_wpos[msg.id[n]-1].y-yr)*sin_thetar;
    //  The error associated with the landmark measure is higher if the
    // landmark is further away.
    //  We consider a variance of 0.05 m^2 if less than or equal to 1 m,
    // and 0.5 m^2 if equal to 8 m.
    if( y.at<double>(n*2,0) < 1.0 )
      W.at<double>(n*2,n*2) = 0.05;
    else
      W.at<double>(n*2,n*2) = 0.0643*y.at<double>(n*2,0)-0.0143;
    if( y.at<double>(n*2+1,0) < 1.0 )
      W.at<double>(n*2+1,n*2+1) = 0.05;
    else
      W.at<double>(n*2+1,n*2+1) = 0.0643*y.at<double>(n*2+1,0)-0.0143;
  }

  // The update step consists on
  // ---> Updating the state
  //  x(k+1|k+1) = x(k+1|k) + R*v
  //
  // ---> And updating the covariance matrix
  //  P(k+1|k+1) = P(k+1|k) - R*H(k+1)*P(k+1|k)
  //
  // with
  //  v = y(k+1) - h(x(k+1|k),k+1)
  //  S = H(k+1)*P(k+1|k)*H(k+1)' + W(k+1)
  //  R = P(k+1|K)*H(k+1)'*S^(-1)
  cv::Mat v(msg.num_markers*2, 1, CV_64F);
  cv::Mat S(msg.num_markers*2, msg.num_markers*2, CV_64F);
  cv::Mat R(3, msg.num_markers*2, CV_64F);
  // Compute v
  v = y-h;
  // Compute S
  S = H*_P*H.t() + W;
  // Compute R
  R = _P*H.t()*S.inv();
  // Now update the state
  _state += R*v;
  // And update the state covariance matrix
  _P += -R*H*_P;
#endif
}

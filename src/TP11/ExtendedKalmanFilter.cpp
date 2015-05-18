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

//// Specify if the particle filter steps should run
#define STEP_PREDICTION 1
#define STEP_UPDATE 1


ExtendedKalmanFilter::ExtendedKalmanFilter()
{

}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}


void ExtendedKalmanFilter::predictStep(const pose_2d deltar)
{
#if STEP_PREDICTION
  // Temporary variables
  double sin_thetar = sin(_state(2,0)),
         cos_thetar = cos(_state(2,0));

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

  double dx =  deltar.x*cos_thetar - deltar.y*sin_thetar;
  double dy =  deltar.x*sin_thetar + deltar.y*cos_thetar;
  double dtheta = deltar.theta;

  // Note that the constant members of Fx(k) were already set previously, so
  // we will just update the 1st and 2nd positions of the last column.
  _Fx(0,2) = -dy;
  _Fx(1,2) =  dx;
  // Note that the constant members of Fv(k) were already set previously, so
  // we will just update the 1st and 2nd positions of the first 2 columns.
  _Fu(0,0) =  cos_thetar;
  _Fu(0,1) = -sin_thetar;
  _Fu(1,0) =  sin_thetar;
  _Fu(1,1) =  cos_thetar;
  // Update the state covariance
  _P = _Fx*_P*_Fx.t() + _Fu*_V*_Fu.t();

  // --> State
  // x(k+1|k) = f(x(k|k),u(k),k) <=>
  // <=> | xr(k+1|k) | = | xr(k) + Δx(k) |
  //     | yr(k+1|k) | = | yr(k) + Δy(k) |
  //     | θr(k+1|k) | = | θr(k) + Δθ(k) |
  // but where the movements are extracted directly from the odometry (see
  // above).
  _state(0,0) += dx;
  _state(1,0) += dy;
  _state(2,0) = normalize(_state(2,0)+dtheta);
#endif
}


void ExtendedKalmanFilter::updateStep(const pose_2d z,
                                      const cv::Mat_<double> W)
{
#if STEP_UPDATE
  // Before doing the actual update process, let us build the y(k+1)
  // (measures) vector, the h(k+1) (expected measures) vector and
  // corresponding H(k+1) matrix.
  //
  //  z_j(k+1) = h_j(k+1) = | xr_j(k+1) |
  //                        | yr_j(k+1) |
  //                        | θr_j(k+1) |
  //
  //  H_j(k+1) =
  // | 1 0 0 |
  // | 0 1 0 |
  // | 0 0 1 |
  //
  cv::Mat_<double> h = _state.clone();

  // The update step consists on
  // ---> Updating the state
  //  x(k+1|k+1) = x(k+1|k) + R*v
  //
  // ---> And updating the covariance matrix
  //  P(k+1|k+1) = P(k+1|k) - R*H(k+1)*P(k+1|k)
  //
  // with
  //  v = z(k+1) - h(x(k+1|k),k+1)
  //  S = H(k+1)*P(k+1|k)*H(k+1)' + W(k+1)
  //  R = P(k+1|K)*H(k+1)'*S^(-1)
  cv::Mat_<double> v(3, 1);
  cv::Mat_<double> s(3, 3);
  cv::Mat_<double> K(3, 3);
  // Compute v
  v(0,0) = z.x - h(0,0);
  v(1,0) = z.y - h(1,0);
  v(2,0) = normalize(z.theta - h(2,0));
  // Compute S
  s = _H*_P*_H.t() + W;
  // Compute K
  K = _P*_H.t()*s.inv();
  // Now update the state
  _state += K*v;
  // And update the state covariance matrix
  _P += -K*_H*_P;
#endif
}

#!/usr/bin/env python3
'''
Copyright (c) 2019, Hugo Costelha
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
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 4
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Revision $Id$

@package docstring
Implementation of an Extended Kalman Filter.
'''

# Our functions
from markers_msgs.msg import Markers
import ar_utils.LocalFrameWorldFrameTransformations as lfwft
from ar_utils.utils import normalize

# Matrices and OpenCV related functions
import numpy as np
from numpy.linalg import inv
from math import sin, cos, atan2, sqrt


class ExtendedKalmanFilter:
    ''' Extended Kalman Filter implementation '''
    def __init__(self):
        ''' Initilize Kalman filter related variables '''
        # Initially we do not have any landmark, so the state holds only x, y
        # and theta estimate of the robot, all of type double, and initially
        # set to 0.
        self.state = np.zeros((3, 1))  # EKF state
        self.num_landmarks = 0
        # Dictionary that maps the andmark real id to our internal index
        self.marker_id_to_idx = {}
        # EKF process covariance (confidence associated with current state)
        # (Σ in the course theoretic material). Assume sone uncertainty in the
        # beggining
        self.P = np.array([[0.1, 0.0, 0.0],
                           [0.0, 0.1, 0.0],
                           [0.0, 0.0, 0.1]])
        # EKF dynamics covariance matrix, that is, covariance matrix associated
        # with the robot motion/actuation
        self.V = np.array([[0.1**2, 0.0, 0.0],
                           [0.0, 0.03**2, 0.0],
                           [0.0, 0.0, 0.2**2]])
        # EKF process dynamics jacobian (prediction) regarding the state
        # Only elements (1,3) and (2,3) need to be updated later on, so we only
        # set the correct values for the other ones now (positions (1,3) and
        # (2,3) will be updated in runtime).
        self.Jfxv = np.eye(3, 3)
        # EKF process dynamics jacobian (prediction) regarding the inputs
        # Only elements (0,0), (0,1), (1,0) and (1,1) need to be updated, so we
        # set the correct values for the other ones for now (positions (0,0),
        # (0,1), (1,0) and (1,1) will be updated in runtime).
        self.Jfuv = np.array([[0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0]])
        # Observation (sensor) noise covariance.
        # This could be updated in runtime to take into account that
        # observations of landmarks further away from the robot have larger
        #  noise, as done in the EKF tutorial.
        self.W = np.array([[0.1, 0.0],
                           [0.0, 0.1]])

        # These matrices will be fully updated on runtime, so no need to set
        # values for them now.
        self.v = np.empty((2, 1))  # Inovation factor (error)
        self.s = np.empty((2, 2))  # Innovation covariance
        # ekf.K  # Kalman gain (size will change over time)
        # Vector for observation values
        self.z = np.empty((2, 1))
        # Vector for observation model (expected observation values)
        self.h = np.empty((2, 1))
        # Matrix for the observation model jacobian
        self.Jh = np.empty((2, 3+self.num_landmarks))

        # Jacobian of the landmark world position observation with respect to
        # the robot pose. Only the last columm will need update in runtime.
        self.Jhnxv = np.array([[1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0]])
        # Jacobian of the landmark world position observation with respect to
        # the robot observation values. Will be fully updated in runtime
        self.Jhnz = np.empty((2, 2))

    def predictStep(self, delta_xr: float, delta_yr: float,
                    delta_thetar: float):
        '''
        EKF predict step.
        delta_x - amount of x displacement (robot coordinates).
        delta_y - amount of y displacement (robot coordinates).
        delta_theta, - amount of rotation (robot coordinates).
        '''

        ''' Step 1 - Prediction:
        --> State
            x(k+1|k) = f(x(k|k),u(k),k) <=>
                <=> | xr(k+1|k) | = | xr(k) + Δx(k) |
                    | yr(k+1|k) | = | yr(k) + Δy(k) |
                    | θr(k+1|k) | = | θr(k) + Δθ(k) |
                where
                    Δx = Δxr*cos(θr(k))-Δyr*sin(θr(k))
                    Δy = Δxr*sin(θr(k))+Δyr*cos(θr(k))
            with the motion beigin extracted directly from the odometry
        (see above).
        '''
        # Temporary variables
        sin_thetar = sin(self.state[2, 0])
        cos_thetar = cos(self.state[2, 0])

        dx = delta_xr * cos_thetar - delta_yr * sin_thetar
        dy = delta_xr * sin_thetar + delta_yr * cos_thetar
        dtheta = delta_thetar

        self.state[0, 0] += dx
        self.state[1, 0] += dy
        self.state[2, 0] = normalize(self.state[2, 0] + dtheta)

        # The remainder of state states (landmark positions) stays the same,
        # since the landmakrs are fixed and do not move with the robot.
        '''
        --> State covariance update, given the prediction above

              P(k+1|k) = Jfx(k)P(k|k)Jfx(k)' + Jfu(k)V(k)Jfu(k)'
                where (disregarding (k))
                    Jfx = | Jfxv 0 |,
                          |  0   I |
                    Jfu = | Jfuv |
                          |  0   |
                    V(k) is constant (initialized at the beggining),
             with  I being an identity matrix with width/height equal to the
             number of detected landmarks so far, and
                    Jfxv = | 1 0 -Δxr.sin(θr)-Δyr.cos(θr) | = | 1 0 -Δy(k) |
                           | 0 1  Δxr.cos(θr)-Δyr.sin(θr) |   | 0 1  Δx(k) |
                           | 0 0             1            |   | 0 0    1   |
                    Jfuv = | cos(θr) -sin(θr) 0 |
                           | sin(θr)  cos(θr) 0 |
                           |  0          0    1 |

             Given the sparse nature of the matrices, the process covariance
             matrix can be updated in submatrices as follows.
             Considering P = | Pv   Pvm |, computing P can be written as:
                             | Pvm' Pm  |
                 P(k+1|k) = | Jfxv.Pv.Jfxv'+Jfxu.V.Jfxu' Jfxv.Pvm |
                            |         (Jfxv.Pvm)'            Pm   |
             As expected, a change in the robot pose thus not influence the
            correlation between the various landmark estimates.
        '''

        # Note that the constant members of these matrices were already set
        # previously, so we will just update the other values.
        self.Jfxv[0, 2] = -dy
        self.Jfxv[1, 2] = dx
        self.Jfuv[0, 0] = cos_thetar
        self.Jfuv[0, 1] = -sin_thetar
        self.Jfuv[1, 0] = sin_thetar
        self.Jfuv[1, 1] = cos_thetar
        # Remeber that .T is the trasnpose and @ is used for matrix
        # multiplication
        self.P[0:3, 0:3] = \
            self.Jfxv @ self.P[0:3, 0:3] @ self.Jfxv.T + \
            self.Jfuv @ self.V @ self.Jfuv.T
        if self.num_landmarks > 0:
            self.P[0:3, 3:3+2*self.num_landmarks] = \
                self.Jfxv @ self.P[0:3, 3:3+2*self.num_landmarks]
            self.P[3:3+2*self.num_landmarks, 0:3] = \
                self.P[0:3, 3:3+2*self.num_landmarks].T

    def updateStep(self, msg: Markers):
        '''
        Run the update step.
        msg - detected markers information;
        '''

        # Do the update step (or augmentation step if it is a new landmark) for
        # each detected landmark
        for n in range(msg.num_markers):
            # Store the sensor values in z
            self.z = np.array([[msg.range[n]],
                               [msg.bearing[n]]])

            # If this is the first time we are seeing this marker, we need to
            # augment our state vector and state covariance with its
            # information. In this case we do not perform the update state, we
            # only do theaugmentation.
            if msg.id[n] not in self.marker_id_to_idx:
                # This is a new marker, lets add it
                self.marker_id_to_idx[msg.id[n]] = self.num_landmarks
                self.num_landmarks += 1
                print(f'Found new landmark with id {msg.id[n]} (stored with ' +
                      f'index {self.marker_id_to_idx[msg.id[n]]}).')

                '''
                Perform state augmentation

                Given the previous state and the landmark observation
                  Xaug = | X  |
                         | xl |
                         | yl |
                The associated covariance is given by
                  Paug = |  Pv  Pvm 0 |
                         | Pvm'  Pm 0 |
                         |  0    0  R |
                  where R is the covariance associated with the sensor noise.

                The expected landmark world position coordinates can be written
                as:
                   hn = | xr + d*cos(θr+b) |
                        | yr + d*sin(θr+b) |
                  where d and b are the observed distance and bearing.

                As such, the augmented state is basically the previous state
                plus the x and y world coordinates of the new detected
                landmark, which can be written as:
                   X = |  X |
                       | hn |
                In this case the state covariance update can be approximated
                by:
                   P = Jhn.Paug.Jhn'
                  where Jhn is the jacobian of the observation model with
                  respect to the augmented state:
                   Jhn = |  Iv    0   0  |
                         |   0   Im   0  |
                         | Jhnxv  0 Jhnz |
                Here Iv is an identity matrix with the number of columms and
                rows equal to the number of robot pose variables, and Im is an
                identity matrix with the number of columms and rows equal to
                the number of previoulsy nown landmark positions variables.
                Jhxxv and Jhnz are the jacobians of hn with respect to the
                robot pose state variables and the new landmark observation
                variables, respectively:

                   Jhnxv = | 1 0 -d.sin(θr+b) |
                           | 0 1  d.cos(θr+b) |
                   Jhnz = | cos(θr+b) -d.sin(θr+b) |
                          | sin(θr+b)  d.cos(θr+b) |

                Finnally, computing the new covariance can be written as
                   P = |    Pv      Pvm               Pv.Jhnxv'           |
                       |    Pvm'     Pm              Pvm'.Jhnxv'          |
                       | Jhnxv.Pv' Jhnxv.Pvm Jhnxv.Pv.Jhnxv'+Jhnz.R.Jhnz' |
                We can thus have a faster computation of these values.
                '''

                # Compute the expected landmark world position
                cs = cos(self.state[2, 0] + self.z[1, 0])
                sn = sin(self.state[2, 0] + self.z[1, 0])

                # Vector for observation model (initialization)
                h = np.array([[self.state[0, 0] + self.z[0, 0]*cs],
                              [self.state[1, 0] + self.z[0, 0]*sn]])

                # Compute the jacobians
                # NOTE: We could improve performance here by avoinding
                # repeating some of the computations, but I leave it this way
                # to improve readability.
                self.Jhnxv[0, 2] = -self.z[0, 0]*sn
                self.Jhnxv[1, 2] = self.z[0, 0]*cs
                self.Jhnz[0, 0] = cs
                self.Jhnz[0, 1] = -self.z[0, 0]*sn
                self.Jhnz[1, 0] = sn
                self.Jhnz[1, 1] = self.z[0, 0]*cs

                # Finnally update the state and its covariance.
                # Note that, for the covariance matrix, we will update only the
                # final row and then make the last columm equal to that row
                # transposed.

                # Add vector h to the end of the state vector
                self.state = np.append(self.state, h, axis=0)
                oldP = self.P.copy()  # Store old P value

                # Increase _P (and _tempP, and _tempI)
                self.P = np.empty((3+2*self.num_landmarks,
                                   3+2*self.num_landmarks))

                # This is the part of the covariance P which does not change
                # with new landmarks detected (robot  pose and previous
                # landmarks).
                self.P[0:3+2*(self.num_landmarks-1),
                       0:3+2*(self.num_landmarks-1)] = \
                    oldP[0:3+2*(self.num_landmarks-1),
                         0:3+2*(self.num_landmarks-1)]

                # Update last line with 1st submatrix, Jhnxv.Pv'
                self.P[3+2*(self.num_landmarks-1):3+2*self.num_landmarks,
                       0:3] = self.Jhnxv @ oldP[0:3, 0:3]

                # Update last line with Jhnxv.Pvm (if there were previous
                # landmarks).
                if self.num_landmarks > 1:
                    self.P[3+2*(self.num_landmarks-1):3+2*self.num_landmarks,
                           3:3+2*(self.num_landmarks-1)] = \
                        self.Jhnxv @ oldP[0:3, 3:3+2*(self.num_landmarks-1)]
                # Copy the two computed submatrices from the "last line",
                # transposed to the "last colummm".
                self.P[0:3+2*(self.num_landmarks-1),
                       3+2*(self.num_landmarks-1):3+2*self.num_landmarks] = \
                    self.P[3+2*(self.num_landmarks-1):3+2*self.num_landmarks,
                           0:3+2*(self.num_landmarks-1)].T

                # Compute the bottom-right submatrix with
                #  Jhnxv.Pv.Jhnxv'+Jhnz.W.Jhnz'
                self.P[3+2*(self.num_landmarks-1):3+2*self.num_landmarks,
                       3+2*(self.num_landmarks-1):3+2*self.num_landmarks] = \
                    self.Jhnxv @ oldP[0:3, 0:3] @ self.Jhnxv.T + \
                    self.Jhnz @ self.W @ self.Jhnz.T

                # Increase the matrix used for the observation model jacobian,
                # needed when processing observations of previously detected
                # landmarks.
                self.Jh = np.empty((2, 3+2*self.num_landmarks))

                # Proceed to the next landmark (skip the update step for this
                # new landmark)
                continue

            '''
            The expected sensor reading can be obtained from the robot
            estimated pose and the detected landmark position in world
            coordinates. Here j denotes the landmarl id.

               y_j(k+1) = | xl_j(k+1) |
                          | yl_j(k+1) |

              zhat = h_j(k+1) =
                    |     sqrt((xl_j-xr(k+1))²+(yl_j-yr(k+1))²)      |
                    | atan2(yl_j-yr(k+1), (xl_j-xr(k+1))) - θxr(k+1) |

            The Jacobian with respect to the state variables is given as
            follows:
               Jh_j(k+1) =
        |-(xl_j-xr)/d  -(yl_j-yr)/d   0 ...  (xl_j-xr)/d  (yl_j-yr)/d  0 ... |
        | (yl_j-yr)/d² -(xl_j-xr)/d² -1 ... -(yl_j-yr)/d² (xl_j-xr)/d² 0 ... |
              where d = sqrt((xl_j-xr(k+1))²+(yl_j-yr(k+1))²).
            Note that it considers not just the robot pose variables, but also
            the landmaks position variables.

            Given these, one can update the state using the EKF as follows:
               v = z - hj <-- Inovation
               s = Jh.P.Jh' + W <-- Inovation confidence
               K = Pa.Jh'.s⁻¹ <-- Kalman gain

               X = X + K.v
               P = P - K.S.K'
            '''

            landmark_idx = self.marker_id_to_idx[msg.id[n]]
            # Get beacon estimated world posture from our state estimate.
            marker_wpos = lfwft.Point2D(self.state[3+landmark_idx*2, 0],
                                        self.state[4+landmark_idx*2, 0])

            # Compute expected distance measure from the robot to the marker.
            d = sqrt((marker_wpos.x - self.state[0, 0])**2 +
                     (marker_wpos.y - self.state[1, 0])**2)

            # Compute expected observation (distance and bearing)
            self.h[0, 0] = d
            self.h[1, 0] = atan2(marker_wpos.y-self.state[1, 0],
                                 marker_wpos.x-self.state[0, 0]) - \
                self.state[2, 0]

            # Compute the observation Jacobian
            dlx = self.state[3+landmark_idx*2, 0] - self.state[0, 0]
            dly = self.state[4+landmark_idx*2, 0] - self.state[1, 0]
            d_sq = d**2
            self.Jh[:] = 0.0
            self.Jh[0, 0] = -dlx/d
            self.Jh[0, 1] = -dly/d
            self.Jh[0, 3+landmark_idx*2] = dlx/d
            self.Jh[0, 4+landmark_idx*2] = dly/d
            self.Jh[1, 0] = dly/d_sq
            self.Jh[1, 1] = -dlx/d_sq
            self.Jh[1, 2] = -1
            self.Jh[1, 3+landmark_idx*2] = -dly/d_sq
            self.Jh[1, 4+landmark_idx*2] = dlx/d_sq

            # Compute the confidence associated with this measure
            # The expected size of s, the innovation covariance, is (2, 2)
            self.s = self.Jh @ self.P @ self.Jh.T + self.W

            # Compute the Kalman gain.
            # The expected size of K is (3+num_landmarks*2, 2, CV_64F)
            self.K = self.P @ self.Jh.T @ inv(self.s)

            # Compute the error (inovation) of the observation
            self.v = self.z - self.h
            # Make sure the bearing error is between -PI and PI
            self.v[1, 0] = normalize(self.v[1, 0])

            # Finnally update the state and the state covariance matrix
            self.state += self.K @ self.v
            # Make sure theta is between -PI and PI
            self.state[2, 0] = normalize(self.state[2, 0])
            self.P += -self.K @ self.s @ self.K.T

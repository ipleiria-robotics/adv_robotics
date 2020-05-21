#!/usr/bin/env python3

# Copyright (c) 2020, Hugo Costelha
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
# * Neither the name of the Player Project nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE 4 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

'''@package docstring
Implementation of an Extended Kalman Filter.
'''

# Our functions
from utils import normalize

# Matrices and OpenCV related functions
import numpy as np
from numpy.linalg import inv
from math import sin, cos


class ExtendedKalmanFilter:
    ''' Extended Kalman Filter implementation '''
    def __init__(self):
        ''' Initilize Kalman filter related variables '''
        # The EKF state holds X, Y and Theta estimate of the robot, all
        # floating type
        self.state = np.zeros((3, 1))  # EKF state
        # EKF process covariance (confidence associated with current state)
        # Consider some uncertainty in the initial state
        # (Σ in the course theoretic material)
        self.P = np.array([[0.5, 0.0, 0.0],
                           [0.0, 0.5, 0.0],
                           [0.0, 0.0, 0.02]])
        # EKF dynamics covariance matrix, that is, covariance matrix associated
        # with the robot motion/actuation
        self.V = np.array([[0.1**2, 0.0, 0.0],
                           [0.0, 0.03**2, 0.0],
                           [0.0, 0.0, 0.2**2]])
        # EKF process dynamics jacobian (prediction) regarding the state
        # Only elements (1,3) and (2,3) need to be updated later on, so we only
        # set the correct values for the other ones now (positions (1,3) and
        # (2,3) will be updated in runtime).
        self.Fx = np.eye(3, 3)
        # EKF process dynamics jacobian (prediction) regarding the inputs
        # Only elements (0,0), (0,1), (1,0) and (1,1) need to be updated, so we
        # set the correct values for the other ones for now (positions (0,0),
        # (0,1), (1,0) and (1,1) will be updated in runtime).
        self.Fv = np.array([[0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0]])
        # Sensors measure jacobiann
        # In this case it is the identity matrix
        self.H = np.eye(3, 3)
        # Sensors covariance (confidence associated with sensor values)
        # Updated in runtime according to the detected markers
        # self.W = ...

    '''
    Run the EKF predict step.
    delta_x - amount of x displacement (robot coordinates).
    delta_y - amount of y displacement (robot coordinates).
    delta_theta, - amount of rotation (robot coordinates).
    '''
    def predictStep(self, delta_xr: float, delta_yr: float,
                    delta_thetar: float):
        # Temporary variables
        sin_thetar = sin(self.state[2, 0])
        cos_thetar = cos(self.state[2, 0])

        ''' Step 1 - Prediction: '''

        ''' --> State
            x(k+1|k) = f(x(k|k),u(k),k) <=>
                <=> | xr(k+1|k) | = | xr(k) + Δx(k) |
                    | yr(k+1|k) | = | yr(k) + Δy(k) |
                    | θr(k+1|k) | = | θr(k) + Δθ(k) |
            but where the motions are extracted directly from the odometry:
                Δx = Δxr*cos(θr(k))-Δyr*sin(θr(k))
                Δy = Δxr*sin(θr(k))+Δyr*cos(θr(k))
        '''
        dx = delta_xr * cos_thetar - delta_yr * sin_thetar
        dy = delta_xr * sin_thetar + delta_yr * cos_thetar
        dtheta = delta_thetar

        self.state[0, 0] += dx
        self.state[1, 0] += dy
        self.state[2, 0] = normalize(self.state[2, 0] + dtheta)

        ''' --> State covariance

            P(k+1|k) = Fx(k)P(k|k)Fx(k)' + Fv(k)V(k)Fv(k)'

                with Fx(k) = | 1 0 -Δy |
                             | 0 1  Δx |,
                             | 0 0  1  |

                Fv(k) = | cos(θr(k)) -sin(θr(k)) 0 |
                        | sin(θr(k))  cos(θr(k)) 0 |,
                        |     0           0      1 |

                V(k) constant (initialized at the beggining)

            with (as above)
                Δx = Δxr*cos(θr(k))-Δyr*sin(θr(k))
                Δy = Δxr*sin(θr(k))+Δyr*cos(θr(k))
        '''
        # Note that the constant members of Fx(k) were already set previously,
        # so we will just update the 1st and 2nd positions of the last column.
        self.Fx[0, 2] = -dy
        self.Fx[1, 2] = dx
        # Note that the constant members of Fv(k) were already set previously,
        # so we will just update the 1st and 2nd positions of the first 2
        # columns.
        self.Fv[0, 0] = cos_thetar
        self.Fv[0, 1] = -sin_thetar
        self.Fv[1, 0] = sin_thetar
        self.Fv[1, 1] = cos_thetar
        # Update the state covariance
        # Fv.T is the transpose of Fv
        # '@' means multiplication when using numpy
        self.P = self.Fx @ self.P @ self.Fx.T + self.Fv @ self.V @ self.Fv.T

    '''
    Update step.
    z - columm vector with the computed robot X, Y and Theta from the sensors
    W - covariance matriz associated with the sensor data
    '''
    def updateStep(self, z: np.matrix, W: np.matrix):

        '''
        Before doing the actual update process, let us build the z(k+1)
        (measures) vector, the h(k+1) (expected measures) vector and
        corresponding H(k+1) matrix.

            z_j(k+1) = h_j(k+1) = | xr_j(k+1) |
                                  | yr_j(k+1) |
                                  | θr_j(k+1) |

            H_j(k+1) = | 1 0 0 |
                       | 0 1 0 |
                       | 0 0 1 |
        '''
        h = self.state.copy()

        '''
        The update step consists on
            ---> Updating the state
                x(k+1|k+1) = x(k+1|k) + R*v

            ---> And updating the covariance matrix
                P(k+1|k+1) = P(k+1|k) - R*H(k+1)*P(k+1|k)

            with
                v = z(k+1) - h(x(k+1|k),k+1)
                s = H(k+1)*P(k+1|k)*H(k+1)' + W(k+1)
                K = P(k+1|K)*H(k+1)'*s^(-1)
        '''
        # Compute v - will have size (3, 1)
        v = np.array([[z[0, 0] - h[0, 0]],
                      [z[1, 0] - h[1, 0]],
                      [normalize(z[2, 0] - h[2, 0])]])

        # Compute S - will have size (msg.num_markers*2, msg.num_markers*2)
        # '@' means multiplication when using numpy
        s = self.H @ self.P @ self.H.T + W
        # Compute K - will have size (3, msg.num_markers*2)
        # inv(s) is the inverse of s
        K = self.P @ self.H.T @ inv(s)
        # Now update the state
        self.state += K @ v
        # And update the state covariance matrix
        self.P += -K @ self.H @ self.P

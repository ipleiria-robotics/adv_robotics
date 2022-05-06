#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
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
from markers_msgs.msg import Markers
import tw07.LocalFrameWorldFrameTransformations as lfwft
from tw07.utils import normalize

# Matrices and OpenCV related functions
import numpy as np
from numpy.linalg import inv
from math import sin, cos

# Other
from typing import List


class ExtendedKalmanFilter:
    ''' Extended Kalman Filter implementation '''
    def __init__(self):
        ''' Initilize Kalman filter related variables '''
        # The EKF state holds X, Y and Theta estimate of the robot, all
        # floating type
        self.state = np.zeros((3, 1))  # EKF state
        # EKF process covariance (confidence associated with current state)
        # (Σ in the course theoretic material)
        # Consider some uncertainty in the initial state
        self.P = np.array([[0.5, 0.0, 0.0],
                           [0.0, 0.5, 0.0],
                           [0.0, 0.0, 0.02]])

        # EKF dynamics covariance matrix, that is, covariance matrix associated
        # with the robot motion/actuation
        self.V = np.array([[0.05**2, 0.0, 0.0],
                           [0.0, 0.015**2, 0.0],
                           [0.0, 0.0, 0.1**2]])
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

        # Sensors measure jacobian (H)
        # H is computed in runtime, since it depends on the number of detected
        # markers
        # self.H = ...
        # Sensors covariance (confidence associated with sensor values)
        # Updated in runtime according to the detected markers
        # self.W = ...

    '''
    Run the EKF predict step.
    delta_x - amount of x displacement (in robot coordinates).
    delta_y - amount of y displacement (in robot coordinates).
    delta_theta, - amount of rotation (in robot coordinates).
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
    Run the update step.
    const markers_msgs::Markers& msg - detected markers information;
    const point_2d *_markers_wpos - markers world position
    '''
    def updateStep(self, msg: Markers, markers_wpos: List[lfwft.Point2D]):
        # Temporary variables:
        xr = self.state[0, 0]
        yr = self.state[1, 0]
        thetar = self.state[2, 0]
        sin_thetar = sin(thetar)
        cos_thetar = cos(thetar)

        '''
        Before doing the actual update process, let us build the y(k+1)
        (measures) vector, the h(k+1) (expected measures) vector and
        corresponding H(k+1) matrix.

        For each landmark j of the l possibly detected landmarks we have:

            z_j(k+1) = | xl_j(k+1) |
                       | yl_j(k+1) |

            h_j(k+1) =
                | (xl_j-xr(k+1))*cos(θr(k+1))+(yl_j-yr(k+1))*sin(θr(k+1)) |
                |-(xl_j-xr(k+1))*sin(θr(k+1))+(yl_j-yr(k+1))*cos(θr(k+1)) |

          H_j(k+1) =
                |-cos(θr(k+1)) -sin(θr(k+1)) -(xl_j-xr(k+1))*sin(θr(k+1))+
                                              (yl_j-yr(k+1))*cos(θr(k+1)) |
                | sin(θr(k+1)) -cos(θr(k+1)) -(xl_j-xr(k+1))*cos(θr(k+1))-
                                              (yl_j-yr(k+1))*sin(θr(k+1)) |
        '''
        z = np.empty((msg.num_markers*2, 1))
        h = np.empty((msg.num_markers*2, 1))
        self.H = np.empty((msg.num_markers*2, 3))
        self.W = np.zeros((msg.num_markers*2, msg.num_markers*2))
        for n in range(msg.num_markers):
            ''' Compute y (measures in cartesian coordinates) '''
            # Local x coordinate of the landmark n position
            z[n*2, 0] = msg.range[n]*cos(msg.bearing[n])
            # Local y coordinate of the landmark n position
            z[n*2+1, 0] = msg.range[n]*sin(msg.bearing[n])
            ''' Compute h '''
            # NOTE: We could have used the local2world function, as in previous
            # works, but then we needed to store h differently.
            h[n*2, 0] = (markers_wpos[msg.id[n]-1].x-xr)*cos_thetar + \
                        (markers_wpos[msg.id[n]-1].y-yr)*sin_thetar
            h[n*2+1, 0] = -(markers_wpos[msg.id[n]-1].x-xr)*sin_thetar + \
                           (markers_wpos[msg.id[n]-1].y-yr)*cos_thetar
            ''' Compute H '''
            self.H[n*2, 0] = -cos_thetar
            self.H[n*2, 1] = -sin_thetar
            self.H[n*2, 2] = -(markers_wpos[msg.id[n]-1].x-xr)*sin_thetar + \
                              (markers_wpos[msg.id[n]-1].y-yr)*cos_thetar
            self.H[n*2+1, 0] = sin_thetar
            self.H[n*2+1, 1] = -cos_thetar
            self.H[n*2+1, 2] = -(markers_wpos[msg.id[n]-1].x-xr)*cos_thetar - \
                                (markers_wpos[msg.id[n]-1].y-yr)*sin_thetar
            # The error associated with the landmark measure is higher if the
            # landmark is further away.
            #  We consider a variance of 0.05 m^2 if less than or equal to 1 m,
            # and 0.5 m^2 if equal to 8 m. For the values in between, we use a
            # linear function.
            if z[n*2, 0] < 1.0:
                self.W[n*2, n*2] = 0.05
            else:
                self.W[n*2, n*2] = 0.0643*z[n*2, 0]-0.0143
            if z[n*2+1, 0] < 1.0:
                self.W[n*2+1, n*2+1] = 0.05
            else:
                self.W[n*2+1, n*2+1] = 0.0643*z[n*2+1, 0]-0.0143

        '''
        The update step consists on
            ---> Updating the state
                x(k+1|k+1) = x(k+1|k) + K*v

            ---> And updating the covariance matrix
                P(k+1|k+1) = P(k+1|k) - K*H(k+1)*P(k+1|k)

            with
                v = z(k+1) - h(x(k+1|k),k+1)
                s = H(k+1)*P(k+1|k)*H(k+1)' + W(k+1)
                K = P(k+1|K)*H(k+1)'*s^(-1)
        '''
        # Compute v - will have size (msg.num_markers*2, 1)
        v = z-h
        # Compute S - will have size (msg.num_markers*2, msg.num_markers*2)
        # '@' means multiplication when using numpy
        s = self.H @ self.P @ self.H.T + self.W
        # Compute K - will have size (3, msg.num_markers*2)
        # inv(s) is the inverse of s
        K = self.P @ self.H.T @ inv(s)
        # Now update the state
        self.state += K @ v
        # And update the state covariance matrix
        self.P += -K @ self.H @ self.P

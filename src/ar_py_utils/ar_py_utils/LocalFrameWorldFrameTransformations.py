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
This package provides for coordinate transformations between different
reference frames.
'''

from math import sin, cos
from geometry_msgs.msg import Pose2D


class Point2D:
    ''' 2D point class (float/double) '''
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class Point2Di:
    ''' 2D point class (integers) '''
    def __init__(self, x: int = 0, y: int = 0):
        self.x = x
        self.y = y


'''
2D position transformations
'''


def local2WorldPoint(baseFrame: Pose2D, vectorInBaseFrame: Point2D) -> Point2D:
    '''
    Transform a given vector in local coordinates, referenced to localFrame
    into global coordinates.
    @Argument baseFrame - the values of the base frame
    @Argument vectorInLocalFrame - the vector coordinates in the base frame
    @Return vectorInWorldFrame - value to be returned, containing the given
    local vector in world coordinates
    '''
    sinTheta = sin(baseFrame.theta)
    cosTheta = cos(baseFrame.theta)

    # Perform the transformation
    x = (baseFrame.x + vectorInBaseFrame.x*cosTheta
         - vectorInBaseFrame.y*sinTheta)
    y = (baseFrame.y + vectorInBaseFrame.x*sinTheta
         + vectorInBaseFrame.y*cosTheta)

    # Return the result
    return Point2D(x, y)  # vectorInWorldFrame


def world2LocalPoint(baseFrame: Pose2D, vectorInWorldFrame: Point2D) -> Point2D:
    '''
    Transform a given vector in world coordinates, into a vector based on the
    given baseFrame.
    @Argument baseFrame - the value of the base frame
    @Argument vectorInWorldFrame - the vector coordinates in the world frame.
    @Argument vectorInBaseFrame - value to be returned, containing the given
     world vector in local coordinates
    '''
    sinTheta = sin(baseFrame.theta)
    cosTheta = cos(baseFrame.theta)

    # Do the transformation
    x = ((vectorInWorldFrame.x - baseFrame.x)*cosTheta +
         (vectorInWorldFrame.y - baseFrame.y)*sinTheta)
    y = (-(vectorInWorldFrame.x - baseFrame.x)*sinTheta +
          (vectorInWorldFrame.y - baseFrame.y)*cosTheta)

    # Return the result
    return Point2D(x, y)  # vectorInBaseFrame


'''
2D pose transformations (2D position + orientation)
'''


def local2WorldPose(baseFrame: Pose2D, poseInBaseFrame: Pose2D) -> Pose2D:
    '''
    Transform a given pose in local coordinates, referenced to
    localFrame into global coordinates.
    @Argument baseFrame - the values of the base frame
    @Argument poseInLocalFrame - the vector coordinates in the base
    frame
    @Return pose in World frame, containing the given local pose in world
    coordinates
    '''
    sinTheta = sin(baseFrame.theta)
    cosTheta = cos(baseFrame.theta)

    # Perform the transformation
    x = baseFrame.x + poseInBaseFrame.x*cosTheta - poseInBaseFrame.y*sinTheta
    y = baseFrame.y + poseInBaseFrame.x*sinTheta + poseInBaseFrame.y*cosTheta
    theta = baseFrame.theta + poseInBaseFrame.theta

    # Return result
    return Pose2D(x=x, y=y, theta=theta)


def world2LocalPose(baseFrame: Pose2D, poseInWorldFrame: Pose2D) -> Pose2D:
    '''
    Transform a given pose in world coordinates, into a pose based
    on the given baseFrame.
    @Argument baseFrame - the value of the base frame
    @Argument poseInWorldFrame - the pose coordinates in the world
    frame.
    @Return pose in base frame, containing the given world pose in local
    coordinates
    '''
    sinTheta = sin(baseFrame.theta)
    cosTheta = cos(baseFrame.theta)

    # Do the transformation
    x = (poseInWorldFrame.x - baseFrame.x)*cosTheta + \
        (poseInWorldFrame.y - baseFrame.y)*sinTheta
    y = -(poseInWorldFrame.x - baseFrame.x)*sinTheta + \
         (poseInWorldFrame.y - baseFrame.y)*cosTheta
    theta = poseInWorldFrame.theta - baseFrame.theta

    # Return the computed values
    return Pose2D(x=x, y=y, theta=theta)

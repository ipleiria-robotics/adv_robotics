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
This package provides for coordinate transformations between different
reference frames.
'''

from math import sin, cos
from geometry_msgs.msg import Pose2D


class Point2D:
    ''' 2D point class '''
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


def local2World(baseFrame: Pose2D, vectorInBaseFrame: Point2D) -> Point2D:
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


def world2Local(baseFrame: Pose2D, vectorInWorldFrame: Point2D) -> Point2D:
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

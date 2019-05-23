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
'''

# Library packages needed
from __future__ import print_function
from math import atan2, sin, cos

# Matrices and OpenCV related functions
import numpy as np
import cv2


def clipValue(value: float, min: float, max: float) -> float:
    '''Clip a given value to the interval [min, max]'''
    if value > max:
        return max
    elif value < min:
        return min
    else:
        return value


def normalize(angle: float) -> float:
    '''Normalize an angle between -PI and PI'''
    return atan2(sin(angle), cos(angle))


def quaternionToYaw(q) -> float:
    '''Returns the yaw in radians of a quaternion.
    Reimplements part of euler_from_quaternion from the tf package because tf
    doesn't play well in Python 3.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return atan2(t0, t1)


def createLineIterator(P1, P2, img) -> np.matrix:
    """
    Produces and array that consists of the coordinates and intensities of each
    pixel in a line between two points
    Adapted from https://stackoverflow.com/a/32857432

    Parameters:
        - P1: a numpy array with the coordinate of the first point (x,y)
        - P2: a numpy array with the coordinate of the second point (x,y)
        - img: the image being processed
        , where x is the columm and y the row

    Returns:
        - it: a numpy array that consists of the coordinates and intensities of
        each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])
    """
    # Define local variables for readability
    imageH = img.shape[0]
    imageW = img.shape[1]
    P1X = P1[0]
    P1Y = P1[1]
    P2X = P2[0]
    P2Y = P2[1]

    # Difference and absolute difference between points
    # Used to calculate slope and relative location between points
    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    # Predefine numpy array for output based on distance between points
    itbuffer = np.empty(shape=(np.maximum(dYa, dXa), 3), dtype=np.float32)
    itbuffer.fill(np.nan)

    # Obtain coordinates along the line using a form of Bresenham's algorithm
    negY = P1Y > P2Y
    negX = P1X > P2X
    if P1X == P2X:  # Vertical line segment
        itbuffer[:, 0] = P1X
        if negY:
            itbuffer[:, 1] = np.arange(P1Y-1, P1Y-dYa-1, -1)
        else:
            itbuffer[:, 1] = np.arange(P1Y+1, P1Y+dYa+1)
    elif P1Y == P2Y:  # Horizontal line segment
        itbuffer[:, 1] = P1Y
        if negX:
            itbuffer[:, 0] = np.arange(P1X-1, P1X-dXa-1, -1)
        else:
            itbuffer[:, 0] = np.arange(P1X+1, P1X+dXa+1)
    else:  # Diagonal line segment
        steepSlope = dYa > dXa
        if steepSlope:
            slope = dX.astype(np.float32)/dY.astype(np.float32)
            if negY:
                itbuffer[:, 1] = np.arange(P1Y-1, P1Y-dYa-1, -1)
            else:
                itbuffer[:, 1] = np.arange(P1Y+1, P1Y+dYa+1)
            itbuffer[:, 0] = (slope*(itbuffer[:, 1]-P1Y)).astype(np.int) + P1X
        else:
            slope = dY.astype(np.float32)/dX.astype(np.float32)
            if negX:
                itbuffer[:, 0] = np.arange(P1X-1, P1X-dXa-1, -1)
            else:
                itbuffer[:, 0] = np.arange(P1X+1, P1X+dXa+1)
            itbuffer[:, 1] = (slope*(itbuffer[:, 0]-P1X)).astype(np.int) + P1Y

    # Remove points outside of image
    colX = itbuffer[:, 0]
    colY = itbuffer[:, 1]
    itbuffer = itbuffer[(colX >= 0) & (colY >= 0) &
                        (colX < imageW) & (colY < imageH)]

    # Get intensities from img ndarray
    itbuffer[:, 2] = \
        img[itbuffer[:, 1].astype(np.uint8), itbuffer[:, 0].astype(np.uint8)]

    return itbuffer.astype(np.uint8)


def drawPos(image: np.matrix, x: float, y: float, theta: float,
            resolution: float, color: tuple, isRobot: bool = True,
            radius: int = 3):
    '''
    Draws the robot pose or landmark position in the image.
    image - image where the robot is to be drawn
    x - X coordinate of the robot/landmark [m]
    y - Y coordinate of the robot/landmark [m]
    theta - orientation of the robot [rad] (ignored for landmarks)
    resolution - [pixels/m]
    color - (Blue,Green,Red) color to be used.
    isRobot - True if drawing a robot pose, otherwise it is a landmark (
        defaults to true)
    radius - radius of the circle to be drawn (defaults to 3)
    '''

    # Create a circle in the robot/landmark position
    cv2.circle(image,
               (int(round(image.shape[1]/2.0 + x/resolution)),
                int(round(image.shape[0]/2.0 - y/resolution))),
               radius, color, -1)
    if not isRobot:
        # In case of a landmark, draw a thin blak border around the spot
        cv2.circle(image,
                   (int(round(image.shape[1]/2.0 + x/resolution)),
                    int(round(image.shape[0]/2.0 - y/resolution))),
                   radius, (0, 0, 0), 2)
    else:
        # Create a small line showing the orientation (for robot pose only)
        delta_line = 5  # [pixels]
        cv2.line(image,
                 (int(round(image.shape[1]/2.0 + x/resolution)),
                  int(round(image.shape[0]/2.0 - y/resolution))),
                 (int(round(image.shape[1]/2.0 +
                            x/resolution + cos(theta)*delta_line)),
                  int(round(image.shape[0]/2.0 -
                            y/resolution - sin(theta)*delta_line))),
                 color, 2)

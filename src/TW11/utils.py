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

# Library packages needed
from math import atan2, sin, cos
import select
import sys

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


def getKey():
    '''Non-blocking check if a key was pressed and, if so, return it. Otherwise
    return None'''
    # Check if a key was pressed
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    key_pressed = (dr != [])
    # If a key was pressed, read it and return it
    if key_pressed:
        return sys.stdin.read(1)  # Return the pressed key
    else:
        return None  # Return None if no key was pressed


def rpyToQuaternion(roll, pitch, yaw) -> np.matrix:
    '''Returns the quaternian corresponding to a yaw rotation. Reimplements
    part of the tf package because tf doesn't play well in Python 3.
    '''
    halfYaw = yaw * 0.5
    halfPitch = pitch * 0.5
    halfRoll = roll * 0.5
    cosYaw = cos(halfYaw)
    sinYaw = sin(halfYaw)
    cosPitch = cos(halfPitch)
    sinPitch = sin(halfPitch)
    cosRoll = cos(halfRoll)
    sinRoll = sin(halfRoll)
    return np.array([
        sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,  # x
        cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,  # y
        cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,  # z
        cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw])  # w


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


def drawPoses(image: np.matrix, x: float, y: float, theta: float,
              resolution: float, color: tuple,
              is_robot: bool = True, radius: int = 3):
    '''
    Draws the robot or marker position in the image.
    image - image where the robot is to be drawn
    x - X coordinate of the robot [m]
    y - Y coordinate of the robot [m]
    theta - orientation of the robot [rad]
    resolution - [m/pixels]
    color - (Blue,Green,Red) color to be used.
    is_robot - true (default) if the marker corresponds to a robot
    radius  - radius of the circle in pixels (3 by default)
    '''

    if is_robot:
        # Create a circle in the robot position
        cv2.circle(image,
                   (int(round(image.shape[1]/2.0 + x/resolution)),
                    int(round(image.shape[0]/2.0 - y/resolution))),
                   radius, color, -1)

        # Create a small line showing its orientation
        delta_line = 5  # [pixels]
        cv2.line(image,
                 (int(round(image.shape[1]/2.0 + x/resolution)),
                  int(round(image.shape[0]/2.0 - y/resolution))),
                 (int(round(image.shape[1]/2.0 +
                            x/resolution + cos(theta)*delta_line)),
                  int(round(image.shape[0]/2.0 -
                            y/resolution - sin(theta)*delta_line))),
                 color, 2)
    else:
        # Create a circle in the marker position
        cv2.circle(image,
                   (int(round(image.shape[1]/2.0 + x/resolution)),
                    int(round(image.shape[0]/2.0 - y/resolution))),
                   radius, color, 2)


def drawCross(image: np.matrix, x: float, y: float, resolution: float,
              color: tuple, radius: int = 5):
    '''
    Draws a cross in the given image at the given position.
    image - image where the cross is to be drawn
    x - X coordinate of the cross [m]
    y - Y coordinate of the cross [m]
    resolution - [m/pixels]
    color - (Blue,Green,Red) color to be used.
    radius  - radius of the cross in pixels (3 by default)
    '''
    # Draw first line
    cv2.line(image,
             (int(round(image.shape[1]/2.0 + x/resolution - radius)),
              int(round(image.shape[0]/2.0 - y/resolution + radius))),
             (int(round(image.shape[1]/2.0 + x/resolution + radius)),
              int(round(image.shape[0]/2.0 - y/resolution - radius))),
             color, 2)
    # Draw second line
    cv2.line(image,
             (int(round(image.shape[1]/2.0 + x/resolution + radius)),
              int(round(image.shape[0]/2.0 - y/resolution + radius))),
             (int(round(image.shape[1]/2.0 + x/resolution - radius)),
              int(round(image.shape[0]/2.0 - y/resolution - radius))),
             color, 2)

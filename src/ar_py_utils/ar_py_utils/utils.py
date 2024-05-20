#!/usr/bin/env python3

# Copyright (c) 2022, Hugo Costelha
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
import cv2
import datetime
from math import atan2, cos, sin
import matplotlib.pyplot as plt
import multiprocessing
import numpy as np
# from platform import uname
import queue
import subprocess
import sys
import time
import threading

# ROS
from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D, Point2Di
from geometry_msgs.msg import Quaternion, Pose
import rclpy


def quaternionToYaw(q) -> float:
    '''Returns the yaw in radians of a quaternion.
    Reimplements part of euler_from_quaternion from the tf package because tf
    doesn't play well in Python 3.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return atan2(t0, t1)


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
    return Quaternion(
        x=sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
        y=cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
        z=cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
        w=cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw)


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


def meter2cell(pt_in_meters: Point2D, map_origin: Pose, map_resolution: float):
    '''
    Given a point in worlds coordinates the map information, convert to the
    corresponding map cell.
    Note that we assume that the cell map and the world reference frame have
    the X and Y axis paralell and with the same direction.
    '''
    if (map_origin.orientation.x != 0.0) or (map_origin.orientation.y != 0.0) \
       or (map_origin.orientation.z != 0.0) \
       or (map_origin.orientation.w != 1.0):
        raise Exception('meter2cell: Conversion with orientation other than 0'
                        + ' not implemented yet!')

    # Do the actua conversion
    target_pxl = Point2Di(round((pt_in_meters.x-map_origin.position.x) /
                                map_resolution),
                          round((pt_in_meters.y-map_origin.position.y) /
                                map_resolution))
    return target_pxl


def cell2meter(cell_pt: Point2D, map_origin: Pose, map_resolution: float):
    '''
    Given a point in cell coordinates and the map information, convert to a
    point in world coordinates.
    Note that we assume that the cell map and the world reference frame have
    the X and Y axis paralell and with the same direction.
    '''
    if (map_origin.orientation.x != 0.0) or (map_origin.orientation.y != 0.0) \
       or (map_origin.orientation.z != 0.0) \
       or (map_origin.orientation.w != 1.0):
        raise Exception('cell2meter: Conversion with orientation other than 0'
                        + ' not implemented yet!')

    target_meter = Point2D(
        cell_pt.x*map_resolution + map_origin.position.x,
        cell_pt.y*map_resolution + map_origin.position.y)
    return target_meter


def downsampleOccGridMap(occ_grid_map, downsample_factor):
    '''
    Reduces the size of a map while maintaing taking the necessary steps so
    that obstacles do not disappear in the downsampling process.

        occ_grid_map - origina, occupancy grid map as numpy matrix, where -1 is
            unknown space, 0 is free space, 100 is occupied space, and any
            value in between 0 a 100 is an occupancy factor.
        downsample_factor - value greater than 1, of how much to scale down.
            Ideally it should be an odd number.

    To achieve that, we first grow the obstacles by the downsample_factor, and
    only then we downsample the map. The function returns the downsampled map
    of the same type as original map.
    '''

    # OpenCv does not suporte S8 image types for dilation, so use S16
    map_copy = np.asarray(occ_grid_map, dtype=np.int16)

    # Dilate obstacles (obstaces have the highest cell value)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (downsample_factor, downsample_factor))
    cv2.dilate(map_copy, kernel, dst=map_copy, iterations=1)

    # Since unknown cells have -1 value, the abode dilation might lead to part
    # of those areas being replaced by free or almost free cell values, which
    # we do not want to happen. As such, we will return all cells that were -1
    # previously and are not occupied after the dilation, back to -1.
    map_copy[(occ_grid_map == -1) & (map_copy <= 50)] = -1

    # Now we can resize/downsample the map
    resized_map = cv2.resize(map_copy, None,
                             fx=1./downsample_factor,
                             fy=1./downsample_factor,
                             interpolation=cv2.INTER_NEAREST)
    return resized_map.astype(np.int8)


def showImage(img: np.ndarray, title: str = str(datetime.datetime.today()),
              cmap=None):
    """Show an image in a new process without blocking. Usefull for debugging.
    Args:
        img (np.ndarray): Image to be shown
        title (str, optional): Title to be shown. Defaults to
    str(datetime.datetime.today()).
    """

    def plot(q, title, cmap):
        fig = plt.figure()
        fig.suptitle(title)
        try:
            q.get(True, 2.0)  # Wait a couple of seconds
        except queue.Empty:
            print('Not image received to plot...quitting')
            sys.exit()

        plt.imshow(img, cmap=cmap)
        plt.show()
        sys.exit()

    # Create a queue to share data between process
    q = multiprocessing.Queue()

    # Create and start the process
    proc = multiprocessing.Process(None, plot, args=(q, title, cmap))
    proc.start()
    q.put(img)


###############################################################################
# Terminal writing related functions
###############################################################################


def printxy(x, y, text):
    '''Print in a specified screen position'''
    sys.stdout.write('\x1b[%d;%df%s' % (x, y, text))


def clearTerminal():
    '''Clear terminarl
       http://ascii-table.com/ansi-escape-sequences.php'''
    sys.stdout.write('\x1b[2J')


def clearLine():
    '''Clear current terminar line
       http://ascii-table.com/ansi-escape-sequences.php'''
    sys.stdout.write('\x1b[K')

###############################################################################
# Sound related functions
###############################################################################


# def in_wsl() -> bool:
#     return 'WSL' in uname().release


def play_sound(sound_file, play_async=True, cancel_others=False):
    '''Play given sound_file. Should be called only within a ROS node.
       If play_async is True, the method starts the sound play and returns
    immediately, otherwise it waits for the sound to finish.
    '''
    # If cancel_others is true, cancel all running sounds.
    if cancel_others:
        stop_all_sounds()
    else:  # Otherwise, we will wait for other sounds to complete
        # Check if some sound is playing does not work in WSL for now)
        while True:
            result = subprocess.run(
                ['pgrep', 'paplay'], capture_output=True, text=True)
            if result.returncode == 1:
                # No process aplay is running
                break
            time.sleep(0.1)

    if play_async:
        # Asyncrhonous play. Function will continue on background
        thread = threading.Thread(target=play_sound(sound_file, False))
        thread.start()
    else:
        # Synchronous play, will only return when finished
        result = subprocess.run(['paplay', sound_file],
                                capture_output=True, text=True)
        # If something went wrong, output the error
        if result.returncode != 0:
            rclpy.logging._root_logger.warn(
                f'Problem in sound module: {result.stderr}')
            return False
    return True


def stop_all_sounds():
    '''Stop all sounds by shutting down aplay executions'''
    subprocess.run(['killall', 'paplay'], capture_output=True)

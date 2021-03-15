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


# Matrices and OpenCV related functions
import numpy as np

# Library packages needed
from math import ceil, cos, sin
import threading
from skimage.draw import line_aa
import cv2
import PyKDL
from ruamel.yaml import YAML

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import message_filters
from std_srvs.srv import Trigger

# Our functions
from sensor_msgs.msg import LaserScan


class BasicMapping(Node):
    '''
    Basic robot grid-based mapping.
    '''
    def __init__(self):
        '''
        Initializes the class instance.
        '''

        # Map related constants
        # TODO: These could be parameters.
        self.map_filename = 'map.png'
        self.map_resolution = 0.05  # [m/px]
        self.map_height_meters = 6.0  # [m]
        self.map_width_meters = 8.0  # [m]
        self.min_cell_value = 0  # Free space
        self.unkown_cell_value = 64  # Unkonwn space
        self.max_cell_value = 100  # Occuppied space
        self.cell_delta_occ = 4  # Increment update torwards occupied space
        self.cell_delta_free = -2  # Decrement update torwards free space
        height_px = ceil(self.map_height_meters/self.map_resolution)
        width_px = ceil(self.map_width_meters/self.map_resolution)
        self.map_origin = [-width_px/2.*self.map_resolution,  # x
                           -height_px/2.*self.map_resolution,  # y
                           0.]  # z

        # All map points are initialized with 127 (unkown)
        # TODO: The size could be computed and updated in real-time
        self.occ_map = np.full((height_px, width_px),
                               self.unkown_cell_value, np.int8)

        # Control iterations between map file updates
        self.iteration = 0

        # Internal variables
        self.robot_pose = Pose2D()
        self.robot_lin_vel = 0.0  # Store current linear velocity
        self.robot_ang_vel = 0.0  # Store current angular velocity
        self.closest_front_obstacle = 0.0
        self.closest_left_obstacle = 0.0
        self.closest_right_obstacle = 0.0
        self.odom_updated = False  # True if we got an odometry update
        self.laser_updated = False  # True if we got a laser data update
        self.end_program = False  # End infinite loop when True

        # Create robot related objects
        #
        self.robot_name = 'robot_0'

        # Initialize the node itself
        super().__init__('tw03_mapping')

        # TF related initializations
        self.lock = threading.Lock()

        # Setup subscribers using a TimeSynchronizer filter
        # Odometry
        self.sub_odom = message_filters.Subscriber(self, Odometry,
                                                   f'{self.robot_name}/odom')
        # LaserScan
        self.sub_laser = message_filters.Subscriber(
            self, LaserScan, f'/{self.robot_name}/base_scan')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_odom, self.sub_laser], 5, 0.05)
        ts.registerCallback(self.odom_laser_cb)

        # Setup publisher
        self.occ_grid_pub = \
            self.create_publisher(OccupancyGrid, '/robot_0/map', 1)

        # Run periodic callback (to publish the map)
        self.pubtimer = self.create_timer(5.0, self.timer_cb)

        # Provide the map save service.
        self.create_service(Trigger, 'map_save', self.map_saver_svc)

    def timer_cb(self):
        with self.lock:
            # Generate an Occupancy grid and publish it
            curr_time = self.get_clock().now().to_msg()
            occ_grid = OccupancyGrid()
            occ_grid.header.stamp = curr_time
            occ_grid.header.frame_id = f'{self.robot_name}/odom'
            occ_grid.info = \
                MapMetaData(
                    map_load_time=curr_time,
                    resolution=self.map_resolution,
                    width=self.occ_map.shape[1],
                    height=self.occ_map.shape[0],
                    origin=Pose(
                        position=Point(x=self.map_origin[0],
                                       y=self.map_origin[1],
                                       z=self.map_origin[2]),
                        orientation=Quaternion(x=0., y=0., z=0., w=1.)))
            # Convert the image to a [0;100] scale, if needed.
            if self.max_cell_value != 100:
                map2pub = np.asarray(
                    100.*(1.-self.occ_map/self.max_cell_value),
                    dtype=np.int8)
            else:
                map2pub = self.occ_map.copy()
            # Unkown space
            map2pub[self.occ_map == self.unkown_cell_value] = -1
            # Convert to the expected data
            occ_grid.data = map2pub.reshape(map2pub.size).tolist()
            # Publish occupancy grid map
            self.occ_grid_pub.publish(occ_grid)

    def odom_laser_cb(self, msg_odom: Odometry, msg_laser: LaserScan):
        ''' Process odometry and laser data
            We assume that the laser is centered in robot base_footprint frame.
            If that was not the case, we would need to take that in
            consideration.
            We are also ignoring the z coordinate, sine we are working only in
            2D.
        '''

        # Do nothing if the robot is rotating
        if abs(msg_odom.twist.twist.angular.z) > 0.001:
            return

        # Store laser position in map grid coordinates. We are assuming that
        # the laser is aligned with the robot frame, and we will ignore the
        # height, since we are working in 2D only.
        laser_map_coord = np.array(
            [round((self.map_width_meters/2. +
                    msg_odom.pose.pose.position.x)/self.map_resolution),
             round((self.map_height_meters/2. +
                    msg_odom.pose.pose.position.y)/self.map_resolution)],
            dtype=np.int)  # [Col-x, Row-y]
        # Laser frame
        laser_frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                x=msg_odom.pose.pose.orientation.x,
                y=msg_odom.pose.pose.orientation.y,
                z=msg_odom.pose.pose.orientation.z,
                w=msg_odom.pose.pose.orientation.w),
            PyKDL.Vector(
                x=msg_odom.pose.pose.position.x,
                y=msg_odom.pose.pose.position.y,
                z=msg_odom.pose.pose.position.z))

        # Go through all the LRF measurements
        angle = msg_laser.angle_min
        for i in range(len(msg_laser.ranges)):
            if(msg_laser.ranges[i] > msg_laser.range_min):
                '''
                Update map with each sensor reading.

                Important variables (already defined and available):
                - (robot_pose.x, robot_pose.y, robot_pose.theta) ==> Robot pose
                in the world frame;
                - msg.ranges[i] ==> LRF i reading (distance from the LRF to the
                beacon detected by the beacon at the i-th angle);
                '''
                # Create obstacle position in sensor coordinates from i-th
                # laser measurement
                pt_in_laser = PyKDL.Vector(msg_laser.ranges[i]*cos(angle),
                                           msg_laser.ranges[i]*sin(angle),
                                           0.)
                # Transform obstacle position to world frame using the laser
                # frame
                pt_in_world = laser_frame * pt_in_laser

                # Convert pt_in_world from world coordinates to map coordinates
                # (must be int)
                pt_in_map = np.array(
                    [round((self.map_width_meters/2. +
                            pt_in_world[0])/self.map_resolution),  # Col-x
                     round((self.map_height_meters/2. +
                            pt_in_world[1])/self.map_resolution)],  # Row-y
                    dtype=np.int)

                with self.lock:
                    # Update map for free space considering a line from the
                    # laser up the detected laser point. The first and last
                    # point are not included.
                    # In numpy we need to access the matrices as [row, col]
                    rr, cc, val = line_aa(laser_map_coord[1],
                                          laser_map_coord[0],
                                          pt_in_map[1], pt_in_map[0])
                    self.occ_map[rr[1:-1], cc[1:-1]] = \
                        np.clip(self.occ_map[rr[1:-1], cc[1:-1]]
                                + val[1:-1] * self.cell_delta_free,
                                self.min_cell_value, self.max_cell_value)

                    # Update map for occupied space (last point), if applicable
                    if msg_laser.ranges[i] < msg_laser.range_max:
                        self.occ_map[pt_in_map[1], pt_in_map[0]] = np.clip(
                            self.occ_map[pt_in_map[1], pt_in_map[0]]
                            + self.cell_delta_occ,
                            self.min_cell_value,
                            self.max_cell_value)

            # Proceed to next laser measure
            angle += msg_laser.angle_increment

    def map_saver_svc(self, request, response):
        ''' Service provided to allow saving the current map'''
        with self.lock:
            # Scale so that free space is 255, occupied is 0.
            map_save = np.asarray(
                (self.max_cell_value-self.occ_map)/(
                    1.0*self.max_cell_value)*255,
                dtype=np.uint8)
            # We need to invert the map when saving it to a file.
            cv2.imwrite(self.map_filename, np.flip(map_save, 0))

            # Now save typical map information, as shown in
            # https://index.ros.org/p/nav2_map_server/
            with open('map.yaml', 'w') as stream:
                yaml = YAML(typ="safe")
                map_data2 = dict(
                    image=self.map_filename,
                    resolution=self.map_resolution,
                    origin=self.map_origin,
                    negate=0,
                    occupied_thresh=0.65,  # >= 0.65 => occupied
                    free_thresh=0.35,  # <= 0.35 => free
                )
                # Store the YAML file with the data
                yaml.dump(map_data2, stream)
            # Send back the service response
            response.success = True
            response.message = f'Map saved as {self.map_filename}!'
            self.get_logger().info(response.message)
            return response


def main(args=None):
    '''
    Main function.
    '''

    # Output usage information
    print('Basic grid-based map generation.\n' +
          '---------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    map_node = BasicMapping()

    # Get the node executing
    rclpy.spin(map_node)

    # Cleanup memory and shutdown
    map_node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')

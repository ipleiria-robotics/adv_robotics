#!/usr/bin/env python3

# Copyright (c) 2024, Hugo Costelha
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
import threading
import matplotlib.pyplot as plt

# ROS API
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path
from nav2_msgs.msg import Costmap, CostmapMetaData
import message_filters

# Our utility functions
import ar_py_utils.utils as utils
from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D, Point2Di

# Wether to use odometry (if True) or the localization pose (if False).
# The "USE_ODOM = True" should be used only for initial tests, while you are
# not running the localization algorithm.
USE_ODOM = True


class PlannerPotentialFields(Node):
    '''
    Path global planner using potential fields from laser sensor information.
    It publishes a costmap (raw) with the potential fields result and the
    corresponding path
    '''
    def __init__(self):
        '''
        Initializes the class instance.
        '''

        # Prevent simultaneous read/write to the class variables
        self.lock = threading.Lock()

        # Amount of map resolution  scaling to apply to reduce computational
        # cost. Must be greater than or equal to 1 (no scaling if equal to 1)
        self.scale_factor = 3

        # Will hold the repulsive potential map
        self.rep_pot = None

        # Potential factors (these could be parameters)
        self.kr = 0.5  # Repulsive
        self.ka = 2.0  # Atractive

        # Initialize the node itself
        super().__init__('potential_fields_planner')

        # Setup subscribers
        # Map
        # Since the map is only published when the map server starts, we need
        # to get the message that was last pubslihed, even if it as published
        # before we subscribed the topic. To enable that behavior so, we
        # specify the TRANSIENT_LOCAL Durability Policy.
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid,
                                                'map',
                                                self.map_cb, qos_profile)

        # Setup subscribers using a ApproximateTimeSynchronizer filter. We want
        # to get the estimated robot pose which is closest in time from the
        # published goal pose.
        # Setup odometry or pose subscriber, according to USE_ODOM
        if USE_ODOM:  # Use odometry
            # Use odometry should be only until localization is fully working
            self.sub_pose = message_filters.Subscriber(
                self, Odometry, 'odom')
        else:
            # Estimated pose (from localization)
            self.sub_pose = message_filters.Subscriber(
                self, PoseWithCovarianceStamped, 'pose')
        # Goal pose
        self.sub_goal_pose = message_filters.Subscriber(
            self, PoseStamped, 'goal_pose')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_pose, self.sub_goal_pose], 5, 0.1)
        ts.registerCallback(self.goal_pose_cb)

        # Setup publishers
        # Actual potential field as costmap
        self.costmap_pub = self.create_publisher(Costmap, 'pot_costmap', 1)

        # Actual potential field as OccupancyGrid (for RViz)
        self.occ_grid_pub = self.create_publisher(OccupancyGrid,
                                                  'pot_occgrid', 1)

        # Path publisher
        self.path_pub = self.create_publisher(Path, 'path', 1)

    def map_cb(self, msg: OccupancyGrid):
        '''
        Receive an Occupancygrid type message with the map and compute the
        costmap using potential fields
        '''

        # Acquire lock, as we will be dealing with shared data.
        self.lock.acquire()

        # Get data as numpy array
        org_map = np.asanyarray(msg.data)
        # Reshape to expected rectangular size
        org_map = np.reshape(org_map, (msg.info.height, msg.info.width))

        # Store the occupancy map origin
        self.map_origin = msg.info.origin

        # Compute the new resolution to use
        self.new_map_resolution = msg.info.resolution*self.scale_factor

        # The way the agorithm is implemented, it is rather slow. As such, we
        # are using a scaled down version of the map
        self.resized_map = utils.downsampleOccGridMap(org_map,
                                                      self.scale_factor)
        # DEBUG: Store the map on disk for debugging purposes
        dbg_resized_map = np.full_like(self.resized_map, 128, dtype=np.uint8)
        dbg_resized_map[self.resized_map >= 0.] = np.asarray(
            (100. - self.resized_map[self.resized_map >= 0.])/100.*255.,
            dtype=np.uint8)
        # Flip the map to ve visually correct when stored to a file
        # We are using matplotlib to allow saving floating point data
        # plt.imshow(np.flipud(dbg_resized_map), cmap='gray')
        # plt.show()
        plt.imsave('map_resized.png', np.flipud(dbg_resized_map), cmap='gray')

        '''
        Compute repulsive potential map
        '''
        # Repulsive potential map will be the same size as the original map,
        # filled with zeros, and will use floating point numbers
        self.rep_pot = np.zeros_like(self.resized_map, dtype=float)

        # Go trough each map point and fill the costmap according the the
        # distance to all obstacles
        self.max_rep_value = 0.0
        for xr in range(0, self.resized_map.shape[1]):
            for yr in range(0, self.resized_map.shape[0]):
                rep_value = 0
                for xo in range(0, self.rep_pot.shape[1]):
                    for yo in range(0, self.rep_pot.shape[0]):
                        if self.resized_map[yo, xo] < 0:
                            # This cell occupancy is unkown, ignore it
                            continue
                        elif ((xr == xo) and (yr == yo)):
                            # Avoid dividing by 0.
                            rep_value += self.kr * \
                                (self.resized_map[yo, xo]/100.) / \
                                (1.0 * (self.new_map_resolution**2))
                        else:
                            ###################################################
                            # Compute the repulsive potential to add (when the
                            # position being evaluated is not part of an
                            # obstacle)
                            ###################################################
                            rep_value += self.kr * \
                                (self.resized_map[yo, xo]/100.) / \
                                (((xr-xo)**2 + (yr-yo)**2) *
                                 (self.new_map_resolution**2))
                            ###################################################
                self.rep_pot[yr, xr] = rep_value
                if (rep_value > self.max_rep_value):
                    self.max_rep_value = rep_value
        self.get_logger().info(
            f'Max repulsive value: {self.max_rep_value:.2f}')

        # DEBUG: Save the repulsive potential as an image to disk. Values are
        # between 0 (bloack) and 1 (white).
        if self.max_rep_value > 0:
            dbg_rep_pot = self.rep_pot/self.max_rep_value
        else:
            # In this case we only have 0s, there are no obstacles!!
            self.get_logger().warn('No obstacles were found!')
            dbg_rep_pot = self.rep_pot
        # Flip the debug image to store it with the correct visual orientation
        # We are using matplotlib to allow saving floating point data
        # plt.imshow(np.flipud(dbg_rep_pot), cmap='gray')
        # plt.show()
        plt.imsave('map_pot_rep.png', np.flipud(dbg_rep_pot), cmap='gray')

        # Release lock, since we no longer need access to the shared data
        self.lock.release()
        self.get_logger().info('Finished computing the repulsive potential')

    def goal_pose_cb(self, msg_curr_pose,  # PoseWithCovarianceStamped/Odometry
                     msg_goal_pose: PoseStamped):
        '''
        Given a target pose, compute the atractive potential, then the
        resulting potential, and then compute a path from the current
        pose to the target pose using that resulting potential.
        '''

        # Do not continue if the repulsive potential was not yet
        # computed
        with self.lock:
            if self.rep_pot is None:
                self.get_logger().warn(
                    'Got goal pose, but repulsive map is not computed yet!')
                return

        if USE_ODOM:  # Get robot pose from the odometry message
            robot_pose = Pose2D(
                x=msg_curr_pose.pose.pose.position.x,
                y=msg_curr_pose.pose.pose.position.y,
                theta=utils.quaternionToYaw(
                    msg_curr_pose.pose.pose.orientation))
        else:  # Get the robot pose from the PoseWithCovarianceStamped message
            robot_pose = Pose2D(
                x=msg_curr_pose.pose.pose.position.x,
                y=msg_curr_pose.pose.pose.position.y,
                theta=utils.quaternionToYaw(
                    msg_curr_pose.pose.pose.orientation))

        '''
        Compute atractive potential map
        '''
        # Get the robot goal position in the cell coordinates
        curr_target_px = utils.meter2cell(
            Point2D(x=msg_goal_pose.pose.position.x,
                    y=msg_goal_pose.pose.position.y),
            self.map_origin,
            self.new_map_resolution)

        atractive_pot = np.zeros(self.resized_map.shape, dtype=float)
        max_atr_value = 0.0
        for xr in range(0, atractive_pot.shape[1]):
            for yr in range(0, atractive_pot.shape[0]):
                ###############################################################
                # Compute here the atractive potential to add
                ###############################################################
                # atractive_pot[yr, xr] = ...

                ###############################################################

                if (atractive_pot[yr, xr] > max_atr_value):
                    max_atr_value = atractive_pot[yr, xr]
        self.get_logger().info(f'Max atractive value: {max_atr_value:.2f}')

        # DEBUG: Save the atractive potential as an image to disk. Values are
        # between 0 (bloack) and 1 (white).
        dbg_atr_pot = atractive_pot/max_atr_value
        # Flip the debug image to store it with the correct visual orientation
        # plt.imshow(np.flipud(dbg_atr_pot), cmap='gray')
        # plt.show()
        plt.imsave('map_pot_atr.png', np.flipud(dbg_atr_pot), cmap='gray')

        '''
        Compute the resulting potential
        '''
        # Acquire lock, as we will be dealing with shared data.
        self.lock.acquire()

        resulting_pot = self.rep_pot + atractive_pot
        max_res_val = np.max(resulting_pot)

        # DEBUG: Save the resulting potential as an image to disk. Values are
        # between 0 (black) and 1 (white).
        dbg_res_pot = resulting_pot/max_res_val
        # Flip the debug image to store it with the correct visual orientation
        # plt.imshow(np.flipud(dbg_res_pot), cmap='gray')
        # plt.show()
        plt.imsave('map_pot_res.png', np.flipud(dbg_res_pot), cmap='gray')

        # Publish the resulting potential map as a costmap, if there is at
        # least one subscriber
        curr_time_stamp = self.get_clock().now().to_msg()
        if self.costmap_pub.get_subscription_count() > 0:
            costmap = Costmap()
            costmap.header.stamp = curr_time_stamp
            costmap.header.frame_id = 'map'  # TODO: make this a parameter
            costmap.metadata = CostmapMetaData(
                map_load_time=costmap.header.stamp,
                update_time=costmap.header.stamp,
                layer='potential_fields',
                resolution=self.new_map_resolution,
                size_x=dbg_res_pot.shape[1],
                size_y=dbg_res_pot.shape[0],
                origin=self.map_origin
            )
            # Get the data from the resulting potential, scaled to 254
            costmap_data = np.asarray(resulting_pot/max_res_val*254.,
                                      dtype=np.uint8)
            # Set all obstacles to 254
            costmap_data[self.resized_map == 100] = 254
            # Set all unknown cells to 255
            costmap_data[self.resized_map == -1] = 255
            # Store the costmap data for publishing with the expected format
            costmap.data = costmap_data.reshape(costmap_data.size).tolist()
            # Publish the costmap (converted to vector and list)
            self.costmap_pub.publish(costmap)

        # Create and publish the actual potential field as an OccupancyGrid,
        # used, for instance, by RViz. Do so oly if there is at least one
        # subscriber.
        if self.occ_grid_pub.get_subscription_count() > 0:
            occ_grid = OccupancyGrid()
            occ_grid.header.stamp = curr_time_stamp
            occ_grid.header.frame_id = 'map'
            occ_grid.info = \
                MapMetaData(
                    map_load_time=curr_time_stamp,
                    resolution=self.new_map_resolution,
                    width=dbg_res_pot.shape[1],
                    height=dbg_res_pot.shape[0],
                    origin=self.map_origin)
            # Convert the resulting potential to a [0;100] scale, if needed.
            occ_grid_data = np.asarray(resulting_pot/max_res_val*100.,
                                       dtype=np.int8)
            # Set all obstacles to 100
            occ_grid_data[self.resized_map == 100] = 100
            # Set all unknown cells to -1
            occ_grid_data[self.resized_map == -1] = -1
            # Store the costmap data for publishing with the expected format
            occ_grid.data = occ_grid_data.reshape(occ_grid_data.size).tolist()
            # Publish occupancy grid map
            self.occ_grid_pub.publish(occ_grid)

        # Store the robot current/start position in world coordinates
        # as Pose2D
        curr_target_meters = Pose2D(x=robot_pose.x,
                                    y=robot_pose.y,
                                    theta=robot_pose.theta)

        # Generate a path from the resulting potential (costmap)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'  # TODO: make this a parameter
        curr_target_px = utils.meter2cell(curr_target_meters,
                                          self.map_origin,
                                          self.new_map_resolution)
        while True:
            # Add current target to the path
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = path.header.frame_id
            # Store position
            pose.pose.position.x = curr_target_meters.x
            pose.pose.position.y = curr_target_meters.y
            pose.pose.position.z = 0.0
            # Store orientation
            pose.pose.orientation = utils.rpyToQuaternion(
                0., 0., curr_target_meters.theta)
            # Add to the path
            path.poses.append(pose)

            # Search near cell with lower potential
            # Get potential for current pose
            current_potential = resulting_pot[curr_target_px.y,
                                              curr_target_px.x]
            # Assume for now that there is no lower potential around this point
            next_target_px = Point2Di(curr_target_px.x, curr_target_px.y)
            # Do the search on the neighbourhood (using 8-connectivity)
            for x in range(curr_target_px.x-1, curr_target_px.x+2):
                for y in range(curr_target_px.y-1, curr_target_px.y+2):
                    # Do not check points that are outside the map
                    if ((x < 0) or (x >= resulting_pot.shape[1]) or
                            (y < 0) or (y >= resulting_pot.shape[0])):
                        continue
                    # Is this a point with lower potential?
                    if (resulting_pot[y, x] < current_potential):
                        current_potential = resulting_pot[y, x]
                        next_target_px.x = x
                        next_target_px.y = y

            # If the computed next target is the same as the previous target,
            # then this is the last point, and we're finished. Otherwise,
            # continue so has to add the target to the path.
            if ((curr_target_px.x == next_target_px.x) and
                    (curr_target_px.y == next_target_px.y)):
                # Print current potential value
                self.get_logger().info(
                     f'Final potential = {current_potential:.2f}')
                break

            # Get position in world coordinates [meters], to be used in the
            # next while iteration.
            curr_target_px = next_target_px
            curr_pt_meters = utils.cell2meter(
                curr_target_px, self.map_origin, self.new_map_resolution)
            curr_target_meters = Pose2D(
                x=curr_pt_meters.x,
                y=curr_pt_meters.y,
                theta=0.)  # Not be used, set to "0"

        # Publish the path
        self.path_pub.publish(path)

        # Release lock, since we no longer need access to the shared data
        self.lock.release()


def main(args=None):
    '''
    Main function.
    '''

    # Output usage information
    print('Potential fields-based path generation\n' +
          '--------------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    node = PlannerPotentialFields()

    # Get the node executing
    rclpy.spin(node)

    # Cleanup memory and shutdown
    node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')

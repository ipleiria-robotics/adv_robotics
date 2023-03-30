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


# Matrices and OpenCV related functions
import numpy as np

# Library packages needed
from math import sqrt
import threading
import time
from collections import deque

# ROS API
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
import message_filters

# Our utility functions
import ar_py_utils.utils as utils
from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D
from tw06.Graph import Graph, MapPoint, SearchMethods
from tw06.Graph import Node as GraphNode

# Wether to use odometry (if True) or the localization pose (if False).
# The "USE_ODOM = True" should be used only for initial tests, while you are
# not running the localization algorithm.
USE_ODOM = True

# If true, show/publish debug information
DEBUG = True


class SearchAndPlanning(Node):
    '''
    Path generation using breadth-first, depth-first and A* search algorithms.
    It the path given a goal pose and the current robot pose.
    '''
    def __init__(self):
        '''
        Initializes the class instance.
        '''
        # Prevent simultaneous read/write to the class variables
        self.lock = threading.Lock()

        # Will hold the graph for the path search
        self.graph = None

        # Robot name(space)
        self.robot_name = 'robot_0'

        # Initialize the node itself
        super().__init__('search_and_planning')

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
        self.sub_map = self.create_subscription(OccupancyGrid, '/map',
                                                self.map_cb, qos_profile)

        # Setup subscribers using a ApproximateTimeSynchronizer filter. We want
        # to get the estimated robot pose which is closest in time from the
        # published goal pose.
        # Setup odometry or pose subscriber, according to USE_ODOM
        if USE_ODOM:  # Use odometry
            # Use odometry should be only until localization is fully working
            self.sub_pose = message_filters.Subscriber(
                self, Odometry, f'/{self.robot_name}/odom')
        else:
            # Estimated pose (from localization)
            self.sub_pose = message_filters.Subscriber(
                self, PoseStamped, f'{self.robot_name}/pose')
        # Goal pose
        self.sub_goal_pose = message_filters.Subscriber(
            self, PoseStamped, f'{self.robot_name}/goal_pose')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_pose, self.sub_goal_pose], 5, 0.5)
        ts.registerCallback(self.goal_pose_cb)

        # Setup publishers
        if DEBUG:
            # We will use this "color image" for debuggning purposes
            self.dbg_img = None
            # Debug image showing the generated cells
            self.dbg_img_pub = self.create_publisher(
                Image, f'/{self.robot_name}/dbg_search', qos_profile)

        # Path publisher
        self.path_pub = self.create_publisher(Path,
                                              f'/{self.robot_name}/path', 1)

    @staticmethod
    def heuristic(current_position: MapPoint,
                  goal_position: MapPoint) -> float:
        '''
        Compute the heuristic given the current position and the goal position.
        Returns the heuristic value.
        '''
        # Use Euclidean distance
        return sqrt((goal_position.x-current_position.x)**2 +
                    (goal_position.y-current_position.y)**2)

    def map_cb(self, msg: OccupancyGrid):
        '''
        Receive an Occupancygrid type message with the map and store an
        internal copy of it.
        '''

        # Avoid simultaneous access to the graph
        with self.lock:
            # Store map information internally
            self.map_origin = msg.info.origin
            self.map_resolution = msg.info.resolution
            self.occgrid = np.reshape(np.asanyarray(msg.data),
                                      (msg.info.height, msg.info.width))
            # Create/initialize graph with occupancy grid.
            # The debug mode must be set to true to enable publishing the graph
            # image view for debugging purposes.
            self.graph = Graph(self.occgrid, debug_mode=DEBUG)
            if DEBUG:
                # Publish initial map/graph image view.
                self.graph.showGraph(self.dbg_img_pub,
                                     self.get_clock().now().to_msg(),
                                     'map')
            self.get_logger().info('Got and stored local copy of the map')

    def goal_pose_cb(self, msg_curr_pose,  # PoseStamped or Odometry
                     msg_goal_pose: PoseStamped):
        '''
        Given a the robot current and target/goal poses, compute the path from
        the starting pose to the goal pose. We are only considering positions.
        '''

        # Do not continue if the repulsive potential was not yet
        # computed
        with self.lock:
            if self.graph is None:
                self.get_logger().warn(
                    'Got goal pose, but the map was not received yet!')
                return

        # Avoid simultaneous access to the map while searching
        self.lock.acquire()

        if USE_ODOM:  # Get robot pose from the odometry message
            robot_pose = Pose2D(
                x=msg_curr_pose.pose.pose.position.x,
                y=msg_curr_pose.pose.pose.position.y,
                theta=utils.quaternionToYaw(
                    msg_curr_pose.pose.pose.orientation))
        else:  # Get the robot pose from the PoseStamped message
            robot_pose = Pose2D(
                x=msg_curr_pose.pose.position.x,
                y=msg_curr_pose.pose.position.y,
                theta=utils.quaternionToYaw(msg_curr_pose.pose.orientation))

        # Convert start and end poses to map positions (cell coordinates)
        start_position_px = utils.meter2cell(Point2D(x=robot_pose.x,
                                                     y=robot_pose.y),
                                             self.map_origin,
                                             self.map_resolution)
        start_pt = MapPoint(start_position_px.x, start_position_px.y)
        goal_position_px = utils.meter2cell(
            Point2D(x=msg_goal_pose.pose.position.x,
                    y=msg_goal_pose.pose.position.y),
            self.map_origin,
            self.map_resolution)
        goal_pt = MapPoint(goal_position_px.x, goal_position_px.y)

        # Perform the map search and, if successful, and DEBUG is active, show
        # the resulting path in the terminal output as text
        path = self.doSearch(start_pt, goal_pt, SearchMethods.A_STAR)
        if path is not None:
            # Show the last graph image view
            if DEBUG:
                self.get_logger().debug('Showing final searched map')
                self.graph.showGraph(self.dbg_img_pub,
                                     self.get_clock().now().to_msg(),
                                     'map')
                time.sleep(2.0)
                self.get_logger().debug('Showing final path')
                self.graph.showPath(path, self.get_logger(), self.dbg_img_pub,
                                    self.get_clock().now().to_msg(), 'map')
            # Create the path message to be published
            path_to_publish = Path()
            path_to_publish.header.stamp = self.get_clock().now().to_msg()
            path_to_publish.header.frame_id = 'map'  # TODO: use a parameter
            for node in path:
                # Convert from map coordinates to world coordinates
                curr_target = utils.cell2meter(Point2D(node.x, node.y),
                                               self.map_origin,
                                               self.map_resolution)
                # Add current target to the path
                pose = PoseStamped()
                pose.header.stamp = path_to_publish.header.stamp
                pose.header.frame_id = path_to_publish.header.frame_id
                # Store position
                pose.pose.position.x = curr_target.x
                pose.pose.position.y = curr_target.y
                pose.pose.position.z = 0.0
                # Store orientation
                pose.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.0)
                # Add to the path
                path_to_publish.poses.append(pose)

            # Publish the path
            self.path_pub.publish(path_to_publish)

        else:
            self.get_logger().warn(
                'There is no solution for the specified problem!')

        # We are done, reset the graph
        self.graph = Graph(self.occgrid, debug_mode=DEBUG)

        # Release lock, since we no longer need access to the shared data
        self.lock.release()

    def doSearch(self, start_position: MapPoint, goal_position: MapPoint,
                 search_type: SearchMethods) -> bool:
        '''Perform search on a graph.
        Returns true if the solution was found.'''

        # Set the graph goal position
        self.graph.setGoalPosition(goal_position)

        # Create root node at the given start position and add it to the graph
        # map_graph - graph this node belongs to
        # None - no parent
        # 0 - no cost
        # heuristic function
        # start_position
        # None - no action needed to reach this node
        root = GraphNode(self.graph, None, 0, self.heuristic,
                         start_position, None)
        self.graph.addNode(root, True)

        # This variable will get true if we find a solution, i.e., a path from
        # the start position to the goal
        solutionFound = False

        # Output debug line
        self.get_logger().info(
            ' ----> Performing path-planning search in a grid-based map:')

        # List of nodes which were already generated but not yet explored.
        nodesToExplore = deque()

        # Add the root node to the nodes that were already generated, but not
        # yet explored. This will be the first to expanded.
        nodesToExplore.append(root)

        # Keep expanding nodes until we found a solution (a path from start
        # position to the goal position), or until there are no more nodes to
        # explore.
        while (len(nodesToExplore) > 0):
            # Get the first node on the list of nodes to be explored (the node
            # is also removed from the list of nodes to be explored)
            node = nodesToExplore.popleft()

            # Check if the current node is the solution, that is, if its
            # position corresponds to the goal position. If so, the search ends
            # now.
            if ((node.map_position_.x == goal_position.x) and
               (node.map_position_.y == goal_position.y)):
                # We found the solution, leave...
                solutionFound = True
                break

            # Expand node by generating all its children, stored in the
            # newNodes variable.
            newNodes = node.expand()

            # Add the new nodes to the list of nodes that were already
            # generated but not yet explored.
            if (search_type == SearchMethods.DEPTH_FIRST):
                ###############################################################
                # Put code here to update nodesToExplore (Depth-first search)
                ###############################################################
                pass  # REPLACE ME

                ###############################################################
            elif (search_type == SearchMethods.BREADTH_FIRST):
                ###############################################################
                # Put code here to update nodesToExplore (Breadth-first search)
                ###############################################################
                pass  # REPLACE ME

                ###############################################################
            elif (search_type == SearchMethods.A_STAR):
                # Add the nodes such that the ones with lowest total cost are
                # in the beggining.
                for new_node in newNodes:
                    # Look for the node with higher total cost than this one,
                    # and insert the new node before that node.
                    # This could be done in a more efficient way!
                    i = 0
                    while i < len(nodesToExplore):
                        if (nodesToExplore[i].total_cost_ >
                           new_node.total_cost_):
                            break
                        else:
                            i += 1
                    nodesToExplore.insert(i, new_node)

            # Show map with search information debugging purposes
            if DEBUG:
                self.graph.showGraph(self.dbg_img_pub,
                                     self.get_clock().now().to_msg(),
                                     'map')

        # If a solution was found, return the corresponding path, else, return
        # None.
        if solutionFound:
            finalPath = deque()
            # Get goal node
            node = self.graph.nodes_list_[goal_position.label]
            # Cycle through all available nodes starting from the goal to the
            # start node.
            while (True):
                finalPath.appendleft(node.map_position_)
                # get this node parent
                node = node.parent_
                # If this new node is our start position, i.e., it is our root,
                # we are finished
                if (node == self.graph.root_):
                    finalPath.appendleft(node.map_position_)
                    break
            return finalPath
        else:
            return None


def main(args=None):
    '''
    Main function.
    '''

    # Output usage information
    print('Search and Planning-based path generation\n' +
          '--------------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    node = SearchAndPlanning()

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

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

# 3rd party libraries
import numpy as np
from enum import Enum
from typing import Callable, Deque
from collections import deque
import time
import cv2

# ROS libraries
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

MAP_FREE_THRESHOLD = 40


class MapPoint:
    '''Used to access/store a map point (as int and as string)'''
    def __init__(self, x: int, y: int):
        self.x = x  # Column
        self.y = y  # Row
        # Store "(y,x)"
        self.label = "(" + str(x) + "," + str(y) + ")"


class Action:
    '''Used to access/store an action (map point change)'''
    def __init__(self, x: int, y: int):
        self.x = x  # Column
        self.y = y  # Row


class SearchMethods(Enum):
    '''Types of available search methods'''
    DEPTH_FIRST = 0
    BREADTH_FIRST = 1
    A_STAR = 2


class Node:
    ''' Graph-based map node'''
    def __init__(self, graph: 'Graph', parent: 'Node', cost: float,
                 h: Callable[[MapPoint, MapPoint], float],
                 map_position: MapPoint, action: Action):
        # Store received values
        # World state associated with this node (y,columm of the map)
        self.map_position_ = map_position
        # Cost of this node
        self.cost_ = cost
        # Parent node
        self.parent_ = parent
        # Graph this node belongs to
        self.graph_ = graph
        # Heuristic function
        self.hf_ = h
        # Estimated cost from here to the goal (heuristic)
        self.h_ = self.hf_(self.map_position_, self.graph_.goal_position_)
        # Total (estimated) Cost ( = cost_ + h_)
        self.total_cost_ = self.cost_ + self.h_
        # Action that lead to this node
        self.action_ = action
        # Vector of nodes children of this node
        self.children_ = deque()
        # List of unborn children (actions yet to be done)
        self.unborn_children_ = deque()

        '''
        List of available actions
         - We have 4 possible actions
         - Each action has an x and y displacement
         - Each action will correspond to a possible child
         - We use MapPoint type to store the row/column displacement
        '''
        self.actions_ = deque()
        # Up - stay in the same column (x), move one row (y) down
        self.actions_.append(Action(0, 1))
        # Right
        self.actions_.append(Action(1, 0))
        # Left
        self.actions_.append(Action(-1, 0))
        # Down
        self.actions_.append(Action(0, -1))

        # Add all actions as unborn children
        self.unborn_children_.extend(self.actions_)

    def expand(self) -> Deque['Node']:
        '''Create all this node children'''
        # Store all added nodes to be returned by the end of this function
        addedNodes = deque()

        # While there are unborn children, give birth to them
        while len(self.unborn_children_) > 0:
            # Get first unborn child in the list (older itens on the left)
            child_action = self.unborn_children_.popleft()

            # Move on the map
            x = self.map_position_.x + child_action.x
            y = self.map_position_.y + child_action.y

            # Check if we are outside the map limits
            if ((y < 0) or (x < 0) or
                (y >= self.graph_.map_.shape[0]) or
                    (x >= self.graph_.map_.shape[1])):
                # Ignore this one, it is outside the map limits
                continue

            # If the obtained map position is occupied, or unkown, then we skip
            # this position
            if (self.graph_.map_[y, x] > MAP_FREE_THRESHOLD) or \
               (self.graph_.map_[y, x] < 0):
                continue

            # We have a new map point to explore in the future, this will be a
            # new child if not generated previously with lower cost
            map_point = MapPoint(x, y)
            child_cost = self.cost_ + 1

            # Check if the new position was already processed
            if (map_point.label in self.graph_.nodes_list_):
                node = self.graph_.getNode(map_point.label)
                # If the new cost to the node is better, change the node cost
                # and parent given the newly born child cost and parent
                if (child_cost < node.cost_):
                    # Remove the node from the graph. We will read it with the
                    # new cost.
                    self.graph_.removeNode(map_point.label)
                else:
                    # There is already an equal node with lower cost, so forget
                    # this one.
                    continue

            # If we reached this far, then we have a new node
            child = Node(self.graph_, self, child_cost, self.hf_, map_point,
                         child_action)
            # Add node to this node list of nodes
            self.children_.append(child)
            # Add new child to the list of added children
            addedNodes.append(child)
            # Add new child to the list of nodes in the graph
            self.graph_.addNode(child)

        return addedNodes


class Graph:
    def __init__(self, occgrid_map: OccupancyGrid, debug_mode: bool = False):
        '''Stores the map and initializes internal variables'''
        self.nodes_list_ = dict()

        # Store our own copy of the map (it contains occupancy grid-like
        # values)
        self.map_ = occgrid_map

        # If debug mode is on, create auxialiary debug image
        if debug_mode:
            # Image - ROS msg conversion
            self.cvbridge_ = CvBridge()

            self.debug_mode = True
            # Initialize the debug imageto 128 (unknown)
            dbg_img = np.full(occgrid_map.shape, 128, dtype=np.uint8)
            # Convert all "known" cells from [0; 100] to [255; 0]
            dbg_img[occgrid_map >= 0] = \
                np.asarray((100-occgrid_map[occgrid_map >= 0])/100.*255,
                           dtype=np.uint8)
            # Store in color
            self.dbg_img = cv2.cvtColor(dbg_img, cv2.COLOR_GRAY2BGR)
        else:
            self.debug_mode = False

    def setGoalPosition(self, goal_position: MapPoint):
        '''Store the goal position internally'''
        self.goal_position_ = goal_position

    def addNode(self, node: Node, is_root: bool = False):
        '''Add node to the graph.'''
        # If the node was already included previously, do not add it
        if (node.map_position_.label in self.nodes_list_):
            self.get_logger().error(f'Error, node ({node.map_position_.label}'
                                    + ') was already included previously!')
        else:
            # Add node
            self.nodes_list_[node.map_position_.label] = node
            if is_root:
                self.root_ = node
            if self.debug_mode:
                self.dbg_img[node.map_position_.y, node.map_position_.x] = \
                    [0, 0, 255]

    def removeNode(self, node_label: str):
        '''Remove node from graph'''
        # Check if the node is in the graph
        if (node_label not in self.nodes_list_):
            self.get_logger().error(f'Error, node ({node_label}'
                                    + ') is not part of the graph!')
        else:
            # remove node
            del self.nodes_list_[node_label]

    def getNode(self, node_label: str) -> Node:
        '''Get a node with the given label from the graph'''
        if (node_label in self.nodes_list_):
            return self.nodes_list_[node_label]
        else:
            return None

    def showGraph(self, img_pub, stamp, frame_id):
        '''
        Publish an image view of the graph (for debug purposes), given an Image
        pubsliher.
        '''
        if self.debug_mode is False:
            print('Debug mode was not activated during initialization')
        else:
            # Publish graph image converted to a ROS message.
            img_to_publish = self.cvbridge_.cv2_to_imgmsg(
                np.flipud(self.dbg_img), encoding='bgr8')
            img_to_publish.header.stamp = stamp
            img_to_publish.header.frame_id = frame_id
            img_pub.publish(img_to_publish)

    def showPath(self, path: Deque, logger, img_pub, stamp, frame_id):
        '''Show a path from the root node to the given map position'''

        # Go through the list and print all positions
        logger.info(
            f'This is the path from {path[0].label} to {path[-1].label}:')
        for node in path:
            logger.info(f' --> {node.label}')
            if self.debug_mode:
                self.dbg_img[node.y, node.x] = [0, 255, 0]
                img_to_publish = self.cvbridge_.cv2_to_imgmsg(
                    np.flipud(self.dbg_img), encoding='bgr8')
                img_to_publish.header.stamp = stamp
                img_to_publish.header.frame_id = frame_id
                img_pub.publish(img_to_publish)
                # Wait some time just to allow seeing the values go by.
                time.sleep(0.05)

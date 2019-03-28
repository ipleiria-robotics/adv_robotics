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

# 3rd party libraries
import numpy as np
from matplotlib import pyplot as plt
from enum import Enum
from typing import Callable, Deque
from collections import deque

# Our libraries
from utils import eprint

MAP_FREE_THRESHOLD = 127


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
        # Down - stay in the same column (x), move one row (y) down
        self.actions_.append(Action(0, 1))
        # Right
        self.actions_.append(Action(1, 0))
        # Left
        self.actions_.append(Action(-1, 0))
        # Up
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
            if((y < 0) or (x < 0) or
               (y >= self.graph_.map_.shape[0]) or
               (x >= self.graph_.map_.shape[1])):
                # Ignore this one, it is outside the map limits
                continue

            # If the obtained map position is occupied, then we skip this
            # position
            if(self.graph_.map_[y, x] < MAP_FREE_THRESHOLD):
                continue

            # We have a new map point to explore in the future, this will be a
            # new child if not generated previously with lower cost
            map_point = MapPoint(x, y)
            child_cost = self.cost_ + 1

            # Check if the new position was already processed
            if(map_point.label in self.graph_.nodes_list_):
                node = self.graph_.getNode(map_point.label)
                # If the new cost to the node is better, change the node cost
                # and parent given the newly born child cost and parent
                if(child_cost < node.cost_):
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
    def __init__(self, map_filename: str, goal_position: MapPoint):
        '''Stores the map and the root node'''
        self.goal_position_ = goal_position
        self.nodes_list_ = dict()

        # Read the map from the image file
        org_map_color = plt.imread(map_filename)
        # Image is read as a color image with alpha (transparancy) channel, but
        # we just want a grayscale image, so lets keep only the 1st image
        # plane, and we convert it to uint8 (0 to 255 per pixel)
        self.map_ = (org_map_color[:, :, 1]*255).astype(np.uint8)
        # In this stage, the image (map) is stored with floating point value,
        # where 0.0 is black and 1.0 is white.

        # Create a window and show the map
        plt.figure("Map")
        plt.cla()
        plt.imshow(self.map_, cmap='gray')
        plt.pause(0.01)
        # Debug the map values, by showing the map as text in the terminal
        print(self.map_)

    def addNode(self, node: Node, is_root: bool = False):
        '''Add node to the graph'''
        # If the node was already included previously, do not add it
        if(node.map_position_.label in self.nodes_list_):
            eprint('Error, node (' + node.map_position_.label +
                   ') was already included previously!')
        else:
            # Add node
            self.nodes_list_[node.map_position_.label] = node
            if(is_root):
                self.root_ = node

    def removeNode(self, node_label: str):
        '''Remove node from graph'''
        # Check if the node is in the graph
        if(node_label not in self.nodes_list_):
            eprint('Error, node (' + node_label +
                   ') is not part of the graph!')
        else:
            # remove node
            del self.nodes_list_[node_label]

    def getNode(self, node_label: str) -> Node:
        '''Get a node with the given label from the graph'''
        if(node_label in self.nodes_list_):
            return self.nodes_list_[node_label]
        else:
            return None

    def showGraph(self):
        '''Show a text view of the map (for debug purposes)'''
        for y in range(self.map_.shape[0]):
            for x in range(self.map_.shape[1]):
                map_point = MapPoint(x, y)
                if(map_point.label in self.nodes_list_):
                    total_cost = self.nodes_list_[map_point.label].total_cost_
                    print(f'{total_cost:5.2f} ', end='')
                elif(self.map_[y, x] < MAP_FREE_THRESHOLD):
                    print('   *  ', end='')
                else:
                    print('   .  ', end='')
            print('')

        print('\n')

    def showPath(self, goal_position: MapPoint):
        '''Show a path from the root node to the given map position'''
        finalPath = deque()
        # Get goal node
        node = self.nodes_list_[goal_position.label]
        # Cycle through all available nodes starting from the goal to the start
        # node
        while(True):
            finalPath.appendleft(node.map_position_)
            # get this node parent
            node = node.parent_
            # If this new node is our start position, i.e., it is our root, we
            # are finished
            if(node == self.root_):
                finalPath.appendleft(node.map_position_)
                break

        # Map to show path
        map_color = np.empty([self.map_.shape[0], self.map_.shape[1], 3],
                             self.map_.dtype)
        # Initialize this a a colored image form the original greyscale map
        map_color[:, :, 0] = self.map_
        map_color[:, :, 1] = self.map_
        map_color[:, :, 2] = self.map_

        # Now go through the list and print all positions
        print('This is the path from ' + self.root_.map_position_.label +
              ' to ' + goal_position.label + ":")

        # Show the path on the map and as text
        plt.figure("Map")
        for node in finalPath:
            print(' --> ' + node.label, end='')
            map_color[node.y, node.x] = [255, 0, 0]
            plt.cla()
            plt.imshow(map_color)
            plt.pause(0.1)
        print('\n')  # Change line

        # Save image
        plt.imsave('Map_solution.png', map_color)

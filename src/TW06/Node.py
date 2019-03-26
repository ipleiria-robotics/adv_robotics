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

from typing import Callable, List
from collections import deque
import numpy as np

from Graph import Graph

MAP_FREE_THRESHOLD = 0.5


class MapPoint:
    '''Used to access store a map point (as int and as string)'''
    def __init__(self, prow: int, pcolumn: int):
        self.row = prow
        self.column = pcolumn
        # Store "(row,column)"
        self.label = "(" + prow.toString() + "," + pcolumn.toString() + ")"


class Node:
    ''' Graph-based map node'''
    def __init__(self, graph: Graph, parent: 'Node', cost: float,
                 h: Callable[[MapPoint, MapPoint], float],
                 map_position: MapPoint, action: List[int]):
        # Store received values
        # World state associated with this node (row,columm of the map)
        self.map_position_ = map_position
        # Cost of this node
        self.cost_ = cost
        # Total (estimated) Cost ( = cost_ + h_)
        self.total_cost_ = 0.0
        # Parent node
        self.parent_ = parent
        # Graph this node belongs to
        self.graph_ = graph
        # Estimated cost from here to the goal (heuristic)
        self.h_ = 0
        # Heuristic function
        self.hf_ = h
        # Action that lead to this node
        self.action_ = action
        # Vector of nodes children of this node
        self.children_ = []
        # List of unborn children (actions yet to be done)
        self.unborn_children_ = deque()

        '''
        List of available actions
         - We have 4 possible actions
         - Each action has a column and row displacement
         - Each action will correspond to a possible child
        '''
        actions = []
        # Down - stay in the same column, move one row down
        actions[0] = np.array([0, 1], dtype=np.int8)
        # Right
        actions[1] = np.array([1, 0], dtype=np.int8)
        # Left
        actions[2] = np.array([-1, 0], dtype=np.int8)
        # Up
        actions[3] = np.array([0, -1], dtype=np.uint8)

        # Add all actions as unborn children
        for i in range(0, self.actions_.size()):
            self.unborn_children_.append(actions[i])

        # Compute heuristic value
        self.h_ = self.hf_(self.map_position_, self.graph.goal_position_)
        self.total_cost_ = self.cost_ + self.h_

    def expand(self) -> List['Node']:
        '''Create all this node children'''
        # Store all added nodes to be returned by the end of this function
        addedNodes = []

        # While there are unborn children, give birth to them
        while self.unborn_children_.count() == 0:
            # Get first unborn child in the list (older itens on the left)
            child_action = self.unborn_children_.popleft()

            # Move on the map
            row = self.map_position_.row + child_action[1]
            column = self.map_position_.column + child_action[0]

            # Check if we are outside the map limits
            if((row < 0) or (column < 0) or
               (row >= self.graph_.map_.shape(0)) or
               (column >= self.graph_.map_.shape(1))):
                # Ignore this one, it is outside the map limits
                continue

            # If the obtained map position is occupied, then we skip this
            # position
            if(self.graph_.map_[row, column] < MAP_FREE_THRESHOLD):
                continue

            # We have a new map point to explore in the future, this will be a
            # new child if not generated previously with lower cost
            map_point = MapPoint(row, column)
            child_cost = self.cost_ + 1

            # Check if the new position was already processed
            if(self.graph_.isNodeInGraph(map_point.label)):
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

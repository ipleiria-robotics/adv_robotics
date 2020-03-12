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

# 3rd party libraries
from math import sqrt
import time
import sys
from collections import deque
from matplotlib import pyplot as plt

# Our libraries
from Graph import Graph, MapPoint, Node, SearchMethods


def heuristic(current_position: MapPoint, goal_position: MapPoint) -> float:
    '''Compute the heuristic given the current position and the goal position.
       Returns the heuristic value.'''
    # Use Euclidean distance
    return sqrt((goal_position.x-current_position.x)**2 +
                (goal_position.y-current_position.y)**2)


def doSearch(graph: Graph, root: Node, goal_position: MapPoint,
             search_type: SearchMethods) -> bool:
    '''Perform search on a graph.
       Returns true if the solution was found.'''
    # This variable will get true if we find a solution, i.e., a path from the
    # start position to the goal
    solutionFound = False

    # Output debug line
    print(' ----> Performing depth-first search in a grid-based map:\n')

    # Show initial map as text in the terminal
    graph.showGraph()
    time.sleep(0.1)  # Wait 0.1 s, just to allow seeing the number go by.

    # List of nodes which were already generated but not yet explored.
    nodesToExplore = deque()

    # Add the root node to the nodes that were already generated, but not yet
    # explored. This will be the first to expanded.
    nodesToExplore.append(root)

    # Keep expanding nodes until we found a solution (a path from start
    # position to the goal position), or until there are no more nodes to
    # explore.
    while(len(nodesToExplore) > 0):
        # Get the first node on the list of nodes to be explored (the node is
        # also removed from the list of nodes to be explored)
        node = nodesToExplore.popleft()

        # Check if the current node is the solution, that is, if its position
        # corresponds to the goal position. If so, the search ends now.
        if((node.map_position_.x == goal_position.x) and
           (node.map_position_.y == goal_position.y)):
            # We found the solution, leave...
            solutionFound = True
            break

        # Expand node by generating all its children, stored in the newNodes
        # variable.
        newNodes = node.expand()

        # Add the new nodes to the list of nodes that were already generated
        # but not yet explored.
        if(search_type == SearchMethods.DEPTH_FIRST):
            ###################################################################
            # Place code here to update nodesToExplore for Depth-first search
            ###################################################################
            pass  # REPLACE ME

            ###################################################################
        elif(search_type == SearchMethods.BREADTH_FIRST):
            ###################################################################
            # Place code here to update nodesToExplore for Breadth-first search
            ###################################################################
            pass  # REPLACE ME

            ###################################################################
        elif(search_type == SearchMethods.A_STAR):
            # Add the nodes such that the ones with lowest total cost are in
            # the beggining.
            for new_node in newNodes:
                # Look for the node with higher total cost than this one, and
                # insert the new node before that node.
                # This could be done in a more efficient way!
                i = 0
                while i < len(nodesToExplore):
                    if(nodesToExplore[i].total_cost_ > new_node.total_cost_):
                        break
                    else:
                        i += 1
                nodesToExplore.insert(i, new_node)

        # Show map as text for debugging purposes
        graph.showGraph()
        time.sleep(0.1)  # Wait 0.1 s, just to allow seeing the values go by.

    return solutionFound


if __name__ == '__main__':
    # Start and goal positions of the robot
    start_position = MapPoint(6, 1)  # Column (x) 6 and row (y) 1
    goal_position = MapPoint(1, 6)  # Column (x) 1 and row (y) 6

    # Create graph with associated grid map
    map_graph = Graph('map.png', goal_position)

    # Create root node at the given position
    # map_graph - graph this node belongs to
    # None - no parent
    # 0 - no cost
    # heuristic function
    # start_position
    # None - no action needed to reach this node
    root = Node(map_graph, None, 0, heuristic, start_position, None)
    map_graph.addNode(root, True)

    # Perform the map search and, if successful, show the resulting path in the
    # terminal output as text
    if(doSearch(map_graph, root, goal_position, SearchMethods.A_STAR)):
        map_graph.showGraph()
        map_graph.showPath(goal_position)
    else:
        print('There is no solution for the specified problem!')

    # We're done, lets leave successfully, after pressing any key
    # Keep figures open until the user presses 'q' to quit. This is a blocking
    #  statement
    plt.show()
    sys.exit(0)

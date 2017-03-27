/*
Copyright (c) 2012, Hugo Costelha.
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
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "tp06.hpp"
#include "Graph.hpp"
#include "Node.hpp"

#include <iostream> // Access output operations
#include <iomanip> // Allow manipulating the output operations

int main(void)
{
  // Start and goal positions of the robot
  MapPoint start_position(1,6);
  MapPoint goal_position(6,1);

  // Create graph with associated grid map
  Graph map_graph("map.png", goal_position);

  // Create root node at given position (row, columm)
  // &map_graph - graph this node belongs to
  // 0 - no parent
  // 0 - no cost
  // MapPoint(6,11) - node a row 6, column 11
  // Node::NONE - no action needed to reach this node
  Node *root = new Node(&map_graph, 0, 0, heuristic, start_position, std::vector<int>(0));
  map_graph.addNode(root, true);

  // Perform the map search and, if successful, show the resulting path in the
  // terminal output as text
  if( doSearch(&map_graph, root, goal_position, A_STAR) == true )
  {
    map_graph.showGraph();
    map_graph.showPath(goal_position);
  }
  else
    std::cout << "There is no solution for the specified problem!"
              << std::endl;

  // We're done, lets leave successfully, after pressing any key
  cv::waitKey(0);
  return 1;
}

double heuristic(MapPoint current_position, MapPoint goal_position)
{
  // Use Euclidean distance
  return sqrt(pow(goal_position.column-current_position.column, 2) +
              pow(goal_position.row-current_position.row, 2));
}

bool doSearch(Graph* graph, Node *root, MapPoint goal_position,
              SearchMethods search_type)
{
  //  This variable will be true if we find a solution, i.e., a path from the
  // start position to the goal
  bool solutionFound = false;

  // Output debuig line
  std::cout << " ----> Performing depth-first search in a grid-based map:\n\n";

  // Show initial map as text in the terminal
  graph->showGraph();
  cv::waitKey(100); // Wait 100ms, just to allow seeing the number go by.

  // List of nodes which were already generated but not yet explored.
  std::list<Node*> nodesToExplore;
  // Iterator which allows accessing the previous list
  std::list<Node*>::iterator listIt;

  // Add the root node to the nodes that were already generated, but not yet
  //explored. This will be the first to expanded.
  nodesToExplore.push_front(root);

  // Keep expanding nodes until we found a solution (a path from start position
  //to the goal position), or until there are no more nodes to explore.
  while(nodesToExplore.empty() == false)
  {
    // Get first node on the list of nodes to be explored
    listIt = nodesToExplore.begin();
    Node* node = *listIt;

    // Remove the node from the list, since we are already exploring it
    nodesToExplore.erase(listIt);

    // Check if the current node is the solution, that is, if its position
    // corresponds to the goal position. If so, the search ends now.
    if( (node->map_position_.row == goal_position.row ) &&
        (node->map_position_.column == goal_position.column ) )
    {
      // We found the solution, leave...
      solutionFound = true;
      break;
    }

    // Expand node by generating all its children, stored in the newNodes
    //variable.
    std::list<Node*>* newNodes = node->expand();


    // Add the new nodes to the list of nodes that were already generated but
    //not yet explored. The list of new nodes starts in newNodes->begin() and
    // ends on newNodes->end().
    switch(search_type)
    {
      case DEPTH_FIRST:
        //////////////////////////////////////////////////////////////////////
        // Insert code here to update nodesToExplore for Depth-first search
        //////////////////////////////////////////////////////////////////////
         
         
         
        //////////////////////////////////////////////////////////////////////
        break;
      
      case BREADTH_FIRST:
        //////////////////////////////////////////////////////////////////////
        // Insert code here to update nodesToExplore for Breadth-first search
        //////////////////////////////////////////////////////////////////////
         
         
         
        //////////////////////////////////////////////////////////////////////
        break;

      case A_STAR:
        // Add the nodes such that the ones with lowest total cost are in the
        //beggining.
        std::list<Node*>::iterator newIt;
        for( newIt = newNodes->begin(); newIt != newNodes->end(); newIt++ )
        {
          // Look for the node with heuristic higher than this one, and insert the
          //new node before that node.
          // This could be done in a more efficient way!
          std::list<Node*>::iterator it;
          for( it = nodesToExplore.begin(); it != nodesToExplore.end(); it++ )
            if( (*it)->total_cost_ > (*newIt)->total_cost_ )
              break;
          nodesToExplore.insert(it, *newIt);

        }
        break;
    }

    // Delete newNodes, as we no longer needed it
    delete(newNodes);

    // Show map as text for debugging purposes
    graph->showGraph();

    cv::waitKey(100); // Wait 100ms, just to allow seeing the values go by.
    if( cv::waitKey(100) == 27 )
      break;
  }

  return solutionFound;
}


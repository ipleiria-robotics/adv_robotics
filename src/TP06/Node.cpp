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

#include "Node.hpp"
#include "Graph.hpp"

#include <sstream>
#include <iostream>

MapPoint::MapPoint(int prow, int pcolumn):
    row(prow),
    column(pcolumn)
{
  // Store "(row,column)"
  std::stringstream tmpstr;
  tmpstr << "(" << prow << "," << pcolumn << ")";
  label = tmpstr.str();
}


Node::Node(Graph* graph, Node* parent, double cost,
           double (*h)(MapPoint,MapPoint), MapPoint map_position,
           std::vector<int> action):
  map_position_(map_position),
  cost_(cost),
  total_cost_(0),
  parent_(parent),
  graph_(graph),
  h_(0),
  hf_(h),
  action_(action),
  children_(),
  unborn_children_()
{
  // Fill list of available actions
  actions_.resize(4); // We have 4 possible actions:
  // Down
  actions_[0].resize(2); // Each action has a column and row displacement
  actions_[0][0] = 0; // Stay in the same column
  actions_[0][1] = 1; // Move one row down
  // Right
  actions_[1].resize(2);
  actions_[1][0] = 1; // Move right by one column
  actions_[1][1] = 0; // Stay in the same row
  // Left
  actions_[2].resize(2);
  actions_[2][0] = -1; // Move left by one column
  actions_[2][1] = 0; // Stay in the same row
  // Up
  actions_[3].resize(2);
  actions_[3][0] = 0; // Stay in the same column
  actions_[3][1] = -1; // Move up one row

  // Add all actions as unborn children
  for( uint i = 0; i < actions_.size(); i++ )
    unborn_children_.push_back(actions_[i]);

  // Compute heuristic value
  h_ = hf_(map_position, graph->goal_position_);
  total_cost_ = cost + h_;
}

Node::~Node()
{

}

std::list<Node*>* Node::expand(void)
{
  std::list<Node*> *addedNodes = new std::list<Node*>;

  // While there are unborn children, perform their delivery
  while( unborn_children_.empty() == false )
  {
    // Get first unborn child in the list
    std::list< std::vector<int> >::iterator listIt = unborn_children_.begin();

    // Move on the map
    int row = this->map_position_.row + (*listIt)[1];
    int column = this->map_position_.column + (*listIt)[0];

    // Check if we are outside the map limits
    if( (row < 0) || (column < 0) ||
        (row >= graph_->map_.size().height) ||
        (column >= graph_->map_.size().width) )
    {
      // Remove child from unborn children list
      unborn_children_.erase(listIt);
      continue;
    }

    // If the obtained map position is occupied, then we skip this position
    if( (int)graph_->map_.at<uchar>(row, column) < MAP_FREE_THRESHOLD )
    {
      // Remove child from unborn children list
      unborn_children_.erase(listIt);
      continue;
    }

    MapPoint map_point(row,column);
    double child_cost =  cost_ + 1;

    // else, check if the new position was already processed
    if( graph_->isNodeInGraph(map_point.label) )
    {
      Node *node = graph_->getNode(map_point.label);
      // If the new cost to the node is better, change the node cost and parent
      if( child_cost < node->cost_ )
      {
        // Remove the node from the graph. We will readd it with the new cost.
        graph_->removeNode(map_point.label);
      } else
      {
        // Remove child from unborn children list
        unborn_children_.erase(listIt);
        continue;
      }
    }

    // If we reached this far, then we have a new node
    Node *child = new Node(graph_, this, child_cost, hf_, map_point, *listIt);
    // Add node to this node list of nodes
    children_.push_back(child);
    // Add new child to the list of added children
    addedNodes->push_back(child);
    // Add new child to the list of nodes in the graph
    graph_->addNode(child);
    // Remove child from unborn children list
    unborn_children_.erase(listIt);

  }

  return addedNodes;
}


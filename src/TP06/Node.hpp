/*
Copyright (c) 2013, Hugo Costelha.
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

#ifndef _NODE_
#define _NODE_


#include <vector>
#include <list>
#include <string>

#define MAP_FREE_THRESHOLD 127 // Values above this one correspond to free cells

// This is just so that we can use *Graph type variables in class Node
class Graph;

// Integer-based map point
class MapPoint
{
public:
  // Constructor
  MapPoint(int prow, int pcolumn);

  int row;
  int column;

  // Label in form "(row,column)"
  std::string label;
};

// Graph-based map node
class Node
{
public:
  // Constructor
  Node(Graph* graph, Node* parent, double cost,
       double (*h)(MapPoint,MapPoint), MapPoint map_position,
       std::vector<int> action);

  // Destructor
  ~Node(void);

  // Create all this node children
  std::list<Node*>* expand(void);

  // World state associated with this node (row,columm of the map)
  MapPoint map_position_;

  // Cost of this node
  double cost_;

  // Total (estimated) Cost ( = cost_ + h_)
  double total_cost_;

  // Parent node
  Node *parent_;

private:

  // Graph this node belongs to
  Graph* graph_;

  // Estimated cost from here to the goal (heuristic)
  double h_;

  // Heuristic function
  double (*hf_)(MapPoint,MapPoint);

  // Available actions
  std::vector< std::vector<int> > actions_;

  // Action that lead to this node
  std::vector<int> action_;

  // Vector of nodes children of this node
  std::list<Node*> children_;

  // List of unborn children (actions yet to be done)
  std::list< std::vector<int> > unborn_children_;
};

#endif // _NODE_

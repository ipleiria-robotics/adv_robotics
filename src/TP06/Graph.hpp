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

#ifndef _GRAPH_
#define _GRAPH_


#include <map>

#include <opencv2/opencv.hpp>

#include "Node.hpp"

// Graph-based map
class Graph
{
public:
  // Constructor
  // Stores the map and the root node
  Graph(std::string map_filename, MapPoint goal_position);

  // Destructor
  ~Graph(void);

  // Add node to the graph
  void addNode(Node* node, bool is_root = false);

  // Remove node from graph
  void removeNode(std::string node_label);

  // Return pointer to given node
  Node* getNode(std::string node_label);

  // Check if the given node is already included
  // Returns true if it is.
  bool isNodeInGraph(std::string node_label);

  // Show a text view of the map (for debug purposes)
  void showGraph(void);

  // Show a path from the root node to the given map position
  void showPath(MapPoint goalPosition);

  // Show the values stored in the map as text
  void showMapValues();

  // OpenCV matrix with the map
  cv::Mat map_;

  // Goal position (do not write in this value)
  MapPoint goal_position_;

private:
  // Root node of the graph
  Node* root_;

  // Hash-table with all the nodes
  std::map<std::string, Node*> nodes_list_;
};

#endif // _GRAPH_

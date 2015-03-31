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

#include "Graph.hpp"
#include "Node.hpp"

#include <iostream>
#include <iomanip>

Graph::Graph(std::string map_filename, MapPoint goal_position):
  goal_position_(goal_position)
{
  // Read the map from the image file. "0" forces the read to greyscale
  map_ = cv::imread(map_filename, 0);
  if( !map_.data )
  {
    std::cerr << "Map " << map_filename << " not found!" << std::endl;
    return;
  }

  // Create a window and show the map
  cv::namedWindow("Map", 0);
  cv::imshow("Map", map_);
  // Debug the map values, by showing the map as text in the terminal
  showMapValues();
}

Graph::~Graph(void)
{
  // Delete all nodes from memory by using the hash-table
  std::map<std::string,Node*>::iterator mapIt;
  for( mapIt = nodes_list_.begin(); mapIt != nodes_list_.end(); mapIt++)
  {
    delete(mapIt->second);
    mapIt->second = 0;
  }
  nodes_list_.clear();
}

void Graph::addNode(Node* node, bool is_root)
{
  // If the node
  if( isNodeInGraph(node->map_position_.label) )
  {
    std::cerr << "Error, node (" << node->map_position_.label
              << ") was already included!" << std::endl;
  } else
  {
    // Add node
    nodes_list_[node->map_position_.label] = node;
    if( is_root )
        root_ = node;
  }

  return;
}

void Graph::removeNode(std::string node_label)
{
  // Check if the node is in the graph
  if( isNodeInGraph(node_label) == false)
  {
    std::cerr << "Error, node (" << node_label
              << ") is not part of the graph!" << std::endl;
  } else
  {
    // remove node
    nodes_list_.erase(node_label);
  }

  return;
}


Node* Graph::getNode(std::string node_label)
{
  if( isNodeInGraph(node_label) )
      return nodes_list_[node_label];
  else
      return 0;
}


bool Graph::isNodeInGraph(std::string node_label)
{
  // Return true if node already included
  return( nodes_list_.find(node_label) != nodes_list_.end() );
}

void Graph::showGraph(void)
{
  cv::Size mapSize = map_.size();
  for( int j = 0; j < mapSize.height; j++ )
  {
    for( int i = 0; i < mapSize.width; i++ )
    {
      MapPoint map_point(j,i);
      if( isNodeInGraph(map_point.label) )
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2)
                  << std::setw(4) << nodes_list_[map_point.label]->total_cost_ << " ";
      else if ((int)map_.at<uchar>(j, i) < MAP_FREE_THRESHOLD)
        std::cout << "  *  ";
      else
        std::cout << "  .  ";
    }
    std::cout << "\n";
  }

  std::cout << std::endl;
}

void Graph::showPath(MapPoint goal_position)
{
  std::list<MapPoint> finalPath;
  std::list<MapPoint>::iterator listIt;
  // Get goal node
  Node* node = nodes_list_[goal_position.label];
  // Cycle through all available nodes starting from the goal to the start
  //node
  while(1)
  {
    finalPath.push_front(node->map_position_);
    // get this node parent
    node = node->parent_;
    // if this new node is our start position, i.e., it is our root, we're
    //done
    if( node == root_ )
      break;
  }

  // Map to show path
  cv::Mat map_color(map_.size().height, map_.size().width, CV_8UC3);
  cvtColor(map_, map_color, CV_GRAY2RGB);

  // Now go through the list and print all positions
  std::cout << "This is the path from "
            << root_->map_position_.label << " to "
            << goal_position.label << ":\n";
  std::cout << root_->map_position_.label;
  // Show them in the map to.
  map_color.at<cv::Vec3b>(root_->map_position_.row,
                          root_->map_position_.column) = cv::Vec3b(0,0,255);
  for( listIt = finalPath.begin(); listIt != finalPath.end(); listIt++)
  {
    std::cout << " --> " << listIt->label;
    map_color.at<cv::Vec3b>(listIt->row,listIt->column) = cv::Vec3b(0,0,255);
  }
  std::cout << std::endl;
  cv::imshow("Map", map_color);
  cv::imwrite("Map_solution.png", map_color);
}

void Graph::showMapValues()
{
  // For through each (i,j) position of the matrix and output its value.
  // By using (int)matrix->at<uchar>(j, i) we have access to the i column and
  //j row of the matrix, as integer.
  std::cout << " ----> Image values \n\n";
  for( int j=0; j < map_.size().height; j++)
  {
    for( int i=0; i < map_.size().width; i++)
      std::cout << std::setw(3) << (int)map_.at<uchar>(j, i) << " ";
    std::cout << "\n";
  }
  std::cout << std::endl;
}

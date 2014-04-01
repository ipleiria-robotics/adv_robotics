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

#ifndef _TP6_
#define _TP6_

#include <opencv2/opencv.hpp>

class Node;
class Graph;
class MapPoint;

enum SearchMethods
{
  DEPTH_FIRST = 0,
  BREADTH_FIRST,
  A_STAR
};

/**
 * Main function
 * Find a path in the map from the start spot to the goal spot
 */
int main(void);

/**
  * Compute the heuristic given the current position and the goal position.
  * Returns the heuristic value.
  */
double heuristic(MapPoint current_position, MapPoint goal_position);

/**
  * Perform search on a graph.
  * Returns true if the solution was found.
  */
bool doSearch(Graph* graph, Node *root, MapPoint goal_position,
              SearchMethods search_type);

/**
  * Debug function used to show all the matrix values for a greyscale 8 bit
  * image.
  */
void showMatValues(cv::Mat* matrix);

#endif // _TP6_

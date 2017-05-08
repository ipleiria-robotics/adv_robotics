/*
Copyright (c) 2010, Hugo Costelha, based on Brad Kratochvil, Toby Collett, Brian
Gerkey and Andrew Howard example.
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

#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <cv.h>
#include <highgui.h>

/**
  * Normalize an angle between -PI and PI
  */
inline double normalize(double angle)
{
  return atan2(sin(angle), cos(angle));
}


/**
  * Limit a given value within the given interval
  */
inline double clipValue(double value, double min, double max)
{
  if( value > max )
    return max;
  else if( value < min )
    return min;
  else return value;
}


/**
  * Draws the robot position in the image.
  * image - image where the robot is to be drawn
  * x - X coordinate of the robot [m]
  * y - Y coordinate of the robot [m]
  * theta - orientation of the robot [rad]
  * resolution - [pixels/m]
  * color - cv::Scalar(Blue,Green,Red) color to be used.
  */
void drawPostures(cv::Mat &image, double x, double y, double theta,
                  double resolution, cv::Scalar color);

/**
  * Debug function used to show all the matrix values for a 2D matrix.
  */
void showMatValues(cv::Mat* matrix);

#endif // _UTILS_HPP_

/*
Copyright (c) 2014, Hugo Costelha
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

#ifndef _LOCAL_FRAME_WORLD_FRAME_TRANSFORMATIONS_HPP_
#define _LOCAL_FRAME_WORLD_FRAME_TRANSFORMATIONS_HPP_

#include <math.h>
#include <geometry_msgs/msg/pose2_d.hpp> // Pose2D

class point_2d
{
public:
  double x;
  double y;
};

class pose_2d
{
public:
  double x;
  double y;
  double theta;
};

/**
  * Convert given angle from degrees to radians
  */
inline double deg2rad(double angle)
{
  return angle*M_PI/180.0;
}


/**
  * Convert given angle from radians to degrees
  */
inline double rad2deg(double angle)
{
  return angle*180.0/M_PI;
}


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
 * Transform a given vector in local coordinates, referenced to
 * localFrame into global coordinates.
 * @Argument baseFrame - the values of the base frame
 * @Argument vectorInLocalFrame - the vector coordinates in the basee
 * frame
 * @Argument vectorInWorldFrame - value to be returned, containing the
 * given local vector in world coordinates
 */
void local2World( const geometry_msgs::msg::Pose2D baseFrame,
                  const point_2d vectorInBaseFrame,
                  point_2d* vectorInWorldFrame );

/**
 * Transform a given vector in world coordinates, into a vector based
 * on the given baseFrame.
 * @Argument baseFrame - the value of the base frame
 * @Argument vectorInWorldFrame - the vector coordinates in the world
 * frame.
 * @Argument vectorInBaseFrame - value to be returned, containing the
 * given world vector in local coordinates
 */
void world2Local( const geometry_msgs::msg::Pose2D baseFrame,
                  const point_2d vectorInWorldFrame,
                  point_2d* vectorInBaseFrame );


/**
 * Transform a given pose in local coordinates, referenced to
 * localFrame into global coordinates.
 * @Argument baseFrame - the values of the base frame
 * @Argument poseInLocalFrame - the vector coordinates in the base
 * frame
 * @Argument poseInWorldFrame - value to be returned, containing the
 * given local pose in world coordinates
 */
void local2World( const geometry_msgs::msg::Pose2D baseFrame,
                  const geometry_msgs::msg::Pose2D vectorInBaseFrame,
                  geometry_msgs::msg::Pose2D* vectorInWorldFrame );

/**
 * Transform a given pose in world coordinates, into a pose based
 * on the given baseFrame.
 * @Argument baseFrame - the value of the base frame
 * @Argument poseInWorldFrame - the pose coordinates in the world
 * frame.
 * @Argument poseInBaseFrame - value to be returned, containing the
 * given world pose in local coordinates
 */
void world2Local( const geometry_msgs::msg::Pose2D baseFrame,
                  const geometry_msgs::msg::Pose2D vectorInWorldFrame,
                  geometry_msgs::msg::Pose2D* vectorInBaseFrame );


#endif // _LOCAL_FRAME_WORLD_FRAME_TRANSFORMATIONS_HPP_

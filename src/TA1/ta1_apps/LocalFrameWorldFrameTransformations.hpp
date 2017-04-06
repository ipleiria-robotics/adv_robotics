/*
Copyright (c) 2016, Hugo Costelha
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
#include <geometry_msgs/Pose2D.h>

typedef struct point_2d
{
  float x;
  float y;
} point_2d_t;


/**
 * Transform a given vector in local coordinates, referenced to
 * localFrame into global coordinates.
 * @Argument baseFrame - the values of the base frame
 * @Argument vectorInLocalFrame - the vector coordinates in the basee
 * frame
 * @Argument vectorInWorldFrame - value to be returned, containing the
 * given local vector in world coordinates
 */
void local2World( const geometry_msgs::Pose2D baseFrame,
                  const point_2d_t vectorInBaseFrame,
                  point_2d_t* vectorInWorldFrame );

/**
 * Transform a given vector in world coordinates, into a vector based
 * on the given baseFrame.
 * @Argument baseFrame - the value of the base frame
 * @Argument vectorInWorldFrame - the vector coordinates in the world
 * frame.
 * @Argument vectorInBaseFrame - value to be returned, containing the
 * given world vector in local coordinates
 */
void world2Local( const geometry_msgs::Pose2D baseFrame,
                  const point_2d_t vectorInWorldFrame,
                  point_2d_t* vectorInBaseFrame );


#endif // _LOCAL_FRAME_WORLD_FRAME_TRANSFORMATIONS_HPP_

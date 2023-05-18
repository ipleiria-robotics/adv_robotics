/*
Copyright (c) 2019, Hugo Costelha
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

#include "ar_cpp_utils/LocalFrameWorldFrameTransformations.hpp"

///
/// 2D position transformations
///

void local2World( geometry_msgs::msg::Pose2D baseFrame,
                  point_2d vectorInBaseFrame,
                  point_2d* vectorInWorldFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    double x, y;

    // Perform the transformation
    x = baseFrame.x + vectorInBaseFrame.x*cosTheta
                     - vectorInBaseFrame.y*sinTheta;
    y = baseFrame.y + vectorInBaseFrame.x*sinTheta
                     + vectorInBaseFrame.y*cosTheta;

    // Store the result
    vectorInWorldFrame->x = x;
    vectorInWorldFrame->y = y;
}

void world2Local( geometry_msgs::msg::Pose2D baseFrame,
                  point_2d vectorInWorldFrame,
                  point_2d* vectorInBaseFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    double x, y;

    // Do the transformation
    x =  (vectorInWorldFrame.x - baseFrame.x)*cosTheta +
         (vectorInWorldFrame.y - baseFrame.y)*sinTheta;
    y =  -(vectorInWorldFrame.x - baseFrame.x)*sinTheta +
          (vectorInWorldFrame.y - baseFrame.y)*cosTheta;

    // Store the result
    vectorInBaseFrame->x = x;
    vectorInBaseFrame->y = y;
}


///
/// 2D + orientation (pose transformation)
///

void local2World( geometry_msgs::msg::Pose2D baseFrame,
                  geometry_msgs::msg::Pose2D poseInBaseFrame,
                  geometry_msgs::msg::Pose2D* poseInWorldFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    double x, y, theta;

    // Perform the transformation
    x = baseFrame.x + poseInBaseFrame.x*cosTheta
                         - poseInBaseFrame.y*sinTheta;
    y = baseFrame.y + poseInBaseFrame.x*sinTheta
                         + poseInBaseFrame.y*cosTheta;
    theta = baseFrame.theta + poseInBaseFrame.theta;

    // Store the result
    poseInWorldFrame->x = x;
    poseInWorldFrame->y = y;
    poseInWorldFrame->theta = theta;
}

void world2Local( geometry_msgs::msg::Pose2D baseFrame,
                  geometry_msgs::msg::Pose2D poseInWorldFrame,
                  geometry_msgs::msg::Pose2D* poseInBaseFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    double x, y, theta;

    // Do the transformation
    x =  (poseInWorldFrame.x - baseFrame.x)*cosTheta +
         (poseInWorldFrame.y - baseFrame.y)*sinTheta;
    y =  -(poseInWorldFrame.x - baseFrame.x)*sinTheta +
          (poseInWorldFrame.y - baseFrame.y)*cosTheta;
    theta = poseInWorldFrame.theta - baseFrame.theta;

    // Store the computed values
    poseInBaseFrame->x = x;
    poseInBaseFrame->y = y;
    poseInBaseFrame->theta = theta;
}

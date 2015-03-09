/**
 * @Filename LocalFrameWorldFrameTransformations.cpp
 * @Description LocalFrameWorldFrame class implementation.
 * @Status Finished
 * @Maintainer Hugo Costelha
 */

#include "LocalFrameWorldFrameTransformations.hpp"

void local2World( geometry_msgs::Pose2D baseFrame,
                  point_2d_t vectorInBaseFrame,
                  point_2d_t* vectorInWorldFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    float x, y;

    // Perform the transformation
    x = baseFrame.x + vectorInBaseFrame.x*cosTheta
                     - vectorInBaseFrame.y*sinTheta;
    y = baseFrame.y + vectorInBaseFrame.x*sinTheta
                     + vectorInBaseFrame.y*cosTheta;

    // Store the result
    vectorInWorldFrame->x = x;
    vectorInWorldFrame->y = y;
}

void world2Local( geometry_msgs::Pose2D baseFrame,
                  point_2d_t vectorInWorldFrame,
                  point_2d_t* vectorInBaseFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    float x, y;

    // Do the transformation
    x =  (vectorInWorldFrame.x - baseFrame.x)*cosTheta +
         (vectorInWorldFrame.y - baseFrame.y)*sinTheta;
    y =  -(vectorInWorldFrame.x - baseFrame.x)*sinTheta +
          (vectorInWorldFrame.y - baseFrame.y)*cosTheta;

    // Store the result
    vectorInBaseFrame->x = x;
    vectorInBaseFrame->y = y;
}

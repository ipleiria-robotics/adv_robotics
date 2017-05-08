/**
 * @Filename LocalFrameWorldFrameTransformations.cpp
 * @Description LocalFrameWorldFrame class implementation.
 * @Status Finished
 * @Version $Id: LocalFrameWorldFrameTransformations.cpp 2447 2007-06-20 13:39:59Z hcostelha $
 * @Maintainer Hugo Costelha
 */

#include "LocalFrameWorldFrameTransformations.hpp"

///
/// 2D position transformations
///

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


///
/// 2D + orientation (pose transformation)
///

void local2World( geometry_msgs::Pose2D baseFrame,
                  geometry_msgs::Pose2D poseInBaseFrame,
                  geometry_msgs::Pose2D* poseInWorldFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    float x, y, theta;

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

void world2Local( geometry_msgs::Pose2D baseFrame,
                  geometry_msgs::Pose2D poseInWorldFrame,
                  geometry_msgs::Pose2D* poseInBaseFrame )
{
    double sinTheta = sin(baseFrame.theta);
    double cosTheta = cos(baseFrame.theta);
    float x, y, theta;

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


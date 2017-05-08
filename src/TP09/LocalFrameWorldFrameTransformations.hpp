/**
 * @Filename LocalFrameWorldFrameTransformations.hpp
 * @Description LocalFrameWorldFrameTransformations class header
 * @Status Finished
 * @Version $Id: LocalFrameWorldFrameTransformations.hpp 2447 2007-06-20 13:39:59Z hcostelha $
 * @Maintainer Hugo Costelha
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


/**
 * Transform a given pose in local coordinates, referenced to
 * localFrame into global coordinates.
 * @Argument baseFrame - the values of the base frame
 * @Argument poseInLocalFrame - the vector coordinates in the base
 * frame
 * @Argument poseInWorldFrame - value to be returned, containing the
 * given local pose in world coordinates
 */
void local2World( const geometry_msgs::Pose2D baseFrame,
                  const geometry_msgs::Pose2D vectorInBaseFrame,
                  geometry_msgs::Pose2D* vectorInWorldFrame );

/**
 * Transform a given pose in world coordinates, into a pose based
 * on the given baseFrame.
 * @Argument baseFrame - the value of the base frame
 * @Argument poseInWorldFrame - the pose coordinates in the world
 * frame.
 * @Argument poseInBaseFrame - value to be returned, containing the
 * given world pose in local coordinates
 */
void world2Local( const geometry_msgs::Pose2D baseFrame,
                  const geometry_msgs::Pose2D vectorInWorldFrame,
                  geometry_msgs::Pose2D* vectorInBaseFrame );


#endif // _LOCAL_FRAME_WORLD_FRAME_TRANSFORMATIONS_HPP_

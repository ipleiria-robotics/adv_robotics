#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Point, Pose2D
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

from ar_py_utils.utils import rpyToQuaternion
from numpy import radians

"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    # Nav2 initialization
    nav2 = BasicNavigator(namespace='/robot_0')

    # List of targets the robot should pass by (X [m], Y [m], theta [deg])
    targets = [Pose2D(x=2.0, y=2.0, theta=45.0),  # 1
               Pose2D(x=6.0, y=7.0, theta=90.0),  # 2
               Pose2D(x=-6.5, y=-1.0, theta=-90.0),  # 3
               Pose2D(x=-1.0, y=-7.0, theta=0.0)]  # 4
    num_targets = len(targets)

    # Generate list of waypoints using ROS format from the targets list above
    waypoints = []
    for pose2d in targets:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # pose.header.stamp = To be set latter
        pose.pose.position = Point(x=pose2d.x, y=pose2d.y, z=0.)
        pose.pose.orientation = rpyToQuaternion(0., 0., radians(pose2d.theta))
        waypoints.append(pose)

    # Go through all waypoints indefenitely
    j = 0
    num_fails = 0
    while True:
        target = waypoints[j]

        # Compute path to target
        # use_start=False means that the robot will use its current pose as the
        # the start pose for the path planning. If False, the robot will
        # use the pose provided in the start argument.
        path = nav2.getPath(start=PoseStamped(),  # Not used
                            goal=target,
                            planner_id="GridBased",
                            use_start=False)
        # If no path exists, try the next target
        if path is None:
            print(f'No path found for target {j}, trying next target!')
            j = (j+1) % num_targets
            continue

        # Otherwise, smooth the path
        smoothed_path = nav2.smoothPath(path)

        # Go through the path to the next waypoint
        nav2.followPath(smoothed_path, controller_id='FollowPath',
                        goal_checker_id='general_goal_checker')

        # While the robot is moving, we can do whatever we want...
        i = 0
        last_distance_to_goal = None
        while not nav2.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = nav2.getFeedback()
            if feedback:
                last_distance_to_goal = feedback.distance_to_goal
                if i % 5 == 0:
                    print(f'Distance to target: {feedback.distance_to_goal} m')
                    i = 0

            # Example on how to cancel the task (not beig used for now)
            if False:
                nav2.cancelTask()

        # Do something depending on the return code
        result = nav2.getResult()
        if result == TaskResult.SUCCEEDED:
            # Proceed to the next target
            j = (j+1) % num_targets
            print(f'Goal succeeded, going for target {j}!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            # Retry the same target
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            # If it failed, but the distance to the goal is small, proceed to
            # the next target
            if (last_distance_to_goal is not None) and \
               (last_distance_to_goal < 0.05):
                j = (j+1) % num_targets
                num_fails = 0
            elif num_fails >= 2:
                # If we failed too much, Move to the next target
                j = (j+1) % num_targets
                num_fails = 0
            else:
                num_fails += 1
        else:
            print('Goal has an invalid return status!')

    # Shutdown nav2 nodes
    nav2.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

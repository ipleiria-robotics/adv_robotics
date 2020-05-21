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


This applications controls the simulation for the second laboratory work, part
of the Advanced Robotics class in the Electrical Engineering Masters at the 
Polytechnic of Leiria, Portugal.
It simulates the timings and responses regarding picking up and leaving the parts,
taking into account the robot position. It also simulates the robot charging and
discharging.
*/

// ROS API
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // Odometry
#include <geometry_msgs/Pose2D.h> // Pose2D
#include <control_msgs/JointControllerState.h>
#include <tf/tf.h> // Geometry transformations

// Our files
#include "LocalFrameWorldFrameTransformations.hpp"
#include "tw11/StartCharging.h"

// Other includes
#include <mutex>
#include <atomic>

#define RAD2DEG(x) x/3.1415927*180

// Battery simulation related variables
static float battery_level(80.0);  // Start with 80% level
static std::atomic_bool is_charging(false);  // True if charging, false otherwise
static ros::Publisher batttery_level_pub;
static ros::Publisher vel_pub;
// For now we assume a simple model, with a constant battery discharge and
// charge 
#define BATT_DISCHARGE_DELTA 0.055555  // Per 0.1 sec 
#define BATT_CHARGE_DELTA 1  // Per iteration, ~15xDISCHARGE

// Robot position and velocity, used to control the charging too
static geometry_msgs::Pose2D global_robot_pose, global_robot_speed;
std::mutex pose_vel_mutex;

// Maximum error to consider in desired orientation
#define MAX_X_OFFSET 0.20 // [m]
#define MAX_Y_OFFSET 0.20 // [m]
#define MAX_ANG_OFFSET 0.17 // [rad] (10 deg)
// Maxumum speed to grab/drop the parts
#define MAX_LIN_SPEED 0.01 // [m/s]
#define MAX_ANG_SPEED 0.017 // [rad/s] (1º/s)
// Charging pose / location
static pose_2d charging_pose = {0.0, -2.0, -M_PI/2};


/**
 * Service callback for charging requests
 */
bool startChargeRequest(tw11::StartCharging::Request &req,
                        tw11::StartCharging::Response &res)
{
    geometry_msgs::Pose2D robot_pose, robot_speed;
    // Store a local copy of the robot pose and velocity
    pose_vel_mutex.lock();
    robot_pose = global_robot_pose;
    robot_speed = global_robot_speed;
    pose_vel_mutex.unlock();

    // Are we cancelling the charge?
    if(req.charge == false)
    {
        is_charging = false;
        res.charging = false;
        return true;
    }

    // If the robot is not stopped, or too far, return false response
    if((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED ||
       (abs(robot_speed.theta) > MAX_ANG_SPEED)) ||
       (abs(robot_pose.x - charging_pose.x) > MAX_X_OFFSET) ||
       (abs(robot_pose.y - charging_pose.y) > MAX_Y_OFFSET) ||
       (abs(robot_pose.theta - charging_pose.theta) > MAX_ANG_OFFSET))
    {
        if((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED) ||
           (abs(robot_speed.theta) > MAX_ANG_SPEED))
           std::cout << "Robot is moving, unable to start charging..." << std::endl;
        if((abs(robot_pose.x - charging_pose.x) > MAX_X_OFFSET) ||
           (abs(robot_pose.y - charging_pose.y) > MAX_Y_OFFSET) ||
           (abs(robot_pose.theta - charging_pose.theta) > MAX_ANG_OFFSET))
           std::cout << "Robot not in charging pose, unable to start charging..." << std::endl;
        is_charging = false;
        res.charging = false;
    } else
    {
        is_charging = true;
        res.charging = true;
    }
    return true;
}

/**
 * Periodic callback to control charging/discharging
 */
void batteryCallback(const rosgraph_msgs::Clock& clk)
{
    if(is_charging)
    {
        // Check if our position is still good, and that we are stopped
        geometry_msgs::Pose2D robot_pose, robot_speed;
        pose_vel_mutex.lock();
        robot_pose = global_robot_pose;
        robot_speed = global_robot_speed;
        pose_vel_mutex.unlock();
        if( (abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED ||
            (abs(robot_speed.theta) > MAX_ANG_SPEED)) ||
            (abs(robot_pose.x - charging_pose.x) > MAX_X_OFFSET) ||
            (abs(robot_pose.y - charging_pose.y) > MAX_Y_OFFSET) ||
            (abs(robot_pose.theta - charging_pose.theta) > MAX_ANG_OFFSET))
        {
            if((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED) ||
               (abs(robot_speed.theta) > MAX_ANG_SPEED))
               std::cout << "Robot is moving, stop charging..." << std::endl;
            if((abs(robot_pose.x - charging_pose.x) > MAX_X_OFFSET) ||
               (abs(robot_pose.y - charging_pose.y) > MAX_Y_OFFSET) ||
               (abs(robot_pose.theta - charging_pose.theta) > MAX_ANG_OFFSET))
                std::cout << "Robot not in charging pose, stop charging..." << std::endl;
            is_charging = false;
        } else
        {
            if(battery_level < 100 - BATT_CHARGE_DELTA)
                battery_level += BATT_CHARGE_DELTA;
            else
                battery_level = 100;
        }
    } else
    {
        if(battery_level > BATT_DISCHARGE_DELTA)
            battery_level -= BATT_DISCHARGE_DELTA;
        else
            battery_level = 0;
    }

    // Pubish the battery level
    std_msgs::UInt8 batlevel;
    batlevel.data = battery_level;
    batttery_level_pub.publish(batlevel);

    // If the battery level is too low, the robot should not move
    if(battery_level < 5)
    {
        geometry_msgs::Twist vel_cmd;
        vel_cmd.angular.z = 0.0;
        vel_cmd.linear.x = 0.0;
        vel_pub.publish(vel_cmd);
    }
}


/**
  * Receive and store current robot real pose
  */
void realPoseCallback(const nav_msgs::Odometry& msg)
{
    // Store real, error-free pose values given by the simulator
    static geometry_msgs::Pose2D robot_pose, robot_speed;
    point_2d part_pos, part_local_pos;
    robot_pose.x = msg.pose.pose.position.x;
    robot_pose.y = msg.pose.pose.position.y;
    robot_pose.theta = tf::getYaw(msg.pose.pose.orientation);
    robot_speed.x = msg.twist.twist.linear.x;
    robot_speed.y = msg.twist.twist.linear.y;
    robot_speed.theta = msg.twist.twist.angular.z;

    // Update global values
    pose_vel_mutex.lock();
    global_robot_pose = robot_pose;
    global_robot_speed = robot_speed;
    pose_vel_mutex.unlock();
}


/**
  * Main function for controlling the simulation.
  */
int main(int argc, char** argv)
{
    // Init ROS
    ros::init(argc, argv, "battery_manager");

    /// ROS variables/objects
    ros::NodeHandle nh; // Node handle
    std::string robot_name = "/robot_0/";

    // Publisher for the battery level
    batttery_level_pub = nh.advertise<std_msgs::UInt8>("battery/level", 1, true);
    // Publisher for the velocity commands    
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    /// Setup subscribers
    // Real, error-free robot pose (for debug purposes only)
    ros::Subscriber sub_real_pose = nh.subscribe(robot_name + "base_pose_ground_truth", 1, realPoseCallback);
    // Baterry manager (subscriber to the clock)
    ros::Subscriber sub_bat = nh.subscribe("/clock", 1, batteryCallback);

    // Advertise the batter charging service
    ros::ServiceServer batt_service = nh.advertiseService("battery/charge", startChargeRequest);

    // Infinite loop
    ros::spin();

    return 0;
}

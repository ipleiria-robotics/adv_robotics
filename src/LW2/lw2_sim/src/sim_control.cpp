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
#include "lw2_sim/StartCharging.h"

// Other includes
#include <mutex>
#include <atomic>

#define RAD2DEG(x) x/3.1415927*180

// Forklift related variables
static control_msgs::JointControllerState forklift_status;
static double forklift_desired_pos = 0.0; // Received forklift position command
#define FORKLIFT_SPEED 0.03  // [m/s]
#define FORKLIFT_MARGIN 0.005  // [%]m]
// Forklift information
#define FORKLIFT_DOWN 0.0   // Down position
#define FORKLIFT_UP   0.07  // Up position
static bool forklift_up = false,
            forklift_down = false;

// Battery simulation related variables
static float battery_level(80.0);  // Start with 80% level
static std::atomic_bool is_charging(false);  // True if charging, false otherwise
static ros::Publisher batttery_level_pub;
static ros::Publisher vel_pub;
// For now we assume a simple model, with a constant battery discharge and
// charge 
#define BATT_DISCHARGE_DELTA 0.11111  // Per 0.1 sec 
#define BATT_CHARGE_DELTA 1  // Per iteration, ~15xDISCHARGE

// Robot position and velocity, used to control the charging too
static geometry_msgs::Pose2D global_robot_pose, global_robot_speed;
std::mutex pose_vel_mutex;

// Maximum error to consider in desired orientation
#define MAX_X_OFFSET 0.35 // [m]
#define MAX_DIST_ERROR 0.05 // [m]
#define MAX_ANG_ERROR 0.07 // [rad] (4º)
// Maxumum speed to grab/drop the parts
#define MAX_LIN_SPEED 0.01 // [m/s]
#define MAX_ANG_SPEED 0.017 // [rad/s] (1º/s)

// Store the positions of the parts approach pose
#define NUM_PARTS 5 // Number of available parts/input/outputs
#define NUM_PMAN 8 // Number of processing machines
#define DA 0 // Distance to part in [m]
static pose_2d input_parts[NUM_PARTS] =
    {{2.60, -2.48+DA, -M_PI_2}, // 1
     {2.23, -2.48+DA, -M_PI_2}, // 2
     {1.86, -2.48+DA, -M_PI_2}, // 3
     {1.49, -2.48+DA, -M_PI_2}, // 4
     {1.12, -2.48+DA, -M_PI_2}};// 5
static pose_2d process_parts[NUM_PMAN] =
    {{1.10-DA, 0.18, 0.0}, // R1 1
     {1.46+DA, 0.18, M_PI}, // R2 2
     {1.10-DA, -0.18, 0.0}, // R3 3
     {1.46+DA, -0.18, M_PI}, // R4 4
     {-1.12+DA, 0.18, M_PI}, // L1 5
     {-1.48-DA, 0.18, 0.0}, // L2 6
     {-1.12+DA, -0.18, M_PI}, // L3 7
     {-1.48-DA, -0.18, 0.0}};// L4 8
static pose_2d output_parts[NUM_PARTS] =
    {{-2.62, 2.50-DA, -M_PI_2}, // 1
     {-2.25, 2.50-DA, M_PI_2}, // 2
     {-1.88, 2.50-DA, M_PI_2}, // 3
     {-1.51, 2.50-DA, M_PI_2}, // 4
     {-1.14, 2.50-DA, M_PI_2}};// 5
static pose_2d charging_pose = {0.0, -2.0, -M_PI};

enum part_state_t {
  PART_UNPROCESSED = 0, // Part was not yet processed
  PART_IN_PROCESS, // Part is being processed
  PART_PROCESSED, // Part was already processed
  PART_DELIVERED, // Part was already processed and delivered at its destination
  // From this point onwards, there are internal states only, not to be sent
  PART_UNPROCESSED_PICKING, // Unprocessed part is being picked up
  PART_UNPROCESSED_PICKED_UP, // Unprocessed part was picked up
  PART_IN_PROCESS_DROPING, // Part being droped intro processing machine
  PART_PROCESSED_PICKING, // Processed part is being picked up
  PART_PROCESSED_PICKED_UP, // Processed part was picked up
  PART_PROCESSED_DELIVERED_DROPING // Part is being droped for delivery
};

// Parts status
static ros::Publisher parts_status_pub;
static std_msgs::UInt8MultiArray parts_status_msg;
static part_state_t parts_state[NUM_PARTS] = {PART_UNPROCESSED,
                                              PART_UNPROCESSED,
                                              PART_UNPROCESSED,
                                              PART_UNPROCESSED,
                                              PART_UNPROCESSED};
// Parts location
// 0:4 - Input wharehouse n
// 5 - On the move
// 6:13 - In processing machine n-5
// 14:18 - Input wharehouse n-13
static uint parts_location[NUM_PARTS] = {0, 1, 2, 3, 4};
// Parts processing timestamp
static ros::Time parts_timestamps[NUM_PARTS];


/**
 * Service callback for charging requests
 */
bool startChargeRequest(lw2_sim::StartCharging::Request &req,
                        lw2_sim::StartCharging::Response &res)
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
       (abs(robot_pose.x - charging_pose.x) > 0.20) ||
       (abs(robot_pose.y - charging_pose.y) > 0.20) )
    {
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
            (abs(robot_pose.x - charging_pose.x) > 0.20) ||
            (abs(robot_pose.y - charging_pose.y) > 0.20) )
        {
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
  * Receive and store desired forklift value
  */
void forkliftCallback(const std_msgs::Float64& msg)
{
    // Process forklift request (store desired position)
    forklift_desired_pos = msg.data;
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

    // If the robot is moving, do nothing
    // Assume we cannot interact with the parts in motion
    bool excess_speed = false;
    if((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED ||
       (abs(robot_speed.theta) > MAX_ANG_SPEED)))
        excess_speed = true;

    // Change each part information according to the forklift position
    //and robot pose
    bool part_ext_status_changed = false;
    for(uint part_num=0; part_num < NUM_PARTS; part_num++)
    {
        double dangle;
        switch(parts_state[part_num])
        {
            // Part is still unprocessed
            case PART_UNPROCESSED:
                // We cannot start picking up a part if we are moving
                if(excess_speed)
                    continue;
                // Get this part position
                part_pos.x = input_parts[part_num].x;
                part_pos.y = input_parts[part_num].y;
                world2Local(robot_pose, part_pos, &part_local_pos);
                // Get the angle error
                dangle = atan2(part_local_pos.y, part_local_pos.x);
                // Check if the robot is enough near the approach pose and the forklift is down
                if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET) &&
                   (abs(dangle) < MAX_ANG_ERROR) && forklift_down)
                {
                    // The robot is well placed to pick the part
                    parts_state[part_num] = PART_UNPROCESSED_PICKING;
                    std::cout << "Part " << part_num+1 << " is being picked up from input station..." << std::endl;
                }
                break;

            // Part is being picked up
            case PART_UNPROCESSED_PICKING:
                // We cannot start picking up a part if we are moving
                if(excess_speed)
                    continue;
                // Get the distance error
                part_pos.x = input_parts[part_num].x;
                part_pos.y = input_parts[part_num].y;
                world2Local(robot_pose, part_pos, &part_local_pos);
                // Get the angle error
                dangle = atan2(part_local_pos.y, part_local_pos.x);
                // Check if the robot is enough near the approach pose and the forklift is up
                if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET) &&
                   (abs(dangle) < MAX_ANG_ERROR) && forklift_up)
                {
                    // The robot picked the part and now can tansport it
                    parts_state[part_num] = PART_UNPROCESSED_PICKED_UP;
                    std::cout << "Part " << part_num+1 << " was picked up from input station..." << std::endl;
                }
                break;

            // Part is being transported
            case PART_UNPROCESSED_PICKED_UP:
                // We cannot start picking up a part if we are moving
                if(excess_speed)
                    continue;
                // Check if the robot is able to drop the part in a processing machine
                for(uint pb_num=0; pb_num < NUM_PMAN; pb_num++)
                {
                    // Get this part position
                    part_pos.x = process_parts[pb_num].x;
                    part_pos.y = process_parts[pb_num].y;
                    world2Local(robot_pose, part_pos, &part_local_pos);
                    if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET))
                    {
                        // Check the angle and the forklift position
                        dangle = atan2(part_local_pos.y, part_local_pos.x);
                        if((abs(dangle) < MAX_ANG_ERROR) && (forklift_up == false))
                        {
                            // The robot is ready to start placing the part
                            parts_state[part_num] = PART_IN_PROCESS_DROPING;
                            parts_location[part_num] = pb_num;
                            std::cout << "Part " << part_num+1 << " is being placed for processing..." << std::endl;
                            break; // No need to check for others
                        }
                    }
                }
                break;

            // Part is being dropped in a processing station
            case PART_IN_PROCESS_DROPING:
                // We cannot start picking up a part if we are moving
                if(excess_speed)
                    continue;
                // If the robot position is still fine and the forklift is down
                //the part is droped into the processing machine
                // Get this part position
                part_pos.x = process_parts[parts_location[part_num]].x;
                part_pos.y = process_parts[parts_location[part_num]].y;
                world2Local(robot_pose, part_pos, &part_local_pos);

                if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET))
                {
                    dangle = atan2(part_local_pos.y, part_local_pos.x);
                    if((abs(dangle) < MAX_ANG_ERROR) && forklift_down)
                    {
                        // The robot has placed the part, it is now processing
                        parts_state[part_num] = PART_IN_PROCESS;
                        part_ext_status_changed = true;
                        parts_status_msg.data[part_num] = PART_IN_PROCESS;
                        parts_timestamps[part_num] = msg.header.stamp+ros::Duration(20.0+(rand()*1.0/RAND_MAX)*30);
                        std::cout << "Part " << part_num+1 << " is being processed..." << std::endl;
                        break; // No need to check for others
                    }
                }
                break;

            // Part is being processed in a processing station
            case PART_IN_PROCESS:
                // If enough time passed by, part is done processing
                if(msg.header.stamp >= parts_timestamps[part_num])
                {
                    parts_state[part_num] = PART_PROCESSED;
                    part_ext_status_changed = true;
                    parts_status_msg.data[part_num] = PART_PROCESSED;
                    std::cout << "Part " << part_num+1 << " has finished processing..." << std::endl;
                }
                break;

            // Part is done processing, we can now pick it up
            case PART_PROCESSED:
                // We cannot start picking up a part if we are moving
                if(excess_speed)
                  continue;
                // If the robot position is fine and the forklift is down
                //the part is being picked up in processing machine
                // Get this part position
                part_pos.x = process_parts[parts_location[part_num]].x;
                part_pos.y = process_parts[parts_location[part_num]].y;
                world2Local(robot_pose, part_pos, &part_local_pos);
                if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET))
                {
                    dangle = atan2(part_local_pos.y, part_local_pos.x);
                    if((abs(dangle) < MAX_ANG_ERROR) && forklift_down)
                    {
                        parts_state[part_num] = PART_PROCESSED_PICKING;
                        std::cout << "Part " << part_num+1 << " is being picked up from processing..." << std::endl;
                    }
                }
                break;

            // Part is being picked up in the processing machine
            case PART_PROCESSED_PICKING:
                // We cannot start picking up a part if we are moving
                if(excess_speed)
                    continue;
                // If the robot position is fine and the forklift is up
                //the part is picked up from the processing machine
                // Get this part position
                part_pos.x = process_parts[parts_location[part_num]].x;
                part_pos.y = process_parts[parts_location[part_num]].y;
                world2Local(robot_pose, part_pos, &part_local_pos);
                if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET))
                {
                    dangle = atan2(part_local_pos.y, part_local_pos.x);
                    if((abs(dangle) < MAX_ANG_ERROR) && forklift_up)
                    {
                        parts_state[part_num] = PART_PROCESSED_PICKED_UP;
                        std::cout << "Part " << part_num+1 << " was picked up from processing..." << std::endl;
                    }
                }
                break;

            // Part is being transported, it can be delivered
            case PART_PROCESSED_PICKED_UP:
                // We cannot start dropping down a part if we are moving
                if(excess_speed)
                    continue;
                // Check if the robot is able to drop the part in a processing machine
                for(uint pb_num=0; pb_num < NUM_PARTS; pb_num++)
                {

                    // Get this part position
                    part_pos.x = output_parts[pb_num].x;
                    part_pos.y = output_parts[pb_num].y;
                    world2Local(robot_pose, part_pos, &part_local_pos);
                    if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET))
                    {
                        // Check the angle and the forklift position
                        dangle = atan2(part_local_pos.y, part_local_pos.x);
                        if((abs(dangle) < MAX_ANG_ERROR) && forklift_down)
                        {
                            // The robot is ready to start placing the part
                            parts_state[part_num] = PART_PROCESSED_DELIVERED_DROPING;
                            parts_location[part_num] = pb_num;
                            std::cout << "Part " << part_num+1 << " is being delivered..." << std::endl;
                            break; // No need to check for others
                        }
                    }
                }
                break;

            // Part is being delivered
            case PART_PROCESSED_DELIVERED_DROPING:
                // We cannot start dropping down a part if we are moving
                if(excess_speed)
                    continue;
                // If the robot position is still fine and the forklift is down
                //the part is delivered
                // Get this part position
                part_pos.x = output_parts[parts_location[part_num]].x;
                part_pos.y = output_parts[parts_location[part_num]].y;
                world2Local(robot_pose, part_pos, &part_local_pos);
                if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET))
                {
                    dangle = atan2(part_local_pos.y, part_local_pos.x);
                    if((abs(dangle) < MAX_ANG_ERROR) && forklift_down)
                    {
                        // The robot has placed the part, it now delivered
                        parts_state[part_num] = PART_DELIVERED;
                        part_ext_status_changed = true;
                        parts_status_msg.data[part_num] = PART_DELIVERED;
                        std::cout << "Part " << part_num+1 << " has been delivered..." << std::endl;
                        break; // No need to check for others
                    }
                }
                break;

            case PART_DELIVERED:
                // Nothing to do/check
                break;

            //default:
            //    ROS_ERROR("Unknown part state!!");
        }
    }

    if(part_ext_status_changed)
        parts_status_pub.publish(parts_status_msg);
}


/**
  * Main function for controlling the simulation.
  */
int main(int argc, char** argv)
{
    // Init ROS
    ros::init(argc, argv, "sim_control");

    /// ROS variables/objects
    ros::NodeHandle nh; // Node handle
    std::string robot_name = "/robot_0/";
    // Forklift related
    forklift_status.process_value = 0.0;
    forklift_down = true;
    // Parts related
    parts_status_msg.data.clear();
    for(uint i=0; i < NUM_PARTS; i++)
        parts_status_msg.data.push_back(PART_UNPROCESSED);

    // Publisher for the forklift status
    ros::Publisher forklift_pub = nh.advertise<control_msgs::JointControllerState>(robot_name + "forklift_position_controller/state", 1, true);
    forklift_pub.publish(forklift_status); // Publish initial status
    // Publisher for the parts status
    parts_status_pub = nh.advertise<std_msgs::UInt8MultiArray>("parts_sensor", 1, true);
    parts_status_pub.publish(parts_status_msg);
    // Publisher for the battery level
    batttery_level_pub = nh.advertise<std_msgs::UInt8>("battery/level", 1, true);
    // Publisher for the velocity commands    
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    /// Setup subscribers
    // Real, error-free robot pose (for debug purposes only)
    ros::Subscriber sub_real_pose = nh.subscribe(robot_name + "base_pose_ground_truth", 1, realPoseCallback);
    // Setup subscriber for the forlift commands
    ros::Subscriber sub_f = nh.subscribe(robot_name + "forklift_position_controller/command", 1, forkliftCallback);
    // Baterry manager (subscriber to the clock)
    ros::Subscriber sub_bat = nh.subscribe("/clock", 1, batteryCallback);

    // Advertise the batter charging service
    ros::ServiceServer batt_service = nh.advertiseService("battery/charge", startChargeRequest);

    // Infinite loop
    ros::Rate cycle(10.0); // Rate when no key is being pressed

    while(ros::ok())
    {
        ros::spinOnce();
        double delta = forklift_desired_pos - forklift_status.process_value;
        // Process forklift data and publish if we have a new value
        if( abs(delta) > 0.001 )
        {
            // Move forklift
            if(delta > 0)
                forklift_status.process_value =
                        clipValue(forklift_status.process_value+
                                    FORKLIFT_SPEED*cycle.expectedCycleTime().toSec(),
                                  FORKLIFT_DOWN, FORKLIFT_UP);
            else
                forklift_status.process_value =
                        clipValue(forklift_status.process_value-
                                    FORKLIFT_SPEED*cycle.expectedCycleTime().toSec(),
                                  FORKLIFT_DOWN, FORKLIFT_UP);
            if(forklift_status.process_value < FORKLIFT_DOWN+FORKLIFT_MARGIN)
            {
                if(forklift_down == false)
                {
                    forklift_down = true;
                    forklift_up = false;
                    std::cout << "Forklift is down..." << std::endl;
                }
            } else if(forklift_status.process_value > FORKLIFT_UP-FORKLIFT_MARGIN)
            {
                if(forklift_up == false)
                {
                    forklift_down = false;
                    forklift_up = true;
                    std::cout << "Forklift is up..." << std::endl;
                }
            } else
            {
                forklift_down = false;
                forklift_up = false;
            }
            // Publish new value, if changed
            forklift_status.error = forklift_desired_pos - forklift_status.process_value;
            forklift_pub.publish(forklift_status);
        }

        cycle.sleep(); // Limit the amount of messages when no key is pressed
    }
    return 0;
}

/*
Copyright (c) 2021, Hugo Costelha
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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp> // Pose2D
#include <nav_msgs/msg/odometry.hpp> // Odometry
#include <sensor_msgs/msg/battery_state.hpp> // BatteryState
#include <tf2/utils.h> // Geometry transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Our files
#include "ar_cpp_utils/LocalFrameWorldFrameTransformations.hpp"
#include "ar_utils/srv/start_charging.hpp"
#include "ar_utils/msg/forklift_state.hpp"

// Other includes
#include <mutex>
#include <atomic>
#include <math.h>

#define RAD2DEG(x) x/3.1415927*180
#define NUM_PARTS 5 // Number of available parts/input/outputs
#define NUM_PMAN 8 // Number of processing machines

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

class SimControl : public rclcpp::Node
{
  public:
    SimControl() : Node("sim_control")
    {
      /// ROS variables/objects
      std::string robot_name = "/robot_0/";

      // Forklift related
      forklift_state_.position = 0.0;
      forklift_state_.moving = false;
      forklift_down_ = true;

      // Battery information
      battery_state_.voltage = 12.0;
      battery_state_.temperature = NAN;  // Not measured
      battery_state_.current = NAN;
      battery_state_.charge = NAN;
      battery_state_.capacity = NAN;
      battery_state_.design_capacity = NAN;
      battery_state_.percentage = 0.80;
      battery_state_.power_supply_status = 
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      battery_state_.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
      battery_state_.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      battery_state_.present = true;
      battery_state_.cell_voltage = {NAN, NAN, NAN};
      battery_state_.cell_temperature = {NAN, NAN, NAN};
      battery_state_.location = "MAIN_SLOT";
      battery_state_.serial_number = "SIMULATED_BATTERY";

      // Parts related
      parts_status_msg_.data.clear();
      for(uint i=0; i < NUM_PARTS; i++)
        parts_status_msg_.data.push_back(PART_UNPROCESSED);

      // Publisher for the forklift status
      pub_forklift_ = create_publisher<ar_utils::msg::ForkliftState>(
          robot_name + "forklift/state", 
          rclcpp::QoS(1).reliable().transient_local());
      // Publisher for the parts status
      pub_parts_status_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
            "parts_sensor", 
            rclcpp::QoS(1).reliable().transient_local());
      // Publisher for the battery level
      pub_batttery_state_ = create_publisher<sensor_msgs::msg::BatteryState>(
            "battery/state", rclcpp::QoS(rclcpp::KeepLast(1)));
      // Publisher for the velocity commands
      pub_vel_ = create_publisher<geometry_msgs::msg::Twist>(
        robot_name + "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)));

      /// Setup subscribers
      // Real, error-free robot pose (for debug purposes only)
      sub_real_pose_ = create_subscription<nav_msgs::msg::Odometry>(
          robot_name + "base_pose_ground_truth", 1,
          std::bind(&SimControl::realPoseCallback, this,
                    std::placeholders::_1));
      // Setup subscriber for the forlift commands
      sub_forklift_ = create_subscription<std_msgs::msg::Float32>(
        robot_name + "forklift/goal_position", 1,
          std::bind(&SimControl::forkliftCallback, this,
                    std::placeholders::_1));
      // Baterry manager (subscriber to the clock)
      sub_bat_ = create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", 1,
          std::bind(&SimControl::batteryCallback, this,
          std::placeholders::_1));

      // Advertise the batter charging service
      // Advertise the battery charging service
      svc_charge_ = create_service<ar_utils::srv::StartCharging>(
          robot_name + "battery/charge",
          std::bind(&SimControl::startChargeRequest,
                    this, 
                    std::placeholders::_1,
                    std::placeholders::_2));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Simulation control started...");

      // Start the main loop using a timer at 10 times per second
      main_loop_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimControl::mainLoop, this));
    }

    /**
     * Service callback for charging requests
     */
    bool startChargeRequest(
      const std::shared_ptr<ar_utils::srv::StartCharging::Request> req,
      std::shared_ptr<ar_utils::srv::StartCharging::Response> res)
    {
      // Store a local copy of the robot pose and velocity
      pose_vel_mutex_.lock();
      geometry_msgs::msg::Pose2D robot_pose = global_robot_pose_;
      geometry_msgs::msg::Pose2D robot_speed = global_robot_speed_;
      pose_vel_mutex_.unlock();

      battery_state_mutex_.lock();
      // Are we cancelling the charge?
      if(req->charge == false)
      {
        battery_state_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        res->charging = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery not charging...");
        battery_state_mutex_.unlock();
        return true;
      }

      // If the robot is not stopped, or too far, return false response
      if((abs(sqrt(pow(robot_speed.x,2) + 
                   pow(robot_speed.y,2))) > MAX_LIN_SPEED_ ||
         (abs(robot_speed.theta) > MAX_ANG_SPEED_)) ||
         (abs(robot_pose.x - charging_pose_.x) > 0.20) ||
         (abs(robot_pose.y - charging_pose_.y) > 0.20) )
      {
        battery_state_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        res->charging = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery not charging...");
      } else
      {
        battery_state_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        res->charging = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery is charging...");
      }
      battery_state_mutex_.unlock();
      return true;
    }

    /**
     * Periodic callback to control charging/discharging
     */
    void batteryCallback(const rosgraph_msgs::msg::Clock::SharedPtr)
    {
      battery_state_mutex_.lock();
      if(battery_state_.power_supply_status != 
         sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING)
      {
        // Check if our position is still good, and that we are stopped
        pose_vel_mutex_.lock();
        geometry_msgs::msg::Pose2D robot_pose = global_robot_pose_;
        geometry_msgs::msg::Pose2D robot_speed = global_robot_speed_;
        pose_vel_mutex_.unlock();
        if((abs(sqrt(pow(robot_speed.x,2) + 
                     pow(robot_speed.y,2))) > MAX_LIN_SPEED_ ||
           (abs(robot_speed.theta) > MAX_ANG_SPEED_)) ||
           (abs(robot_pose.x - charging_pose_.x) > 0.20) ||
           (abs(robot_pose.y - charging_pose_.y) > 0.20) )
        {
          battery_state_.power_supply_status = 
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        } else
        {
          if(battery_state_.percentage < 1.0 - BATT_CHARGE_DELTA_)
          {
            battery_state_.percentage += BATT_CHARGE_DELTA_;
          } else {
            battery_state_.percentage = 1.0;
            battery_state_.power_supply_status =
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
          }
        }
      } else
      {
        if(battery_state_.percentage > BATT_DISCHARGE_DELTA_)
          battery_state_.percentage -= BATT_DISCHARGE_DELTA_;
        else
          battery_state_.percentage = 0;
      }

      // Pubish the battery state
      pub_batttery_state_->publish(battery_state_);

      // If the battery level is too low, the robot should not move
      if(battery_state_.percentage < 0.03)
      {
        geometry_msgs::msg::Twist vel_cmd;
        vel_cmd.angular.z = 0.0;
        vel_cmd.linear.x = 0.0;
        pub_vel_->publish(vel_cmd);
      }
      battery_state_mutex_.unlock();
    }

    /**
     * Receive and store desired forklift value
     */
    void forkliftCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      // Process forklift request (store desired position)
      forklift_desired_pos_ = msg->data;
    }

    /**
     * Receive and store current robot real pose
     */
    void realPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Store real, error-free pose values given by the simulator
      geometry_msgs::msg::Pose2D robot_pose, robot_speed;
      point_2d part_pos, part_local_pos;
      robot_pose.x = msg->pose.pose.position.x;
      robot_pose.y = msg->pose.pose.position.y;
      robot_pose.theta = tf2::getYaw(msg->pose.pose.orientation);
      robot_speed.x = msg->twist.twist.linear.x;
      robot_speed.y = msg->twist.twist.linear.y;
      robot_speed.theta = msg->twist.twist.angular.z;

      // Update global values
      pose_vel_mutex_.lock();
      global_robot_pose_ = robot_pose;
      global_robot_speed_ = robot_speed;
      pose_vel_mutex_.unlock();

      // If the robot is moving, do nothing
      // Assume we cannot interact with the parts in motion
      bool excess_speed = false;
      if((abs(sqrt(pow(robot_speed.x,2) + 
                   pow(robot_speed.y,2))) > MAX_LIN_SPEED_ ||
         (abs(robot_speed.theta) > MAX_ANG_SPEED_)))
        excess_speed = true;

      // Change each part information according to the forklift position
      //and robot pose
      bool part_ext_status_changed = false;
      for(uint part_num=0; part_num < NUM_PARTS; part_num++)
      {
        float dangle;
        switch(parts_state_[part_num])
        {
          // Part is still unprocessed
          case PART_UNPROCESSED:
            // We cannot start picking up a part if we are moving
            if(excess_speed)
              continue;
            // Get this part position
            part_pos.x = input_parts_[part_num].x;
            part_pos.y = input_parts_[part_num].y;
            world2Local(robot_pose, part_pos, &part_local_pos);
            // Get the angle error
            dangle = atan2(part_local_pos.y, part_local_pos.x);
            // Check if the robot is enough near the approach pose and the
            // forklift is down
            if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET_) &&
               (abs(dangle) < MAX_ANG_ERROR_) && forklift_down_)
            {
              // The robot is well placed to pick the part
              parts_state_[part_num] = PART_UNPROCESSED_PICKING;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                          "Part %d is being picked up from input station...",
                          part_num+1);
            }
            break;

          // Part is being picked up
          case PART_UNPROCESSED_PICKING:
            // We cannot start picking up a part if we are moving
            if(excess_speed)
              continue;
            // Get the distance error
            part_pos.x = input_parts_[part_num].x;
            part_pos.y = input_parts_[part_num].y;
            world2Local(robot_pose, part_pos, &part_local_pos);
            // Get the angle error
            dangle = atan2(part_local_pos.y, part_local_pos.x);
            // Check if the robot is enough near the approach pose and the
            // forklift is up
            if((part_local_pos.x > 0.0) && 
               (part_local_pos.x < MAX_X_OFFSET_) &&
               (abs(dangle) < MAX_ANG_ERROR_) && forklift_up_)
            {
              // The robot picked the part and now can tansport it
              parts_state_[part_num] = PART_UNPROCESSED_PICKED_UP;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                          "Part %d was picked up from input station...",
                          part_num+1);
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
              part_pos.x = process_parts_[pb_num].x;
              part_pos.y = process_parts_[pb_num].y;
              world2Local(robot_pose, part_pos, &part_local_pos);
              if((part_local_pos.x > 0.0) &&
                 (part_local_pos.x < MAX_X_OFFSET_))
              {
                // Check the angle and the forklift position
                dangle = atan2(part_local_pos.y, part_local_pos.x);
                if((abs(dangle) < MAX_ANG_ERROR_) && (forklift_up_ == false))
                {
                  // The robot is ready to start placing the part
                  parts_state_[part_num] = PART_IN_PROCESS_DROPING;
                  parts_location_[part_num] = pb_num;
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                              "Part %d is being placed for processing...",
                              part_num+1);
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
            part_pos.x = process_parts_[parts_location_[part_num]].x;
            part_pos.y = process_parts_[parts_location_[part_num]].y;
            world2Local(robot_pose, part_pos, &part_local_pos);

            if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET_))
            {
              dangle = atan2(part_local_pos.y, part_local_pos.x);
              if((abs(dangle) < MAX_ANG_ERROR_) && forklift_down_)
              {
                // The robot has placed the part, it is now processing
                parts_state_[part_num] = PART_IN_PROCESS;
                part_ext_status_changed = true;
                parts_status_msg_.data[part_num] = PART_IN_PROCESS;
                int32_t dt = lrint(20.0+(rand()*1.0/RAND_MAX)*30);
                parts_timestamps_[part_num].sec = msg->header.stamp.sec + dt;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d is being processed...",
                            part_num+1);
                break; // No need to check for others
              }
            }
            break;

          // Part is being processed in a processing station
          case PART_IN_PROCESS:
            // If enough time passed by, part is done processing
            // Do not care about the nano secs
            if(msg->header.stamp.sec >= parts_timestamps_[part_num].sec)
            {
              parts_state_[part_num] = PART_PROCESSED;
              part_ext_status_changed = true;
              parts_status_msg_.data[part_num] = PART_PROCESSED;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                          "Part %d has finished processing...",
                          part_num+1);
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
            part_pos.x = process_parts_[parts_location_[part_num]].x;
            part_pos.y = process_parts_[parts_location_[part_num]].y;
            world2Local(robot_pose, part_pos, &part_local_pos);
            if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET_))
            {
              dangle = atan2(part_local_pos.y, part_local_pos.x);
              if((abs(dangle) < MAX_ANG_ERROR_) && forklift_down_)
              {
                parts_state_[part_num] = PART_PROCESSED_PICKING;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d is being picked up from processing...",
                            part_num+1);
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
            part_pos.x = process_parts_[parts_location_[part_num]].x;
            part_pos.y = process_parts_[parts_location_[part_num]].y;
            world2Local(robot_pose, part_pos, &part_local_pos);
            if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET_))
            {
              dangle = atan2(part_local_pos.y, part_local_pos.x);
              if((abs(dangle) < MAX_ANG_ERROR_) && forklift_up_)
              {
                parts_state_[part_num] = PART_PROCESSED_PICKED_UP;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d was picked up from processing...",
                            part_num+1);
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
              part_pos.x = output_parts_[pb_num].x;
              part_pos.y = output_parts_[pb_num].y;
              world2Local(robot_pose, part_pos, &part_local_pos);
              if((part_local_pos.x > 0.0) && 
                 (part_local_pos.x < MAX_X_OFFSET_))
              {
                // Check the angle and the forklift position
                dangle = atan2(part_local_pos.y, part_local_pos.x);
                if((abs(dangle) < MAX_ANG_ERROR_) && forklift_up_)
                {
                  // The robot is ready to start placing the part
                  parts_state_[part_num] = PART_PROCESSED_DELIVERED_DROPING;
                  parts_location_[part_num] = pb_num;
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                              "Part %d is being delivered...",
                              part_num+1);
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
            part_pos.x = output_parts_[parts_location_[part_num]].x;
            part_pos.y = output_parts_[parts_location_[part_num]].y;
            world2Local(robot_pose, part_pos, &part_local_pos);
            if((part_local_pos.x > 0.0) && (part_local_pos.x < MAX_X_OFFSET_))
            {
              dangle = atan2(part_local_pos.y, part_local_pos.x);
              if((abs(dangle) < MAX_ANG_ERROR_) && forklift_down_)
              {
                // The robot has placed the part, it now delivered
                parts_state_[part_num] = PART_DELIVERED;
                part_ext_status_changed = true;
                parts_status_msg_.data[part_num] = PART_DELIVERED;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d has been delivered...",
                            part_num+1);
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
        pub_parts_status_->publish(parts_status_msg_);
    }

    void mainLoop()
    {
      // If we are shutting down, cancl the timer callback
      if(rclcpp::ok() == false)
      {
        main_loop_timer_->cancel();
        return;
      }

      if(first_time_)
      {
        pub_parts_status_->publish(parts_status_msg_);
        pub_forklift_->publish(forklift_state_);
        last_time_ = get_clock()->now();
        first_time_ = false;
      }

      // Else, usual business
      float delta = forklift_desired_pos_ - forklift_state_.position;
      rclcpp::Time current_time = get_clock()->now();
      double dt = (current_time - last_time_).seconds();
      last_time_ = current_time;
      // Process forklift data and publish if we have a new value
      if( abs(delta) > 0.001 )
      {
        // Move forklift
        if(delta > 0)
          forklift_state_.position =
            clipValue(forklift_state_.position + FORKLIFT_SPEED_*dt,
                      FORKLIFT_DOWN_, FORKLIFT_UP_);
        else
          forklift_state_.position =
            clipValue(forklift_state_.position - FORKLIFT_SPEED_*dt,
                      FORKLIFT_DOWN_, FORKLIFT_UP_);
        if(forklift_state_.position < FORKLIFT_DOWN_ + FORKLIFT_MARGIN_)
        {
          if(forklift_down_ == false)
          {
            forklift_down_ = true;
            forklift_up_ = false;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Forklift is down...");
          }
        } else if(forklift_state_.position > FORKLIFT_UP_ - FORKLIFT_MARGIN_)
        {
          if(forklift_up_ == false)
          {
            forklift_down_ = false;
            forklift_up_ = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Forklift is up...");
          }
        } else
        {
          forklift_down_ = false;
          forklift_up_ = false;
        }
        if(forklift_desired_pos_ - forklift_state_.position <= 0.001)
          forklift_state_.moving = false;
        else
          forklift_state_.moving = true;
        // Publish new value, if changed
        pub_forklift_->publish(forklift_state_);
      }
    }

  private:
    bool first_time_ = true;
    // Timer for the periodic callback (main loop)
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    // Forklift related variables
    rclcpp::Publisher<ar_utils::msg::ForkliftState>::SharedPtr pub_forklift_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_forklift_;
    ar_utils::msg::ForkliftState forklift_state_;
    rclcpp::Time last_time_;
    float forklift_desired_pos_ = 0.0; // Received forklift position command
    const float FORKLIFT_SPEED_ = 0.03;  // [m/s]
    const float FORKLIFT_MARGIN_ = 0.005;  // [%] (m)
    // Forklift information
    const float FORKLIFT_DOWN_ = 0.0;  // Down position
    const float FORKLIFT_UP_ = 0.07;  // Up position
    bool forklift_up_ = false,
         forklift_down_ = false;

    // Battery simulation related variables
    sensor_msgs::msg::BatteryState battery_state_;
    std::mutex battery_state_mutex_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_bat_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr 
        pub_batttery_state_;
    rclcpp::Service<ar_utils::srv::StartCharging>::SharedPtr svc_charge_;
    // For now we assume a simple model, with a constant battery discharge and
    // charge 
    const float BATT_DISCHARGE_DELTA_ = 0.0011111;  // Per 0.1 sec 
    const float BATT_CHARGE_DELTA_ = 0.017;  // Per iteration, ~15xDISCHARGE

    // Robot position and velocity, used to control the charging too
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_real_pose_;
    geometry_msgs::msg::Pose2D global_robot_pose_, global_robot_speed_;
    std::mutex pose_vel_mutex_;

    // Maximum error to consider in desired orientation
    const float MAX_X_OFFSET_ = 0.35; // [m]
    const float MAX_DIST_ERROR_ = 0.05; // [m]
    const float MAX_ANG_ERROR_ = 0.07; // [rad] (4º)
    // Maxumum speed to grab/drop the parts
    const float MAX_LIN_SPEED_ = 0.01; // [m/s]
    const float MAX_ANG_SPEED_ = 0.017; // [rad/s] (1º/s)

    // Store the positions of the parts approach pose
    const float DA_ = 0.0; // Distance to part in [m]
    const pose_2d input_parts_[NUM_PARTS] =
      {{2.60, -2.48+DA_, -M_PI_2}, // 1
       {2.23, -2.48+DA_, -M_PI_2}, // 2
       {1.86, -2.48+DA_, -M_PI_2}, // 3
       {1.49, -2.48+DA_, -M_PI_2}, // 4
       {1.12, -2.48+DA_, -M_PI_2}};// 5
    const pose_2d process_parts_[NUM_PMAN] =
      {{1.10-DA_, 0.18, 0.0}, // R1 1
       {1.46+DA_, 0.18, M_PI}, // R2 2
       {1.10-DA_, -0.18, 0.0}, // R3 3
       {1.46+DA_, -0.18, M_PI}, // R4 4
       {-1.12+DA_, 0.18, M_PI}, // L1 5
       {-1.48-DA_, 0.18, 0.0}, // L2 6
       {-1.12+DA_, -0.18, M_PI}, // L3 7
       {-1.48-DA_, -0.18, 0.0}};// L4 8
    const pose_2d output_parts_[NUM_PARTS] =
      {{-2.62, 2.50-DA_, -M_PI_2}, // 1
       {-2.25, 2.50-DA_, M_PI_2}, // 2
       {-1.88, 2.50-DA_, M_PI_2}, // 3
       {-1.51, 2.50-DA_, M_PI_2}, // 4
       {-1.14, 2.50-DA_, M_PI_2}};// 5
    const pose_2d charging_pose_ = {0.0, -2.0, -M_PI};

    // Parts status
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      pub_parts_status_;
    std_msgs::msg::UInt8MultiArray parts_status_msg_;
    part_state_t parts_state_[NUM_PARTS] = {PART_UNPROCESSED,
                                            PART_UNPROCESSED,
                                            PART_UNPROCESSED,
                                            PART_UNPROCESSED,
                                            PART_UNPROCESSED};

    // Parts location
    // 0:4 - Input wharehouse n
    // 5 - On the move
    // 6:13 - In processing machine n-5
    // 14:18 - Input wharehouse n-13
    uint parts_location_[NUM_PARTS] = {0, 1, 2, 3, 4};
    // Parts processing timestamp
    builtin_interfaces::msg::Time parts_timestamps_[NUM_PARTS];
};


/**
  * Main function for controlling the simulation.
  */
int main(int argc, char** argv)
{
  // Init ROS
  rclcpp::init(argc, argv);
  // Infinite loop with one instance of our class
  rclcpp::spin(std::make_shared<SimControl>());
  rclcpp::shutdown();
  return 0;
}

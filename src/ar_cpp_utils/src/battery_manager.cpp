/*
Copyright (c) 2024, Hugo Costelha
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
WARRANTIES OF MERCHANTABILITY AND FITNESS150 FOR A PARTICULAR PURPOSE ARE
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
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp> // Pose2D
#include <nav_msgs/msg/odometry.hpp> // Odometry
#include <sensor_msgs/msg/battery_state.hpp> // BatteryState
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Our files
#include "ar_cpp_utils/LocalFrameWorldFrameTransformations.hpp"
#include "ar_utils/srv/start_charging.hpp"

// Other includes
#include <mutex>
#include <atomic>

#define RAD2DEG(x) x/3.1415927*180

// For now we assume a simple model, with a constant battery discharge and
// charge 
#define BATT_DISCHARGE_DELTA 0.00055555  // Per 0.1 sec 
#define BATT_CHARGE_DELTA 0.01  // Per iteration, ~15xDISCHARGE

// Maximum error to consider in desired orientation
#define MAX_X_OFFSET 0.35 // [m]
#define MAX_Y_OFFSET 0.35 // [m]
#define MAX_ANG_OFFSET 0.26 // [rad] (15 deg)
// Maxumum speed to grab/drop the parts
#define MAX_LIN_SPEED 0.01 // [m/s]
#define MAX_ANG_SPEED 0.017 // [rad/s] (1º/s)


class BatteryManager : public rclcpp::Node
{
  public:
    BatteryManager() : Node("battery_manager")
    {
      /// Battery start/default values
      battery_sate_.voltage = 12.0;
      battery_sate_.temperature = NAN;  // Not measured
      battery_sate_.current = NAN;
      battery_sate_.charge = NAN;
      battery_sate_.capacity = NAN;
      battery_sate_.design_capacity = NAN;
      battery_sate_.percentage = 0.80;
      battery_sate_.power_supply_status = 
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      battery_sate_.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
      battery_sate_.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      battery_sate_.present = true;
      battery_sate_.cell_voltage = {NAN, NAN, NAN};
      battery_sate_.cell_temperature = {NAN, NAN, NAN};
      battery_sate_.location = "MAIN_SLOT";
      battery_sate_.serial_number = "SIMULATED_BATTERY";

      /// ROS variables/objects

      // Publisher for the battery state
      batttery_level_pub_ = 
          create_publisher<sensor_msgs::msg::BatteryState>(
            "battery/state", rclcpp::QoS(rclcpp::KeepLast(1)));
      // Publisher for the velocity commands    
      vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)));

      /// Setup subscribers
      // Real, error-free robot pose (for debug purposes only)
      sub_real_pose_ = 
        create_subscription<nav_msgs::msg::Odometry>(
          "base_pose_ground_truth", 1,
          std::bind(&BatteryManager::odomCallback, this,
                    std::placeholders::_1));

      // Clock
      sub_bat_ = create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", 1,
          std::bind(&BatteryManager::batteryCallback, this,
          std::placeholders::_1));

      // Advertise the battery charging service
      service_ = create_service<ar_utils::srv::StartCharging>(
          "battery/charge",
          std::bind(&BatteryManager::startChargeRequest,
                    this, 
                    std::placeholders::_1,
                    std::placeholders::_2));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery manager started...");
    }

    /**
     * Service callback for charging requests
     */
    bool startChargeRequest(
        const std::shared_ptr<ar_utils::srv::StartCharging::Request> request,
        std::shared_ptr<ar_utils::srv::StartCharging::Response> response)
    {
      geometry_msgs::msg::Pose2D robot_pose, robot_speed;
      // Store a local copy of the robot pose and velocity
      pose_vel_mutex_.lock();
      robot_pose = robot_pose_;
      robot_speed = robot_speed_;
      pose_vel_mutex_.unlock();

      // Are we cancelling the charge?
      if(request->charge == false)
      {
        if(is_charging_ == false)
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot was already not charging...");
        else
        {
          is_charging_ = false;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot stopped charging...");
        }
        response->charging = false;
        return true;
      }

      // If the robot is not stopped, or too far, return false response
      if ((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED ||
          (abs(robot_speed.theta) > MAX_ANG_SPEED)) ||
          (abs(robot_pose.x - charging_pose_.x) > MAX_X_OFFSET) ||
          (abs(robot_pose.y - charging_pose_.y) > MAX_Y_OFFSET) ||
          (abs(robot_pose.theta - charging_pose_.theta) > MAX_ANG_OFFSET))
      {
        if ((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED) ||
            (abs(robot_speed.theta) > MAX_ANG_SPEED))
          RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                      "Robot is moving, unable to start charging...");
        if ((abs(robot_pose.x - charging_pose_.x) > MAX_X_OFFSET) ||
            (abs(robot_pose.y - charging_pose_.y) > MAX_Y_OFFSET) ||
            (abs(robot_pose.theta - charging_pose_.theta) > MAX_ANG_OFFSET))
          RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Robot not in charging pose, unable to start charging...");
        is_charging_ = false;
        response->charging = false;
      } else
      {
	      if (is_charging_ == true)
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot was already charging...");
        else
        {
          is_charging_ = true;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot started charging...");
        }
        response->charging = true;
      }
      return true;
    }

    /**
     * Periodic callback to control charging/discharging
     */
    void batteryCallback(const rosgraph_msgs::msg::Clock::SharedPtr)
    {
      if(is_charging_)
      {
        // Check if our position is still good, and that we are stopped
        geometry_msgs::msg::Pose2D robot_pose, robot_speed;
        pose_vel_mutex_.lock();
        robot_pose = robot_pose_;
        robot_speed = robot_speed_;
        pose_vel_mutex_.unlock();
        if((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED ||
           (abs(robot_speed.theta) > MAX_ANG_SPEED)) ||
           (abs(robot_pose.x - charging_pose_.x) > MAX_X_OFFSET) ||
           (abs(robot_pose.y - charging_pose_.y) > MAX_Y_OFFSET) ||
           (abs(robot_pose.theta - charging_pose_.theta) > MAX_ANG_OFFSET))
        {
          if((abs(sqrt(pow(robot_speed.x,2)+pow(robot_speed.y,2))) > MAX_LIN_SPEED) ||
             (abs(robot_speed.theta) > MAX_ANG_SPEED))
            RCLCPP_WARN(
                rclcpp::get_logger("rclcpp"),
                "Robot is moving, stop charging...");
          if((abs(robot_pose.x - charging_pose_.x) > MAX_X_OFFSET) ||
              (abs(robot_pose.y - charging_pose_.y) > MAX_Y_OFFSET) ||
              (abs(robot_pose.theta - charging_pose_.theta) > MAX_ANG_OFFSET))
            RCLCPP_WARN(
                rclcpp::get_logger("rclcpp"),
                "Robot not in charging pose, stop charging...");
          is_charging_ = false;
          battery_sate_.power_supply_status = 
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        } else
        {
          if(battery_sate_.percentage < 1.0 - BATT_CHARGE_DELTA)
          {
            battery_sate_.percentage += BATT_CHARGE_DELTA;
            battery_sate_.power_supply_status = 
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
          } else
          {
            battery_sate_.percentage = 1.0;
            battery_sate_.power_supply_status = 
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
          }
        }
      } else
      {
        if(battery_sate_.percentage > BATT_DISCHARGE_DELTA)
          battery_sate_.percentage -= BATT_DISCHARGE_DELTA;
        else
          battery_sate_.percentage = 0.0;
        battery_sate_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      }

      // Publish the battery state
      batttery_level_pub_->publish(battery_sate_);

      // If the battery level is too low, the robot should not move
      if(battery_sate_.percentage < 0.05)
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("rclcpp"),
          *get_clock(),
          2000,
          "Low battery, robot might not move correctly...");
        geometry_msgs::msg::Twist vel_cmd;
        vel_cmd.angular.z = 0.0;
        vel_cmd.linear.x = 0.0;
        vel_pub_->publish(vel_cmd);
      }
    }

    /**
     * Receive and store current robot odomoetry and velocity
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Update global values
      pose_vel_mutex_.lock();
      robot_pose_.x = msg->pose.pose.position.x;
      robot_pose_.y = msg->pose.pose.position.y;
      robot_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
      robot_speed_.x = msg->twist.twist.linear.x;
      robot_speed_.y = msg->twist.twist.linear.y;
      robot_speed_.theta = msg->twist.twist.angular.z;
      pose_vel_mutex_.unlock();
    }

  private:
    // Battery simulation related variables
    sensor_msgs::msg::BatteryState battery_sate_;
    std::atomic<bool> is_charging_{false};  // True if charging, false otherwise
    //std::atomic_bool BatteryManager::is_charging_{false};  // True if charging, false otherwise

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
       batttery_level_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_real_pose_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_bat_;
    // Services
    rclcpp::Service<ar_utils::srv::StartCharging>::SharedPtr service_;

    // Robot position and velocity, used to control the charging too
    geometry_msgs::msg::Pose2D robot_pose_, robot_speed_;
    std::mutex pose_vel_mutex_;

    // Charging pose / location
    const pose_2d charging_pose_ = {0.0, -2.0, -M_PI/2.0};
};


/**
  * Main function for controlling the simulation.
  */
int main(int argc, char** argv)
{
  // Init ROS
  rclcpp::init(argc, argv);
  // Infinite loop with one instance of our class
  rclcpp::spin(std::make_shared<BatteryManager>());
  rclcpp::shutdown();
  return 0;
}

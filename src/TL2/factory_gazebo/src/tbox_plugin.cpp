/*
    Copyright (c) 2013, Hugo Costelha
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "tbox_plugin.hpp"

#define TBOX_DEBUG 1

namespace gazebo
{
#define RAD2DEG(x) x/3.1415927*180
// Maximum error to consider in desired orientation
#define MAX_ANG_ERROR 0.17 // [rad] (10º)

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TBoxModelPlugin)

  // Constructor
  TBoxModelPlugin::TBoxModelPlugin():
    part_state_(PART_UNPROCESSED), num_machines_(8),
    num_out_warehouses_(5), first_iteration_(true)
  {
    // Build colors vector
    colors_.resize(NUM_COLORS);
    colors_[RED] = "Gazebo/RedGlow";
    colors_[YELLOW] = "Gazebo/Yellow";
    colors_[GREEN] = "Gazebo/Green";
    colors_[BLUE] = "Gazebo/Blue";


    // Build machines positions (area limits)
    machines_position_.resize(num_machines_);
    double xbase = -1.87,
           xdelta = 0.35,
           ybase = -0.50,
           ydelta = 0.50;
    double new_xbase = xbase;
    uint machine_num = 0;
    for(uint x=0; x < 2; x++) // For each X side
    {
      for(uint y=0; y < 2; y++) // For each Y side
      {
        machines_position_[machine_num].minX = new_xbase;
        machines_position_[machine_num].minY = ybase;
        machines_position_[machine_num].maxX = new_xbase+xdelta;
        machines_position_[machine_num].maxY = ybase+ydelta;
        machine_num++;
        machines_position_[machine_num].minX = new_xbase;
        machines_position_[machine_num].minY = ybase+ydelta;
        machines_position_[machine_num].maxX = new_xbase+xdelta;
        machines_position_[machine_num].maxY = ybase+2*ydelta;
        machine_num++;
        new_xbase += xdelta;
      }
      new_xbase = -xbase-2*xdelta;
    }

    // Build warehouses positions (area limits)
    // Create warehouse working area
    out_warehouses_position_.resize(num_out_warehouses_);
    xbase = -3.30,
    xdelta = 0.50,
    ybase = 3.05,
    ydelta = 0.35;
    for(uint n=0; n < 5; n++)
    {
      out_warehouses_position_[n].minX = xbase;
      out_warehouses_position_[n].minY = ybase;
      out_warehouses_position_[n].maxX = xbase+xdelta;
      out_warehouses_position_[n].maxY = ybase+ydelta;
      xbase += xdelta;
    }

    // Assign a random process time to this box
    process_duration_ = 20.0+(rand()*1.0/RAND_MAX)*30;
    //process_duration_ = 3.0;

    ROS_INFO_STREAM("TBox plugin created (with random processing time of " << process_duration_ << "seg.");
  }

  // Destructor
  TBoxModelPlugin::~TBoxModelPlugin()
  {
  }

  // Load the controller
  void TBoxModelPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
  {
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    parent_ = _parent;
    world_ = parent_->GetWorld();

    if (!parent_) { gzthrow("TBox controller requires a Model as its parent"); }

    // Find base link in the plugin parameters
    std::string baseLinkName;
    if (!_sdf->HasElement("baseLink"))
    {
      baseLinkName = "tbox_base_link";
      ROS_WARN("TBox plugin missing <baseLink>, defaults to %s", baseLinkName.c_str());
    } else
      baseLinkName = _sdf->GetElement("baseLink")->Get<std::string>();

    // Find LED link in the plugin parameters
    std::string ledVisualName;
    if (!_sdf->HasElement("ledVisual"))
    {
      ledVisualName = "tbox_LED";
      ROS_WARN("TBox plugin missing <ledLink>, defaults to %s", ledVisualName.c_str());
    } else
      ledVisualName = _sdf->GetElement("ledVisual")->Get<std::string>();

    // Get LED link visual name
    physics::LinkPtr base_link = parent_->GetLink(baseLinkName);
    if(!base_link)
      ROS_ERROR("Could not get base link");
    ledParentName_ = base_link->GetScopedName();
    ledVisualName_ = ledParentName_ + "::" + ledVisualName;

    // Start node and transport for canging color
    node_ = transport::NodePtr(new transport::Node());
    node_->Init(parent_->GetWorld()->GetName());
    visPub_ = node_->Advertise<msgs::Visual>("~/visual", 10);

    // listen to the update event (broadcast every simulation iteration)
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TBoxModelPlugin::UpdateChild, this));

    ROS_INFO("TBox plugin loaded");
  }

  // Update the controller
  void TBoxModelPlugin::UpdateChild()
  {
    math::Pose boxPose = parent_->GetWorldPose();

#if TBOX_DEBUG
    gzdbg << "----\n"
          << parent_->GetName() << " pos: "
          << boxPose.pos.x << " " << boxPose.pos.y << " " << boxPose.pos.z << " "
          << RAD2DEG(boxPose.rot.z) << " "
          << RAD2DEG(boxPose.pos.y) << " "
          << RAD2DEG(boxPose.pos.z) << "\n";
    if( checkPartInMachine(boxPose) )
      gzdbg << parent_->GetName() << " is in a machine.\n";
    else
      gzdbg << parent_->GetName() << " is NOT in a machine.\n";
    if( checkPartInOutWarehouse(boxPose) )
      gzdbg << parent_->GetName() << " is in an output warehouse.\n";
    else
      gzdbg << parent_->GetName() << " is NOT in an output warehouse.\n";
#endif
    // If the robot is holding the part, do nothing
    if( boxPose.pos.z > 0.01 )
      return;

    msgs::Visual visualMsg;
    // Set the visual's parent name. This should be unique.
    visualMsg.set_name(ledVisualName_);
    // Set the visual's name. This should be unique.
    visualMsg.set_parent_name(ledParentName_);

    if( first_iteration_ )
    {
      // Set initial conditions
      part_state_ = PART_UNPROCESSED;
      part_previous_pose_ = parent_->GetWorldPose();
      part_color_ = RED;
      ROS_INFO_STREAM(parent_->GetName() << " goes " << colors_[part_color_]);
      visualMsg.mutable_material()->mutable_script()->set_name(colors_[part_color_]);
      visPub_->Publish(visualMsg);
      first_iteration_ = false;
    }

    // Usual (non-first-iteration) loop
    part_color_t desired_color = part_color_;
    common::Time timestamp;
    timestamp = world_->GetSimTime();

    // Check situation 1
    if( part_state_ == PART_UNPROCESSED )
    {
      // Check if a part is inside a machine
      if( checkPartInMachine(boxPose) )
      {
        part_state_ = PART_IN_PROCESS;
        timestamp_ = timestamp;
        desired_color = YELLOW;
        ROS_INFO_STREAM(" --> " << parent_->GetName() << " is being processed.");
      }
    } else if( part_state_ == PART_IN_PROCESS )
    {
      // Check if the part is inside a machine.
      if( checkPartInMachine(boxPose) )
      {
  //      gzdbg << (timestamp - timestamp_).double() << "\n.";
        if(timestamp - timestamp_ > process_duration_ )
        {
          // Option 3
          part_state_ = PART_PROCESSED;
          desired_color = GREEN;
          ROS_INFO_STREAM(" --> " << parent_->GetName() << " has been processed.");
        }

      } else
      {
        // Option 2
        ROS_INFO_STREAM(" --> " << parent_->GetName() << " processing stopped.");
        desired_color = RED;
        part_state_ = PART_UNPROCESSED;
      }
    } else if( part_state_== PART_PROCESSED )
    {
      if( checkPartInOutWarehouse(boxPose) )
      {
        // Option 4
        part_state_ = PART_DELIVERED;
        desired_color = BLUE;
        ROS_INFO_STREAM("--> " << parent_->GetName() << " has been delivered.");
      }
    }

    // Change part color if needed
    if( desired_color != part_color_ )
    {
      ROS_INFO_STREAM(parent_->GetName() << " goes " << colors_[desired_color]);
      visualMsg.mutable_material()->mutable_script()->set_name(colors_[desired_color]);
      visPub_->Publish(visualMsg);
      part_color_ = desired_color;
    }
  }

  bool TBoxModelPlugin::checkPartInMachine(math::Pose &object_pose)
  {
    for( uint i =0; i < num_machines_; i++ )
    {
      if( (object_pose.pos.x >= machines_position_[i].minX) &&
          (object_pose.pos.x <= machines_position_[i].maxX) &&
          (object_pose.pos.y >= machines_position_[i].minY) &&
          (object_pose.pos.y <= machines_position_[i].maxY) &&
          (fabs(object_pose.rot.GetPitch()) < MAX_ANG_ERROR) &&
          (fabs(object_pose.rot.GetRoll()) < MAX_ANG_ERROR) )
        return true;
    }

    return false;
  }

  bool TBoxModelPlugin::checkPartInOutWarehouse(math::Pose &object_pose)
  {
    for( uint i = 0; i < num_out_warehouses_; i++ )
    {
      if( (object_pose.pos.x >= out_warehouses_position_[i].minX) &&
          (object_pose.pos.x <= out_warehouses_position_[i].maxX) &&
          (object_pose.pos.y >= out_warehouses_position_[i].minY) &&
          (object_pose.pos.y <= out_warehouses_position_[i].maxY) &&
          (fabs(object_pose.rot.GetPitch()) < MAX_ANG_ERROR) &&
          (fabs(object_pose.rot.GetRoll()) < MAX_ANG_ERROR) )
        return true;
    }
    return false;
  }
}

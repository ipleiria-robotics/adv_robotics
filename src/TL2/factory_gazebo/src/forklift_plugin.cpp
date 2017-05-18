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


#include "forklift_plugin.hpp"

#define FORKLIFT_DEBUG 1

namespace gazebo
{
  // Constructor
  ForkliftModelPlugin::ForkliftModelPlugin()
  {
    ROS_INFO_STREAM("Forlift plugin created .");
  }

  // Destructor
  ForkliftModelPlugin::~ForkliftModelPlugin()
  {
  }

  // Load the controller
  void ForkliftModelPlugin::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf )
  {
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    model_ = _model;

    if (!model_)
    {
      gzthrow("Forlift controller requires a Model as its parent");
    } else
    {
      ROS_INFO_STREAM("Forlift plugin atached to." << model_->GetName());
    }

    double startPosition = 0;
    // Check that the velocity element exists, then read the value
    if (_sdf->HasElement("startPosition"))
      startPosition = _sdf->Get<double>("startPosition");

    // Get the first joint (prismatic joint).
    this->joint_ = _model->GetJoints()[0];

    // Setup a P-controller, with a gain of 0.1.
    this->pid_ = common::PID(0.1, 0, 0);

    // Apply the P-controller to the joint.
    this->model_->GetJointController()->SetPositionPID(this->joint_->GetScopedName(), this->pid_);

    // Set the joint's target positions.
    this->model_->GetJointController()->SetPositionTarget(this->joint_->GetScopedName(), startPosition);

    // listen to the update event (broadcast every simulation iteration)
//    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ForkliftModelPlugin::UpdateChild, this));

    // Create the node
    this->node_ = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
    this->node_->Init(this->model->GetWorld()->GetName());
#else
    this->node_->Init(this->model->GetWorld()->Name());
#endif

    // Create a topic name
    std::string topicName = "~/" + this->model->GetName() + "/pos_cmd";

    // Subscribe to the topic, and register a callback
    this->sub_ = this->node_->Subscribe(topicName,
       &ForkliftPlugin::OnMsg, this);

    ROS_INFO("Forklift plugin loaded and controller started");
    gzdbg << "Forklift plugin loaded and controller startedd";
  }

  // Update the controller
/*  void ForkliftModelPlugin::UpdateChild()
  {
    math::Pose boxPose = parent_->GetWorldPose();

#if FORKLIFT_DEBUG
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
*/

  void ForkliftModelPlugin::SetPosition(const double &position)
  {
    // Set the joint's target velocity.
    this->model_->GetJointController()->SetPositionTarget(
        this->joint_->GetScopedName(), position);
  }

  void OnMsg(ConstVector3dPtr &msg)
  {
    this->SetPosition(msg->x());
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ForkliftModelPlugin)

}

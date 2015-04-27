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

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class pos4
  {
  public:
    double minX;
    double minY;
    double maxX;
    double maxY;
  };

  class TBoxModelPlugin : public ModelPlugin
  {
  public:
    TBoxModelPlugin();

    virtual ~TBoxModelPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void UpdateChild();

  private:
    //< Check if part is in a machine
    bool checkPartInMachine(math::Pose &object_pose);
    //< Check if part is in a warehouse
    bool checkPartInOutWarehouse(math::Pose &object_pose);

    physics::WorldPtr world_;
    std::string ledVisualName_;
    std::string ledParentName_;
    physics::ModelPtr parent_;
    event::ConnectionPtr updateConnection_;
    transport::NodePtr node_;
    transport::PublisherPtr visPub_;

    bool first_iteration_;

    // Gazebo used colors
    std::vector<std::string> colors_;

    enum part_color_t {
      RED=0,
      YELLOW,
      GREEN,
      BLUE,
      NUM_COLORS // Number of available colors
    };

    enum part_state_t {
      PART_UNPROCESSED = 0, // Part was not yet processed
      PART_IN_PROCESS, // Part is being processed
      PART_PROCESSED, // Part was already processed
      PART_DELIVERED // Part was already processed and delivered at its destination
    };

    // Control the status of this part
    part_state_t part_state_;
    // Part timestamp
    common::Time  timestamp_;
    // Previous part pose
    math::Pose part_previous_pose_;
    // Parts color
    part_color_t part_color_;

    // Number of available machines
    uint num_machines_;
    uint num_out_warehouses_;

    // Store where each machine is positioned, so as to know if a part was placed
    //there.
    std::vector<pos4> machines_position_;

    // Store where each output warehouse is positioned, so as to know if a part
    //was placedthere.
    std::vector<pos4> out_warehouses_position_;

    float process_duration_;
  };
}

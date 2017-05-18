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

#ifndef _FORKLIFT_PLUGIN_HH_
#define _FORKLIFT_PLUGIN_HH_

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ForkliftModelPlugin : public ModelPlugin
  {
  public:
    ForkliftModelPlugin();

    virtual ~ForkliftModelPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    void SetPosition(const double &position);
  protected:
    virtual void UpdateChild();

  private:
    void OnMsg(ConstVector3dPtr &_msg);

    physics::ModelPtr model_;
//    event::ConnectionPtr updateConnection_;
//    transport::NodePtr node_;
//    transport::PublisherPtr visPub_;

    //< Pointer to the joint.
    private: physics::JointPtr joint_;

    //< A PID controller for the joint.
    private: common::PID pid_;

    //< A node used for transport
    private: transport::NodePtr node_;

    //< A subscriber to a named topic.
    private: transport::SubscriberPtr sub_;
  };
}

#endif

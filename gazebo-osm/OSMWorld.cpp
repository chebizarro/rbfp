// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include "OSMWorld.h"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(OSMWorld);

void OSMWorld::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(_parent->Name());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub =
            node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory msg;

    // Model file to load
    msg.set_sdf_filename("model://cylinder");

    // Pose to initialize the model to
    msgs::Set(msg.mutable_pose(),
              ignition::math::Pose3d(
                      ignition::math::Vector3d(1, -2, 0),
                      ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    factoryPub->Publish(msg);

}
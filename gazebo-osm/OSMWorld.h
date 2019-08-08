// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_OSMWORLD_H
#define RBPF_OSMWORLD_H

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

using namespace gazebo;

class OSMWorld : public WorldPlugin {

public:
    void Load(physics::WorldPtr, sdf::ElementPtr) override;


};


#endif //RBPF_OSMWORLD_H

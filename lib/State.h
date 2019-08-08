// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_STATE_H
#define RBPF_STATE_H

#include <memory>
#include <rxcpp/rx.hpp>
using namespace rxcpp;

struct State {

    enum class Activity { in_vehicle, walking, still };

    struct shared {
        float x;
        float y;
        float theta;
        Activity activity;
        rxsc::scheduler::clock_type::time_point timestamp;
    };
    std::shared_ptr<shared> data = std::make_shared<shared>();
};



#endif //RBPF_STATE_H

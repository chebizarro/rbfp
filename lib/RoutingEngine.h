// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_ROUTINGENGINE_H
#define RBPF_ROUTINGENGINE_H

#include "Route.h"
#include <rxcpp/rx.hpp>
using namespace rxcpp;
using namespace rxcpp::rxo;
using namespace rxcpp::rxs;

class RoutingEngine {

    virtual observable<Route> getRoute(Point, Point) noexcept = 0;

};


#endif //RBPF_ROUTINGENGINE_H

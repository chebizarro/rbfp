// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_OSMOFFLINEROUTINGENGINE_H
#define RBPF_OSMOFFLINEROUTINGENGINE_H


#include "RoutingEngine.h"

class OSMOfflineRoutingEngine : public RoutingEngine {

public:
    observable<Route> getRoute(Point, Point) noexcept override;

};


#endif //RBPF_OSMOFFLINEROUTINGENGINE_H

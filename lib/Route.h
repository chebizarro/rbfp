// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_ROUTE_H
#define RBPF_ROUTE_H


#include <vector>
#include "Point.h"

struct Route {

    Point start;
    Point end;
    std::vector<Point> path;

};


#endif //RBPF_ROUTE_H

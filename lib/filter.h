// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_FILTER_H
#define RBPF_FILTER_H

#include <functional>

namespace filter {


    inline std::function<observable<Reducer>> predict() {
        return [=](observable<Reducer> r) {

        };
    }

}


#endif //RBPF_FILTER_H
